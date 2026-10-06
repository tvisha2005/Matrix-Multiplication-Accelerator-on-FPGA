`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// vga_display.v  -  Draw three 3x3 matrices + quantisation mode label on 640x480 VGA
//                   PIPELINED VERSION - 5-stage pipeline at 100 MHz
//
// CHANGES FROM ORIGINAL
// ---------------------
//  1. New port: quant_mode[1:0]  - selects the live Q-mode to display on screen.
//  2. font_rom address widened from 7-bit to 8-bit ({char[4:0], row[2:0]})
//     to accommodate new characters Q/./I/N/T (indices 16-20).
//  3. All character-index pipeline registers widened from 4-bit to 5-bit.
//  4. New display region (bottom-centre): shows current quantisation mode
//     as a 4-character string:
//       mode=00 -> "INT8"   (I N T 8)
//       mode=01 -> "Q3.5"   (Q 3 . 5)
//       mode=10 -> "Q511"   (Q 5 1 1)   [abbreviated Q5.11]
//       mode=11 -> "Q115"   (Q 1 1 5)   [abbreviated Q1.15]
//     Displayed in magenta.
//  5. Timing fix: Stage 2b inserted to break long BCD path (see pipeline summary).
//  6. Matrices A and B widened to 4 chars/cell (64px) to display 3-digit
//     signed values, e.g. -128.  Cell layout: [sign][H][T][U].
//     B matrix screen origin moved from X=256 to X=240 to preserve the
//     16px gap after A (which now ends at X=223).
//
// PIPELINE SUMMARY
//   S1:  Region detect, offsets, ccol/crow/cidx, f_col/f_row,
//        flat_idx, label regions, grid flags, MODE region/char.
//   S2:  Matrix element mux, sign + |value|.
//   S2b: BCD upper digits (TT, rem0, TH, rem1) registered.   <-- NEW, breaks timing
//   S3:  BCD lower digits (H,T,U) + character-index selection.
//   S4:  Font ROM drive + pixel bit extraction.
//   S5:  Final RGB output register.
//
// Pipeline latency = 5 pixel clocks.
// Horizontal origins in S1 are shifted -5 to compensate.
//////////////////////////////////////////////////////////////////////////////////

module vga_display (
    input            clk100,
    input            reset,
    input  [71:0]    matA,
    input  [71:0]    matB,
    input  [143:0]   matC,
    input            done,
    input  [1:0]     quant_mode,   // live: 00=INT8 01=Q3.5 10=Q511 11=Q115
    output reg [3:0] red,
    output reg [3:0] green,
    output reg [3:0] blue,
    output           hsync,
    output           vsync
);

// ============================================================
//  1. VGA Sync
// ============================================================
wire [9:0] hpos, vpos;
wire       active, pclk_en;

vga_sync u_sync (
    .clk100  (clk100),
    .reset   (reset),
    .hsync   (hsync),
    .vsync   (vsync),
    .active  (active),
    .hpos    (hpos),
    .vpos    (vpos),
    .pclk_en (pclk_en)
);

// ============================================================
//  2. Latch result matrix on done pulse
// ============================================================
reg [143:0] matC_lat = 144'd0;
always @(posedge clk100) begin
    if (reset)     matC_lat <= 144'd0;
    else if (done) matC_lat <= matC;
end

// ============================================================
//  3. Unpack matrix elements (purely combinational)
// ============================================================
wire signed [7:0]  eA [0:8];
wire signed [7:0]  eB [0:8];
wire signed [15:0] eC [0:8];

genvar gi;
generate
    for (gi = 0; gi < 9; gi = gi + 1) begin : unpack
        assign eA[gi] = $signed(matA    [gi*8  +: 8 ]);
        assign eB[gi] = $signed(matB    [gi*8  +: 8 ]);
        assign eC[gi] = $signed(matC_lat[gi*16 +: 16]);
    end
endgenerate

// ============================================================
//  4. Two Font ROM instances (async read, driven from Stage 4)
//     Address is 8-bit: {char[4:0], row[2:0]}
// ============================================================
reg  [7:0] fr_addr_cell;
wire [7:0] fr_data_cell;
font_rom u_fr_cell (.addr(fr_addr_cell), .data(fr_data_cell));

reg  [7:0] fr_addr_lbl;    // shared for matrix labels AND mode display (mutually exclusive)
wire [7:0] fr_data_lbl;
font_rom u_fr_lbl  (.addr(fr_addr_lbl),  .data(fr_data_lbl));

// ============================================================
//  Helper function: mode string character lookup
//  Returns the font_rom character index for (quant_mode, char_position).
//
//  char_position 0..3 within the 4-char mode string:
//    INT8 : I(18) N(19) T(20) 8(8)
//    Q3.5 : Q(16) 3(3)  .(17) 5(5)
//    Q511 : Q(16) 5(5)  1(1)  1(1)
//    Q115 : Q(16) 1(1)  1(1)  5(5)
// ============================================================
function [4:0] mode_char_idx;
    input [1:0] qmode;
    input [1:0] cpos;
    begin
        case ({qmode, cpos})
            // INT8
            4'b00_00: mode_char_idx = 5'd18;  // I
            4'b00_01: mode_char_idx = 5'd19;  // N
            4'b00_10: mode_char_idx = 5'd20;  // T
            4'b00_11: mode_char_idx = 5'd8;   // 8
            // Q3.5
            4'b01_00: mode_char_idx = 5'd16;  // Q
            4'b01_01: mode_char_idx = 5'd3;   // 3
            4'b01_10: mode_char_idx = 5'd17;  // .
            4'b01_11: mode_char_idx = 5'd5;   // 5
            // Q5.11 -> displayed as "Q511"
            4'b10_00: mode_char_idx = 5'd16;  // Q
            4'b10_01: mode_char_idx = 5'd5;   // 5
            4'b10_10: mode_char_idx = 5'd1;   // 1
            4'b10_11: mode_char_idx = 5'd1;   // 1
            // Q1.15 -> displayed as "Q115"
            4'b11_00: mode_char_idx = 5'd16;  // Q
            4'b11_01: mode_char_idx = 5'd1;   // 1
            4'b11_10: mode_char_idx = 5'd1;   // 1
            4'b11_11: mode_char_idx = 5'd5;   // 5
            default:  mode_char_idx = 5'd15;  // space
        endcase
    end
endfunction

// ============================================================
//  STAGE 1 - Region & Geometry
//  Combinational inputs: hpos, vpos (raw from vga_sync)
//  hpos origins are shifted -4 to compensate for 4-cycle pipeline latency.
// ============================================================

// A: origin X=32, 4 chars/cell x 64px, 3 cells = 192px wide. pipeline-compensated: 32-5=27.
wire signed [10:0] ax_off = {1'b0, hpos} - 11'd27;   // 32 - 5
wire signed [10:0] ay_off = {1'b0, vpos} - 11'd60;
// B: origin X=240 (A ends at 223, x-symbol at [224,239], B starts at 240). pipeline-compensated: 240-5=235.
wire signed [10:0] bx_off = {1'b0, hpos} - 11'd235;  // 240 - 5
wire signed [10:0] by_off = {1'b0, vpos} - 11'd60;
// Matrix C: each cell is 112px wide (7 chars x 16px), 3 cells = 336px total.
// Screen origin X=464, pipeline-compensated: 464-5=459.
wire signed [10:0] cx_off = {1'b0, hpos} - 11'd147;  // 464 - 5
wire signed [10:0] cy_off = {1'b0, vpos} - 11'd196;

// A/B now 192px wide (3 cells x 64px each, 4 chars/cell)
wire s1c_in_A = (!ax_off[10]) && (ax_off < 11'd192) &&
                (!ay_off[10]) && (ay_off < 11'd96);
wire s1c_in_B = (!bx_off[10]) && (bx_off < 11'd192) &&
                (!by_off[10]) && (by_off < 11'd96);
// Matrix C is now 336px wide (3 cols x 112px)
wire s1c_in_C = (!cx_off[10]) && (cx_off < 11'd336) &&
                (!cy_off[10]) && (cy_off < 11'd96);
wire s1c_in_mat = s1c_in_A | s1c_in_B | s1c_in_C;

// For A/B: use 8-bit mx. For C: need 9 bits (up to 335).
wire [7:0] s1c_mx    = s1c_in_A ? ax_off[7:0] : s1c_in_B ? bx_off[7:0] : 8'd0; // A/B only
wire [8:0] s1c_mx_c  = cx_off[8:0];   // C-specific (0..335)
wire [6:0] s1c_my    = s1c_in_A ? ay_off[6:0] : s1c_in_B ? by_off[6:0] : cy_off[6:0];

// ---- A/B column/row geometry (64px cells, 4 chars/cell) ----
wire [1:0] s1c_ccol_ab = (s1c_mx < 8'd64) ? 2'd0 : (s1c_mx < 8'd128) ? 2'd1 : 2'd2;
wire [1:0] s1c_crow    = (s1c_my < 7'd32) ? 2'd0 : (s1c_my < 7'd64) ? 2'd1 : 2'd2;

wire [6:0] s1c_pxc7_ab = (s1c_ccol_ab == 2'd0) ? s1c_mx[6:0] :
                          (s1c_ccol_ab == 2'd1) ? (s1c_mx[6:0] - 7'd64) :
                                                   (s1c_mx[6:0] - 7'd128);
wire [5:0] s1c_pxc_ab  = s1c_pxc7_ab[5:0];

// A/B cidx (4 chars, 2-bit: 0..3)
wire [1:0] s1c_cidx_ab = (s1c_pxc_ab < 6'd16) ? 2'd0 :
                          (s1c_pxc_ab < 6'd32) ? 2'd1 :
                          (s1c_pxc_ab < 6'd48) ? 2'd2 : 2'd3;

// ---- C column/row geometry (112px cells, 7 chars/cell) ----
wire [1:0] s1c_ccol_c  = (s1c_mx_c < 9'd112) ? 2'd0 :
                          (s1c_mx_c < 9'd224) ? 2'd1 : 2'd2;

// Pixel-within-C-cell using 9-bit arithmetic
wire [8:0] s1c_cpxc9   = (s1c_ccol_c == 2'd0) ? s1c_mx_c :
                          (s1c_ccol_c == 2'd1) ? (s1c_mx_c - 9'd112) :
                                                   (s1c_mx_c - 9'd224);
wire [6:0] s1c_cpxc_v  = s1c_cpxc9[6:0];   // 0..111 within cell

// C cidx (7 chars, 3-bit)
wire [2:0] s1c_cidx_c  = (s1c_cpxc_v < 7'd16)  ? 3'd0 :
                          (s1c_cpxc_v < 7'd32)  ? 3'd1 :
                          (s1c_cpxc_v < 7'd48)  ? 3'd2 :
                          (s1c_cpxc_v < 7'd64)  ? 3'd3 :
                          (s1c_cpxc_v < 7'd80)  ? 3'd4 :
                          (s1c_cpxc_v < 7'd96)  ? 3'd5 : 3'd6;

// Unified ccol (always the matrix-column, 0..2)
wire [1:0] s1c_ccol    = s1c_in_C ? s1c_ccol_c : s1c_ccol_ab;

// Pixel-within-char x (font col = divide by 2)
// For A/B use pxc_ab; for C use cpxc_v
wire [5:0] s1c_pxc     = s1c_in_C ? s1c_cpxc_v[5:0] : s1c_pxc_ab;

wire [4:0] s1c_pyc  = (s1c_crow == 2'd0) ? s1c_my[4:0] :
                      (s1c_crow == 2'd1) ? (s1c_my[4:0] - 5'd32) :
                                           (s1c_my[4:0] - 5'd64);

// cidx unified as 3-bit; A/B padded to 3-bit
wire [2:0] s1c_cidx = s1c_in_C ? s1c_cidx_c : {1'b0, s1c_cidx_ab};

// Font glyph row/col (2x scale -> divide by 2)
wire [2:0] s1c_f_col = s1c_pxc[3:1];
wire [2:0] s1c_f_row = s1c_pyc[3:1];

wire s1c_in_txt = (s1c_pyc < 5'd16);

// Flat element index (row-major)
wire [3:0] s1c_flat_idx = (s1c_crow == 2'd0) ? {2'd0, s1c_ccol} :
                           (s1c_crow == 2'd1) ? (4'd3 + {2'd0, s1c_ccol}) :
                                                (4'd6 + {2'd0, s1c_ccol});

// Grid lines
wire [7:0] s1c_b_mx = bx_off[7:0];
wire [6:0] s1c_b_my = by_off[6:0];
wire [8:0] s1c_c_mx = cx_off[8:0];
wire [6:0] s1c_c_my = cy_off[6:0];

// A/B grid: cell borders at 0, 63, 127, 191
wire s1c_gA = s1c_in_A && (s1c_mx == 8'd0   || s1c_mx == 8'd63  ||
                            s1c_mx == 8'd127 || s1c_mx == 8'd191 ||
                            s1c_my == 7'd0   || s1c_my == 7'd31  ||
                            s1c_my == 7'd63  || s1c_my == 7'd95);
wire s1c_gB = s1c_in_B && (s1c_b_mx == 8'd0   || s1c_b_mx == 8'd63  ||
                            s1c_b_mx == 8'd127 || s1c_b_mx == 8'd191 ||
                            s1c_b_my == 7'd0   || s1c_b_my == 7'd31  ||
                            s1c_b_my == 7'd63  || s1c_b_my == 7'd95);
// C grid: 3 cells x 112px wide, borders at 0, 111, 223, 335
wire s1c_gC = s1c_in_C && (s1c_c_mx == 9'd0   || s1c_c_mx == 9'd111 ||
                            s1c_c_mx == 9'd223 || s1c_c_mx == 9'd335 ||
                            s1c_c_my == 7'd0   || s1c_c_my == 7'd31  ||
                            s1c_c_my == 7'd63  || s1c_c_my == 7'd95);
wire s1c_any_grid = s1c_gA | s1c_gB | s1c_gC;

// Matrix label regions (A, B, C, x, =)
// These use raw hpos (no shift) with constants pre-corrected by -4.
// Label A: centred over A matrix (X=32..223 -> centre=128), 16px wide -> [120,135]
wire s1c_inLA = (hpos >= 10'd120) && (hpos < 10'd136) &&
                (vpos >= 10'd30)  && (vpos < 10'd46);
// Label B: centred over B matrix (X=240..431 -> centre=336), 16px wide -> [328,343]
wire s1c_inLB = (hpos >= 10'd328) && (hpos < 10'd344) &&
                (vpos >= 10'd30)  && (vpos < 10'd46);
// Matrix C is now 336px wide starting at X=464; centre at X=632.
wire s1c_inLC = (hpos >= 10'd312) && (hpos < 10'd328) &&
                (vpos >= 10'd166)  && (vpos < 10'd182);
// x-symbol: sits in the 16px gap between A (ends 223) and B (starts 240) -> [224,239]
wire s1c_inMX = (hpos >= 10'd224) && (hpos < 10'd240) &&
                (vpos >= 10'd88)  && (vpos < 10'd104);
wire s1c_inEQ = (hpos >= 10'd124) && (hpos < 10'd140) &&
                (vpos >= 10'd228)  && (vpos < 10'd244);
wire s1c_inLBL = s1c_inLA | s1c_inLB | s1c_inLC | s1c_inMX | s1c_inEQ;

wire [9:0] s1c_lox10 = s1c_inLA ? (hpos - 10'd120) :
                        s1c_inLB ? (hpos - 10'd328) :
                        s1c_inLC ? (hpos - 10'd312) :
                        s1c_inMX ? (hpos - 10'd224) :
                                   (hpos - 10'd124);
wire [9:0] s1c_loy10 = (s1c_inLA | s1c_inLB) ?
                        (vpos - 10'd30) : (s1c_inLC) ? (vpos - 10'd166) : (s1c_inMX) ? (vpos - 10'd88) : (vpos - 10'd228);

wire [2:0] s1c_l_fcol = s1c_lox10[3:1];
wire [2:0] s1c_l_frow = s1c_loy10[3:1];

// Label character index (5-bit)
wire [4:0] s1c_lbl_ch = s1c_inLA ? 5'd11 : s1c_inLB ? 5'd12 :
                         s1c_inLC ? 5'd13 : s1c_inMX ? 5'd21 : 5'd14;

// ---- Quantisation mode display (bottom-centre) --------------------------
// Actual screen position: hpos=[288,351], vpos=[460,475]
// Stage-1 comparison (shifted -5): hpos=[283,346], vpos=[460,475]
// 4 chars * 16px wide = 64px, 16px tall (2x scaled 8x8 glyph)
wire s1c_inLM = (hpos >= 10'd283) && (hpos < 10'd347) &&
                (vpos >= 10'd460) && (vpos < 10'd476);

wire [9:0] s1c_lm_off_x = hpos - 10'd283;
wire [9:0] s1c_lm_off_y = vpos - 10'd460;

// Character position within the 4-char mode string (each char 16px wide)
wire [1:0] s1c_mode_cpos = (s1c_lm_off_x < 10'd16) ? 2'd0 :
                            (s1c_lm_off_x < 10'd32) ? 2'd1 :
                            (s1c_lm_off_x < 10'd48) ? 2'd2 : 2'd3;

// Character index from function (uses live quant_mode)
wire [4:0] s1c_mode_ch = mode_char_idx(quant_mode, s1c_mode_cpos);

// Within-character pixel position (2x scale: divide by 2)
wire [2:0] s1c_m_fcol = s1c_lm_off_x[3:1];
wire [2:0] s1c_m_frow = s1c_lm_off_y[3:1];

// --- Stage 1 pipeline registers ---
reg        s1_active;
reg        s1_in_A,    s1_in_B,    s1_in_C,    s1_in_mat,  s1_in_txt;
reg [2:0]  s1_cidx;
reg [2:0]  s1_f_col,   s1_f_row;
reg [3:0]  s1_flat_idx;
reg        s1_inLA,    s1_inLB,    s1_inLC,    s1_inLBL;
reg [2:0]  s1_l_fcol,  s1_l_frow;
reg [4:0]  s1_lbl_ch;
reg        s1_any_grid;
reg        s1_inLM;
reg [4:0]  s1_mode_ch;
reg [2:0]  s1_m_fcol,  s1_m_frow;

always @(posedge clk100) begin
    if (reset) begin
        s1_active<=0; s1_in_A<=0; s1_in_B<=0; s1_in_C<=0;
        s1_in_mat<=0; s1_in_txt<=0; s1_cidx<=0;
        s1_f_col<=0; s1_f_row<=0; s1_flat_idx<=0;
        s1_inLA<=0; s1_inLB<=0; s1_inLC<=0; s1_inLBL<=0;
        s1_l_fcol<=0; s1_l_frow<=0; s1_lbl_ch<=0; s1_any_grid<=0;
        s1_inLM<=0; s1_mode_ch<=0; s1_m_fcol<=0; s1_m_frow<=0;
    end else if (pclk_en) begin
        s1_active    <= active;
        s1_in_A      <= s1c_in_A;      s1_in_B     <= s1c_in_B;
        s1_in_C      <= s1c_in_C;      s1_in_mat   <= s1c_in_mat;
        s1_in_txt    <= s1c_in_txt;    s1_cidx     <= s1c_cidx;
        s1_f_col     <= s1c_f_col;     s1_f_row    <= s1c_f_row;
        s1_flat_idx  <= s1c_flat_idx;
        s1_inLA      <= s1c_inLA;      s1_inLB     <= s1c_inLB;
        s1_inLC      <= s1c_inLC;      s1_inLBL    <= s1c_inLBL;
        s1_l_fcol    <= s1c_l_fcol;    s1_l_frow   <= s1c_l_frow;
        s1_lbl_ch    <= s1c_lbl_ch;    s1_any_grid <= s1c_any_grid;
        s1_inLM      <= s1c_inLM;
        s1_mode_ch   <= s1c_mode_ch;
        s1_m_fcol    <= s1c_m_fcol;    s1_m_frow   <= s1c_m_frow;
    end
end

// ============================================================
//  STAGE 2 - Matrix Mux + Sign + Abs
//  Inputs: s1_flat_idx, s1_in_A/B/C, eA/eB/eC arrays
// ============================================================

wire signed [15:0] s2c_selA =
    (s1_flat_idx==4'd0)?{{8{eA[0][7]}},eA[0]}:(s1_flat_idx==4'd1)?{{8{eA[1][7]}},eA[1]}:
    (s1_flat_idx==4'd2)?{{8{eA[2][7]}},eA[2]}:(s1_flat_idx==4'd3)?{{8{eA[3][7]}},eA[3]}:
    (s1_flat_idx==4'd4)?{{8{eA[4][7]}},eA[4]}:(s1_flat_idx==4'd5)?{{8{eA[5][7]}},eA[5]}:
    (s1_flat_idx==4'd6)?{{8{eA[6][7]}},eA[6]}:(s1_flat_idx==4'd7)?{{8{eA[7][7]}},eA[7]}:
                                               {{8{eA[8][7]}},eA[8]};

wire signed [15:0] s2c_selB =
    (s1_flat_idx==4'd0)?{{8{eB[0][7]}},eB[0]}:(s1_flat_idx==4'd1)?{{8{eB[1][7]}},eB[1]}:
    (s1_flat_idx==4'd2)?{{8{eB[2][7]}},eB[2]}:(s1_flat_idx==4'd3)?{{8{eB[3][7]}},eB[3]}:
    (s1_flat_idx==4'd4)?{{8{eB[4][7]}},eB[4]}:(s1_flat_idx==4'd5)?{{8{eB[5][7]}},eB[5]}:
    (s1_flat_idx==4'd6)?{{8{eB[6][7]}},eB[6]}:(s1_flat_idx==4'd7)?{{8{eB[7][7]}},eB[7]}:
                                               {{8{eB[8][7]}},eB[8]};

wire signed [15:0] s2c_selC =
    (s1_flat_idx==4'd0)?eC[0]:(s1_flat_idx==4'd1)?eC[1]:(s1_flat_idx==4'd2)?eC[2]:
    (s1_flat_idx==4'd3)?eC[3]:(s1_flat_idx==4'd4)?eC[4]:(s1_flat_idx==4'd5)?eC[5]:
    (s1_flat_idx==4'd6)?eC[6]:(s1_flat_idx==4'd7)?eC[7]:eC[8];

wire signed [15:0] s2c_cell_val = s1_in_A ? s2c_selA : s1_in_B ? s2c_selB : s2c_selC;
wire               s2c_is_neg   = s2c_cell_val[15];
wire [15:0]        s2c_cell_abs = s2c_is_neg ? (~s2c_cell_val + 16'd1) : s2c_cell_val[15:0];

// --- Stage 2 pipeline registers ---
reg        s2_active;
reg        s2_in_A,    s2_in_B,    s2_in_C,    s2_in_mat,  s2_in_txt;
reg [2:0]  s2_cidx;
reg [2:0]  s2_f_col,   s2_f_row;
reg        s2_inLA,    s2_inLB,    s2_inLC,    s2_inLBL;
reg [2:0]  s2_l_fcol,  s2_l_frow;
reg [4:0]  s2_lbl_ch;
reg        s2_any_grid;
reg        s2_is_neg;
reg [15:0] s2_cell_abs;
reg        s2_inLM;
reg [4:0]  s2_mode_ch;
reg [2:0]  s2_m_fcol,  s2_m_frow;

always @(posedge clk100) begin
    if (reset) begin
        s2_active<=0; s2_in_A<=0; s2_in_B<=0; s2_in_C<=0;
        s2_in_mat<=0; s2_in_txt<=0; s2_cidx<=0;
        s2_f_col<=0; s2_f_row<=0;
        s2_inLA<=0; s2_inLB<=0; s2_inLC<=0; s2_inLBL<=0;
        s2_l_fcol<=0; s2_l_frow<=0; s2_lbl_ch<=0; s2_any_grid<=0;
        s2_is_neg<=0; s2_cell_abs<=0;
        s2_inLM<=0; s2_mode_ch<=0; s2_m_fcol<=0; s2_m_frow<=0;
    end else if (pclk_en) begin
        s2_active   <= s1_active;
        s2_in_A     <= s1_in_A;     s2_in_B    <= s1_in_B;
        s2_in_C     <= s1_in_C;     s2_in_mat  <= s1_in_mat;
        s2_in_txt   <= s1_in_txt;   s2_cidx    <= s1_cidx;
        s2_f_col    <= s1_f_col;    s2_f_row   <= s1_f_row;
        s2_inLA     <= s1_inLA;     s2_inLB    <= s1_inLB;
        s2_inLC     <= s1_inLC;     s2_inLBL   <= s1_inLBL;
        s2_l_fcol   <= s1_l_fcol;   s2_l_frow  <= s1_l_frow;
        s2_lbl_ch   <= s1_lbl_ch;   s2_any_grid <= s1_any_grid;
        s2_is_neg   <= s2c_is_neg;  s2_cell_abs <= s2c_cell_abs;
        s2_inLM     <= s1_inLM;
        s2_mode_ch  <= s1_mode_ch;
        s2_m_fcol   <= s1_m_fcol;   s2_m_frow  <= s1_m_frow;
    end
end

// ============================================================
//  STAGE 2b - BCD Upper Digits (registered)
//
//  Registers TT (ten-thousands) and rem0, TH (thousands) and rem1
//  from s2_cell_abs.  This breaks the long S2->S3 combinational
//  path (was ~20 ns, violating 10 ns at 100 MHz) into two halves.
//
//  All sideband signals from S2 are also flopped here so they
//  remain synchronised through the extra stage.
// ============================================================

// -- Combinational upper-BCD (fed from S2 registers) --
wire [3:0] s2bc_TT =
    (s2_cell_abs >= 16'd30000) ? 4'd3 :
    (s2_cell_abs >= 16'd20000) ? 4'd2 :
    (s2_cell_abs >= 16'd10000) ? 4'd1 : 4'd0;

wire [15:0] s2bc_tt_sub =
    (s2bc_TT==4'd3) ? 16'd30000 :
    (s2bc_TT==4'd2) ? 16'd20000 :
    (s2bc_TT==4'd1) ? 16'd10000 : 16'd0;

wire [15:0] s2bc_rem0 = s2_cell_abs - s2bc_tt_sub;

wire [3:0] s2bc_TH =
    (s2bc_rem0 >= 16'd9000) ? 4'd9 : (s2bc_rem0 >= 16'd8000) ? 4'd8 :
    (s2bc_rem0 >= 16'd7000) ? 4'd7 : (s2bc_rem0 >= 16'd6000) ? 4'd6 :
    (s2bc_rem0 >= 16'd5000) ? 4'd5 : (s2bc_rem0 >= 16'd4000) ? 4'd4 :
    (s2bc_rem0 >= 16'd3000) ? 4'd3 : (s2bc_rem0 >= 16'd2000) ? 4'd2 :
    (s2bc_rem0 >= 16'd1000) ? 4'd1 : 4'd0;

wire [15:0] s2bc_th_sub =
    (s2bc_TH==4'd9) ? 16'd9000 : (s2bc_TH==4'd8) ? 16'd8000 :
    (s2bc_TH==4'd7) ? 16'd7000 : (s2bc_TH==4'd6) ? 16'd6000 :
    (s2bc_TH==4'd5) ? 16'd5000 : (s2bc_TH==4'd4) ? 16'd4000 :
    (s2bc_TH==4'd3) ? 16'd3000 : (s2bc_TH==4'd2) ? 16'd2000 :
    (s2bc_TH==4'd1) ? 16'd1000 : 16'd0;

wire [15:0] s2bc_rem1 = s2bc_rem0 - s2bc_th_sub;

// -- S2b pipeline registers --
reg        s2b_active;
reg        s2b_in_A,    s2b_in_B,    s2b_in_C,    s2b_in_mat,  s2b_in_txt;
reg [2:0]  s2b_cidx;
reg [2:0]  s2b_f_col,   s2b_f_row;
reg        s2b_inLA,    s2b_inLB,    s2b_inLC,    s2b_inLBL;
reg [2:0]  s2b_l_fcol,  s2b_l_frow;
reg [4:0]  s2b_lbl_ch;
reg        s2b_any_grid;
reg        s2b_is_neg;
reg [15:0] s2b_cell_abs; // full abs still needed for A/B H/T/U path
reg [3:0]  s2b_TT;
reg [3:0]  s2b_TH;
reg [15:0] s2b_rem1;     // remainder after subtracting TT*10000 + TH*1000
reg        s2b_inLM;
reg [4:0]  s2b_mode_ch;
reg [2:0]  s2b_m_fcol,  s2b_m_frow;

always @(posedge clk100) begin
    if (reset) begin
        s2b_active<=0; s2b_in_A<=0; s2b_in_B<=0; s2b_in_C<=0;
        s2b_in_mat<=0; s2b_in_txt<=0; s2b_cidx<=0;
        s2b_f_col<=0; s2b_f_row<=0;
        s2b_inLA<=0; s2b_inLB<=0; s2b_inLC<=0; s2b_inLBL<=0;
        s2b_l_fcol<=0; s2b_l_frow<=0; s2b_lbl_ch<=0; s2b_any_grid<=0;
        s2b_is_neg<=0; s2b_cell_abs<=0;
        s2b_TT<=0; s2b_TH<=0; s2b_rem1<=0;
        s2b_inLM<=0; s2b_mode_ch<=0; s2b_m_fcol<=0; s2b_m_frow<=0;
    end else if (pclk_en) begin
        s2b_active   <= s2_active;
        s2b_in_A     <= s2_in_A;     s2b_in_B    <= s2_in_B;
        s2b_in_C     <= s2_in_C;     s2b_in_mat  <= s2_in_mat;
        s2b_in_txt   <= s2_in_txt;   s2b_cidx    <= s2_cidx;
        s2b_f_col    <= s2_f_col;    s2b_f_row   <= s2_f_row;
        s2b_inLA     <= s2_inLA;     s2b_inLB    <= s2_inLB;
        s2b_inLC     <= s2_inLC;     s2b_inLBL   <= s2_inLBL;
        s2b_l_fcol   <= s2_l_fcol;   s2b_l_frow  <= s2_l_frow;
        s2b_lbl_ch   <= s2_lbl_ch;   s2b_any_grid <= s2_any_grid;
        s2b_is_neg   <= s2_is_neg;   s2b_cell_abs <= s2_cell_abs;
        s2b_TT       <= s2bc_TT;
        s2b_TH       <= s2bc_TH;
        s2b_rem1     <= s2bc_rem1;
        s2b_inLM     <= s2_inLM;
        s2b_mode_ch  <= s2_mode_ch;
        s2b_m_fcol   <= s2_m_fcol;   s2b_m_frow  <= s2_m_frow;
    end
end

// ============================================================
//  STAGE 3 - BCD Lower Digits + Character Select
//
//  For matrix A/B (8-bit, max abs = 128): 3 chars wide → cidx 0..2
//    pos0 = sign or leading space
//    pos1 = tens (or space if zero)
//    pos2 = units
//
//  For matrix C (16-bit signed, range -32768..32767): 7 chars wide → cidx 0..6
//    5-digit BCD: TThou(0..3), Thou(0..9), H(0..9), T(0..9), U(0..9)
//    Layout:
//      cidx 0 : sign '-' (5'd10) or space (5'd15)
//      cidx 1 : TThou digit or space if leading zero
//      cidx 2 : Thou  digit or space if leading zero
//      cidx 3 : H     digit or space if leading zero
//      cidx 4 : T     digit or space if leading zero
//      cidx 5 : U     digit  (always shown)
//      cidx 6 : space (padding — cell is 7 chars but value fits in 6)
// ============================================================

// ---- 5-digit BCD for C matrix (lower half, from s2b registers) ----
// TT and TH already registered in s2b; continue from s2b_rem1.

// Step 3: hundreds digit
wire [3:0] s3c_H =
    (s2b_rem1 >= 16'd900) ? 4'd9 : (s2b_rem1 >= 16'd800) ? 4'd8 :
    (s2b_rem1 >= 16'd700) ? 4'd7 : (s2b_rem1 >= 16'd600) ? 4'd6 :
    (s2b_rem1 >= 16'd500) ? 4'd5 : (s2b_rem1 >= 16'd400) ? 4'd4 :
    (s2b_rem1 >= 16'd300) ? 4'd3 : (s2b_rem1 >= 16'd200) ? 4'd2 :
    (s2b_rem1 >= 16'd100) ? 4'd1 : 4'd0;

wire [15:0] s3c_h_sub =
    (s3c_H==4'd9) ? 16'd900 : (s3c_H==4'd8) ? 16'd800 :
    (s3c_H==4'd7) ? 16'd700 : (s3c_H==4'd6) ? 16'd600 :
    (s3c_H==4'd5) ? 16'd500 : (s3c_H==4'd4) ? 16'd400 :
    (s3c_H==4'd3) ? 16'd300 : (s3c_H==4'd2) ? 16'd200 :
    (s3c_H==4'd1) ? 16'd100 : 16'd0;

wire [15:0] s3c_rem2 = s2b_rem1 - s3c_h_sub;

// Step 4: tens digit
wire [3:0] s3c_T =
    (s3c_rem2 >= 16'd90) ? 4'd9 : (s3c_rem2 >= 16'd80) ? 4'd8 :
    (s3c_rem2 >= 16'd70) ? 4'd7 : (s3c_rem2 >= 16'd60) ? 4'd6 :
    (s3c_rem2 >= 16'd50) ? 4'd5 : (s3c_rem2 >= 16'd40) ? 4'd4 :
    (s3c_rem2 >= 16'd30) ? 4'd3 : (s3c_rem2 >= 16'd20) ? 4'd2 :
    (s3c_rem2 >= 16'd10) ? 4'd1 : 4'd0;

wire [15:0] s3c_t_sub =
    (s3c_T==4'd9) ? 16'd90 : (s3c_T==4'd8) ? 16'd80 :
    (s3c_T==4'd7) ? 16'd70 : (s3c_T==4'd6) ? 16'd60 :
    (s3c_T==4'd5) ? 16'd50 : (s3c_T==4'd4) ? 16'd40 :
    (s3c_T==4'd3) ? 16'd30 : (s3c_T==4'd2) ? 16'd20 :
    (s3c_T==4'd1) ? 16'd10 : 16'd0;

wire [15:0] s3c_rem3 = s3c_rem2 - s3c_t_sub;
wire [3:0]  s3c_U    = s3c_rem3[3:0];

// ---- Leading-zero flags for C (suppress leading zeros) ----
wire s3c_tt_nz = (s2b_TT != 4'd0);
wire s3c_th_nz = s3c_tt_nz || (s2b_TH != 4'd0);
wire s3c_h_nz  = s3c_th_nz || (s3c_H  != 4'd0);
wire s3c_t_nz  = s3c_h_nz  || (s3c_T  != 4'd0);

// ---- Character select for C (7 positions, cidx 0..6) ----
wire [4:0] s3c_C_ch0 = s2b_is_neg ? 5'd10 : 5'd15;              // sign or space
wire [4:0] s3c_C_ch1 = s3c_tt_nz ? {1'b0, s2b_TT} : 5'd15;     // TThou or space
wire [4:0] s3c_C_ch2 = s3c_th_nz ? {1'b0, s2b_TH} : 5'd15;     // Thou  or space
wire [4:0] s3c_C_ch3 = s3c_h_nz  ? {1'b0, s3c_H}  : 5'd15;   // H     or space
wire [4:0] s3c_C_ch4 = s3c_t_nz  ? {1'b0, s3c_T}  : 5'd15;   // T     or space
wire [4:0] s3c_C_ch5 = {1'b0, s3c_U};                         // U  (always)
// cidx 6 = space (trailing padding slot)

wire [4:0] s3c_this_ch_C =
    (s2b_cidx == 3'd0) ? s3c_C_ch0 :
    (s2b_cidx == 3'd1) ? s3c_C_ch1 :
    (s2b_cidx == 3'd2) ? s3c_C_ch2 :
    (s2b_cidx == 3'd3) ? s3c_C_ch3 :
    (s2b_cidx == 3'd4) ? s3c_C_ch4 :
    (s2b_cidx == 3'd5) ? s3c_C_ch5 : 5'd15;  // cidx 6 = space

// ---- Character select for A/B (4 positions, cidx 0..3) ----
// cidx0 = '-' or space, cidx1 = H or space, cidx2 = T or space, cidx3 = U (always shown)
wire [4:0] s3c_dch0 = s2b_is_neg             ? 5'd10  : 5'd15;
wire [4:0] s3c_dch1 = (s3c_H == 4'd0)        ? 5'd15  : {1'b0, s3c_H};
wire [4:0] s3c_dch2 = (s3c_H==4'd0 && s3c_T==4'd0) ? 5'd15 : {1'b0, s3c_T};
wire [4:0] s3c_dch3 = {1'b0, s3c_U};

wire [4:0] s3c_this_ch_AB =
    (s2b_cidx[1:0] == 2'd0) ? s3c_dch0 :
    (s2b_cidx[1:0] == 2'd1) ? s3c_dch1 :
    (s2b_cidx[1:0] == 2'd2) ? s3c_dch2 : s3c_dch3;

// Final character mux: C uses 7-char path, A/B uses 3-char path
wire [4:0] s3c_this_ch = s2b_in_C ? s3c_this_ch_C : s3c_this_ch_AB;


// --- Stage 3 pipeline registers ---
reg        s3_active;
reg        s3_in_mat,   s3_in_txt,  s3_in_C;
reg [2:0]  s3_f_col,    s3_f_row;
reg        s3_inLA,     s3_inLB,    s3_inLC,    s3_inLBL;
reg [2:0]  s3_l_fcol,   s3_l_frow;
reg [4:0]  s3_lbl_ch;
reg        s3_any_grid;
reg        s3_is_neg;
reg [4:0]  s3_this_ch;
reg        s3_inLM;
reg [4:0]  s3_mode_ch;
reg [2:0]  s3_m_fcol,   s3_m_frow;

always @(posedge clk100) begin
    if (reset) begin
        s3_active<=0; s3_in_mat<=0; s3_in_txt<=0; s3_in_C<=0;
        s3_f_col<=0; s3_f_row<=0;
        s3_inLA<=0; s3_inLB<=0; s3_inLC<=0; s3_inLBL<=0;
        s3_l_fcol<=0; s3_l_frow<=0; s3_lbl_ch<=0; s3_any_grid<=0;
        s3_is_neg<=0; s3_this_ch<=0;
        s3_inLM<=0; s3_mode_ch<=0; s3_m_fcol<=0; s3_m_frow<=0;
    end else if (pclk_en) begin
        s3_active   <= s2b_active;
        s3_in_mat   <= s2b_in_mat;    s3_in_txt  <= s2b_in_txt;
        s3_in_C     <= s2b_in_C;
        s3_f_col    <= s2b_f_col;     s3_f_row   <= s2b_f_row;
        s3_inLA     <= s2b_inLA;      s3_inLB    <= s2b_inLB;
        s3_inLC     <= s2b_inLC;      s3_inLBL   <= s2b_inLBL;
        s3_l_fcol   <= s2b_l_fcol;    s3_l_frow  <= s2b_l_frow;
        s3_lbl_ch   <= s2b_lbl_ch;    s3_any_grid <= s2b_any_grid;
        s3_is_neg   <= s2b_is_neg;    s3_this_ch  <= s3c_this_ch;
        s3_inLM     <= s2b_inLM;
        s3_mode_ch  <= s2b_mode_ch;
        s3_m_fcol   <= s2b_m_fcol;    s3_m_frow  <= s2b_m_frow;
    end
end

// ============================================================
//  STAGE 4 - Font ROM Drive + Pixel Extraction
//  Both matrix labels and mode display share fr_lbl (mutually exclusive).
//  Drive addresses from Stage 3 outputs (async ROM -> same-cycle data).
//  Register outputs into s4_*.
// ============================================================

// Cell ROM: character data for matrix element digits
always @(*) begin
    fr_addr_cell = {s3_this_ch, s3_f_row};   // {5-bit char, 3-bit row} = 8 bits
end

// Label ROM: shared for matrix labels (A/B/C/x/=) and mode display string.
// Priority: matrix label takes precedence; else mode display; else dummy (no pixel lit).
always @(*) begin
    if (s3_inLBL)
        fr_addr_lbl = {s3_lbl_ch, s3_l_frow};    // matrix label character
    else if (s3_inLM)
        fr_addr_lbl = {s3_mode_ch, s3_m_frow};   // mode display character
    else
        fr_addr_lbl = 8'd0;                       // default: index 0, no pixel lit
end

// Pixel bit extraction (2x-scaled glyphs)
wire s4c_cell_pix = s3_in_txt && fr_data_cell[7 - s3_f_col];
wire s4c_lbl_pix  = s3_inLBL  && fr_data_lbl [7 - s3_l_fcol];
wire s4c_mode_pix = s3_inLM   && fr_data_lbl [7 - s3_m_fcol];

// --- Stage 4 pipeline registers ---
reg        s4_active;
reg        s4_in_mat,  s4_in_C;
reg        s4_inLA,    s4_inLB,    s4_inLC,    s4_inLBL;
reg        s4_any_grid;
reg        s4_is_neg;
reg        s4_cell_pix, s4_lbl_pix, s4_mode_pix;

always @(posedge clk100) begin
    if (reset) begin
        s4_active<=0; s4_in_mat<=0; s4_in_C<=0;
        s4_inLA<=0; s4_inLB<=0; s4_inLC<=0; s4_inLBL<=0;
        s4_any_grid<=0; s4_is_neg<=0;
        s4_cell_pix<=0; s4_lbl_pix<=0; s4_mode_pix<=0;
    end else if (pclk_en) begin
        s4_active   <= s3_active;
        s4_in_mat   <= s3_in_mat;   s4_in_C    <= s3_in_C;
        s4_inLA     <= s3_inLA;     s4_inLB    <= s3_inLB;
        s4_inLC     <= s3_inLC;     s4_inLBL   <= s3_inLBL;
        s4_any_grid <= s3_any_grid; s4_is_neg  <= s3_is_neg;
        s4_cell_pix <= s4c_cell_pix;
        s4_lbl_pix  <= s4c_lbl_pix;
        s4_mode_pix <= s4c_mode_pix;
    end
end

// ============================================================
//  STAGE 5 - Final RGB Output Register
//  Priority (low -> high): background -> grid -> cell text
//                           -> matrix labels -> mode label
// ============================================================

always @(posedge clk100) begin
    if (!s4_active) begin
        red <= 4'h0; green <= 4'h0; blue <= 4'h0;
    end else begin
        // Background: dark navy
        red <= 4'h0; green <= 4'h0; blue <= 4'h2;

        // Grid lines: mid-grey
        if (s4_any_grid) begin
            red <= 4'h5; green <= 4'h5; blue <= 4'h5;
        end

        // Cell text (higher priority than grid)
        if (s4_in_mat && s4_cell_pix) begin
            if (s4_is_neg) begin
                red <= 4'hF; green <= 4'h2; blue <= 4'h2;  // negative: red
            end else if (s4_in_C) begin
                red <= 4'h0; green <= 4'hF; blue <= 4'h4;  // result: bright green
            end else begin
                red <= 4'hF; green <= 4'hF; blue <= 4'hF;  // input: white
            end
        end

        // Matrix labels A, B (yellow), C (cyan), x and = (white)
        if (s4_lbl_pix) begin
            if (s4_inLA || s4_inLB) begin
                red <= 4'hF; green <= 4'hE; blue <= 4'h0;  // A, B: yellow
            end else if (s4_inLC) begin
                red <= 4'h0; green <= 4'hF; blue <= 4'hF;  // C: cyan
            end else begin
                red <= 4'hF; green <= 4'hF; blue <= 4'hF;  // x, =: white
            end
        end

        // Quantisation mode label (highest priority, magenta)
        if (s4_mode_pix) begin
            red <= 4'hF; green <= 4'h4; blue <= 4'hF;
        end
    end
end

endmodule
