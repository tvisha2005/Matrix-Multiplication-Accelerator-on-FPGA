`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// top.v  -  Matrix Multiplication & VGA Display - ZedBoard (Zynq-7000)
//
// All matrix input and control signals come from a Vivado VIO (Virtual I/O)
// IP core.  No physical switches or buttons are used for input.
//
// =========================================================================
// VIO CONFIGURATION IN VIVADO IP CATALOG
// =========================================================================
// Component Name   : vio_0
//
// INPUT PROBES  (design -> VIO dashboard, READ by user):
//   Input Probe Count : 2
//   probe_in0  width=1  : mult_done  (pulses HIGH for 1 cycle when C is ready)
//   probe_in1  width=1  : mult_busy  (HIGH while multiplier is running)
//
// OUTPUT PROBES (VIO dashboard -> design, WRITTEN by user):
//   Output Probe Count : 6
//   probe_out0  width=8  : vio_val        8-bit signed element value to load
//   probe_out1  width=4  : vio_sel        element index 0-8 (row*3+col)
//   probe_out2  width=1  : vio_mat        0=Matrix A, 1=Matrix B
//   probe_out3  width=1  : vio_load       toggle 0->1->0 to write element
//   probe_out4  width=1  : vio_start      toggle 0->1->0 to start multiply
//   probe_out5  width=2  : vio_quant_mode 00=INT8, 01=Q3.5, 10=Q5.11, 11=Q1.15
//
// =========================================================================
// HOW TO USE IN VIVADO HARDWARE MANAGER
// =========================================================================
//   STEP 1 - Set quantization mode (probe_out5):
//       00 = Integer 8-bit (no fixed-point)
//       01 = Q3.5  (8-bit:  3 integer + 5 fractional bits)
//       10 = Q5.11 (16-bit: 5 integer + 11 fractional bits)
//       11 = Q1.15 (16-bit: 1 integer + 15 fractional bits)
//
//   STEP 2 - Load matrix elements:
//       To load Matrix A element at row=1, col=2 (flat index 5) with value 42:
//         set probe_out0 = 8'd42    (value)
//         set probe_out1 = 4'd5    (element index  = row*3+col)
//         set probe_out2 = 1'b0    (select matrix A)
//         toggle probe_out3: 0 -> 1 -> 0   (triggers single load)
//       Repeat for all 18 elements (9 for A, 9 for B).
//
//   STEP 3 - Start multiply:
//       Toggle probe_out4: 0 -> 1 -> 0
//       Watch result matrix C appear in GREEN on the VGA monitor.
//       probe_in1 goes HIGH during computation.
//       probe_in0 pulses HIGH on completion.
//
//   STEP 4 - To re-run with different data, load new values then toggle
//       probe_out4 again.  Quantization mode is latched at start_pe so
//       set probe_out5 before toggling probe_out4.
//
// VGA outputs: red[3:0], green[3:0], blue[3:0], hsync, vsync
// The VGA screen shows:
//   - Matrix A (white/red for negative) on the left
//   - Matrix B (white/red for negative) in the center
//   - Matrix C = A*B (green/red for negative) on the right
//   - Current quantization mode label at the bottom centre
//////////////////////////////////////////////////////////////////////////////////

module top (
    input        clk,       // 100 MHz system clock - only physical input
    // VGA outputs
    output [3:0] red,
    output [3:0] green,
    output [3:0] blue,
    output       hsync,
    output       vsync
);

//=========================================================================
// VIO instantiation - individual probes (NO single packed bus)
//
// In Vivado IP Catalog:
//   Search "VIO", add IP, configure as:
//   Input Probe Count  = 2
//     PROBE_IN0 Width  = 1   (mult_done)
//     PROBE_IN1 Width  = 1   (mult_busy)
//   Output Probe Count = 6
//     PROBE_OUT0 Width = 8   (value to load)
//     PROBE_OUT1 Width = 4   (element select index)
//     PROBE_OUT2 Width = 1   (matrix select)
//     PROBE_OUT3 Width = 1   (load pulse)
//     PROBE_OUT4 Width = 1   (start multiply)
//     PROBE_OUT5 Width = 2   (quantization mode)
//=========================================================================

// Individual signal wires driven by VIO output probes
wire [7:0] vio_val;        // 8-bit signed value to write into selected element
wire [3:0] vio_sel;        // element index 0..8  (row*3 + col, row-major)
wire       vio_mat;        // 0 = Matrix A,  1 = Matrix B
wire       vio_load;       // rising edge  -> load one element
wire       vio_start;      // rising edge  -> start matrix multiply
wire [1:0] vio_quant_mode; // quantization mode (see header)

// VIO input wires (design status -> VIO dashboard)
wire       mult_done;   // driven by matrix_mult output
wire       mult_busy;   // driven by FSM assign below

vio_0 u_vio (
    .clk        (clk),
    // Design -> dashboard (inputs to VIO)
    .probe_in0  (mult_done),
    .probe_in1  (mult_busy),
    // Dashboard -> design (outputs from VIO)
    .probe_out0 (vio_val),
    .probe_out1 (vio_sel),
    .probe_out2 (vio_mat),
    .probe_out3 (vio_load),
    .probe_out4 (vio_start),
    .probe_out5 (vio_quant_mode)
);

//=========================================================================
// Rising-edge detectors on vio_load and vio_start
// The user toggles the bit in the VIO dashboard (0->1->0).
// We fire a single-clock pulse on the rising edge.
//=========================================================================
reg vio_load_d  = 1'b0;
reg vio_start_d = 1'b0;

wire load_pe  = vio_load  & ~vio_load_d;   // single-cycle load pulse
wire start_pe = vio_start & ~vio_start_d;  // single-cycle start pulse

always @(posedge clk) begin
    vio_load_d  <= vio_load;
    vio_start_d <= vio_start;
end

//=========================================================================
// Latch quantization mode on start_pe so it cannot change mid-computation
//=========================================================================
reg [1:0] quant_mode = 2'b00;   // latched at start of each multiplication

always @(posedge clk) begin
    if (start_pe)
        quant_mode <= vio_quant_mode;
end

//=========================================================================
// Matrix element storage
// matA[0..8] and matB[0..8] are 8-bit signed, row-major:
//   index = row*3 + col
//   [0][0]=0, [0][1]=1, [0][2]=2
//   [1][0]=3, [1][1]=4, [1][2]=5
//   [2][0]=6, [2][1]=7, [2][2]=8
//=========================================================================
reg signed [7:0] matA [0:8];
reg signed [7:0] matB [0:8];

integer ii;
initial begin
    for (ii = 0; ii < 9; ii = ii + 1) begin
        matA[ii] = 8'sd0;
        matB[ii] = 8'sd0;
    end
end

// Write one element on load_pe.
// vio_sel is 4-bit; guard against index > 8.
always @(posedge clk) begin
    if (load_pe && (vio_sel <= 4'd8)) begin
        if (vio_mat == 1'b0)
            matA[vio_sel] <= $signed(vio_val);
        else
            matB[vio_sel] <= $signed(vio_val);
    end
end

//=========================================================================
// Pack matrices into 72-bit flat vectors
//=========================================================================
wire [71:0] A_flat;
wire [71:0] B_flat;

genvar gi;
generate
    for (gi = 0; gi < 9; gi = gi + 1) begin : pack
        assign A_flat[gi*8 +: 8] = matA[gi];
        assign B_flat[gi*8 +: 8] = matB[gi];
    end
endgenerate

//=========================================================================
// Multiplication control FSM
// start_pe triggers: reset multiplier for 8 cycles -> 1 idle -> run
//=========================================================================
localparam MS_IDLE  = 3'd0;
localparam MS_RESET = 3'd1;
localparam MS_WAIT  = 3'd2;
localparam MS_RUN   = 3'd3;
localparam MS_DONE  = 3'd4;

reg [2:0] mstate      = MS_IDLE;
reg [3:0] rst_cnt     = 4'd0;
reg       mult_reset  = 1'b0;
reg       mult_enable = 1'b0;
wire [143:0] C_flat;

always @(posedge clk) begin
    mult_reset  <= 1'b0;   // default off
    mult_enable <= 1'b0;   // default off

    case (mstate)
        MS_IDLE: begin
            if (start_pe) begin
                rst_cnt <= 4'd0;
                mstate  <= MS_RESET;
            end
        end
        MS_RESET: begin
            mult_reset <= 1'b1;
            rst_cnt    <= rst_cnt + 4'd1;
            if (rst_cnt == 4'd7) mstate <= MS_WAIT;
        end
        MS_WAIT: begin
            mstate <= MS_RUN;       // one dead cycle after reset
        end
        MS_RUN: begin
            mult_enable <= 1'b1;
            if (mult_done) mstate <= MS_DONE;
        end
        MS_DONE: begin
            mstate <= MS_IDLE;      // done seen, return to idle
        end
        default: mstate <= MS_IDLE;
    endcase
end

// mult_busy: high whenever state is not IDLE (wire driven by assign)
assign mult_busy = (mstate != MS_IDLE);

//=========================================================================
// Matrix multiplier instance
// quant_mode is latched at start_pe and held stable during computation
//=========================================================================
matrix_mult u_mult (
    .clk        (clk),
    .reset      (mult_reset),
    .enable     (mult_enable),
    .A          (A_flat),
    .B          (B_flat),
    .quant_mode (quant_mode),
    .C          (C_flat),
    .done       (mult_done)
);

//=========================================================================
// VGA display instance
// vio_quant_mode (live) is used only to display the current mode setting
// on screen; the actual computation used quant_mode (latched).
//=========================================================================
vga_display u_display (
    .clk100     (clk),
    .reset      (1'b0),
    .matA       (A_flat),
    .matB       (B_flat),
    .matC       (C_flat),
    .done       (mult_done),
    .quant_mode (vio_quant_mode),   // live - shows what mode is selected
    .red        (red),
    .green      (green),
    .blue       (blue),
    .hsync      (hsync),
    .vsync      (vsync)
);

endmodule
