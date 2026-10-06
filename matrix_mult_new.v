`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// matrix_mult.v  -  Fixed-Point 3x3 Matrix Multiplier (pipelined, 100 MHz)
//
// FIXED-POINT QUANTIZATION SUPPORT
// ---------------------------------
// quant_mode[1:0] selects the arithmetic interpretation of the 8-bit inputs:
//
//   2'b00  INT8  : plain 8-bit signed integers.
//                  Inputs stored as 16-bit sign-extended.
//                  Accumulator = 32-bit (prevents overflow).
//                  Result = accumulator[15:0]  (no shift)
//
//   2'b01  Q3.5  : 8-bit fixed-point, 3 integer + 5 fractional bits.
//                  Range    = [-4.0 .. +3.96875]
//                  Inputs stored as 16-bit sign-extended.
//                  Product  = Q3.5 x Q3.5 = Q7.10 (accumulated in 32-bit)
//                  Result   = accumulator >>> 5, take [15:0]
//
//   2'b10  Q5.11 : 16-bit fixed-point, 5 integer + 11 fractional bits.
//                  Range    = [-16.0 .. +15.9995]
//                  Inputs   = 8-bit VIO value sign-extended to 16-bit.
//                  Product  = Q5.11 x Q5.11 = Q11.22 (accumulated in 32-bit)
//                  Result   = accumulator >>> 11, take [15:0]
//
//   2'b11  Q1.15 : 16-bit fixed-point, 1 integer + 15 fractional bits.
//                  Range    = [-1.0 .. +0.999969]
//                  Inputs   = 8-bit VIO value sign-extended to 16-bit.
//                  Product  = Q1.15 x Q1.15 = Q3.30 (accumulated in 32-bit)
//                  Result   = accumulator >>> 15, take [15:0]
//
// PIPELINE STRUCTURE
// ------------------
//   State LOAD  : unpack 8-bit inputs to 16-bit (sign-extend), clear accumulators
//   State FETCH : Stage-1 - register the two 16-bit operands
//   Stage 2 MAC : 16-bit x 16-bit signed -> 32-bit product, accumulate
//   State DRAIN : absorbs last in-flight MAC
//   State OUTPUT: arithmetic-right-shift, pack lower 16 bits into C
//
// Total latency = 31 clock cycles.
// Interface: A, B = 72-bit (9 x 8-bit); C = 144-bit (9 x 16-bit).
//////////////////////////////////////////////////////////////////////////////////

module matrix_mult (
    input              clk,
    input              reset,       // active-high synchronous reset
    input              enable,      // hold high; re-trigger by pulsing reset then enable
    input  [71:0]      A,           // Matrix A, 9 x 8-bit signed elements
    input  [71:0]      B,           // Matrix B, 9 x 8-bit signed elements
    input  [1:0]       quant_mode,  // 00=INT8  01=Q3.5  10=Q5.11  11=Q1.15
    output reg [143:0] C,           // Matrix C = A*B, 9 x 16-bit signed elements
    output reg         done         // pulses high one clock when result ready
);

localparam QUANT_INT8  = 2'b00;
localparam QUANT_Q35   = 2'b01;
localparam QUANT_Q511  = 2'b10;
localparam QUANT_Q115  = 2'b11;

reg load_type;
// Internal matrices: 16-bit signed (8-bit inputs sign-extended on LOAD)
reg signed [15:0] matA [0:2][0:2];
reg signed [15:0] matB [0:2][0:2];

// 32-bit accumulator: safe for all modes
//   INT8 max: 3 x 127 x 127 = 48387 (fits in 17 bits, 32-bit is safe)
//   Q5.11 / Q1.15: 16x16 product needs 32 bits
reg signed [31:0] matC [0:2][0:2];

// Blocking-assigned temp for arithmetic right-shift in OUTPUT state.
// Used as a combinational intermediate within the clocked always block -
// read in the same for-loop iteration before the non-blocking assignment
// to C.  Synthesizes correctly in Vivado.
reg signed [31:0] result_word;

localparam IDLE   = 3'd0;
localparam LOAD   = 3'd1;
localparam FETCH  = 3'd2;
localparam DRAIN  = 3'd3;
localparam OUTPUT = 3'd4;

reg [2:0] state;
reg [1:0] row, col, k_idx;

// Stage-1 pipeline registers
reg signed [15:0] op_a, op_b;
reg [1:0]         p_row, p_col;
reg               p_valid;

integer pi, pj;

always @(posedge clk) begin
    if (reset) begin
        state   <= IDLE;
        done    <= 1'b0;
        C       <= 144'd0;
        row     <= 2'd0;
        col     <= 2'd0;
        k_idx   <= 2'd0;
        op_a    <= 16'd0;
        op_b    <= 16'd0;
        p_row   <= 2'd0;
        p_col   <= 2'd0;
        p_valid <= 1'b0;
        load_type <= 1'b0;
        for (pi = 0; pi < 3; pi = pi + 1)
            for (pj = 0; pj < 3; pj = pj + 1) begin
                matA[pi][pj] <= 16'd0;
                matB[pi][pj] <= 16'd0;
                matC[pi][pj] <= 32'd0;
            end
    end
    else begin
        done    <= 1'b0;
        p_valid <= 1'b0;

        // Stage 2: MAC - fires every cycle p_valid is asserted
        if (p_valid)
            matC[p_row][p_col] <= matC[p_row][p_col] + op_a * op_b;
        load_type<=quant_mode[1];
        case (state)

            IDLE: begin
                if (enable) state <= LOAD;
            end

            // Sign-extend 8-bit inputs to 16-bit; clear 32-bit accumulators.
            // Mode only affects OUTPUT (right-shift amount); LOAD is mode-agnostic.
            LOAD: begin
                for (pi = 0; pi < 3; pi = pi + 1)
                    if(!load_type) begin
                        for (pj = 0; pj < 3; pj = pj + 1) begin
                            matA[pi][pj] <= {{8{A[(pi*3+pj)*8+7]}}, A[(pi*3+pj)*8 +: 8]};
                            matB[pi][pj] <= {{8{B[(pi*3+pj)*8+7]}}, B[(pi*3+pj)*8 +: 8]};
                            matC[pi][pj] <= 32'd0;
                        end
                    end
                    else begin
                        for (pj = 0; pj < 3; pj = pj + 1) begin
                            matA[pi][pj] <= {A[(pi*3+pj)*8 +: 8], 8'b0};
                            matB[pi][pj] <= {B[(pi*3+pj)*8 +: 8], 8'b0};
                            matC[pi][pj] <= 32'd0;
                        end
                    end
                row   <= 2'd0;
                col   <= 2'd0;
                k_idx <= 2'd0;
                state <= FETCH;
            end

            // Stage 1: register 16-bit operands; MAC fires next cycle.
            // 27 iterations total; move to DRAIN on iteration 27.
            FETCH: begin
                op_a    <= matA[row][k_idx];
                op_b    <= matB[k_idx][col];
                p_row   <= row;
                p_col   <= col;
                p_valid <= 1'b1;

                if (k_idx == 2'd2) begin
                    k_idx <= 2'd0;
                    if (col == 2'd2) begin
                        col <= 2'd0;
                        if (row == 2'd2) begin
                            row   <= 2'd0;
                            state <= DRAIN;
                        end else
                            row <= row + 2'd1;
                    end else
                        col <= col + 2'd1;
                end else
                    k_idx <= k_idx + 2'd1;
            end

            // p_valid=1 from last FETCH fires the final MAC this cycle.
            // p_valid defaults to 0 so no further MACs fire.
            DRAIN: begin
                state <= OUTPUT;
            end

            // All 27 MACs complete.  Arithmetic right-shift by fractional bits,
            // then pack lower 16 bits into C.
            //
            // result_word is a blocking-assigned temp that holds the shifted 32-bit
            // value before the non-blocking assignment extracts [15:0] into C.
            // This pattern is legal and synthesises correctly in Vivado/XST.
            OUTPUT: begin
                for (pi = 0; pi < 3; pi = pi + 1)
                    for (pj = 0; pj < 3; pj = pj + 1) begin
                        case (quant_mode)
                            QUANT_INT8: result_word = matC[pi][pj];
                            QUANT_Q35:  result_word = $signed(matC[pi][pj]) >>> 5;
                            QUANT_Q511: result_word = $signed(matC[pi][pj]) >>> 11;
                            QUANT_Q115: result_word = $signed(matC[pi][pj]) >>> 15;
                            default:    result_word = matC[pi][pj];
                        endcase
                        C[(pi*3+pj)*16 +: 16] <= result_word[15:0];
                    end
                done  <= 1'b1;
                state <= IDLE;
            end

            default: state <= IDLE;

        endcase
    end
end

endmodule
