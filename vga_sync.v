`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// vga_sync.v
// 640x480 @ 60 Hz VGA timing generator.
// Input clock: 100 MHz  -> divided by 4 internally -> 25 MHz pixel clock
//
// Horizontal timing (pixels @ 25 MHz):
//   Active   640
//   Front porch 16
//   Sync pulse  96  (hsync LOW)
//   Back porch  48
//   Total      800
//
// Vertical timing (lines):
//   Active   480
//   Front porch 10
//   Sync pulse   2  (vsync LOW)
//   Back porch  33
//   Total      525
//
// Outputs:
//   hpos, vpos  : current pixel coordinates (only valid when active=1)
//   active      : high when inside the visible 640x480 area
//   hsync, vsync: sync signals (active LOW as per VGA standard)
//   pclk_en     : one-100MHz-cycle strobe marking each 25MHz pixel clock edge
//////////////////////////////////////////////////////////////////////////////////
module vga_sync (
    input        clk100,    // 100 MHz system clock
    input        reset,
    output       hsync,
    output       vsync,
    output       active,
    output [9:0] hpos,
    output [9:0] vpos,
    output       pclk_en    // pixel clock enable (25 MHz strobe)
);

// ---- 100MHz -> 25MHz divider (divide by 4) ----
reg [1:0] clk_div = 2'd0;
always @(posedge clk100)
    if (reset) clk_div <= 2'd0;
    else       clk_div <= clk_div + 2'd1;

assign pclk_en = (clk_div == 2'd3); // strobe on every 4th 100MHz cycle

// ---- Counters ----
reg [9:0] h_cnt = 10'd0;
reg [9:0] v_cnt = 10'd0;

always @(posedge clk100) begin
    if (reset) begin
        h_cnt <= 10'd0;
        v_cnt <= 10'd0;
    end
    else if (pclk_en) begin
        if (h_cnt == 10'd799) begin
            h_cnt <= 10'd0;
            if (v_cnt == 10'd524)
                v_cnt <= 10'd0;
            else
                v_cnt <= v_cnt + 10'd1;
        end
        else
            h_cnt <= h_cnt + 10'd1;
    end
end

// ---- Sync & active ----
// hsync LOW during h = 656..751
assign hsync  = ~((h_cnt >= 10'd656) && (h_cnt < 10'd752));
// vsync LOW during v = 490..491
assign vsync  = ~((v_cnt >= 10'd490) && (v_cnt < 10'd492));
assign active = (h_cnt < 10'd640) && (v_cnt < 10'd480);
assign hpos   = h_cnt;
assign vpos   = v_cnt;

endmodule