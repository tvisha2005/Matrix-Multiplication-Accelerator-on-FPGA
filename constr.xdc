## ============================================================
## top.xdc  -  ZedBoard  -  Matrix Multiplication VGA Display
## VIO replaces all switch/button inputs, so only clock and
## VGA pins are needed here.
## ============================================================

## ----- System Clock (100 MHz) -----
set_property PACKAGE_PIN Y9      [get_ports clk]
set_property IOSTANDARD LVCMOS33 [get_ports clk]
create_clock -period 10.000 -name sys_clk [get_ports clk]

## ----- VGA Red -----
set_property PACKAGE_PIN V20     [get_ports {red[0]}]
set_property PACKAGE_PIN U20     [get_ports {red[1]}]
set_property PACKAGE_PIN V19     [get_ports {red[2]}]
set_property PACKAGE_PIN V18     [get_ports {red[3]}]
set_property IOSTANDARD LVCMOS33 [get_ports {red[0]}]
set_property IOSTANDARD LVCMOS33 [get_ports {red[1]}]
set_property IOSTANDARD LVCMOS33 [get_ports {red[2]}]
set_property IOSTANDARD LVCMOS33 [get_ports {red[3]}]

## ----- VGA Green -----
set_property PACKAGE_PIN AB22    [get_ports {green[0]}]
set_property PACKAGE_PIN AA22    [get_ports {green[1]}]
set_property PACKAGE_PIN AB21    [get_ports {green[2]}]
set_property PACKAGE_PIN AA21    [get_ports {green[3]}]
set_property IOSTANDARD LVCMOS33 [get_ports {green[0]}]
set_property IOSTANDARD LVCMOS33 [get_ports {green[1]}]
set_property IOSTANDARD LVCMOS33 [get_ports {green[2]}]
set_property IOSTANDARD LVCMOS33 [get_ports {green[3]}]

## ----- VGA Blue -----
set_property PACKAGE_PIN Y21     [get_ports {blue[0]}]
set_property PACKAGE_PIN Y20     [get_ports {blue[1]}]
set_property PACKAGE_PIN AB20    [get_ports {blue[2]}]
set_property PACKAGE_PIN AB19    [get_ports {blue[3]}]
set_property IOSTANDARD LVCMOS33 [get_ports {blue[0]}]
set_property IOSTANDARD LVCMOS33 [get_ports {blue[1]}]
set_property IOSTANDARD LVCMOS33 [get_ports {blue[2]}]
set_property IOSTANDARD LVCMOS33 [get_ports {blue[3]}]

## ----- VGA Sync -----
set_property PACKAGE_PIN AA19    [get_ports hsync]
set_property PACKAGE_PIN Y19     [get_ports vsync]
set_property IOSTANDARD LVCMOS33 [get_ports hsync]
set_property IOSTANDARD LVCMOS33 [get_ports vsync]