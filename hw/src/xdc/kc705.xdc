#create_clock -period 5.000 -name clk_p -waveform {0.000 2.500} [get_ports {clk_in1_p }]
#create_clock -period 5.000 -name clk_n -waveform {2.500 5.000} [get_ports {clk_in1_n }]
#set_property CLOCK_DEDICATED_ROUTE FALSE [get_nets clk_IBUF]
#set_property CLOCK_DEDICATED_ROUTE FALSE [get_nets reset_IBUF]

################################################################################
# Constraint file for the Xilinx KC705 development board
################################################################################
# ## Clock Signal using MIG XDC
#set_property -dict {PACKAGE_PIN AD12 IOSTANDARD LVDS} [get_ports sysclk_p]
#create_clock -period 5.000 -name clk_p -waveform {0.000 2.500} -add [get_ports sysclk_p]
#set_property -dict {PACKAGE_PIN AD11 IOSTANDARD LVDS} [get_ports sysclk_n]
#create_clock -period 5.000 -name clk_n -waveform {2.500 5.000} -add [get_ports sysclk_n]

# ################################################################################

set_property DCI_CASCADE {32 34} [get_iobanks 33]

## Reset buttons
set_property -dict {PACKAGE_PIN AB7 IOSTANDARD LVCMOS15} [get_ports reset]

## User buttons: Up, Down, Left, Right, and Center respectively.
#set_property -dict {PACKAGE_PIN AA12 IOSTANDARD LVCMOS15} [get_ports {usr_btn[0]}]
#set_property -dict {PACKAGE_PIN AB12 IOSTANDARD LVCMOS15} [get_ports {usr_btn[1]}]
#set_property -dict {PACKAGE_PIN AC6 IOSTANDARD LVCMOS15} [get_ports {usr_btn[2]}]
#set_property -dict {PACKAGE_PIN AG5 IOSTANDARD LVCMOS15} [get_ports {usr_btn[3]}]
#set_property -dict {PACKAGE_PIN G12 IOSTANDARD LVCMOS25} [get_ports {usr_btn[4]}]

## UART
set_property -dict {PACKAGE_PIN K24 IOSTANDARD LVCMOS25} [get_ports tx]
set_property -dict {PACKAGE_PIN M19 IOSTANDARD LVCMOS25} [get_ports rx]

## LEDs
#set_property -dict {PACKAGE_PIN AB8 IOSTANDARD LVCMOS15} [get_ports {usr_led[0]}]
#set_property -dict {PACKAGE_PIN AA8 IOSTANDARD LVCMOS15} [get_ports {usr_led[1]}]
#set_property -dict {PACKAGE_PIN AC9 IOSTANDARD LVCMOS15} [get_ports {usr_led[2]}]
#set_property -dict {PACKAGE_PIN AB9 IOSTANDARD LVCMOS15} [get_ports {usr_led[3]}]
#set_property -dict {PACKAGE_PIN G19 IOSTANDARD LVCMOS25} [get_ports {usr_led[4]}]

## Switches
#set_property -dict {PACKAGE_PIN Y29  IOSTANDARD LVCMOS25} [get_ports {usr_sw[0]}]
#set_property -dict {PACKAGE_PIN W29  IOSTANDARD LVCMOS25} [get_ports {usr_sw[1]}]
#set_property -dict {PACKAGE_PIN AA28 IOSTANDARD LVCMOS25} [get_ports {usr_sw[2]}]
#set_property -dict {PACKAGE_PIN Y28  IOSTANDARD LVCMOS25} [get_ports {usr_sw[3]}]

## 1620 LCD
#set_property -dict {PACKAGE_PIN AB10 IOSTANDARD LVCMOS15} [get_ports { LCD_E }]
#set_property -dict {PACKAGE_PIN AB13 IOSTANDARD LVCMOS15} [get_ports { LCD_RW }]
#set_property -dict {PACKAGE_PIN Y11  IOSTANDARD LVCMOS15} [get_ports { LCD_RS }]
#set_property -dict {PACKAGE_PIN Y10  IOSTANDARD LVCMOS15} [get_ports { LCD_D[3] }]
#set_property -dict {PACKAGE_PIN AA11 IOSTANDARD LVCMOS15} [get_ports { LCD_D[2] }]
#set_property -dict {PACKAGE_PIN AA10 IOSTANDARD LVCMOS15} [get_ports { LCD_D[1] }]
#set_property -dict {PACKAGE_PIN AA13 IOSTANDARD LVCMOS15} [get_ports { LCD_D[0] }]

