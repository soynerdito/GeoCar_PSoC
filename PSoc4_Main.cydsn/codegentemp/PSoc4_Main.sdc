# THIS FILE IS AUTOMATICALLY GENERATED
# Project: C:\Projects\PSoC\Bootloading_PSoC5_LP\PSoc4_Main.cydsn\PSoc4_Main.cyprj
# Date: Sun, 22 Dec 2013 22:54:27 GMT
#set_units -time ns
create_clock -name {ADC_ACCELEROMETER_intClock(FFB)} -period 500 -waveform {0 250} [list [get_pins {ClockBlock/ff_div_7}]]
create_clock -name {PSOC5_SCBCLK(FFB)} -period 8666.6666666666661 -waveform {0 4333.33333333333} [list [get_pins {ClockBlock/ff_div_2}]]
create_clock -name {CyHFCLK} -period 41.666666666666664 -waveform {0 20.8333333333333} [list [get_pins {ClockBlock/hfclk}]]
create_generated_clock -name {ADC_ACCELEROMETER_intClock} -source [get_pins {ClockBlock/hfclk}] -edges {1 13 25} [list]
create_generated_clock -name {PSOC5_SCBCLK} -source [get_pins {ClockBlock/hfclk}] -edges {1 209 417} -nominal_period 8666.6666666666661 [list]
create_generated_clock -name {BLUE_IntClock} -source [get_pins {ClockBlock/hfclk}] -edges {1 313 627} [list [get_pins {ClockBlock/udb_div_0}]]
create_clock -name {CyIMO} -period 41.666666666666664 -waveform {0 20.8333333333333} [list [get_pins {ClockBlock/imo}]]
create_clock -name {CySYSCLK} -period 41.666666666666664 -waveform {0 20.8333333333333} [list [get_pins {ClockBlock/sysclk}]]
create_clock -name {CyILO} -period 31250 -waveform {0 15625} [list [get_pins {ClockBlock/ilo}]]
create_clock -name {CyLFCLK} -period 31250 -waveform {0 15625} [list [get_pins {ClockBlock/lfclk}]]

set_false_path -from [get_pins {__ONE__/q}]

# Component constraints for C:\Projects\PSoC\Bootloading_PSoC5_LP\PSoc4_Main.cydsn\TopDesign\TopDesign.cysch
# Project: C:\Projects\PSoC\Bootloading_PSoC5_LP\PSoc4_Main.cydsn\PSoc4_Main.cyprj
# Date: Sun, 22 Dec 2013 22:54:16 GMT
