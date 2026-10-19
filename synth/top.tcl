cd [file normalize [file dirname [info script]]/..]
file mkdir build/vivado
read_verilog [glob rtl/*.v]
read_xdc -mode out_of_context synth/clocks.xdc
synth_design -top spark_clip_top -part xc7a35tcpg236-1 -mode out_of_context
opt_design
place_design
route_design
report_utilization -file build/vivado/utilization.txt
report_timing_summary -max_paths 5 -file build/vivado/timing.txt
report_cdc -file build/vivado/cdc.txt
report_power -file build/vivado/power.txt