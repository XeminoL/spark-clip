create_clock -name adc_clk -period 10.000 [get_ports adc_clk]
create_clock -name sys_clk -period 10.000 [get_ports sys_clk]
set_clock_groups -asynchronous -group adc_clk -group sys_clk
set_max_delay -datapath_only -from [get_cells -hierarchical -filter {NAME =~ *write_gray_reg*}] -to [get_cells -hierarchical -filter {NAME =~ *write_gray_meta_reg*}] 5.000
set_max_delay -datapath_only -from [get_cells -hierarchical -filter {NAME =~ *read_gray_reg*}] -to [get_cells -hierarchical -filter {NAME =~ *read_gray_meta_reg*}] 5.000