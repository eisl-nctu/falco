#*****************************************************************************************
#  Create the aquial_bare workspace.
#*****************************************************************************************

# Set the reference directory for source file relative paths
set origin_dir "."

# Set the project name
set proj_name "falco"

variable script_file
set script_file "${proj_name}.tcl"

# Create project
create_project ${proj_name} ./${proj_name} -part xc7k325tffg900-2

# Set project properties
set obj [current_project]
set_property -name "board_part" -value "xilinx.com:kc705:part0:1.6" -objects $obj
set_property -name "default_lib" -value "xil_defaultlib" -objects $obj
set_property -name "sim.ip.auto_export_scripts" -value "1" -objects $obj
set_property -name "simulator_language" -value "Mixed" -objects $obj
set_property -name "xpm_libraries" -value "XPM_CDC XPM_MEMORY" -objects $obj

# Create 'sources_1' fileset (if not found)
if {[string equal [get_filesets -quiet sources_1] ""]} {
  create_fileset -srcset sources_1
}

# Set 'sources_1' fileset object
set obj [get_filesets sources_1]

set files [list \
 [file normalize "$origin_dir/src/core_rtl/issue_queue/int_balance_picker.sv" ]\
 [file normalize "$origin_dir/src/core_rtl/issue_queue/int_issue_queue.sv" ]\
 [file normalize "$origin_dir/src/core_rtl/issue_queue/int_rob_tag_replay_unit.sv" ]\
 [file normalize "$origin_dir/src/core_rtl/issue_queue/empty_entry_finder.sv" ]\
 [file normalize "$origin_dir/src/core_rtl/issue_queue/empty_entry_finder8_wrapper.sv" ]\
 [file normalize "$origin_dir/src/core_rtl/Cache/L1_cache_pkg.sv" ]\
 [file normalize "$origin_dir/src/core_rtl/Commit/reorder_buffer.sv" ]\
 [file normalize "$origin_dir/src/core_rtl/LoadStoreUnit/combine_data_unit.sv" ]\
 [file normalize "$origin_dir/src/core_rtl/register_read/register_read.sv" ]\
 [file normalize "$origin_dir/src/core_rtl/issue_queue/mem_replay_unit.sv" ]\
 [file normalize "$origin_dir/src/core_rtl/execute/ALU.sv" ]\
 [file normalize "$origin_dir/src/core_rtl/Instruction_Decode/ID_stage_io.sv" ]\
 [file normalize "$origin_dir/src/core_rtl/utility/sram_dp.v" ]\
 [file normalize "$origin_dir/src/core_rtl/execute/exe_stage_io.sv" ]\
 [file normalize "$origin_dir/src/core_rtl/Rename_Dispatch/register_map_table.sv" ]\
 [file normalize "$origin_dir/src/core_rtl/Rename_Dispatch/RNDS_stage.sv" ]\
 [file normalize "$origin_dir/src/core_rtl/Address_Generate_stage/AGU_io.sv" ]\
 [file normalize "$origin_dir/src/core_rtl/LoadStoreUnit/load_forward_unit.sv" ]\
 [file normalize "$origin_dir/src/core_rtl/core_top.sv" ]\
 [file normalize "$origin_dir/src/core_rtl/pipeline_recovery/pipeline_control_recovery_io.sv" ]\
 [file normalize "$origin_dir/src/core_rtl/execute/fast_mul.sv" ]\
 [file normalize "$origin_dir/src/core_rtl/TCM/tcm.sv" ]\
 [file normalize "$origin_dir/src/core_rtl/issue_queue/int_iq_ctr_rob_tag_balance_selector.sv" ]\
 [file normalize "$origin_dir/src/core_rtl/execute/divider.sv" ]\
 [file normalize "$origin_dir/src/core_rtl/Instruction_Fetch/IF_stage.sv" ]\
 [file normalize "$origin_dir/src/core_rtl/Instruction_Fetch/gshare_branch_predictor.sv" ]\
 [file normalize "$origin_dir/src/core_rtl/LoadStoreUnit/load_store_unit_io.sv" ]\
 [file normalize "$origin_dir/src/core_rtl/issue_queue/int_issue_queue_io.sv" ]\
 [file normalize "$origin_dir/src/core_rtl/LoadStoreUnit/store_buffer.sv" ]\
 [file normalize "$origin_dir/src/core_rtl/execute/alu_csr_bc_execute_group.sv" ]\
 [file normalize "$origin_dir/src/core_rtl/utility/distri_ram2r1w.v" ]\
 [file normalize "$origin_dir/src/core_rtl/register_read/physical_register_file.sv" ]\
 [file normalize "$origin_dir/src/core_rtl/LoadStoreUnit/load_buffer_io.sv" ]\
 [file normalize "$origin_dir/src/core_rtl/issue_queue/mem_iq_balance_selector_wo_store.sv" ]\
 [file normalize "$origin_dir/src/core_rtl/LoadStoreUnit/last_fetch_store_table.sv" ]\
 [file normalize "$origin_dir/src/core_rtl/LoadStoreUnit/load_buffer.sv" ]\
 [file normalize "$origin_dir/src/core_rtl/Commit/committed_map_table.sv" ]\
 [file normalize "$origin_dir/src/core_rtl/falco_top.sv" ]\
 [file normalize "$origin_dir/src/core_rtl/AXI/core_axi_device_port.v" ]\
 [file normalize "$origin_dir/src/core_rtl/Address_Generate_stage/AGU.sv" ]\
 [file normalize "$origin_dir/src/core_rtl/Rename_Dispatch/prf_freelist.sv" ]\
 [file normalize "$origin_dir/src/core_rtl/Include/Falco_pkg.sv" ]\
 [file normalize "$origin_dir/src/core_rtl/Rename_Dispatch/busy_list.sv" ]\
 [file normalize "$origin_dir/src/core_rtl/Rename_Dispatch/rename_dispatch_io.sv" ]\
 [file normalize "$origin_dir/src/core_rtl/Instruction_Decode/decoder.sv" ]\
 [file normalize "$origin_dir/src/core_rtl/issue_queue/mem_issue_queue_io.sv" ]\
 [file normalize "$origin_dir/src/core_rtl/Instruction_Decode/ID_stage.sv" ]\
 [file normalize "$origin_dir/src/core_rtl/LoadStoreUnit/mem_access.sv" ]\
 [file normalize "$origin_dir/src/core_rtl/pipeline_recovery/pipeline_control_recovery.sv" ]\
 [file normalize "$origin_dir/src/core_rtl/Instruction_Fetch/program_counter.sv" ]\
 [file normalize "$origin_dir/src/core_rtl/Instruction_Fetch/IF_stage_io.sv" ]\
 [file normalize "$origin_dir/src/core_rtl/LoadStoreUnit/load_store_unit.sv" ]\
 [file normalize "$origin_dir/src/core_rtl/Commit/commit_stage_io.sv" ]\
 [file normalize "$origin_dir/src/core_rtl/execute/branch_unit.sv" ]\
 [file normalize "$origin_dir/src/core_rtl/LoadStoreUnit/store_buffer_io.sv" ]\
 [file normalize "$origin_dir/src/core_rtl/utility/sram_dual_port.sv" ]\
 [file normalize "$origin_dir/src/core_rtl/CSR/csr_io.sv" ]\
 [file normalize "$origin_dir/src/core_rtl/LoadStoreUnit/mem_access_io.sv" ]\
 [file normalize "$origin_dir/src/core_rtl/execute/alu_muldiv_execute_group.sv" ]\
 [file normalize "$origin_dir/src/core_rtl/LoadStoreUnit/store_set_id_table.sv" ]\
 [file normalize "$origin_dir/src/core_rtl/register_read/register_read_io.sv" ]\
 [file normalize "$origin_dir/src/core_rtl/CSR/csr.sv" ]\
 [file normalize "$origin_dir/src/core_rtl/issue_queue/mem_issue_queue.sv" ]\
 [file normalize "$origin_dir/src/core_rtl/Instruction_Fetch/BTB.sv" ]\
 [file normalize "$origin_dir/src/soc_rtl/falco_wrapper.v" ]\
 [file normalize "$origin_dir/src/soc_rtl/falco_syn.v" ]\
 [file normalize "$origin_dir/src/soc_rtl/falco_axi_vip_tb.sv"]\
 [file normalize "$origin_dir/src/mem/total_coremark.mem"]\
 [file normalize "$origin_dir/src/mem/total_dhry.mem"]\
 [file normalize "$origin_dir/src/mem/initial_table.mem"]\
]
set imported_files [import_files -fileset sources_1 $files]

set_property top falco_syn [current_fileset]

set file "total_coremark.mem"
set file_obj [get_files -of_objects [get_filesets sources_1] [list "*$file"]]
set_property -name "file_type" -value "Memory File" -objects $file_obj

set file "total_dhry.mem"
set file_obj [get_files -of_objects [get_filesets sources_1] [list "*$file"]]
set_property -name "file_type" -value "Memory File" -objects $file_obj

set file "initial_table.mem"
set file_obj [get_files -of_objects [get_filesets sources_1] [list "*$file"]]
set_property -name "file_type" -value "Memory File" -objects $file_obj

set_property top falco_axi_vip_tb [get_filesets sim_1]
set_property top_lib xil_defaultlib [get_filesets sim_1]

# Import and set 'constrs_1' fileset file properties for local files
set files [list \
 [file normalize "$origin_dir/src/xdc/kc705.xdc"]\
]
set imported_files [import_files -fileset constrs_1 $files]
set files "*.xdc"
set file_objs [get_files -of_objects [get_filesets constrs_1] [list "$files"]]
set_property -name "file_type" -value "XDC" -objects $file_objs

create_ip -name clk_wiz -vendor xilinx.com -library ip -version 6.0 -module_name clk_wiz_0
set_property -dict [list \
    CONFIG.CLK_IN1_BOARD_INTERFACE {sys_diff_clock} \
    CONFIG.RESET_BOARD_INTERFACE {reset} \
    CONFIG.CLKOUT1_REQUESTED_OUT_FREQ {75} \
    CONFIG.PRIM_SOURCE {Differential_clock_capable_pin} \
    CONFIG.PRIM_IN_FREQ {200.000} \
    CONFIG.CLKIN1_JITTER_PS {50.0} \
    CONFIG.MMCM_DIVCLK_DIVIDE {8} \
    CONFIG.MMCM_CLKFBOUT_MULT_F {40.125} \
    CONFIG.MMCM_CLKIN1_PERIOD {5.000} \
    CONFIG.MMCM_CLKIN2_PERIOD {10.0} \
    CONFIG.MMCM_CLKOUT0_DIVIDE_F {13.375} \
    CONFIG.CLKOUT1_JITTER {223.920} \
    CONFIG.CLKOUT1_PHASE_ERROR {236.795}] [get_ips clk_wiz_0]
generate_target all [get_files ${proj_name}/${proj_name}.srcs/sources_1/ip/clk_wiz_0/clk_wiz_0.xci]

create_ip -name axi_uartlite -vendor xilinx.com -library ip -version 2.0 -module_name axi_uartlite_0
set_property -dict [list \
    CONFIG.C_BAUDRATE {115200} \
    CONFIG.C_S_AXI_ACLK_FREQ_HZ {75000000} \
    CONFIG.C_S_AXI_ACLK_FREQ_HZ_d {75} \
    CONFIG.UARTLITE_BOARD_INTERFACE {rs232_uart}] [get_ips axi_uartlite_0]
generate_target all [get_files  ${proj_name}/${proj_name}.srcs/sources_1/ip/axi_uartlite_0/axi_uartlite_0.xci]

create_ip -name axi_vip -vendor xilinx.com -library ip -version 1.1 -module_name axi_vip_0
set_property -dict [list CONFIG.PROTOCOL {AXI4LITE} CONFIG.INTERFACE_MODE {SLAVE} CONFIG.SUPPORTS_NARROW {0} CONFIG.HAS_BURST {0} CONFIG.HAS_LOCK {0} CONFIG.HAS_CACHE {0} CONFIG.HAS_REGION {0} CONFIG.HAS_QOS {0}] [get_ips axi_vip_0]
generate_target all [get_files  ${proj_name}/${proj_name}.srcs/sources_1/ip/axi_vip_0/axi_vip_0.xci]
catch { config_ip_cache -export [get_ips -all axi_vip_0] }
export_ip_user_files -of_objects [get_files ${proj_name}/${proj_name}.srcs/sources_1/ip/axi_vip_0/axi_vip_0.xci] -no_script -sync -force -quiet
create_ip_run [get_files -of_objects [get_fileset sources_1] ${proj_name}/${proj_name}.srcs/sources_1/ip/axi_vip_0/axi_vip_0.xci]

