`timescale 1ns/1ps
// =============================================================================
//  Program : core_top.sv
//  Author  : Hon-Chou Dai
//  Date    : May/15/2021
// -----------------------------------------------------------------------------
//  Description:
//  This is the Falco pipeline.
// -----------------------------------------------------------------------------
//  Revision information:
//
//    August/07/2021, by Chun-Wei Chao:
//      Add CSR unit & Load buffer
//
// -----------------------------------------------------------------------------
//  License information:
//
//  This software is released under the BSD-3-Clause Licence,
//  see https://opensource.org/licenses/BSD-3-Clause for details.
//  In the following license statements, "software" refers to the
//  "source code" of the complete hardware/software system.
//
//  Copyright 2022,
//                    Embedded Intelligent Systems Lab (EISL)
//                    Deparment of Computer Science
//                    National Yang Ming Chiao Tung Uniersity (NYCU)
//                    Hsinchu, Taiwan.
//
//  All rights reserved.
//
//  Redistribution and use in source and binary forms, with or without
//  modification, are permitted provided that the following conditions are met:
//
//  1. Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//
//  2. Redistributions in binary form must reproduce the above copyright notice,
//     this list of conditions and the following disclaimer in the documentation
//     and/or other materials provided with the distribution.
//
//  3. Neither the name of the copyright holder nor the names of its contributors
//     may be used to endorse or promote products derived from this software
//     without specific prior written permission.
//
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
//  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
//  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
//  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
//  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
//  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
//  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
//  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
//  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
//  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
//  POSSIBILITY OF SUCH DAMAGE.
// =============================================================================

import Falco_pkg::*;

module core_top(
    input                           clk,
    input                           rst,

    //program counter address at reset
    input   [XLEN_WIDTH-1:0]        init_pc,

    //debug bus
    output  logic [1:0]             core_commit_count,
    output  committed_map_update_t  committed_update, /*vivado debug purpose*/

    //Instruction port (from TCM or ICache)
    output  instruction_req_t       instruction_req,
    input   instruction_resp_t      instruction_resp,

    output  logic                   load_dmem_stall,
    //Load check hit port
    output  core_load_ck_hit_req_t  load_ck_hit_req,
    input   core_load_hit_resp_t    load_hit_resp,

    //Load data response port
    output  core_load_data_req_t    load_data_req,
    input   core_load_data_resp_t   load_data_resp,

    //Store port
    output  core_store_req_t        store_req,
    input   core_store_resp_t       store_resp
);

pipeline_control_recovery_io pipe_ctrl_io_inst();
IF_stage_io if_stage_io_inst();
ID_stage_io id_stage_io_inst();
rename_dispatch_io rnds_stage_io_inst();
int_issue_queue_io int_iq_io_inst();
mem_issue_queue_io mem_iq_io_inst();
register_read_io register_read_io_inst();
AGU_io agu_io_inst();
exe_stage_io exe_stage_io_inst();
load_store_unit_io lsu_io_inst();
store_buffer_io sdb_io_inst();
load_buffer_io ldb_io_inst();
commit_stage_io commit_stage_io_inst();
mem_access_io mem_access_io_inst();
csr_io csr_io_inst();
pc_t IF_instr0_predict_pc;
pc_t IF_instr1_predict_pc;
logic exe_stage_muldiv_busy;

pipeline_control_recovery pipeline_ctrl(
    .clk(clk),
    .rst(rst),
    .DMEM_access_stall (load_dmem_stall),
    .io(pipe_ctrl_io_inst),
    .csr_io(csr_io_inst)
);

IF_stage instruction_fetch (
    .clk(clk),
    .rst(rst),

    .init_pc(init_pc),

    .instruction_req(instruction_req),
    .instruction_resp(instruction_resp),
    .instr0_predict_pc(IF_instr0_predict_pc),
    .instr1_predict_pc(IF_instr1_predict_pc),
    .if_stage_io(if_stage_io_inst),
    .exe_io(exe_stage_io_inst),
    .lsu_io(lsu_io_inst),
    .commit_io(commit_stage_io_inst),
    .pipe_ctrl_io(pipe_ctrl_io_inst)
);

ID_stage instruction_decode(
    .clk(clk),
    .rst(rst),

    .instr0_predict_pc(IF_instr0_predict_pc),
    .instr1_predict_pc(IF_instr1_predict_pc),
    .if_stage_io(if_stage_io_inst),
    .id_stage_io(id_stage_io_inst),
    .lsu_io(lsu_io_inst),
    .pipe_ctrl_io(pipe_ctrl_io_inst)
);

RNDS_stage rename_dispatch_stage(
    .clk(clk),
    .rst(rst),

    .id_io(id_stage_io_inst),
    .rnds_io(rnds_stage_io_inst),
    .exe_io(exe_stage_io_inst),
    .lsu_io(lsu_io_inst),
    .commit_io(commit_stage_io_inst),
    .pipe_ctrl_io(pipe_ctrl_io_inst),
    .mem_issue_io(mem_iq_io_inst),
    .int_issue_io(int_iq_io_inst)
);

int_issue_queue INT_IQ(
    .clk(clk),
    .rst(rst),

    .muldiv_busy(exe_stage_muldiv_busy),
    .rnds_io(rnds_stage_io_inst),
    .issue_io(int_iq_io_inst),
    .mem_iq_io(mem_iq_io_inst),
    .broadcast_io(exe_stage_io_inst),
    .lsu_io(lsu_io_inst),
    .commit_io(commit_stage_io_inst),
    .pipe_ctrl_io(pipe_ctrl_io_inst)
);

mem_issue_queue MEM_IQ(
    .clk(clk),
    .rst(rst),

    .rnds_io(rnds_stage_io_inst),
    .issue_io(mem_iq_io_inst),
    .grant_io(int_iq_io_inst),
    .lsu_io(lsu_io_inst),
    .broadcast_io(exe_stage_io_inst),
    .pipe_ctrl_io(pipe_ctrl_io_inst)
);

register_read register_read_stage(
    .clk(clk),
    .rst(rst),

    .int_iq_io(int_iq_io_inst),
    .mem_iq_io(mem_iq_io_inst),
    .wb_io(exe_stage_io_inst),
    .pipe_ctrl_io(pipe_ctrl_io_inst),
    .ma_io(mem_access_io_inst),
    .rr_io(register_read_io_inst)
);

alu_csr_bc_execute_group alu_csr_bc_execute_group_inst(
    .clk(clk),
    .rst(rst),
    .rr_io(register_read_io_inst),
    .ma_io(mem_access_io_inst),
    .exe_io(exe_stage_io_inst),
    .csr_io(csr_io_inst),
    .pipe_ctrl_io(pipe_ctrl_io_inst)
);

alu_muldiv_execute_group alu_muldiv_execute_group_inst(
    .clk(clk),
    .rst(rst),

    .rr_io(register_read_io_inst),
    .exe_io(exe_stage_io_inst),
    .ma_io(mem_access_io_inst),
    .pipe_ctrl_io(pipe_ctrl_io_inst),

    .muldiv_busy(exe_stage_muldiv_busy)
);

AGU address_generate_unit(
    .clk(clk),
    .rst(rst),
    .rr_io(register_read_io_inst),
    .exe_io(exe_stage_io_inst),
    .agu_io(agu_io_inst),
    .sdb_io(sdb_io_inst),
    .ma_io(mem_access_io_inst),
    .pipe_ctrl_io(pipe_ctrl_io_inst)
);

load_store_unit LSU(
    .clk(clk),
    .rst(rst),
    .agu_io(agu_io_inst),
    .pipe_ctrl_io(pipe_ctrl_io_inst),
    .lsu_io(lsu_io_inst),
    .sdb_io(sdb_io_inst),
    .ldb_io(ldb_io_inst),
    .load_ck_hit_req(load_ck_hit_req),
    .load_hit_resp(load_hit_resp)
);

store_buffer SDB(
    .clk(clk),
    .rst(rst),
    .sdb_io(sdb_io_inst),
    .rob_io(commit_stage_io_inst),
    .pipe_ctrl_io(pipe_ctrl_io_inst),
    .store_req(store_req),
    .store_resp(store_resp)
);

load_buffer LDB(
    .clk(clk),
    .rst(rst),
    .sdb_io(sdb_io_inst),
    .rob_io(commit_stage_io_inst),
    .pipe_ctrl_io(pipe_ctrl_io_inst),
    .ldb_io(ldb_io_inst)
);

mem_access MemAccess(
    .clk(clk),
    .rst(rst),
    .load_data_resp(load_data_resp),
    .ma_io(mem_access_io_inst),
    .lsu_io(lsu_io_inst),
    .pipe_ctrl_io(pipe_ctrl_io_inst)
);
reorder_buffer ROB(
    .clk(clk),
    .rst(rst),

    .core_commit_count(core_commit_count),

    .pipe_ctrl_io(pipe_ctrl_io_inst),
    .rnds_io(rnds_stage_io_inst),
    .exe_io(exe_stage_io_inst),
    .lsu_io(lsu_io_inst),
    .ma_io(mem_access_io_inst),
    .commit_io(commit_stage_io_inst),
    .csr_io(csr_io_inst)
);

committed_map_table committed_map_table_inst(
    .clk(clk),
    .rst(rst),

    .io(commit_stage_io_inst)
);

csr_file csr(
    .clk_i(clk),
    .rst_i(rst),

    .io(csr_io_inst)
);

`ifdef FALCO_SIM_DEBUG

logic committed_map_table_valid_dbg;

always @(posedge clk) begin
    committed_map_table_valid_dbg <= committed_update.map_update_0 || committed_update.map_update_1;
end

function [7:0] is_committed_map_table_update;
    /*verilator public*/
    begin
        is_committed_map_table_update = committed_map_table_valid_dbg ? 1 : 0;
    end
endfunction

task ver_dump_committed_gpr_value;
    /*verilator public*/
    $display("x0: %d",register_read_stage.prf.prf_file[committed_map_table_inst.committed_map_table[0]] );
    $display("x1: %d",register_read_stage.prf.prf_file[committed_map_table_inst.committed_map_table[1]] );
    $display("x2: %d",register_read_stage.prf.prf_file[committed_map_table_inst.committed_map_table[2]] );
    $display("x3: %d",register_read_stage.prf.prf_file[committed_map_table_inst.committed_map_table[3]] );
    $display("x4: %d",register_read_stage.prf.prf_file[committed_map_table_inst.committed_map_table[4]] );
    $display("x5: %d",register_read_stage.prf.prf_file[committed_map_table_inst.committed_map_table[5]] );
    $display("x6: %d",register_read_stage.prf.prf_file[committed_map_table_inst.committed_map_table[6]] );
    $display("x7: %d",register_read_stage.prf.prf_file[committed_map_table_inst.committed_map_table[7]] );
    $display("x8: %d",register_read_stage.prf.prf_file[committed_map_table_inst.committed_map_table[8]] );
    $display("x9: %d",register_read_stage.prf.prf_file[committed_map_table_inst.committed_map_table[9]] );
    $display("x10: %d",register_read_stage.prf.prf_file[committed_map_table_inst.committed_map_table[10]] );
    $display("x11: %d",register_read_stage.prf.prf_file[committed_map_table_inst.committed_map_table[11]] );
    $display("x12: %d",register_read_stage.prf.prf_file[committed_map_table_inst.committed_map_table[12]] );
    $display("x13: %d",register_read_stage.prf.prf_file[committed_map_table_inst.committed_map_table[13]] );
    $display("x14: %d",register_read_stage.prf.prf_file[committed_map_table_inst.committed_map_table[14]] );
    $display("x15: %d",register_read_stage.prf.prf_file[committed_map_table_inst.committed_map_table[15]] );
    $display("x16: %d",register_read_stage.prf.prf_file[committed_map_table_inst.committed_map_table[16]] );
    $display("x17: %d",register_read_stage.prf.prf_file[committed_map_table_inst.committed_map_table[17]] );
    $display("x18: %d",register_read_stage.prf.prf_file[committed_map_table_inst.committed_map_table[18]] );
    $display("x19: %d",register_read_stage.prf.prf_file[committed_map_table_inst.committed_map_table[19]] );
    $display("x20: %d",register_read_stage.prf.prf_file[committed_map_table_inst.committed_map_table[20]] );
    $display("x21: %d",register_read_stage.prf.prf_file[committed_map_table_inst.committed_map_table[21]] );
    $display("x22: %d",register_read_stage.prf.prf_file[committed_map_table_inst.committed_map_table[22]] );
    $display("x23: %d",register_read_stage.prf.prf_file[committed_map_table_inst.committed_map_table[23]] );
    $display("x24: %d",register_read_stage.prf.prf_file[committed_map_table_inst.committed_map_table[24]] );
    $display("x25: %d",register_read_stage.prf.prf_file[committed_map_table_inst.committed_map_table[25]] );
    $display("x26: %d",register_read_stage.prf.prf_file[committed_map_table_inst.committed_map_table[26]] );
    $display("x27: %d",register_read_stage.prf.prf_file[committed_map_table_inst.committed_map_table[27]] );
    $display("x28: %d",register_read_stage.prf.prf_file[committed_map_table_inst.committed_map_table[28]] );
    $display("x29: %d",register_read_stage.prf.prf_file[committed_map_table_inst.committed_map_table[29]] );
    $display("x30: %d",register_read_stage.prf.prf_file[committed_map_table_inst.committed_map_table[30]] );
    $display("x31: %d",register_read_stage.prf.prf_file[committed_map_table_inst.committed_map_table[31]] );
endtask

`endif

endmodule
