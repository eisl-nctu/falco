`timescale 1ns/1ps
// =============================================================================
//  Program : RNDS_stage.sv
//  Author  : Hon-Chou Dai
//  Date    : May/15/2021
// -----------------------------------------------------------------------------
//  Description:
//  Rename two instructions' rd register in this stage, and mark the instruction ready or
//  not according to busy_list's output.
// -----------------------------------------------------------------------------
//  Revision information:
//
//    August/07/2021, by Chun-Wei Chao:
//      Add LFST in this stage
//    November/18/2021, by Chun-Wei Chao:
//      Add input port to freelist & register map table for checkpoint recovery mechanism.
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

`timescale 1ns/1ps
// =============================================================================
//  Program : RNDS_stage.sv
//  Author  : Hon-Chou Dai
//  Date    :
// -----------------------------------------------------------------------------
//  Description:
//
// -----------------------------------------------------------------------------
//  Revision information:
//
//    None.
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

module RNDS_stage(
    input clk,
    input rst,

    ID_stage_io.RNDS_stage id_io,
    rename_dispatch_io.RNDS_stage rnds_io,
    exe_stage_io.RNDS_stage exe_io,
    load_store_unit_io.RNDS_stage lsu_io,
    commit_stage_io.RNDS_stage commit_io, //prf recycle and map recovery
    pipeline_control_recovery_io.RNDS_stage pipe_ctrl_io,
    mem_issue_queue_io.RNDS_stage mem_issue_io,
    int_issue_queue_io.RNDS_stage int_issue_io
);

logic wait_csr;

//For register rename request
logic reg_rename_instr0_req;
logic reg_rename_instr1_req;
logic instr0_ready_alloc; //may need some fix so isolate FL request and ROB allocation request,
                          // but currently they are use same logic
                          //If I assure they can use same logic, instr0_ready_alloc,FL_instr0_req,rob.instr0_req
                          // would be merge to make more readibilty
logic instr1_ready_alloc;
logic FL_instr0_check_top,FL_instr1_check_top;

// instr valid from ID stage
logic instr0_pre_req;
logic instr1_pre_req;
logic instr0_is_mem_op;
logic instr1_is_mem_op;

prf_specifier_t instr0_rd_new_prf; //from Freelist
prf_specifier_t instr1_rd_new_prf; //from Freelist
prf_specifier_t instr0_rd_prf; //To RNDS io
prf_specifier_t instr1_rd_prf; //TO RNDS io

prf_specifier_t instr0_rd_stale_prf;
prf_specifier_t instr1_rd_stale_prf;

prf_specifier_t instr0_rs1_prf;
prf_specifier_t instr1_rs1_prf;

prf_specifier_t instr0_rs2_prf;
prf_specifier_t instr1_rs2_prf;

//prf register value ready
logic instr0_rs1_reg_ready;
logic instr0_rs2_reg_ready;
logic instr1_rs1_reg_ready;
logic instr1_rs2_reg_ready;
//prf register value ready or operand don't need prf register (immediate or pc ...)
logic instr0_rs1_operand_ready;
logic instr0_rs2_operand_ready;
logic instr1_rs1_operand_ready;
logic instr1_rs2_operand_ready;


// =====================================================
// combination path for input to RNDS pipeline register
// =====================================================
// pre_ - prefix use for preparing RNDS stage pipeline register value
int_dispatch_pack_t pre_int_instr0_pack; //mask unnecessary data in decoded_instr0
int_dispatch_pack_t pre_int_instr1_pack; //mask unnecessary data in decocde_instr1
mem_dispatch_pack_t pre_mem_instr0_pack; //mask uncecessary data in decoded_instr0
mem_dispatch_pack_t pre_mem_instr1_pack; //mask uncecessary data in decoded_instr1
//branch_info_t pre_int_pack0_br_info;
//branch_info_t pre_int_pack1_br_info;
//spec_tag_t pre_mem_pack0_br_mask;
//spec_tag_t pre_mem_pack1_br_mask;

logic pre_int_pack0_valid;
logic pre_int_pack1_valid;
logic pre_mem_pack0_valid;
logic pre_mem_pack1_valid;
logic pre_int_instr0_rs1_ready;
logic pre_int_instr0_rs2_ready;
logic pre_int_instr1_rs1_ready;
logic pre_int_instr1_rs2_ready;
logic pre_mem_instr0_rs1_ready;
logic pre_mem_instr0_rs2_ready;
logic pre_mem_instr1_rs1_ready;
logic pre_mem_instr1_rs2_ready;

//Store set 
logic [STORE_ID_WIDTH-1 : 0]        store_counter/*verilator public*/;
logic [STORE_ID_WIDTH-1 : 0]        store_next_counter/*verilator public*/;
// logic [SSIT_WIDTH-1 : 0]            instr0_pc; //from io
// logic [SSIT_WIDTH-1 : 0]            instr1_pc; //from io
// logic                               instr0_is_store; //from io
// logic                               instr1_is_store; //from io
logic [STORE_ID_WIDTH-1 : 0]        instr0_store_id;
logic [STORE_ID_WIDTH-1 : 0]        instr1_store_id;
logic                               instr0_predict_result/*verilator public*/;
logic                               instr1_predict_result/*verilator public*/;
// logic                               issue_store; //from io
// logic [SSIT_WIDTH-1 : 0]            issue_store_pc; //from io
// logic [STORE_ID_WIDTH-1 : 0]        issue_store_id; //from io
//`ifdef FALCO_SIM_DEBUG
logic [LFST_WIDTH-1 : 0]            instr0_store_set_id/*verilator public*/;
logic [LFST_WIDTH-1 : 0]            instr1_store_set_id/*verilator public*/;
//`endif
logic                               violation; //from io
logic [SSIT_WIDTH-1 : 0]            recovery_insrt0_pc; //from io
logic [SSIT_WIDTH-1 : 0]            recovery_insrt1_pc; //from io
logic                               device_violation; //from io
logic [SSIT_WIDTH-1 : 0]            device_violation_pc; //from io

always_ff @(posedge clk) begin
    if(rst)
        wait_csr <= 0;
    else if((instr0_ready_alloc && id_io.decoded_instr0.is_csr_instr) || 
            (instr1_ready_alloc && id_io.decoded_instr1.is_csr_instr))
        wait_csr <= 1;
    else if(wait_csr == 1 && rnds_io.rob_is_empty)
        wait_csr <= 0;
    else 
        wait_csr <= wait_csr;
end



//TODO: mask branch missprediction and S-TYPE B-TYPE instruction
always_comb instr0_pre_req = id_io.instr0_valid;
always_comb instr1_pre_req = id_io.instr1_valid;
always_comb id_io.instr0_issue = instr0_ready_alloc;
always_comb id_io.instr1_issue = instr1_ready_alloc;
//TODO: only 2 instruction can dispatch then perform dispatch?
always_comb instr0_ready_alloc = instr0_pre_req &&
                                 FL_instr0_check_top &&
                                 rnds_io.rob_instr0_check_top &&
                                 ~pipe_ctrl_io.RNDS_stall && ~pipe_ctrl_io.RNDS_flush &&
                                 !wait_csr &&
                                 ((id_io.decoded_instr0.is_csr_instr && rnds_io.rob_is_empty) || ~id_io.decoded_instr0.is_csr_instr);
always_comb instr1_ready_alloc = instr1_pre_req &&
                                 FL_instr1_check_top &&
                                 rnds_io.rob_instr1_check_top &&
                                 ~pipe_ctrl_io.RNDS_stall && ~pipe_ctrl_io.RNDS_flush &&
                                 !wait_csr &&
                                 ((id_io.decoded_instr1.is_csr_instr && rnds_io.rob_is_empty) || ~id_io.decoded_instr1.is_csr_instr) &&
                                 !(id_io.decoded_instr0.is_csr_instr && id_io.instr0_valid);

always_comb reg_rename_instr0_req = instr0_ready_alloc && id_io.decoded_instr0.is_use_rd_field;
always_comb reg_rename_instr1_req = instr1_ready_alloc && id_io.decoded_instr1.is_use_rd_field;
//seperate Reg map table instr0_req signal (mask those which don't need rd field instr , such as sw)
//mask S-TYPE B-TYPE instruction


always_comb instr0_is_mem_op = (id_io.decoded_instr0.mem_we || id_io.decoded_instr0.mem_re) && id_io.instr0_valid;
always_comb instr1_is_mem_op = (id_io.decoded_instr1.mem_we || id_io.decoded_instr1.mem_re) && id_io.instr1_valid;

prf_freelist freelist(
    .clk(clk),
    .rst(rst),
//TO RNDS,RNDS must check freelist have element to pop
    .instr0_check_top(FL_instr0_check_top),
    .instr1_check_top(FL_instr1_check_top),
    .instr0_req(reg_rename_instr0_req),
    .instr1_req(reg_rename_instr1_req),
    .instr0_free_prf(instr0_rd_new_prf),
    .instr1_free_prf(instr1_rd_new_prf),
//From ROB
    .push_stale_rd0_valid(commit_io.recycle_freelist_0_valid),
    .push_stale_rd1_valid(commit_io.recycle_freelist_1_valid),
    .push_stale_rd0(commit_io.recycle_freelist_prf_0),
    .push_stale_rd1(commit_io.recycle_freelist_prf_1),
//Recovery
    .recovery_flush(pipe_ctrl_io.recovery_flush),
    .recovery_rollback(pipe_ctrl_io.recovery_rollback),
    .recovery_no_copy(rnds_io.recovery_no_copy),
    .recovery_target_rob_tag(rnds_io.recovery_target_rob_tag),
    .instr0_rob_tag(rnds_io.instr0_rob_tag),
    .instr1_rob_tag(rnds_io.instr1_rob_tag),
    .instr0_valid(instr0_pre_req),
    .instr1_valid(instr1_pre_req)
);

register_map_table Map_Table(
    .clk(clk),
    .rst(rst),
//From ROB
    .recovery_mode(pipe_ctrl_io.recovery_procedure),
    .recovery_map_arf_0(commit_io.recovery_arf_0), //0 older than 1
    .recovery_map_prf_0(commit_io.recovery_old_prf_0),
    .recovery_map_arf_1(commit_io.recovery_arf_1),
    .recovery_map_prf_1(commit_io.recovery_old_prf_1),
    .recovery_map_0_valid(commit_io.recovery_arf_map_0_valid),
    .recovery_map_1_valid(commit_io.recovery_arf_map_1_valid),
//TO: RNDS
    .instr0_rd(id_io.decoded_instr0.rd_addr),
    .instr0_rs1(id_io.decoded_instr0.rs1_addr),
    .instr0_rs2(id_io.decoded_instr0.rs2_addr),
    .instr0_rd_new_prf(instr0_rd_new_prf),
    .instr0_rd_new_map_valid(reg_rename_instr0_req),
    .instr0_rd_stale_prf(instr0_rd_stale_prf),
    .instr0_rs1_prf(instr0_rs1_prf),
    .instr0_rs2_prf(instr0_rs2_prf),

    .instr1_rd(id_io.decoded_instr1.rd_addr),
    .instr1_rs1(id_io.decoded_instr1.rs1_addr),
    .instr1_rs2(id_io.decoded_instr1.rs2_addr),
    .instr1_rd_new_prf(instr1_rd_new_prf),
    .instr1_rd_new_map_valid(reg_rename_instr1_req),
    .instr1_rd_stale_prf(instr1_rd_stale_prf),
    .instr1_rs1_prf(instr1_rs1_prf),
    .instr1_rs2_prf(instr1_rs2_prf),
//Recovery
    .recovery_flush(pipe_ctrl_io.recovery_flush),
    .recovery_no_copy(rnds_io.recovery_no_copy),
    .recovery_target_rob_tag(rnds_io.recovery_target_rob_tag),
    .instr0_rob_tag(rnds_io.instr0_rob_tag),
    .instr1_rob_tag(rnds_io.instr1_rob_tag),
    .instr0_valid(instr0_pre_req),
    .instr1_valid(instr1_pre_req)
);



busy_list Busy_list(
    .clk(clk),
    .rst(rst),
// RNDS
    .instr0_rs1_prf(instr0_rs1_prf),
    .instr0_rs2_prf(instr0_rs2_prf),
    .instr1_rs1_prf(instr1_rs1_prf),
    .instr1_rs2_prf(instr1_rs2_prf),
    .instr0_rs1_ready(instr0_rs1_reg_ready),
    .instr0_rs2_ready(instr0_rs2_reg_ready),
    .instr1_rs1_ready(instr1_rs1_reg_ready),
    .instr1_rs2_ready(instr1_rs2_reg_ready),
    .instr0_rd_prf(instr0_rd_prf),
    .instr1_rd_prf(instr1_rd_prf),
    .instr0_rd_valid(instr0_ready_alloc), //TODO: rd_valid condtion?
    .instr1_rd_valid(instr1_ready_alloc),
// From EXE
    .BCAST_alu_csr_bc(exe_io.BCAST_alu_csr_bc),
    .BCAST_ALU_1(exe_io.BCAST_ALU_1),
    .BCAST_muldiv(exe_io.BCAST_muldiv),
// From LSU
    .BCAST_load(mem_issue_io.BCAST_ld_spec),
// From int issue queue
    .int_issue0_rd_valid(int_issue_io.issue_alu0_rd_valid),
    .int_issue0_rd_prf(int_issue_io.issue_alu0_rd),
    .int_issue1_rd_valid(int_issue_io.issue_alu1_rd_valid),
    .int_issue1_rd_prf(int_issue_io.issue_alu1_rd)
);

last_fetch_store_table LFST(
    .clk(clk),
    .rst(rst),
    .instr0_pc(id_io.decoded_instr0.pc[SSIT_WIDTH-1+2 : 2]), 
    .instr1_pc(id_io.decoded_instr1.pc[SSIT_WIDTH-1+2 : 2]),
    .instr0_is_store(id_io.decoded_instr0.mem_we),
    .instr1_is_store(id_io.decoded_instr1.mem_we),
    .instr0_store_id(store_counter),
    .instr1_store_id(store_next_counter),
    .instr0_store_set_id(id_io.instr0_store_set_id),
    .instr1_store_set_id(id_io.instr1_store_set_id),
    .instr0_predict_result(instr0_predict_result),
    .instr1_predict_result(instr1_predict_result),

    // from mem_issue
    .issue_store(rnds_io.issue_store), //from io
    .issue_store_id(rnds_io.issue_store_id), //from io
    .issue_store_set_id(rnds_io.issue_store_set_id) //from io
);

always_comb begin
    //mask x0
    instr0_rd_prf = (id_io.decoded_instr0.rd_addr == 0) ? 0 : instr0_rd_new_prf;
    instr1_rd_prf = (id_io.decoded_instr1.rd_addr == 0) ? 0 : instr1_rd_new_prf;
end

always_comb begin
    //operand == x0, always ready
    instr0_rs1_operand_ready = instr0_rs1_reg_ready || 
                               (id_io.decoded_instr0.operand0_sel != OPERAND0_SEL_RS1 && 
                               ~id_io.decoded_instr0.is_branch) || //branch must use operand0
                               id_io.decoded_instr0.rs1_addr == 0;
    instr0_rs2_operand_ready = instr0_rs2_reg_ready || 
                               ((id_io.decoded_instr0.operand1_sel == OPERAND1_SEL_IMM ||
                               id_io.decoded_instr0.operand1_sel == OPERAND1_SEL_DUMMY0) &&
                               ~id_io.decoded_instr0.is_branch) || //branch must use operand1
                               id_io.decoded_instr0.rs2_addr == 0; //DUMMY should not happen 
    instr1_rs1_operand_ready = instr1_rs1_reg_ready || 
                               (id_io.decoded_instr1.operand0_sel != OPERAND0_SEL_RS1 &&
                               ~id_io.decoded_instr1.is_branch) ||  //branch must use operand0
                               id_io.decoded_instr1.rs1_addr == 0;
    instr1_rs2_operand_ready = instr1_rs2_reg_ready || 
                               ((id_io.decoded_instr1.operand1_sel == OPERAND1_SEL_IMM || 
                               id_io.decoded_instr1.operand1_sel == OPERAND1_SEL_DUMMY0) && 
                               ~id_io.decoded_instr1.is_branch) || //branch must use operand1
                               id_io.decoded_instr1.rs2_addr == 0; //DUMMY should not happen 
end


//===================================================
//ROB related logic
//===================================================
//ROB allocation
/* //TODO: RNDS ROB allocation change Reg pipe
always_ff @(posedge clk) begin
    if (rst) begin
        rnds_io.instr0_req <= 0;
        rnds_io.instr1_req <= 0;
        rnds_io.instr0_br_mask <= 0;
        rnds_io.instr1_br_mask <= 0;
        rnds_io.instr0_stale_rd <= 0;
        rnds_io.instr1_stale_rd <= 0;
        rnds_io.instr0_rd_arf <= 0;
        rnds_io.instr1_rd_arf <= 0;
        rnds_io.instr0_rd_prf <= 0;
        rnds_io.instr1_rd_prf <= 0;
        rnds_io.instr0_pc <= 0;
        rnds_io.instr1_pc <= 0;
        rnds_io.instr0_stale_rd_valid <= 0;
        rnds_io.instr1_stale_rd_valid <= 0;
    end else begin
        rnds_io.instr0_req <= instr0_ready_alloc;
        rnds_io.instr1_req <= instr1_ready_alloc;
        rnds_io.instr0_br_mask <= id_io.instr0_br_info.br_mask;
        rnds_io.instr1_br_mask <= id_io.instr1_br_info.br_mask;
        rnds_io.instr0_stale_rd <= instr0_rd_stale_prf;
        rnds_io.instr1_stale_rd <= instr1_rd_stale_prf;
        rnds_io.instr0_rd_arf <= id_io.decoded_instr0.rd_addr;
        rnds_io.instr1_rd_arf <= id_io.decoded_instr1.rd_addr;
        rnds_io.instr0_rd_prf <= instr0_rd_prf;
        rnds_io.instr1_rd_prf <= instr1_rd_prf;
        rnds_io.instr0_pc <= id_io.decoded_instr0.pc;
        rnds_io.instr1_pc <= id_io.decoded_instr1.pc;
        rnds_io.instr0_stale_rd_valid <= id_io.decoded_instr0.is_use_rd_field;
        rnds_io.instr1_stale_rd_valid <= id_io.decoded_instr1.is_use_rd_field;
    end
end
*/
//ROB allocation
always_comb rnds_io.instr0_req = instr0_ready_alloc;
always_comb rnds_io.instr1_req = instr1_ready_alloc;
always_comb rnds_io.instr0_stale_rd = instr0_rd_stale_prf;
always_comb rnds_io.instr1_stale_rd = instr1_rd_stale_prf;
always_comb rnds_io.instr0_rd_arf = id_io.decoded_instr0.rd_addr;
always_comb rnds_io.instr1_rd_arf = id_io.decoded_instr1.rd_addr;
always_comb rnds_io.instr0_rd_prf = instr0_rd_prf;
always_comb rnds_io.instr1_rd_prf = instr1_rd_prf;
always_comb rnds_io.instr0_pc = id_io.decoded_instr0.pc;
always_comb rnds_io.instr1_pc = id_io.decoded_instr1.pc;
always_comb rnds_io.instr0_is_store_op = id_io.decoded_instr0.mem_we;
always_comb rnds_io.instr1_is_store_op = id_io.decoded_instr1.mem_we;
always_comb rnds_io.instr0_stale_rd_valid = id_io.decoded_instr0.is_use_rd_field;
always_comb rnds_io.instr1_stale_rd_valid = id_io.decoded_instr1.is_use_rd_field;
//===================================================
//Issue logic
//===================================================

always_comb begin
    if (~instr0_is_mem_op && id_io.instr0_valid) begin
        pre_int_instr0_pack.predict_pc = id_io.decoded_instr0.predict_pc;
        pre_int_instr0_pack.pc = id_io.decoded_instr0.pc;
        pre_int_instr0_pack.immediate = id_io.decoded_instr0.immediate;
        pre_int_instr0_pack.operand0_sel = id_io.decoded_instr0.operand0_sel;
        pre_int_instr0_pack.operand1_sel = id_io.decoded_instr0.operand1_sel;
        pre_int_instr0_pack.ctrl_signal = id_io.decoded_instr0.ctrl_signal;
        pre_int_instr0_pack.alu_muldiv_sel = id_io.decoded_instr0.alu_muldiv_sel;
        pre_int_instr0_pack.shift_sel = id_io.decoded_instr0.shift_sel;
        pre_int_instr0_pack.is_branch = id_io.decoded_instr0.is_branch;
        pre_int_instr0_pack.is_jal = id_io.decoded_instr0.is_jal;
        pre_int_instr0_pack.is_jalr = id_io.decoded_instr0.is_jalr;
        pre_int_instr0_pack.is_csr_instr = id_io.decoded_instr0.is_csr_instr;
        pre_int_instr0_pack.csr_addr = id_io.decoded_instr0.csr_addr;
        pre_int_instr0_pack.csr_imm = id_io.decoded_instr0.csr_imm;
        pre_int_instr0_pack.csr_data = id_io.decoded_instr0.csr_data;
        pre_int_instr0_pack.sys_jump = id_io.decoded_instr0.sys_jump;
        pre_int_instr0_pack.sys_jump_csr_addr = id_io.decoded_instr0.sys_jump_csr_addr;
        pre_int_instr0_pack.rd_addr = instr0_rd_prf;  //new map
        pre_int_instr0_pack.rs1_addr = instr0_rs1_prf;
        pre_int_instr0_pack.rs2_addr = instr0_rs2_prf;
        pre_int_instr0_pack.rob_tag = rnds_io.instr0_rob_tag;
        pre_int_instr0_pack.bhsr = id_io.instr0_BHSR;
    end else begin //assign instr1 to int_instr0 but int_pack1 will disable
        pre_int_instr0_pack.predict_pc = id_io.decoded_instr1.predict_pc;
        pre_int_instr0_pack.pc = id_io.decoded_instr1.pc;
        pre_int_instr0_pack.immediate = id_io.decoded_instr1.immediate;
        pre_int_instr0_pack.operand0_sel = id_io.decoded_instr1.operand0_sel;
        pre_int_instr0_pack.operand1_sel = id_io.decoded_instr1.operand1_sel;
        pre_int_instr0_pack.ctrl_signal = id_io.decoded_instr1.ctrl_signal;
        pre_int_instr0_pack.alu_muldiv_sel = id_io.decoded_instr1.alu_muldiv_sel;
        pre_int_instr0_pack.shift_sel = id_io.decoded_instr1.shift_sel;
        pre_int_instr0_pack.is_branch = id_io.decoded_instr1.is_branch;
        pre_int_instr0_pack.is_jal = id_io.decoded_instr1.is_jal;
        pre_int_instr0_pack.is_jalr = id_io.decoded_instr1.is_jalr;
        pre_int_instr0_pack.is_csr_instr = id_io.decoded_instr1.is_csr_instr;
        pre_int_instr0_pack.csr_addr = id_io.decoded_instr1.csr_addr;
        pre_int_instr0_pack.csr_imm = id_io.decoded_instr1.csr_imm;
        pre_int_instr0_pack.csr_data = id_io.decoded_instr1.csr_data;
        pre_int_instr0_pack.sys_jump = id_io.decoded_instr1.sys_jump;
        pre_int_instr0_pack.sys_jump_csr_addr = id_io.decoded_instr1.sys_jump_csr_addr;
        pre_int_instr0_pack.rd_addr = instr1_rd_prf; //new map
        pre_int_instr0_pack.rs1_addr = instr1_rs1_prf;
        pre_int_instr0_pack.rs2_addr = instr1_rs2_prf;
        pre_int_instr0_pack.rob_tag = rnds_io.instr1_rob_tag;
        pre_int_instr0_pack.bhsr = id_io.instr1_BHSR;
    end
end

always_comb begin
    //always assign instr1 to int_instr1
    pre_int_instr1_pack.predict_pc = id_io.decoded_instr1.predict_pc;
    pre_int_instr1_pack.pc = id_io.decoded_instr1.pc;
    pre_int_instr1_pack.immediate = id_io.decoded_instr1.immediate;
    pre_int_instr1_pack.operand0_sel = id_io.decoded_instr1.operand0_sel;
    pre_int_instr1_pack.operand1_sel = id_io.decoded_instr1.operand1_sel;
    pre_int_instr1_pack.ctrl_signal = id_io.decoded_instr1.ctrl_signal;
    pre_int_instr1_pack.alu_muldiv_sel = id_io.decoded_instr1.alu_muldiv_sel;
    pre_int_instr1_pack.shift_sel = id_io.decoded_instr1.shift_sel;
    pre_int_instr1_pack.is_branch = id_io.decoded_instr1.is_branch;
    pre_int_instr1_pack.is_jal = id_io.decoded_instr1.is_jal;
    pre_int_instr1_pack.is_jalr = id_io.decoded_instr1.is_jalr;
    pre_int_instr1_pack.is_csr_instr = id_io.decoded_instr1.is_csr_instr;
    pre_int_instr1_pack.csr_addr = id_io.decoded_instr1.csr_addr;
    pre_int_instr1_pack.csr_imm = id_io.decoded_instr1.csr_imm;
    pre_int_instr1_pack.csr_data = id_io.decoded_instr1.csr_data;
    pre_int_instr1_pack.sys_jump = id_io.decoded_instr1.sys_jump;
    pre_int_instr1_pack.sys_jump_csr_addr = id_io.decoded_instr1.sys_jump_csr_addr;
    pre_int_instr1_pack.rd_addr = instr1_rd_prf; // new map
    pre_int_instr1_pack.rs1_addr = instr1_rs1_prf;
    pre_int_instr1_pack.rs2_addr = instr1_rs2_prf;
    pre_int_instr1_pack.rob_tag = rnds_io.instr1_rob_tag;
    pre_int_instr1_pack.bhsr = id_io.instr1_BHSR;
end

always_comb begin
    if (instr0_is_mem_op && id_io.instr0_valid) begin
        pre_mem_instr0_pack.immediate = id_io.decoded_instr0.immediate;
        pre_mem_instr0_pack.store_set_pc = id_io.decoded_instr0.pc[SSIT_WIDTH-1+2 : 2];
        pre_mem_instr0_pack.predict_no_vilation = instr0_predict_result;
        pre_mem_instr0_pack.store_id = store_counter;
        pre_mem_instr0_pack.mem_load_ext_sel = id_io.decoded_instr0.mem_load_ext_sel;
        pre_mem_instr0_pack.mem_is_store = id_io.decoded_instr0.mem_we;
        pre_mem_instr0_pack.mem_input_sel = id_io.decoded_instr0.mem_input_sel;
        pre_mem_instr0_pack.rd_addr = instr0_rd_prf;
        pre_mem_instr0_pack.rs1_addr = instr0_rs1_prf;
        pre_mem_instr0_pack.rs2_addr = instr0_rs2_prf;
        pre_mem_instr0_pack.rob_tag = rnds_io.instr0_rob_tag;
        pre_mem_instr0_pack.bhsr = id_io.instr0_BHSR;
// `ifdef FALCO_SIM_DEBUG
        pre_mem_instr0_pack.pc = id_io.decoded_instr0.pc;
        pre_mem_instr0_pack.store_set_id = id_io.instr0_store_set_id;
// `endif
    end else begin //assign instr1 to mem_instr0 but mem_pack1 will disable
        pre_mem_instr0_pack.immediate = id_io.decoded_instr1.immediate;
        pre_mem_instr0_pack.store_set_pc = id_io.decoded_instr1.pc[SSIT_WIDTH-1+2 : 2];
        pre_mem_instr0_pack.predict_no_vilation = instr1_predict_result;
        pre_mem_instr0_pack.store_id = store_counter;
        pre_mem_instr0_pack.mem_load_ext_sel = id_io.decoded_instr1.mem_load_ext_sel;
        pre_mem_instr0_pack.mem_is_store = id_io.decoded_instr1.mem_we;
        pre_mem_instr0_pack.mem_input_sel = id_io.decoded_instr1.mem_input_sel;
        pre_mem_instr0_pack.rd_addr = instr1_rd_prf;
        pre_mem_instr0_pack.rs1_addr = instr1_rs1_prf;
        pre_mem_instr0_pack.rs2_addr = instr1_rs2_prf;
        pre_mem_instr0_pack.rob_tag = rnds_io.instr1_rob_tag;
        pre_mem_instr0_pack.bhsr = id_io.instr1_BHSR;
// `ifdef FALCO_SIM_DEBUG
        pre_mem_instr0_pack.pc = id_io.decoded_instr1.pc;
        pre_mem_instr0_pack.store_set_id = id_io.instr1_store_set_id;
// `endif
    end
end

always_comb begin
    //always assign instr1 to mem_instr1
    pre_mem_instr1_pack.immediate = id_io.decoded_instr1.immediate;
    pre_mem_instr1_pack.store_set_pc = id_io.decoded_instr1.pc[SSIT_WIDTH-1+2 : 2];
    pre_mem_instr1_pack.predict_no_vilation = instr1_predict_result;
    pre_mem_instr1_pack.store_id = store_next_counter;
    pre_mem_instr1_pack.mem_load_ext_sel = id_io.decoded_instr1.mem_load_ext_sel;
    pre_mem_instr1_pack.mem_is_store = id_io.decoded_instr1.mem_we;
    pre_mem_instr1_pack.mem_input_sel = id_io.decoded_instr1.mem_input_sel;
    pre_mem_instr1_pack.rd_addr = instr1_rd_prf;
    pre_mem_instr1_pack.rs1_addr = instr1_rs1_prf;
    pre_mem_instr1_pack.rs2_addr = instr1_rs2_prf;
    pre_mem_instr1_pack.rob_tag = rnds_io.instr1_rob_tag;
    pre_mem_instr1_pack.bhsr = id_io.instr1_BHSR;
// `ifdef FALCO_SIM_DEBUG
    pre_mem_instr1_pack.pc = id_io.decoded_instr1.pc;
    pre_mem_instr1_pack.store_set_id = id_io.instr1_store_set_id;
// `endif
end
//INT IQ
//should we force if instr1 valid then instr0 also valid?
always_comb begin
    if (instr0_ready_alloc && ~instr0_is_mem_op)
        pre_int_pack0_valid = 1;
    else if (instr1_ready_alloc && ~instr1_is_mem_op)
        pre_int_pack0_valid = 1;
    else
        pre_int_pack0_valid = 0;
end
always_comb pre_int_pack1_valid = instr1_ready_alloc && ~instr1_is_mem_op && 
                            (instr0_ready_alloc && ~instr0_is_mem_op ); 
                            //if instr0 no dispatch id stage's instr0, instr1 should not dispatch id stags's instr1

//MEM IQ
//should we force if instr1 valid then instr0 also valid?
always_comb begin
    if (instr0_is_mem_op && instr0_ready_alloc) 
        pre_mem_pack0_valid = 1;
    else if (instr1_is_mem_op && instr1_ready_alloc)
        pre_mem_pack0_valid = 1;
    else
        pre_mem_pack0_valid = 0;
end

always_comb pre_mem_pack1_valid = instr1_ready_alloc && instr1_is_mem_op &&
                                   (instr0_ready_alloc && instr0_is_mem_op);
                            //if instr0 no dispatch id stage's instr0, instr1 should not dispatch id stags's instr1

always_comb begin
    if (~instr0_is_mem_op && id_io.instr0_valid) begin
        pre_int_instr0_rs1_ready = instr0_rs1_operand_ready;
        pre_int_instr0_rs2_ready = instr0_rs2_operand_ready;
        //pre_int_pack0_br_info = id_io.instr0_br_info;
    end else begin
        pre_int_instr0_rs1_ready = instr1_rs1_operand_ready;
        pre_int_instr0_rs2_ready = instr1_rs2_operand_ready;
        //pre_int_pack0_br_info = id_io.instr1_br_info;
    end
end

always_comb begin
    pre_int_instr1_rs1_ready = instr1_rs1_operand_ready;
    pre_int_instr1_rs2_ready = instr1_rs2_operand_ready;
    //pre_int_pack1_br_info = id_io.instr1_br_info;
end

always_comb begin
    if (instr0_is_mem_op && id_io.instr0_valid) begin
        pre_mem_instr0_rs1_ready = instr0_rs1_operand_ready;
        pre_mem_instr0_rs2_ready = instr0_rs2_operand_ready;
        //pre_mem_pack0_br_mask = id_io.instr0_br_info.br_mask;
    end else begin
        pre_mem_instr0_rs1_ready = instr1_rs1_operand_ready;
        pre_mem_instr0_rs2_ready = instr1_rs2_operand_ready;
        //pre_mem_pack0_br_mask = id_io.instr1_br_info.br_mask;
    end
end

always_comb begin
    pre_mem_instr1_rs1_ready = instr1_rs1_operand_ready;
    pre_mem_instr1_rs2_ready = instr1_rs2_operand_ready;
    //pre_mem_pack1_br_mask = id_io.instr1_br_info.br_mask;
end

always_ff @(posedge clk) begin
    if (rst) begin
        rnds_io.int_pack0 <= 0;
        rnds_io.int_pack1 <= 0;
        rnds_io.int_pack0_valid <= 0;
        rnds_io.int_pack1_valid <= 0;
        //rnds_io.int_pack0_br_info <= 0;
        //rnds_io.int_pack1_br_info <= 0;
        rnds_io.int_instr0_rs1_ready <= 0;
        rnds_io.int_instr0_rs2_ready <= 0;
        rnds_io.int_instr1_rs1_ready <= 0;
        rnds_io.int_instr1_rs2_ready <= 0;
    end else if (pipe_ctrl_io.recovery_stall) begin
        rnds_io.int_pack0 <= rnds_io.int_pack0;
        rnds_io.int_pack1 <= rnds_io.int_pack1;
        rnds_io.int_pack0_valid <= IsBrROBKill(pipe_ctrl_io.recovery_flush_BCAST, rnds_io.int_pack0.rob_tag) ?
                                   0 : rnds_io.int_pack0_valid;
        rnds_io.int_pack1_valid <= IsBrROBKill(pipe_ctrl_io.recovery_flush_BCAST, rnds_io.int_pack1.rob_tag) ?
                                   0 : rnds_io.int_pack1_valid;
        rnds_io.int_instr0_rs1_ready <= rnds_io.int_instr0_rs1_ready;
        rnds_io.int_instr0_rs2_ready <= rnds_io.int_instr0_rs2_ready;
        rnds_io.int_instr1_rs1_ready <= rnds_io.int_instr1_rs1_ready;
        rnds_io.int_instr1_rs2_ready <= rnds_io.int_instr1_rs2_ready;
    end else begin
        rnds_io.int_pack0 <= pre_int_instr0_pack;
        rnds_io.int_pack1 <= pre_int_instr1_pack;
        rnds_io.int_pack0_valid <= IsBrROBKill(pipe_ctrl_io.recovery_flush_BCAST, pre_int_instr0_pack.rob_tag) ?
                                   0 : pre_int_pack0_valid;
        rnds_io.int_pack1_valid <= IsBrROBKill(pipe_ctrl_io.recovery_flush_BCAST, pre_int_instr1_pack.rob_tag) ?
                                   0 : pre_int_pack1_valid;
        /*rnds_io.int_pack0_br_info <= BrInfoUpdate(
            pre_int_pack0_br_info,
            pipe_ctrl_io.bp_update_tag,
            pipe_ctrl_io.bp_validation
        );
        rnds_io.int_pack1_br_info <= BrMaskUpdate(
            pre_int_pack1_br_info,
            pipe_ctrl_io.bp_update_tag,
            pipe_ctrl_io.bp_validation
        );*/
        rnds_io.int_instr0_rs1_ready <= pre_int_instr0_rs1_ready;
        rnds_io.int_instr0_rs2_ready <= pre_int_instr0_rs2_ready;
        rnds_io.int_instr1_rs1_ready <= pre_int_instr1_rs1_ready;
        rnds_io.int_instr1_rs2_ready <= pre_int_instr1_rs2_ready;
    end
end

always_ff @(posedge clk) begin
    if (rst) begin
        rnds_io.mem_pack0 <= 0;
        rnds_io.mem_pack1 <= 0;
        rnds_io.mem_pack0_valid <= 0;
        rnds_io.mem_pack1_valid <= 0;
        //rnds_io.mem_pack0_br_mask <= 0;
        //rnds_io.mem_pack1_br_mask <= 0;
        rnds_io.mem_instr0_rs1_ready <= 0;
        rnds_io.mem_instr0_rs2_ready <= 0;
        rnds_io.mem_instr1_rs1_ready <= 0;
        rnds_io.mem_instr1_rs2_ready <= 0;
    end else if ( pipe_ctrl_io.recovery_stall ) begin
        rnds_io.mem_pack0 <= rnds_io.mem_pack0;
        rnds_io.mem_pack1 <= rnds_io.mem_pack1;
        rnds_io.mem_pack0_valid <= IsBrROBKill(pipe_ctrl_io.recovery_flush_BCAST, rnds_io.mem_pack0.rob_tag) ?
                                   0 : rnds_io.mem_pack0_valid;
        rnds_io.mem_pack1_valid <= IsBrROBKill(pipe_ctrl_io.recovery_flush_BCAST, rnds_io.mem_pack1.rob_tag) ?
                                   0 : rnds_io.mem_pack1_valid;
        rnds_io.mem_instr0_rs1_ready <= rnds_io.mem_instr0_rs1_ready;
        rnds_io.mem_instr0_rs2_ready <= rnds_io.mem_instr0_rs2_ready;
        rnds_io.mem_instr1_rs1_ready <= rnds_io.mem_instr1_rs1_ready;
        rnds_io.mem_instr1_rs2_ready <= rnds_io.mem_instr1_rs2_ready;
    end else begin
        rnds_io.mem_pack0 <= pre_mem_instr0_pack;
        rnds_io.mem_pack1 <= pre_mem_instr1_pack;
        rnds_io.mem_pack0_valid <= IsBrROBKill(pipe_ctrl_io.recovery_flush_BCAST,pre_mem_instr0_pack.rob_tag) ?
                         0: pre_mem_pack0_valid;
        rnds_io.mem_pack1_valid <= IsBrROBKill(pipe_ctrl_io.recovery_flush_BCAST,pre_mem_instr1_pack.rob_tag) ?
                         0: pre_mem_pack1_valid;
        /*rnds_io.mem_pack0_br_mask <= BrMaskUpdate(
            pre_mem_pack0_br_mask,
            pipe_ctrl_io.bp_update_tag,
            pipe_ctrl_io.bp_validation
        );
        rnds_io.mem_pack1_br_mask <= BrMaskUpdate(
            pre_mem_pack1_br_mask,
            pipe_ctrl_io.bp_update_tag,
            pipe_ctrl_io.bp_validation
        );*/
        rnds_io.mem_instr0_rs1_ready <= pre_mem_instr0_rs1_ready;
        rnds_io.mem_instr0_rs2_ready <= pre_mem_instr0_rs2_ready;
        rnds_io.mem_instr1_rs1_ready <= pre_mem_instr1_rs1_ready;
        rnds_io.mem_instr1_rs2_ready <= pre_mem_instr1_rs2_ready;
    end
end

always_ff @(posedge clk) begin
    if(rst) store_counter <= 1;
    else if(store_counter == 255 && pre_mem_instr0_pack.mem_is_store) store_counter <= 1 
                        + (instr1_is_mem_op && id_io.instr1_valid && pre_mem_instr1_pack.mem_is_store ? 1 : 0);
    else store_counter <= store_counter + pre_mem_instr0_pack.mem_is_store 
                        + (instr1_is_mem_op && id_io.instr1_valid && pre_mem_instr1_pack.mem_is_store ? 1 : 0);
end
always_comb store_next_counter = ~(pre_mem_instr0_pack.mem_is_store) ? store_counter : 
                        (store_counter == 255) ? 1 : store_counter+1;
//===================================================
//stall logic
//===================================================
//TODO: ROB stale reg recycle request with queue_avail singal AND gate
always_comb pipe_ctrl_io.freelist_empty = (instr0_pre_req && ~FL_instr0_check_top) || (instr1_pre_req && ~FL_instr1_check_top);
always_comb pipe_ctrl_io.int_iq_full = (instr0_pre_req && ~instr0_is_mem_op && ~rnds_io.int_iq_instr0_check_top) || 
                                       (instr1_pre_req && ~instr1_is_mem_op && ~rnds_io.int_iq_instr1_check_top);
always_comb pipe_ctrl_io.mem_iq_full = (instr0_pre_req && instr0_is_mem_op && ~rnds_io.mem_iq_instr0_check_top) ||
                            (instr1_pre_req && instr1_is_mem_op && ~rnds_io.mem_iq_instr1_check_top);
always_comb pipe_ctrl_io.rob_full = (instr0_pre_req && ~rnds_io.rob_instr0_check_top) || (instr1_pre_req && ~rnds_io.rob_instr1_check_top);
always_comb pipe_ctrl_io.csr_stall = ((id_io.decoded_instr0.is_csr_instr && id_io.instr0_valid) || 
                                    (id_io.decoded_instr1.is_csr_instr && id_io.instr1_valid)) || wait_csr;

endmodule
