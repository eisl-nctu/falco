`timescale 1ns/1ps
// =============================================================================
//  Program : reorder_buffer.sv
//  Author  : Hon-Chou Dai
//  Date    : May/15/2021
// -----------------------------------------------------------------------------
//  Description:
//  This module handle instruction information for in-order commit.
//  When recovery happened, it will send recovery information to other module.
// -----------------------------------------------------------------------------
//  Revision information:
//
//    August/07/2021, by Chun-Wei Chao:
//      Revise for handle recovery cause from branch_miss or load_store_violation.
//    November/18/2021, by Chun-Wei Chao:
//      Revise for checkpoint recovery mechanism.
//      When recovery happen, Falco will first flush all instruction after 
//      branch or store instruction, then jump to appropriate checkpoint,
//      finally rollback to recycle physical register and remap the register file.
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

module reorder_buffer(
    input clk,
    input rst,

    //debug performance counter
    output logic [1:0] core_commit_count,
 
    pipeline_control_recovery_io.ROB pipe_ctrl_io,
    rename_dispatch_io.ROB rnds_io,
    exe_stage_io.ROB exe_io,
    load_store_unit_io.ROB lsu_io,
    mem_access_io.ROB ma_io,
    csr_io.ROB csr_io,
    commit_stage_io.ROB commit_io
);
typedef struct packed {
    pc_t trunc_pc; //4 byte alignment pc
    logic busy;
    logic issue;
    logic finished;
    prf_specifier_t stale_prf;
    logic stale_valid;
    arf_specifier_t rd_arf; //redundant ??
    prf_specifier_t rd_prf; //redundant ??
    logic valid;
    logic is_store;
} rob_entry_t;



rob_entry_t                 rob[ROB_ENTRY_NUM-1:0];
logic [ROB_ENTRY_WIDTH:0]   counter/*verilator public*/;// alloc ~ commit
logic [ROB_ENTRY_WIDTH:0]   space_counter/*verilator public*/; //alloc ~ stale
logic [ROB_ENTRY_WIDTH-1:0] alloc_ptr /*verilator public*/;
logic [ROB_ENTRY_WIDTH-1:0] alloc_next_ptr/*verilator public*/;
logic [ROB_ENTRY_WIDTH-1:0] commit_ptr/*verilator public*/;
logic [ROB_ENTRY_WIDTH-1:0] commit_next_ptr/*verilator public*/;
logic [ROB_ENTRY_WIDTH-1:0] recovery_ptr /*verilator public*/;
logic [ROB_ENTRY_WIDTH-1:0] recovery_pre_ptr /*verilator public*/;
logic [ROB_ENTRY_WIDTH-1:0] recovery_target /*verilator public*/; //recovery to which entry, debug only
logic [ROB_ENTRY_WIDTH-1:0] recovery_start_point /*verilator public*/;
logic [ROB_ENTRY_WIDTH:0]   recovery_counter/*verilator public*/; //decrement counter
logic [ROB_ENTRY_WIDTH:0]   recovery_rollback_counter/*verilator public*/; //decrement counter
logic [2:0]   recovery_rollback_counter_temp1/*verilator public*/; //decrement counter
logic [ROB_ENTRY_WIDTH:0]   recovery_rollback_counter_temp2/*verilator public*/; //decrement counter

//Debug
pc_t commit_0_pc;
pc_t commit_1_pc;

logic [ROB_ENTRY_WIDTH-1:0] pre_alloc_ptr;

assign alloc_next_ptr = alloc_ptr + 1;
assign commit_next_ptr = commit_ptr + 1;

logic push;
logic commit_pop;
logic [1:0] commit_count;
logic [1:0] push_count;
logic squash;
logic [ROB_ENTRY_WIDTH-1:0] squash_count;


logic commit_instr0;
logic commit_instr1;


// recycle prf from commit registr
prf_specifier_t recycle_stale_rd0;
prf_specifier_t recycle_stale_rd1;
logic recycle_stale_rd0_valid;
logic recycle_stale_rd1_valid;
// recycle prf from branch miss dst reg
arf_specifier_t recovery_arf_rd0;
arf_specifier_t recovery_arf_rd1;
logic recovery_entry_rd0_valid;
logic recovery_entry_rd1_valid;
prf_specifier_t recovery_old_map_prf_rd0;
prf_specifier_t recovery_old_map_prf_rd1;
prf_specifier_t recovery_new_alloc_prf_rd0;
prf_specifier_t recovery_new_alloc_prf_rd1;
logic recovery_start_delay; //update alloc, space , counter
logic recovery_lock; //lock other procedure

logic branch_miss_delay;
logic branch_miss_first/*verilator public*/;
logic branch_miss/*verilator public*/; //use to debug
always_comb branch_miss = (exe_io.exe_alu_csr_bc.valid && exe_io.exe_alu_csr_bc.is_branch && pipe_ctrl_io.bp_PrMiss);

always_comb recovery_lock = (pipe_ctrl_io.recovery_procedure || pipe_ctrl_io.recovery_start);
// commit logic
assign commit_instr0 = rob[commit_ptr].valid && rob[commit_ptr].finished && 
                       ~recovery_lock;
assign commit_instr1 = commit_instr0 && rob[commit_next_ptr].valid && 
                       rob[commit_next_ptr].finished && ~recovery_lock;
assign commit_pop = (commit_instr0 || commit_instr1) && ~recovery_lock;
assign commit_count = commit_instr0 + commit_instr1;

//Debug
assign commit_0_pc = rob[commit_ptr].trunc_pc;
assign commit_1_pc = rob[commit_next_ptr].trunc_pc;

always_ff @(posedge clk) begin
    if (rst)
        core_commit_count <= 0;
    else
        core_commit_count <= commit_count;
end

// Don't use recovery lock, due to stale_recycle_ptr need one cycle (recovery_start) to update, current committed
// instruction

// allocate logic
assign push = (rnds_io.instr0_req || rnds_io.instr1_req) && ~recovery_lock;


//TO committed map table 
always_comb begin
commit_io.committed_update.map_update_0 = 
    commit_instr0 && rob[commit_ptr].stale_valid; // check it contain stale reg map
commit_io.committed_update.map_update_1 = 
    commit_instr1 && rob[commit_next_ptr].stale_valid; //check it contain stale reg map
commit_io.committed_update.map_arf_0 = rob[commit_ptr].rd_arf;
commit_io.committed_update.map_prf_0 = rob[commit_ptr].rd_prf;
commit_io.committed_update.map_arf_1 = rob[commit_next_ptr].rd_arf;
commit_io.committed_update.map_prf_1 = rob[commit_next_ptr].rd_prf;
    
end

always_comb push_count = rnds_io.instr0_req + rnds_io.instr1_req;


// TO RNDS io logic (allocation and recycle logic)
always_comb
    if (commit_instr0 && rob[commit_ptr].stale_valid)
        recycle_stale_rd0_valid = 1;
    else if (commit_instr1 && rob[commit_next_ptr].stale_valid)
        recycle_stale_rd0_valid = 1;
    else
        recycle_stale_rd0_valid = 0;
always_comb
    if (commit_instr0 && rob[commit_ptr].stale_valid &&
        commit_instr1 && rob[commit_next_ptr].stale_valid ) begin
        recycle_stale_rd1_valid = 1;
    end else begin
        recycle_stale_rd1_valid = 0;
    end

assign rnds_io.rob_instr0_check_top = (space_counter >= 1);
assign rnds_io.rob_instr1_check_top = (space_counter >= 2);
always_comb rnds_io.rob_is_empty = (space_counter == ROB_ENTRY_NUM);
assign rnds_io.instr0_rob_tag = alloc_ptr;
assign rnds_io.instr1_rob_tag = rnds_io.instr0_req ? alloc_next_ptr : alloc_ptr;

always_comb
    if (commit_instr0 && rob[commit_ptr].stale_valid)
        recycle_stale_rd0 = rob[commit_ptr].stale_prf;
    else if (commit_instr1 && rob[commit_next_ptr].stale_valid)
        recycle_stale_rd0 = rob[commit_next_ptr].stale_prf;
    else
        recycle_stale_rd0 = 0; //x0 for error

always_comb recycle_stale_rd1 = rob[commit_next_ptr].stale_prf;

always_ff @(posedge clk) begin
    if (rst)
        alloc_ptr <= 0;
    else if (push)
        alloc_ptr <= alloc_ptr + push_count; //let it overflow
    else if (recovery_start_delay)
        alloc_ptr <= alloc_ptr > recovery_counter ?
                     alloc_ptr - recovery_counter : 
                     alloc_ptr + ROB_ENTRY_NUM - recovery_counter; 
    else
        alloc_ptr <= alloc_ptr;
end

always_ff @(posedge clk) begin
    if (rst)
        commit_ptr <= 0;
    else if (commit_pop)
        commit_ptr <= commit_ptr + commit_count; //let if overflow
    else
        commit_ptr <= commit_ptr;
end

always_ff @(posedge clk) begin
    if (rst) begin
        counter <= 0;
    end else if (recovery_start_delay) begin
        counter <= counter > recovery_counter ? 
                   counter - recovery_counter :
                   counter + ROB_ENTRY_NUM - recovery_counter;
    end else begin
        case ({push,commit_pop})
            2'b00: counter <= counter;
            2'b01: counter <= counter - commit_count;
            2'b10: counter <= counter + push_count;
            2'b11: counter <= counter + push_count - commit_count; //overflow???
            default: counter <= counter;
        endcase
    end
end

always_ff @(posedge clk) begin
    if (rst) begin
        space_counter <= ROB_ENTRY_NUM; //init 64 free reg
    end else if (recovery_start_delay) begin
        space_counter <= space_counter + recovery_counter;
    end else begin
        case ({push,commit_pop})
            2'b00: space_counter <= space_counter;
            2'b01: space_counter <= space_counter + commit_count;
            2'b10: space_counter <= space_counter - push_count;
            2'b11: space_counter <= space_counter - push_count + commit_count; //overflow???
            default: space_counter <= space_counter;
        endcase
    end
end

always_comb commit_io.cur_commit_rob_tag = commit_ptr;

always_comb begin
    if (pipe_ctrl_io.recovery_rollback) begin
        commit_io.recycle_freelist_prf_0 = recovery_new_alloc_prf_rd1; //1 -> prf_0, because 1 1 is latest than 0
        commit_io.recycle_freelist_prf_1 = recovery_new_alloc_prf_rd0;
        commit_io.recycle_freelist_0_valid = recovery_entry_rd1_valid;
        commit_io.recycle_freelist_1_valid = recovery_entry_rd0_valid;
    end else begin
        commit_io.recycle_freelist_prf_0 = recycle_stale_rd0;
        commit_io.recycle_freelist_prf_1 = recycle_stale_rd1;
        commit_io.recycle_freelist_0_valid = recycle_stale_rd0_valid;
        commit_io.recycle_freelist_1_valid = recycle_stale_rd1_valid;
    end
end

// =======================================================================
// Recovery logic
// =======================================================================

always_ff @(posedge clk) begin
    if (rst)
        recovery_start_delay <= 0;
    else
        recovery_start_delay <= pipe_ctrl_io.recovery_start;
end

always_ff @(posedge clk) begin
    if (rst)
        recovery_target <= 0;
    else if (lsu_io.store_set_violation && ~branch_miss_first)
        recovery_target <= lsu_io.store_set_rob_tag;
    else if (exe_io.exe_alu_csr_bc.valid && exe_io.exe_alu_csr_bc.is_branch) begin
        recovery_target <= exe_io.exe_alu_csr_bc.commit_addr + 1;
    end else begin
        recovery_target <= recovery_target;
    end
end
always_comb rnds_io.recovery_target_rob_tag = recovery_target;

always_ff @(posedge clk) begin
    if (rst)
        recovery_start_point <= 0;
    else if (lsu_io.store_set_violation && ~branch_miss_first)
        recovery_start_point <= alloc_ptr == 0 ? 63 : alloc_ptr - 1;
    else if (exe_io.exe_alu_csr_bc.valid && exe_io.exe_alu_csr_bc.is_branch) begin
        recovery_start_point <= alloc_ptr == 0 ? 63 : alloc_ptr - 1;
    end else begin
        recovery_start_point <= recovery_start_point;
    end
end

always_ff @(posedge clk) begin
    if (rst)
        recovery_rollback_counter <= 0;
    else if (pipe_ctrl_io.recovery_flush && (recovery_target != alloc_ptr))
        recovery_rollback_counter <= recovery_rollback_counter_temp1 >= recovery_rollback_counter_temp2 ?
                                recovery_rollback_counter_temp2 + 1 : recovery_rollback_counter_temp1;
    else if (pipe_ctrl_io.recovery_rollback)
        recovery_rollback_counter <= recovery_rollback_counter >= 2 ? recovery_rollback_counter - 2 : 0;
    else
        recovery_rollback_counter <= recovery_rollback_counter;
end

always_ff @(posedge clk) begin
    if (rst)
        branch_miss_delay <= 0;
    else if ((lsu_io.store_set_violation && branch_miss_first) || 
            (exe_io.exe_alu_csr_bc.valid && exe_io.exe_alu_csr_bc.is_branch && pipe_ctrl_io.bp_PrMiss))
        branch_miss_delay <= 1;
    else if (pipe_ctrl_io.recovery_rollback)
        branch_miss_delay <= 0;
    else
        branch_miss_delay <= branch_miss_delay;

end

always_comb branch_miss_first = ((lsu_io.store_set_rob_tag > exe_io.exe_alu_csr_bc.commit_addr) ^
                                (lsu_io.store_set_rob_tag > commit_ptr) ^
                                (exe_io.exe_alu_csr_bc.commit_addr >= commit_ptr)) &&
                                (exe_io.exe_alu_csr_bc.valid && exe_io.exe_alu_csr_bc.is_branch && pipe_ctrl_io.bp_PrMiss);
always_comb commit_io.branch_miss_first = branch_miss_first;
always_comb recovery_rollback_counter_temp1 = (8-recovery_target[2:0]);
always_comb recovery_rollback_counter_temp2 = (recovery_start_point >= recovery_target) ? 
                (recovery_start_point - recovery_target) : (recovery_start_point + 64 - recovery_target);

always_comb pre_alloc_ptr = alloc_ptr == 0 ? ROB_ENTRY_NUM - 1 : alloc_ptr - 1;

always_ff @(posedge clk) begin
    if (rst)
        recovery_counter <= 0;
    else if (lsu_io.store_set_violation && ~branch_miss_first)
        recovery_counter <= 1 + ((pre_alloc_ptr >= lsu_io.store_set_rob_tag) ?
                             pre_alloc_ptr - lsu_io.store_set_rob_tag :
                             pre_alloc_ptr + ROB_ENTRY_NUM - lsu_io.store_set_rob_tag);
    else if (exe_io.exe_alu_csr_bc.valid && exe_io.exe_alu_csr_bc.is_branch)
        recovery_counter <= (pre_alloc_ptr >= exe_io.exe_alu_csr_bc.commit_addr) ?
                             pre_alloc_ptr - exe_io.exe_alu_csr_bc.commit_addr :
                             pre_alloc_ptr + ROB_ENTRY_NUM - exe_io.exe_alu_csr_bc.commit_addr;
    else
        recovery_counter <= recovery_counter;
end

always_comb rnds_io.recovery_no_copy = (recovery_rollback_counter_temp1 >= recovery_rollback_counter_temp2) ||
                    (recovery_target == alloc_ptr);

always_comb begin
    if (lsu_io.store_set_violation && ~branch_miss_first)
        csr_io.recovery_distance = 1 + ((pre_alloc_ptr >= lsu_io.store_set_rob_tag) ?
                             pre_alloc_ptr - lsu_io.store_set_rob_tag :
                             pre_alloc_ptr + ROB_ENTRY_NUM - lsu_io.store_set_rob_tag);
    else if (exe_io.exe_alu_csr_bc.valid && exe_io.exe_alu_csr_bc.is_branch)
        csr_io.recovery_distance = (pre_alloc_ptr >= exe_io.exe_alu_csr_bc.commit_addr) ?
                             pre_alloc_ptr - exe_io.exe_alu_csr_bc.commit_addr :
                             pre_alloc_ptr + ROB_ENTRY_NUM - exe_io.exe_alu_csr_bc.commit_addr;
    else csr_io.recovery_distance = 0;
end
always_comb csr_io.start_recovery_tag = 
    (exe_io.exe_alu_csr_bc.valid && exe_io.exe_alu_csr_bc.is_branch) ? 
    exe_io.exe_alu_csr_bc.commit_addr : lsu_io.store_set_rob_tag ;


always_comb recovery_pre_ptr = (recovery_ptr == 0 ? ROB_ENTRY_NUM - 1 :  recovery_ptr - 1);

always_ff @(posedge clk) begin
    if (rst)
        recovery_ptr <= 0;
    else if (pipe_ctrl_io.recovery_rollback &&
             recovery_rollback_counter >= 0 ) begin
        recovery_ptr <= (recovery_rollback_counter >= 2 ? recovery_ptr - 2 : recovery_ptr - 1);
    end else begin
        recovery_ptr <= recovery_rollback_counter_temp1 >= recovery_rollback_counter_temp2 ?
                    recovery_start_point : {recovery_target[5:3], 3'b111};
    end
end

always_comb begin
    if (recovery_rollback_counter == 0 && pipe_ctrl_io.recovery_rollback)
        pipe_ctrl_io.ROB_recovery_finished = 1;
    else
        pipe_ctrl_io.ROB_recovery_finished = 0;
end

always_comb begin
    if (recovery_rollback_counter >= 1 && rob[recovery_ptr].stale_valid) begin
        recovery_entry_rd1_valid = 1;
        recovery_new_alloc_prf_rd1 = rob[recovery_ptr].rd_prf;
        recovery_old_map_prf_rd1 = rob[recovery_ptr].stale_prf;
        recovery_arf_rd1 = rob[recovery_ptr].rd_arf;
    end else if (recovery_rollback_counter >= 2 && rob[recovery_pre_ptr].stale_valid) begin
        recovery_entry_rd1_valid = 1;
        recovery_new_alloc_prf_rd1 = rob[recovery_pre_ptr].rd_prf;
        recovery_old_map_prf_rd1 = rob[recovery_pre_ptr].stale_prf;
        recovery_arf_rd1 = rob[recovery_pre_ptr].rd_arf;
    end else begin
        recovery_entry_rd1_valid = 0;
        recovery_new_alloc_prf_rd1 = 0;
        recovery_old_map_prf_rd1 = 0;
        recovery_arf_rd1 = 0;
    end
end

always_comb begin
    if (recovery_rollback_counter >= 2 && rob[recovery_ptr].stale_valid &&
        rob[recovery_pre_ptr].stale_valid) begin
        recovery_entry_rd0_valid = 1;
        recovery_new_alloc_prf_rd0 = rob[recovery_pre_ptr].rd_prf;
        recovery_old_map_prf_rd0 = rob[recovery_pre_ptr].stale_prf;
        recovery_arf_rd0 = rob[recovery_pre_ptr].rd_arf;
    end else begin
        recovery_entry_rd0_valid = 0;
        recovery_new_alloc_prf_rd0 = 0;
        recovery_old_map_prf_rd0 = 0;
        recovery_arf_rd0 = 0;
    end
end

always_comb begin
    if (pipe_ctrl_io.recovery_rollback) begin
        commit_io.recovery_old_prf_0 <= recovery_old_map_prf_rd0;
        commit_io.recovery_arf_0 <= recovery_arf_rd0;
        commit_io.recovery_arf_map_0_valid <= recovery_entry_rd0_valid;
        commit_io.recovery_old_prf_1 <= recovery_old_map_prf_rd1;
        commit_io.recovery_arf_1 <= recovery_arf_rd1;
        commit_io.recovery_arf_map_1_valid <= recovery_entry_rd1_valid;
    end else begin
        commit_io.recovery_old_prf_0 <= 0;
        commit_io.recovery_arf_0 <= 0;
        commit_io.recovery_arf_map_0_valid <= 0;
        commit_io.recovery_old_prf_1 <= 0;
        commit_io.recovery_arf_1 <= 0;
        commit_io.recovery_arf_map_1_valid <= 0;
    end
end

always_comb begin
    pipe_ctrl_io.flush_rob_tag_0 = (recovery_target == alloc_ptr) ? recovery_start_point : recovery_target;
    pipe_ctrl_io.flush_rob_tag_1 = recovery_start_point;
    pipe_ctrl_io.flush_rob_tag_0_valid = 0;
    pipe_ctrl_io.flush_rob_tag_1_valid = 0;
end

// =======================================================================
// ROB payload logic
// =======================================================================
genvar idx;

generate
for (idx = 0 ; idx < ROB_ENTRY_NUM ; idx = idx + 1) begin
    always_ff @(posedge clk) begin
        if (rst)
            rob[idx].valid <= 0;
        if (rnds_io.instr0_req && alloc_ptr == idx)
            rob[idx].valid <= 1;
        if (!rnds_io.instr0_req && rnds_io.instr1_req && alloc_ptr == idx)
            rob[idx].valid <= 1;
        if (rnds_io.instr0_req && rnds_io.instr1_req && alloc_next_ptr == idx)
            rob[idx].valid <= 1;
        if (commit_ptr == idx && commit_instr0)
            rob[idx].valid <= 0; //commit
        if (commit_next_ptr == idx && commit_instr1)
            rob[idx].valid <= 0; //commit
        if (pipe_ctrl_io.recovery_flush && ~branch_miss_delay)
            rob[idx].valid <= rob[idx].valid ? ~FlushROB_lsv(idx, recovery_target, recovery_start_point) : rob[idx].valid;
        if (pipe_ctrl_io.recovery_flush && branch_miss_delay)
            rob[idx].valid <= rob[idx].valid && (recovery_target != alloc_ptr) ? 
                    ~FlushROB_br(idx, recovery_target, recovery_start_point) : rob[idx].valid;
    end

    always_ff @(posedge clk) begin
        if (rst)
            rob[idx].finished <= 0; 
        if (rnds_io.instr0_req && alloc_ptr == idx)
            rob[idx].finished <= 0;
        if (!rnds_io.instr0_req && rnds_io.instr1_req && alloc_ptr == idx)
            rob[idx].finished <= 0;
        if (rnds_io.instr0_req && rnds_io.instr1_req && alloc_next_ptr == idx)
            rob[idx].finished <= 0;
        if (exe_io.exe_alu_csr_bc.valid && exe_io.exe_alu_csr_bc.commit_addr == idx)
            rob[idx].finished <= 1;
        if (exe_io.exe_ALU_1.valid && exe_io.exe_ALU_1.commit_addr == idx)
            rob[idx].finished <= 1;
        if (exe_io.exe_muldiv.valid && exe_io.exe_muldiv.commit_addr == idx)
            rob[idx].finished <= 1;
        if (lsu_io.LSU_commit.valid && lsu_io.LSU_commit.commit_addr == idx)
            rob[idx].finished <= 1;
        if (ma_io.load_commit.valid && ma_io.load_commit.commit_addr == idx)
            rob[idx].finished <= 1;
    end

    always_ff @(posedge clk) begin
        if (rst)
            rob[idx].stale_valid <= 0;
        if (rnds_io.instr0_req && alloc_ptr == idx)
            rob[idx].stale_valid <= rnds_io.instr0_stale_rd_valid;
        if (!rnds_io.instr0_req && rnds_io.instr1_req && alloc_ptr == idx)
            rob[idx].stale_valid <= rnds_io.instr1_stale_rd_valid;
        if (rnds_io.instr0_req && rnds_io.instr1_req && alloc_next_ptr == idx)
            rob[idx].stale_valid <= rnds_io.instr1_stale_rd_valid;
    end

    always_ff @(posedge clk) begin
        if (rst)
            rob[idx].is_store <= 0;
        if (rnds_io.instr0_req && alloc_ptr == idx)
            rob[idx].is_store <= rnds_io.instr0_is_store_op;
        if (!rnds_io.instr0_req && rnds_io.instr1_req && alloc_ptr == idx)
            rob[idx].is_store <= rnds_io.instr1_is_store_op;
        if (rnds_io.instr0_req && rnds_io.instr1_req && alloc_next_ptr == idx)
            rob[idx].is_store <= rnds_io.instr1_is_store_op;
    end

    always_ff @(posedge clk) begin
        if (rst) begin
            rob[idx].stale_prf <= 0;
            rob[idx].rd_arf <= 0;
            rob[idx].rd_prf <= 0;
        end

        if (rnds_io.instr0_req && alloc_ptr == idx) begin
            rob[idx].stale_prf <= rnds_io.instr0_stale_rd;
            rob[idx].rd_arf <= rnds_io.instr0_rd_arf;
            rob[idx].rd_prf <= rnds_io.instr0_rd_prf;
            rob[idx].trunc_pc <= rnds_io.instr0_pc;
        end
        if (!rnds_io.instr0_req && rnds_io.instr1_req && alloc_ptr == idx) begin
            rob[idx].stale_prf <= rnds_io.instr1_stale_rd;
            rob[idx].rd_arf <= rnds_io.instr1_rd_arf;
            rob[idx].rd_prf <= rnds_io.instr1_rd_prf;
            rob[idx].trunc_pc <= rnds_io.instr1_pc;
        end
        if (rnds_io.instr0_req && rnds_io.instr1_req && alloc_next_ptr == idx) begin
            rob[idx].stale_prf <= rnds_io.instr1_stale_rd;
            rob[idx].rd_arf <= rnds_io.instr1_rd_arf;
            rob[idx].rd_prf <= rnds_io.instr1_rd_prf;
            rob[idx].trunc_pc <= rnds_io.instr1_pc;
        end
    end
end
endgenerate


// =======================================================================
// Store buffer flush and commit logic and Load buffer
// =======================================================================

always_comb begin
    commit_io.store_commit_0_valid = commit_instr0 && rob[commit_ptr].is_store;
    commit_io.store_commit_1_valid = commit_instr1 && rob[commit_next_ptr].is_store;
end

always_comb begin
    commit_io.commit_instr0 = commit_instr0;
    commit_io.commit_instr1 = commit_instr1;
end

logic recovery_entry_0_valid;
logic recovery_entry_1_valid;

always_ff @(posedge clk) begin
    if (rst) begin
        commit_io.store_flush_0_valid <= 0;
        commit_io.store_flush_1_valid <= 1;
    end else if (pipe_ctrl_io.recovery_procedure) begin
        commit_io.store_flush_0_valid <= recovery_rollback_counter >= 1 && 
                                              rob[recovery_ptr].is_store &&
                                              rob[recovery_ptr].finished;
        commit_io.store_flush_1_valid <= recovery_rollback_counter >= 2 &&
                                              rob[recovery_pre_ptr].is_store && 
                                              rob[recovery_pre_ptr].finished; 
    end else begin
        commit_io.store_flush_0_valid <= 0;
        commit_io.store_flush_1_valid <= 0;
    end
end

function logic FlushROB_lsv(
        input [ROB_ENTRY_WIDTH-1:0] idx,
        input [ROB_ENTRY_WIDTH-1:0] flush_start,
        input [ROB_ENTRY_WIDTH-1:0] flush_end
    );
        begin
            FlushROB_lsv = (flush_end >= idx) ^ (flush_end >= flush_start) ^ (idx >= flush_start);
        end
endfunction

function logic FlushROB_br(
        input [ROB_ENTRY_WIDTH-1:0] idx,
        input [ROB_ENTRY_WIDTH-1:0] flush_start,
        input [ROB_ENTRY_WIDTH-1:0] flush_end
    );
        begin
            FlushROB_br = (flush_end >= idx) ^ (flush_end >= flush_start) ^ (idx >= flush_start);
        end
endfunction

`ifdef FALCO_SIM_DEBUG
logic ls_violation/*verilator public*/;
logic [31:0] ls_rob_tag/*verilator public*/;
logic [31:0] br_miss/*verilator public*/;
logic [31:0] br_rob_tag/*verilator public*/;

always_comb ls_violation = lsu_io.store_set_violation;
always_comb ls_rob_tag = lsu_io.store_set_rob_tag;
always_comb br_miss = exe_io.exe_alu_csr_bc.valid && exe_io.exe_alu_csr_bc.is_branch && pipe_ctrl_io.bp_PrMiss;
always_comb br_rob_tag = exe_io.exe_alu_csr_bc.commit_addr;
`endif

`ifdef FALCO_SIM_DEBUG
    reg commit_0_dly;
    reg commit_1_dly;
    reg commit_0_dly_dly;
    reg commit_1_dly_dly;
    arf_specifier_t instr0_rd_dly;
    arf_specifier_t instr1_rd_dly;
    arf_specifier_t instr0_rd_dly_dly;
    arf_specifier_t instr1_rd_dly_dly;
    rob_tag_t commit_rob_tag_0;
    rob_tag_t commit_rob_tag_1;

    always_ff @(posedge clk) begin
        if (rst) begin
            commit_0_dly <= 0;
            commit_1_dly <= 0;
            instr0_rd_dly <= 0;
            instr1_rd_dly <= 0;
            commit_0_dly_dly <= 0;
            commit_1_dly_dly <= 0;
            instr0_rd_dly_dly <= 0;
            instr1_rd_dly_dly <= 0;
            commit_rob_tag_0 <= 0;
            commit_rob_tag_1 <= 0;
        end else begin
            commit_0_dly <= commit_instr0;
            commit_1_dly <= commit_instr1;
            instr0_rd_dly <= commit_io.committed_update.map_arf_0;
            instr1_rd_dly <= commit_io.committed_update.map_arf_1;
            commit_0_dly_dly <= commit_0_dly;
            commit_1_dly_dly <= commit_1_dly;
            instr0_rd_dly_dly <= instr0_rd_dly;
            instr1_rd_dly_dly <= instr1_rd_dly;
            commit_rob_tag_0 <= commit_ptr;
            commit_rob_tag_1 <= commit_next_ptr;
        end
    end
    
    function [31:0] ver_is_ROB_commit_rob_tag_0;
        /*verilator public*/
        ver_is_ROB_commit_rob_tag_0 = commit_rob_tag_0;
    endfunction

    function [31:0] ver_is_ROB_commit_rob_tag_1;
        /*verilator public*/
        ver_is_ROB_commit_rob_tag_1 = commit_rob_tag_1;
    endfunction

    function [31:0] ver_is_ROB_commit_instr0;
        /*verilator public*/
        ver_is_ROB_commit_instr0 = commit_0_dly ? 1 : 0;
    endfunction

    function [31:0] ver_is_ROB_commit_instr1;
        /*verilator public*/
        ver_is_ROB_commit_instr1 = commit_1_dly ? 1 : 0;
    endfunction

    function [31:0] ver_ROB_commit_instr0_rd;
        /*verilator public*/
        ver_ROB_commit_instr0_rd = {27'b0, instr0_rd_dly};
    endfunction

    function [31:0] ver_ROB_commit_instr1_rd;
        /*verilator public*/
        ver_ROB_commit_instr1_rd = {27'b0,instr1_rd_dly};
    endfunction

    function [31:0] ver_ROB_entry_trunc_pc;
        /*verilator public*/
        input [31:0] index;
        ver_ROB_entry_trunc_pc = rob[index].trunc_pc;
    endfunction

    function [31:0] ver_ROB_entry_valid;
        /*verilator public*/
        input [31:0] index;
        ver_ROB_entry_valid = rob[index].valid;
    endfunction

    function [31:0] ver_ROB_entry_finished;
        /*verilator public*/
        input [31:0] index;
        ver_ROB_entry_finished = rob[index].finished;
    endfunction
    function [31:0] ver_ROB_entry_rd_arf;
        /*verilator public*/
        input [31:0] index;
        ver_ROB_entry_rd_arf = rob[index].rd_arf;
    endfunction

    function [31:0] ver_ROB_entry_rd_prf;
        /*verilator public*/
        input [31:0] index;
        ver_ROB_entry_rd_prf = rob[index].rd_prf;
    endfunction

    function [31:0] ver_store_flush_0;
        /*verilator public*/
        ver_store_flush_0 = commit_io.store_flush_0_valid;
    endfunction

    function [31:0] ver_store_flush_1;
        /*verilator public*/
        ver_store_flush_1 = commit_io.store_flush_1_valid;
    endfunction
`endif

endmodule
