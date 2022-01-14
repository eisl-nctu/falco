`timescale 1ns/1ps
// =============================================================================
//  Program : int_issue_queue.sv
//  Author  : Hon-Chou Dai
//  Date    : May/15/2021
// -----------------------------------------------------------------------------
//  Description:
//  Integer instrction issue queue for Falco. There is a replay unit for handle error 
//  caused from wake up instrucion which has dependency with load instruction early.
// -----------------------------------------------------------------------------
//  Revision information:
//
//    August/07/2021, by Chun-Wei Chao:
//      Add some signal to make insturction wakeup early.
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

module int_issue_queue(
    input clk,
    input rst,

    input logic muldiv_busy,
    rename_dispatch_io.int_iq rnds_io,
    int_issue_queue_io.int_iq issue_io,
    mem_issue_queue_io.int_iq mem_iq_io,
    exe_stage_io.int_iq broadcast_io,
    load_store_unit_io.int_iq lsu_io,
    commit_stage_io.int_iq commit_io,
    pipeline_control_recovery_io.int_iq pipe_ctrl_io
);

//issue -> to register read stage
//dispatch -> dispatch from rename and dispatch stage
integer i;

// issue queue full/empty logic
logic [INT_IQ_WIDTH:0] counter;

int_dispatch_pack_t issue_slot [INT_IQ_NUM];
//Issue queue wakeup logic and select logic
logic valid [INT_IQ_NUM] /*verilator public*/; //entry valid
logic rs1_ready [INT_IQ_NUM] /*verilator public*/;
logic rs2_ready [INT_IQ_NUM] /*verilator public*/;
//branch_info_t issue_slot_br_info [INT_IQ_NUM - 1:0];

//========================================
// Replay Buffer connect line
//========================================
logic replay_issue_first;

logic               wake_up_stall_issue_instr0_valid;
logic               wake_up_stall_issue_instr1_valid;
int_dispatch_pack_t wake_up_stall_issue_instr0_pack;
int_dispatch_pack_t wake_up_stall_issue_instr1_pack;
logic               replay_issue_instr0_valid;
logic               replay_issue_instr1_valid;
int_dispatch_pack_t replay_issue_instr0_pack;
int_dispatch_pack_t replay_issue_instr1_pack;

logic replay_fifo_issue_muldiv; //non poison issue or replay

//========================================
// Issue logic signal
//========================================
logic issue_ready [INT_IQ_NUM] /*verilator public*/; //ready to go
logic issue_rs1 [INT_IQ_NUM];
logic issue_rs2 [INT_IQ_NUM];
logic entry_is_csr_instr[INT_IQ_NUM];
logic entry_is_ctrl_instr[INT_IQ_NUM];
logic entry_is_muldiv_instr[INT_IQ_NUM];

`ifdef POSSEL_REPLAY
logic entry_posion_ld_spec_wake_up[INT_IQ_NUM];
logic entry_posion_iq_spec_wake_up[INT_IQ_NUM];
`else
logic entry_is_critical[INT_IQ_NUM];
`endif

logic [INT_IQ_WIDTH-1:0] issue_slot_idx0;
logic [INT_IQ_WIDTH-1:0] issue_slot_idx1;
logic issue_slot_idx0_valid;
logic issue_slot_idx1_valid;

`ifdef POSSEL_REPLAY
logic issue_slot_idx0_poison;
logic issue_slot_idx1_poison;
`endif

logic muldiv_issuable;
//========================================
// dispatch logic signal
//========================================
logic [INT_IQ_WIDTH-1:0] dispatch_slot_idx0;
logic [INT_IQ_WIDTH-1:0] dispatch_slot_idx1;
logic [INT_IQ_WIDTH-1:0] allocatable_slot_idx0;
logic [INT_IQ_WIDTH-1:0] allocatable_slot_idx1;
logic allocatable_slot_idx0_valid;
logic allocatable_slot_idx1_valid;
logic allocatable_slot_idx0_valid_reg; //need it ?
logic allocatable_slot_idx1_valid_reg; //need it ?

logic dispatch_instr0_valid;
logic dispatch_instr1_valid;

//========================================
// Issue poison
//========================================
`ifdef POSSEL_REPLAY
logic issue_poison_idx0_reg;
logic issue_poison_idx1_reg;
`endif
logic issue_lock;

always_comb 
    issue_lock = pipe_ctrl_io.recovery_stall ||
                 pipe_ctrl_io.load_wake_up_predict_failed ||
                 pipe_ctrl_io.load_depend_replay ||
                 pipe_ctrl_io.INT_IQ_stall;
always_comb 
    dispatch_instr0_valid = rnds_io.int_pack0_valid && 
                            allocatable_slot_idx0_valid_reg &&
                            ~pipe_ctrl_io.recovery_stall;
always_comb
    dispatch_instr1_valid = rnds_io.int_pack1_valid &&
                            allocatable_slot_idx1_valid_reg &&
                            ~pipe_ctrl_io.recovery_stall;


always_ff @(posedge clk) begin
    if (rst) begin
        counter <= INT_IQ_NUM;
    end else begin
        counter <= counter;
    end
end

//=====================================================================
// issue slot logic unit
//=====================================================================
genvar geni;

generate
for (geni = 0 ; geni < INT_IQ_NUM ; geni = geni + 1) begin
    always_ff @(posedge clk) begin
        if (rst) begin
            issue_slot[geni] <= 0;
        end else if (geni == dispatch_slot_idx0 && dispatch_instr0_valid ) begin
            issue_slot[geni] <= rnds_io.int_pack0;
        end else if (geni == dispatch_slot_idx1 && dispatch_instr1_valid ) begin
            issue_slot[geni] <= rnds_io.int_pack1;
        end else if (pipe_ctrl_io.recovery_stall) begin
            issue_slot[geni] <= IsBrROBKill(pipe_ctrl_io.recovery_flush_BCAST, issue_slot[geni].rob_tag) ? 0 : issue_slot[geni];
        end else begin
            issue_slot[geni] <= issue_slot[geni];
        end
    end
end

for (geni = 0 ; geni < INT_IQ_NUM ; geni = geni + 1) begin
    always_ff @(posedge clk) begin
        if (rst) begin
            valid[geni] <= 0;
        end else if (pipe_ctrl_io.recovery_stall) begin
            valid[geni] <= IsBrROBKill(pipe_ctrl_io.recovery_flush_BCAST, issue_slot[geni].rob_tag) ? 0 : valid[geni];
        end else if ( (geni == dispatch_slot_idx0 && dispatch_instr0_valid) ||
                      (geni == dispatch_slot_idx1 && dispatch_instr1_valid)) begin
            valid[geni] <= 1;
        end else if ( ((geni == issue_slot_idx0 && issue_slot_idx0_valid) ||
                       (geni == issue_slot_idx1 && issue_slot_idx1_valid)) &&
                       ~issue_lock) begin
            valid[geni] <= 0;
        end else begin
            valid[geni] <= valid[geni];
        end
    end
end

for (geni = 0 ; geni < INT_IQ_NUM ; geni = geni + 1) begin
    always_ff @(posedge clk) begin
        if (rst) begin
            rs1_ready[geni] <= 0;
        end else if (geni == dispatch_slot_idx0 && dispatch_instr0_valid) begin
            rs1_ready[geni] <= bypass_network(rnds_io.int_pack0.rs1_addr, 
                                              rnds_io.int_instr0_rs1_ready);
        end else if (geni == dispatch_slot_idx1 && dispatch_instr1_valid) begin
            rs1_ready[geni] <= bypass_network(rnds_io.int_pack1.rs1_addr, 
                                              rnds_io.int_instr1_rs1_ready);
        end else if (wakeup_network(issue_slot[geni].rs1_addr) && 
                     ~pipe_ctrl_io.recovery_stall && ~pipe_ctrl_io.load_wake_up_predict_failed) begin //TODO: memory broadcast port
            rs1_ready[geni] <= 1;
        end else begin
            rs1_ready[geni] <= rs1_ready[geni];
        end
    end
end
for (geni = 0 ; geni < INT_IQ_NUM ; geni = geni + 1) begin
    always_ff @(posedge clk) begin
        if (rst) begin
            rs2_ready[geni] <= 0;
        end else if (geni == dispatch_slot_idx0 && dispatch_instr0_valid) begin
            rs2_ready[geni] <= bypass_network(rnds_io.int_pack0.rs2_addr, 
                                              rnds_io.int_instr0_rs2_ready);
        end else if (geni == dispatch_slot_idx1 && dispatch_instr1_valid) begin
            rs2_ready[geni] <= bypass_network(rnds_io.int_pack1.rs2_addr, 
                                              rnds_io.int_instr1_rs2_ready);
        end else if (wakeup_network(issue_slot[geni].rs2_addr) &&
                     ~pipe_ctrl_io.recovery_stall && ~pipe_ctrl_io.load_wake_up_predict_failed) begin //TODO: memory broadcast port
            rs2_ready[geni] <= 1;
        end else begin
            rs2_ready[geni] <= rs2_ready[geni];
        end
    end
end

`ifdef POSSEL_REPLAY

for (geni = 0 ; geni < INT_IQ_NUM ; geni = geni + 1) begin
    always_comb begin
        entry_posion_ld_spec_wake_up[geni] = 
            ld_spec_wakeup_network(issue_slot[geni].rs1_addr) || 
            ld_spec_wakeup_network(issue_slot[geni].rs2_addr); 
    end
end

for (geni = 0 ; geni < INT_IQ_NUM ; geni = geni + 1) begin
    always_comb begin
        entry_posion_iq_spec_wake_up[geni] = 
            iq_spec_wakeup_network(issue_slot[geni].rs1_addr) || 
            iq_spec_wakeup_network(issue_slot[geni].rs2_addr); 
    end
end
`else

for (geni = 0 ; geni < INT_IQ_NUM ; geni = geni + 1) begin
    always_comb begin
        entry_is_critical[geni] = InstrIsCritcal(issue_slot[geni].rob_tag,
                                             commit_io.cur_commit_rob_tag,
                                             lsu_io.cur_lsu_rob_tag);
    end
end

`endif

for (geni = 0 ; geni < INT_IQ_NUM ; geni = geni + 1)
    always_comb
        issue_ready[geni] = (rs1_ready[geni] || issue_rs1[geni]) & (rs2_ready[geni] || issue_rs2[geni]);

for (geni = 0 ; geni < INT_IQ_NUM ; geni = geni + 1)
    always_comb
        issue_rs1[geni] = (issue_slot[geni].rs1_addr == issue_io.issue_alu0_rd && issue_io.issue_alu0_rd_valid) ||
            (issue_slot[geni].rs1_addr == issue_io.issue_alu1_rd && issue_io.issue_alu1_rd_valid);

for (geni = 0 ; geni < INT_IQ_NUM ; geni = geni + 1)
    always_comb
        issue_rs2[geni] = (issue_slot[geni].rs2_addr == issue_io.issue_alu0_rd && issue_io.issue_alu0_rd_valid) ||
            (issue_slot[geni].rs2_addr == issue_io.issue_alu1_rd && issue_io.issue_alu1_rd_valid);

for (geni = 0 ; geni < INT_IQ_NUM ; geni = geni + 1)
    always_comb
        entry_is_csr_instr[geni] = issue_slot[geni].is_csr_instr;

for (geni = 0 ; geni < INT_IQ_NUM ; geni = geni + 1)
    always_comb
        entry_is_ctrl_instr[geni] = issue_slot[geni].is_branch || 
                                    issue_slot[geni].is_jalr || 
                                    issue_slot[geni].is_jal;

for (geni = 0 ; geni < INT_IQ_NUM ; geni = geni + 1)
    always_comb
        entry_is_muldiv_instr[geni] = issue_slot[geni].alu_muldiv_sel;
endgenerate
//=====================================================================
// Allocation unit
//=====================================================================

empty_entry_finder8_wrapper empty_entry_finder(
    .dispatch_slot_idx0(dispatch_slot_idx0),
    .dispatch_slot_idx1(dispatch_slot_idx1),
    .dispatch_slot_idx0_valid(dispatch_instr0_valid),
    .dispatch_slot_idx1_valid(dispatch_instr1_valid),
    .slot_valid(valid),

    .allocatable_slot_idx0(allocatable_slot_idx0),
    .allocatable_slot_idx1(allocatable_slot_idx1),
    .allocatable_slot_idx0_valid(allocatable_slot_idx0_valid),
    .allocatable_slot_idx1_valid(allocatable_slot_idx1_valid)
);

always_ff @(posedge clk) begin
    if (rst) begin
        dispatch_slot_idx0 <= 0;
        dispatch_slot_idx1 <= 1;
        allocatable_slot_idx0_valid_reg <= 1;
        allocatable_slot_idx1_valid_reg <= 1;
    end else begin
        dispatch_slot_idx0 <= allocatable_slot_idx0;
        dispatch_slot_idx1 <= allocatable_slot_idx1;
        allocatable_slot_idx0_valid_reg <= allocatable_slot_idx0_valid;
        allocatable_slot_idx1_valid_reg <= allocatable_slot_idx1_valid;
    end
end

always_comb begin
    rnds_io.int_iq_instr0_check_top = allocatable_slot_idx0_valid;
    rnds_io.int_iq_instr1_check_top = allocatable_slot_idx1_valid;
end

// Issue & Selector unit

always_comb
    replay_fifo_issue_muldiv = (replay_issue_instr1_valid && replay_issue_instr1_pack.alu_muldiv_sel) ||
                               (wake_up_stall_issue_instr1_valid && wake_up_stall_issue_instr1_pack.alu_muldiv_sel);

always_comb
    pipe_ctrl_io.replay_failed_to_issue_muldiv = replay_issue_instr1_valid && 
                                                 replay_issue_instr1_pack.alu_muldiv_sel && 
                                                 ~muldiv_issuable;

`ifdef POSSEL_REPLAY

int_iq_ctr_possel_balance_selector INT_selector(
    .clk(clk),
    .rst(rst),
    .entry_valid(valid),
    .entry_ready(issue_ready),
    .entry_is_ctrl_instr(entry_is_ctrl_instr),
    .entry_is_csr_instr(entry_is_csr_instr),
    .entry_is_muldiv(entry_is_muldiv_instr),
    .ld_poison_wake_up(entry_posion_ld_spec_wake_up),
    .iq_poison_wake_up(entry_posion_iq_spec_wake_up),
    .replay_issue_first(replay_issue_first),
    .dispatch_slot_idx0(dispatch_slot_idx0),
    .dispatch_slot_idx1(dispatch_slot_idx1),
    .dispatch_instr0_valid(dispatch_instr0_valid),
    .dispatch_instr1_valid(dispatch_instr1_valid),
    .muldiv_busy(muldiv_busy),
    .replay_issue_muldiv(replay_fifo_issue_muldiv),
    .icache_miss(pipe_ctrl_io.icache_miss_stall_branch),
    .recovery_flush(pipe_ctrl_io.recovery_stall),
    .non_posion_issue(pipe_ctrl_io.load_wake_up_failed_stall), //keep issue non relative instruction
    .issue_replay(pipe_ctrl_io.load_depend_replay),
    .branch_miss_flush(pipe_ctrl_io.recovery_stall),
    .load_wake_up_kill(pipe_ctrl_io.load_wake_up_predict_failed),
    .muldiv_issuable(muldiv_issuable),
    .issue_slot_idx0(issue_slot_idx0),
    .issue_slot_idx1(issue_slot_idx1),
    .issue_slot_idx0_is_poison(issue_slot_idx0_poison),
    .issue_slot_idx1_is_poison(issue_slot_idx1_poison),
    .issue_slot_idx0_valid(issue_slot_idx0_valid),
    .issue_slot_idx1_valid(issue_slot_idx1_valid)
);


int_posssel_replay_unit Replay_Unit(
    .clk(clk),
    .rst(rst),

    .recovery_flush_BCAST(pipe_ctrl_io.recovery_flush_BCAST),
    .load_wake_up_failed_stall(pipe_ctrl_io.load_wake_up_failed_stall),
    .load_wake_up_predict_failed(pipe_ctrl_io.load_wake_up_predict_failed),
    .load_depend_replay(pipe_ctrl_io.load_depend_replay),
    .replay_muldiv_stall(pipe_ctrl_io.replay_muldiv_stall),

    .issue_instr0_valid(issue_io.instr0_valid),
    .issue_instr1_valid(issue_io.instr1_valid),
    .issue_instr0_poison(issue_poison_idx0_reg),
    .issue_instr1_poison(issue_poison_idx1_reg),
    .issue_instr0_pack(issue_io.instr0_pack),
    .issue_instr1_pack(issue_io.instr1_pack),

    .replay_issue_first(replay_issue_first),
    .wake_up_stall_issue_instr0_valid(wake_up_stall_issue_instr0_valid),
    .wake_up_stall_issue_instr1_valid(wake_up_stall_issue_instr1_valid),
    .wake_up_stall_issue_instr0_pack(wake_up_stall_issue_instr0_pack),
    .wake_up_stall_issue_instr1_pack(wake_up_stall_issue_instr1_pack),
    .replay_issue_instr0_valid(replay_issue_instr0_valid),
    .replay_issue_instr1_valid(replay_issue_instr1_valid),
    .replay_issue_instr0_pack(replay_issue_instr0_pack),
    .replay_issue_instr1_pack(replay_issue_instr1_pack)
);
`else
int_iq_ctr_rob_tag_balance_selector INT_selector(
    .clk(clk),
    .rst(rst),
  
    .entry_valid(valid),
    .entry_ready(issue_ready),
    .entry_is_ctrl_instr(entry_is_ctrl_instr),
    .entry_is_csr_instr(entry_is_csr_instr),
    .entry_is_muldiv(entry_is_muldiv_instr),
    .entry_is_critical(entry_is_critical),
    .replay_issue_first(replay_issue_first),
    
    .dispatch_slot_idx0(dispatch_slot_idx0),
    .dispatch_slot_idx1(dispatch_slot_idx1),
    .dispatch_instr0_valid(dispatch_instr0_valid),
    .dispatch_instr1_valid(dispatch_instr1_valid),
    
    .muldiv_busy(muldiv_busy),
    .replay_issue_muldiv(replay_fifo_issue_muldiv),
    .icache_miss(pipe_ctrl_io.icache_miss_stall_branch),
    .recovery_flush(pipe_ctrl_io.recovery_stall),
    .non_posion_issue(pipe_ctrl_io.load_wake_up_failed_stall), //keep issue non relative instruction
    .issue_replay(pipe_ctrl_io.load_depend_replay),
    .branch_miss_flush(pipe_ctrl_io.recovery_stall),
    .load_wake_up_kill(pipe_ctrl_io.load_wake_up_predict_failed),
    .muldiv_issuable(muldiv_issuable),

    .issue_slot_idx0(issue_slot_idx0),
    .issue_slot_idx1(issue_slot_idx1),
    .issue_slot_idx0_valid(issue_slot_idx0_valid),
    .issue_slot_idx1_valid(issue_slot_idx1_valid)
);


int_rob_tag_replay_unit Replay_Unit(
    .clk(clk),
    .rst(rst),

    .recovery_flush_BCAST(pipe_ctrl_io.recovery_flush_BCAST),
    .recovery_stall(pipe_ctrl_io.recovery_stall),
    .load_wake_up_failed_stall(pipe_ctrl_io.load_wake_up_failed_stall),
    .load_wake_up_predict_failed(pipe_ctrl_io.load_wake_up_predict_failed),
    .load_depend_replay(pipe_ctrl_io.load_depend_replay),
    .replay_muldiv_stall(pipe_ctrl_io.replay_muldiv_stall),

    .cur_commit_tag(commit_io.cur_commit_rob_tag),
    .cur_lsu_tag(lsu_io.cur_lsu_rob_tag),

    .issue_instr0_valid(issue_io.instr0_valid),
    .issue_instr1_valid(issue_io.instr1_valid),
    .issue_instr0_pack(issue_io.instr0_pack),
    .issue_instr1_pack(issue_io.instr1_pack),

    .replay_issue_first(replay_issue_first),
    .wake_up_stall_issue_instr0_valid(wake_up_stall_issue_instr0_valid),
    .wake_up_stall_issue_instr1_valid(wake_up_stall_issue_instr1_valid),
    .wake_up_stall_issue_instr0_pack(wake_up_stall_issue_instr0_pack),
    .wake_up_stall_issue_instr1_pack(wake_up_stall_issue_instr1_pack),
    .replay_issue_instr0_valid(replay_issue_instr0_valid),
    .replay_issue_instr1_valid(replay_issue_instr1_valid),
    .replay_issue_instr0_pack(replay_issue_instr0_pack),
    .replay_issue_instr1_pack(replay_issue_instr1_pack)
);
`endif

//===========================================
// pipeline register
//===========================================


`ifdef POSSEL_REPLAY
always_ff @( posedge clk ) begin
    if (rst) begin
        issue_poison_idx0_reg <= 0;
        issue_poison_idx1_reg <= 0;
    end else if (pipe_ctrl_io.recovery_stall) begin
        issue_poison_idx0_reg <= issue_poison_idx0_reg;
        issue_poison_idx1_reg <= issue_poison_idx1_reg;
    end else if (pipe_ctrl_io.load_wake_up_predict_failed || 
                pipe_ctrl_io.replay_muldiv_stall ) begin
        issue_poison_idx0_reg <= 0;
        issue_poison_idx1_reg <= 0;
    end else if (pipe_ctrl_io.load_depend_replay) begin
        issue_poison_idx0_reg <= 0;
        issue_poison_idx1_reg <= 0;
    end else if (pipe_ctrl_io.load_wake_up_failed_stall && replay_issue_first) begin
        issue_poison_idx0_reg <= 0;
        issue_poison_idx1_reg <= 0;
    end else begin
        issue_poison_idx0_reg <= issue_slot_idx0_poison;
        issue_poison_idx1_reg <= issue_slot_idx1_poison;
    end
end

`endif

//TODO: spec tag and stall
always_ff @( posedge clk ) begin
    if (rst) begin
        issue_io.instr0_valid <= 0;
        issue_io.instr1_valid <= 0;
        issue_io.instr0_pack <= 0;
        issue_io.instr1_pack <= 0;
    end else if (pipe_ctrl_io.INT_IQ_stall) begin
        issue_io.instr0_valid <= issue_io.instr0_valid;
        issue_io.instr1_valid <= issue_io.instr1_valid;
        issue_io.instr0_pack <= issue_io.instr0_pack;
        issue_io.instr1_pack <= issue_io.instr1_pack;
    end else if (pipe_ctrl_io.recovery_stall) begin
        issue_io.instr0_valid <= IsBrROBKill(pipe_ctrl_io.recovery_flush_BCAST, issue_io.instr0_pack.rob_tag) ?
                                 0 : issue_io.instr0_valid;
        issue_io.instr1_valid <= IsBrROBKill(pipe_ctrl_io.recovery_flush_BCAST, issue_io.instr1_pack.rob_tag) ?
                                 0 : issue_io.instr1_valid;
        issue_io.instr0_pack <= IsBrROBKill(pipe_ctrl_io.recovery_flush_BCAST, issue_io.instr0_pack.rob_tag) ?
                                 0 : issue_io.instr0_pack;
        issue_io.instr1_pack <= IsBrROBKill(pipe_ctrl_io.recovery_flush_BCAST, issue_io.instr1_pack.rob_tag) ?
                                 0 : issue_io.instr1_pack;
    end else if (pipe_ctrl_io.load_wake_up_predict_failed || 
                pipe_ctrl_io.replay_muldiv_stall ) begin
        issue_io.instr0_valid <= 0;
        issue_io.instr1_valid <= 0;
        issue_io.instr0_pack <= 0;
        issue_io.instr1_pack <= 0;
    end else if (pipe_ctrl_io.load_depend_replay) begin
        issue_io.instr0_valid <= replay_issue_instr0_valid;
        issue_io.instr1_valid <= replay_issue_instr1_valid;
        issue_io.instr0_pack <= replay_issue_instr0_pack;
        issue_io.instr1_pack <= replay_issue_instr1_pack;
    end else if (pipe_ctrl_io.load_wake_up_failed_stall && replay_issue_first) begin
        issue_io.instr0_valid <= wake_up_stall_issue_instr0_valid;
        issue_io.instr1_valid <= wake_up_stall_issue_instr1_valid;
        issue_io.instr0_pack <= wake_up_stall_issue_instr0_pack;
        issue_io.instr1_pack <= wake_up_stall_issue_instr1_pack;
    end else begin
        issue_io.instr0_valid <= issue_slot_idx0_valid;
        issue_io.instr1_valid <= issue_slot_idx1_valid;
        issue_io.instr0_pack <= issue_slot[issue_slot_idx0];
        issue_io.instr1_pack <= issue_slot[issue_slot_idx1];
    end
end

always_comb begin
    issue_io.issue_alu0_rd_valid = issue_io.instr0_valid && ~issue_io.instr0_pack.is_branch &&
                                      ~issue_io.instr0_pack.alu_muldiv_sel;
    issue_io.issue_alu1_rd_valid = issue_io.instr1_valid && ~issue_io.instr1_pack.is_branch &&
                                      ~issue_io.instr1_pack.alu_muldiv_sel;
    issue_io.issue_alu0_rd = issue_io.instr0_pack.rd_addr;
    issue_io.issue_alu1_rd = issue_io.instr1_pack.rd_addr;
end

function logic InstrIsCritcal(
    input rob_tag_t instr_rob_tag,
    input rob_tag_t commit_rob_tag,
    input rob_tag_t lsu_rob_tag
);
begin
    InstrIsCritcal = (lsu_rob_tag != commit_rob_tag) && 
                      (((lsu_rob_tag > instr_rob_tag) ^ (lsu_rob_tag > commit_rob_tag) ^
                     (instr_rob_tag > commit_rob_tag )) || (instr_rob_tag == commit_rob_tag));
end
endfunction

function logic bypass_network(
    input prf_specifier_t prf_addr,
    input logic ready 
);
    begin
        if ((prf_addr == broadcast_io.BCAST_muldiv.prf_addr && broadcast_io.BCAST_muldiv.valid) || 
            (prf_addr == mem_iq_io.BCAST_ld_spec.prf_addr && mem_iq_io.BCAST_ld_spec.valid) ||
            (prf_addr == issue_io.instr0_pack.rd_addr && issue_io.issue_alu0_rd_valid) ||
            (prf_addr == issue_io.instr1_pack.rd_addr && issue_io.issue_alu1_rd_valid)) begin
            bypass_network = 1;    
        end else begin
            bypass_network = ready;
        end
    end
endfunction

function logic wakeup_network(
    input prf_specifier_t prf_addr
);
    begin
        if ((prf_addr == broadcast_io.BCAST_muldiv.prf_addr && broadcast_io.BCAST_muldiv.valid) || 
            (prf_addr == mem_iq_io.BCAST_ld_spec.prf_addr && mem_iq_io.BCAST_ld_spec.valid) ||
            (prf_addr == issue_io.instr0_pack.rd_addr && issue_io.issue_alu0_rd_valid) ||
            (prf_addr == issue_io.instr1_pack.rd_addr && issue_io.issue_alu1_rd_valid)) begin
            wakeup_network = 1;    
        end else begin
            wakeup_network = 0;
        end
    end
endfunction

`ifdef POSSEL_REPLAY
function logic ld_spec_wakeup_network(
    input prf_specifier_t prf_addr
);
    begin
        if ((prf_addr == mem_iq_io.BCAST_ld_spec.prf_addr && mem_iq_io.BCAST_ld_spec.valid)) begin
            ld_spec_wakeup_network = 1;    
        end else begin
            ld_spec_wakeup_network = 0;
        end
    end
endfunction
function logic iq_spec_wakeup_network(
    input prf_specifier_t prf_addr
);
    begin
        if ((prf_addr == issue_io.instr0_pack.rd_addr && issue_io.issue_alu0_rd_valid && 
             issue_poison_idx0_reg) ||
            (prf_addr == issue_io.instr1_pack.rd_addr && issue_io.issue_alu1_rd_valid &&
             issue_poison_idx1_reg)) begin
            iq_spec_wakeup_network = 1;    
        end else begin
            iq_spec_wakeup_network = 0;
        end
    end
endfunction
`endif

`ifdef FALCO_SIM_DEBUG
logic my_stall /*verilator public*/;
logic my_flush /*verilator public*/;
rob_tag_t my_tag1 /*verilator public*/;
rob_tag_t my_tag2 /*verilator public*/;
logic my_valid1 /*verilator public*/;
logic my_valid2 /*verilator public*/;

always_comb my_stall = pipe_ctrl_io.recovery_stall;
always_comb my_flush = pipe_ctrl_io.recovery_flush_BCAST.recovery_flush;
always_comb my_tag1 = pipe_ctrl_io.recovery_flush_BCAST.flush_tag_0;
always_comb my_tag2 = pipe_ctrl_io.recovery_flush_BCAST.flush_tag_1;
// always_comb my_valid1 = pipe_ctrl_io.recovery_flush_BCAST.
// always_comb my_valid2 = pipe_ctrl_io.recovery_flush_BCAST.

`endif

`ifdef FALCO_SIM_DEBUG
    function [31:0] ver_INT_IQ_pc;
        /*verilator public*/
        input [31:0] index;
        ver_INT_IQ_pc = issue_slot[index[2:0]].pc;
    endfunction

    function [31:0] ver_INT_IQ_predict_pc;
        /*verilator public*/
        input [31:0] index;
        ver_INT_IQ_predict_pc = issue_slot[index[2:0]].predict_pc;
    endfunction

    function [31:0] ver_INT_IQ_rd_addr;
        /*verilator public*/
        input [31:0] index;
        ver_INT_IQ_rd_addr = issue_slot[index[2:0]].rd_addr;
    endfunction
    
    function [31:0] ver_INT_IQ_rs1_addr;
        /*verilator public*/
        input [31:0] index;
        ver_INT_IQ_rs1_addr = issue_slot[index[2:0]].rs1_addr;
    endfunction
    
    function [31:0] ver_INT_IQ_rs2_addr;
        /*verilator public*/
        input [31:0] index;
        ver_INT_IQ_rs2_addr = issue_slot[index[2:0]].rs2_addr;
    endfunction

`endif

endmodule
