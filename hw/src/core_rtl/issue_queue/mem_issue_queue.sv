`timescale 1ns/1ps
// =============================================================================
//  Program : mem_issue_queue.sv
//  Author  : Hon-Chou Dai
//  Date    : May/15/2021
// -----------------------------------------------------------------------------
//  Description:
//  Memory instruction issue queue for Falco.
// -----------------------------------------------------------------------------
//  Revision information:
//
//    August/07/2021, by Chun-Wei Chao:
//      Add int_issue instruction information to wake up instrction early.
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

//2 dispatch 1 issue

module mem_issue_queue(
    input clk,
    input rst,

    rename_dispatch_io.mem_iq rnds_io,
    mem_issue_queue_io.mem_iq issue_io,
    int_issue_queue_io.mem_iq grant_io,
    load_store_unit_io.mem_iq lsu_io,
    exe_stage_io.mem_iq broadcast_io,
    pipeline_control_recovery_io.mem_iq pipe_ctrl_io
);

mem_dispatch_pack_t issue_slot[MEM_IQ_NUM]; //TODO: move some part to SRAM

logic [MEM_IQ_NUM-1:0] depvec [MEM_IQ_NUM]/*verilator public*/;
logic [MEM_IQ_NUM-1:0] cur_depvec/*verilator public*/; //current all dependency
logic [MEM_IQ_NUM-1:0] instr0_depvec/*verilator public*/; //current all dependency after issue calculate
logic [MEM_IQ_NUM-1:0] instr0_depvec_ori/*verilator public*/; //current all dependency after issue calculate
logic [MEM_IQ_NUM-1:0] instr1_depvec/*verilator public*/; //current all dependency after instr0 calculate
// 1 -> there is one old store before and not issue
// vec == 0 this instruction can be issue
// issue priority load > store

logic valid [MEM_IQ_NUM] /*verilator public*/; //entry valid
logic rs1_ready [MEM_IQ_NUM] /*verilator public*/; 
logic rs2_ready [MEM_IQ_NUM] /*verilator public*/;
logic issue_rs1 [MEM_IQ_NUM] /*verilator public*/; 
logic issue_rs2 [MEM_IQ_NUM] /*verilator public*/;
logic issue_ready [MEM_IQ_NUM] /*verilator public*/; //ready to go

logic entry_is_store [MEM_IQ_NUM] /*verilator public*/;
logic [MEM_IQ_NUM-1:0] flush_vec /*verilator public*/;
logic [MEM_IQ_NUM-1:0] issue_flush_vec /*verilator public*/;

// Issue logic
logic [MEM_IQ_WIDTH-1:0]issue_slot_idx;
logic issue_slot_idx_valid;

// ld spec brodcast
rob_tag_t ld_spec_rob_tag;

// Dispatch logic
logic [MEM_IQ_WIDTH-1:0] dispatch_slot_idx0;
logic [MEM_IQ_WIDTH-1:0] dispatch_slot_idx1;
logic [MEM_IQ_WIDTH-1:0] allocatable_slot_idx0;
logic [MEM_IQ_WIDTH-1:0] allocatable_slot_idx1;
logic allocatable_slot_idx0_valid;
logic allocatable_slot_idx1_valid;
logic allocatable_slot_idx0_valid_reg; //need it ?
logic allocatable_slot_idx1_valid_reg; //need it ?

logic dispatch_instr0_valid;
logic dispatch_instr1_valid;

logic issue_lock;

//replay issue
logic replay_issue_instr_valid;
mem_dispatch_pack_t replay_issue_instr_pack;

always_comb 
    issue_lock = pipe_ctrl_io.recovery_stall ||
                 pipe_ctrl_io.MEM_IQ_stall ||
                 pipe_ctrl_io.load_wake_up_predict_failed ||
                 pipe_ctrl_io.load_wake_up_failed_stall ||
                 pipe_ctrl_io.load_depend_replay;

always_comb 
    dispatch_instr0_valid = rnds_io.mem_pack0_valid && 
                            allocatable_slot_idx0_valid_reg &&
                            ~ pipe_ctrl_io.recovery_stall;
always_comb
    dispatch_instr1_valid = rnds_io.mem_pack1_valid && 
                            allocatable_slot_idx1_valid_reg &&
                            ~ pipe_ctrl_io.recovery_stall;


//=====================================================================
// issue slot logic unit
//=====================================================================


genvar geni;

always_comb begin
    for (int i = 0 ; i < MEM_IQ_NUM ; i = i + 1) begin
        cur_depvec[i] = (valid[i]) ? 1 : 0;
    end
end

always_comb begin
    if (issue_slot_idx_valid)
        instr0_depvec = rnds_io.mem_pack0.predict_no_vilation && ~rnds_io.mem_pack0.mem_is_store ? 0 :
                     cur_depvec & (~issue_flush_vec);
    else
        instr0_depvec = rnds_io.mem_pack0.predict_no_vilation && ~rnds_io.mem_pack0.mem_is_store ? 0 : cur_depvec;
end

always_comb begin
    if (issue_slot_idx_valid)
        instr0_depvec_ori = cur_depvec & (~issue_flush_vec);
    else
        instr0_depvec_ori = cur_depvec;
end

always_comb begin
    if (dispatch_instr0_valid)
        instr1_depvec = rnds_io.mem_pack1.predict_no_vilation && ~rnds_io.mem_pack1.mem_is_store ? 0 : 
                instr0_depvec_ori | ({{(MEM_IQ_NUM-1){1'b0}},1'b1} << dispatch_slot_idx0);
    else
        instr1_depvec = rnds_io.mem_pack1.predict_no_vilation && ~rnds_io.mem_pack1.mem_is_store ? 0 : 
                instr0_depvec_ori;
end


generate
for (geni = 0 ; geni < MEM_IQ_NUM ; geni = geni + 1) begin
    always_ff @(posedge clk) begin
        if (rst) begin
            issue_slot[geni] <= 0;
        end else if (geni == dispatch_slot_idx0 && dispatch_instr0_valid ) begin
            issue_slot[geni] <= rnds_io.mem_pack0;
        end else if (geni == dispatch_slot_idx1 && dispatch_instr1_valid ) begin
            issue_slot[geni] <= rnds_io.mem_pack1;
        end else begin
            issue_slot[geni] <= issue_slot[geni];
        end
    end
end

for (geni = 0 ; geni < MEM_IQ_NUM ; geni = geni + 1)
    always_comb
        flush_vec[geni] =  IsBrROBKill(pipe_ctrl_io.recovery_flush_BCAST, issue_slot[geni].rob_tag) ? 1 : 0;

for (geni = 0 ; geni < MEM_IQ_NUM ; geni = geni + 1) begin
    always_ff @(posedge clk) begin
        if (rst) begin
            valid[geni] <= 0;
        end else if (pipe_ctrl_io.recovery_stall) begin
            valid[geni] <= flush_vec[geni] ? 0 : valid[geni];
        end else if ( (geni == issue_slot_idx && issue_slot_idx_valid) && 
                       ~issue_lock) begin
            valid[geni] <= 0;
        end else if ( (geni == dispatch_slot_idx0 && dispatch_instr0_valid) ||
                      (geni == dispatch_slot_idx1 && dispatch_instr1_valid)) begin
            valid[geni] <= 1;
        end else begin
            valid[geni] <= valid[geni];
        end
    end
end

for (geni = 0 ; geni < MEM_IQ_NUM ; geni = geni + 1) begin
    always_ff @(posedge clk) begin
        if (rst) begin
            rs1_ready[geni] <= 0;
        end else if (geni == dispatch_slot_idx0 && dispatch_instr0_valid) begin
            rs1_ready[geni] <= bypass_network(rnds_io.mem_pack0.rs1_addr, 
                                              rnds_io.mem_instr0_rs1_ready);
        end else if (geni == dispatch_slot_idx1 && dispatch_instr1_valid) begin
            rs1_ready[geni] <= bypass_network(rnds_io.mem_pack1.rs1_addr, 
                                              rnds_io.mem_instr1_rs1_ready);
        end else if (wakeup_network(issue_slot[geni].rs1_addr) && 
                     ~pipe_ctrl_io.recovery_stall ) begin //TODO: memory broadcast port
            rs1_ready[geni] <= 1;
        end else begin
            rs1_ready[geni] <= rs1_ready[geni];
        end
    end
end
for (geni = 0 ; geni < MEM_IQ_NUM ; geni = geni + 1) begin
    always_ff @(posedge clk) begin
        if (rst) begin
            rs2_ready[geni] <= 0;
        end else if (geni == dispatch_slot_idx0 && dispatch_instr0_valid) begin
            rs2_ready[geni] <= bypass_network(rnds_io.mem_pack0.rs2_addr, 
                                              rnds_io.mem_instr0_rs2_ready);
        end else if (geni == dispatch_slot_idx1 && dispatch_instr1_valid) begin
            rs2_ready[geni] <= bypass_network(rnds_io.mem_pack1.rs2_addr, 
                                              rnds_io.mem_instr1_rs2_ready);
        end else if (wakeup_network(issue_slot[geni].rs2_addr) && 
                     ~pipe_ctrl_io.recovery_stall) begin //TODO: memory broadcast port
            rs2_ready[geni] <= 1;
        end else begin
            rs2_ready[geni] <= rs2_ready[geni];
        end
    end
end

for (geni = 0 ; geni < MEM_IQ_NUM ; geni = geni + 1)
    always_comb
        issue_rs1[geni] = (issue_slot[geni].rs1_addr == grant_io.issue_alu0_rd && grant_io.issue_alu0_rd_valid) ||
            (issue_slot[geni].rs1_addr == grant_io.issue_alu1_rd && grant_io.issue_alu1_rd_valid);

for (geni = 0 ; geni < MEM_IQ_NUM ; geni = geni + 1)
    always_comb
        issue_rs2[geni] = (issue_slot[geni].rs2_addr == grant_io.issue_alu0_rd && grant_io.issue_alu0_rd_valid) ||
            (issue_slot[geni].rs2_addr == grant_io.issue_alu1_rd && grant_io.issue_alu1_rd_valid);

for (geni = 0 ; geni < MEM_IQ_NUM ; geni = geni + 1)
    always_comb
        issue_ready[geni] = valid [geni] && rs1_ready[geni] && rs2_ready[geni]  && (depvec[geni] == 0);

for (geni = 0 ; geni < MEM_IQ_NUM ; geni = geni + 1)
    always_comb
        entry_is_store[geni] = issue_slot[geni].mem_is_store;

for (geni = 0 ; geni < MEM_IQ_NUM ; geni = geni + 1)
    always_ff @(posedge clk) begin
        if (rst) begin
            depvec[geni] <= 0;
        end else if (geni == dispatch_slot_idx0 && dispatch_instr0_valid) begin
            depvec[geni] <= instr0_depvec;
        end else if (geni == dispatch_slot_idx1 && dispatch_instr1_valid) begin
            depvec[geni] <= instr1_depvec;
        end else if (pipe_ctrl_io.recovery_stall) begin
            depvec[geni] <= depvec[geni] & (~flush_vec);
        end else if (issue_slot_idx_valid) begin
            depvec[geni] <= depvec[geni] & (~issue_flush_vec);
        end else begin
            depvec[geni] <= depvec[geni];
        end
    end

endgenerate

//=====================================================================
// Allocation unit
//=====================================================================

empty_entry_finder8_wrapper empty_entry_finder(
    .dispatch_slot_idx0(dispatch_slot_idx0),
    .dispatch_slot_idx1(dispatch_slot_idx1),
    .dispatch_slot_idx0_valid(dispatch_instr0_valid),
    .dispatch_slot_idx1_valid(dispatch_instr0_valid),
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
    rnds_io.mem_iq_instr0_check_top = allocatable_slot_idx0_valid;
    rnds_io.mem_iq_instr1_check_top = allocatable_slot_idx1_valid;
end

//===========================================
// Issue unit
//===========================================

mem_iq_balance_selector_wo_store selector(
    .clk(clk),
    .rst(rst),
    .issue_ready(issue_ready),
    .issue_lock(issue_lock), //don't issue any instruction when mem pipe busy
    .issue_slot_idx(issue_slot_idx),
    .issue_slot_idx_valid(issue_slot_idx_valid)
);

always_comb begin
    issue_flush_vec = ({{(MEM_IQ_NUM-1){1'b0}},1'b1} << issue_slot_idx);
end


//===========================================
// mem replay unit
//===========================================
mem_replay_unit replay_unit(
    .clk(clk),
    .rst(rst),

    .recovery_stall(pipe_ctrl_io.recovery_stall),
    .recovery_flush_BCAST(pipe_ctrl_io.recovery_flush_BCAST),
    .load_wake_up_failed_stall(pipe_ctrl_io.load_wake_up_failed_stall),
    .load_wake_up_predict_failed(pipe_ctrl_io.load_wake_up_predict_failed),
    .load_depend_replay(pipe_ctrl_io.load_depend_replay),
    .replay_muldiv_stall(pipe_ctrl_io.replay_muldiv_stall),

    .issue_instr_valid(issue_io.instr_valid),
    .issue_instr_pack(issue_io.instr_pack),

    .replay_issue_instr_valid(replay_issue_instr_valid),
    .replay_issue_instr_pack(replay_issue_instr_pack)
);


//===========================================
// pipeline register
//===========================================


//TODO: spec tag and stall
always_ff @( posedge clk ) begin
    if (rst) begin
        issue_io.instr_valid <= 0;
        issue_io.instr_pack <= 0;
    end else if (pipe_ctrl_io.recovery_stall) begin
        issue_io.instr_valid <= IsBrROBKill(pipe_ctrl_io.recovery_flush_BCAST, issue_io.instr_pack.rob_tag) ?
                                 0 : issue_io.instr_valid;
        issue_io.instr_pack <= issue_io.instr_pack;
    end else if (pipe_ctrl_io.SDA_full) begin
        issue_io.instr_valid <= issue_io.instr_valid;
        issue_io.instr_pack <= issue_io.instr_pack;
    end else if (pipe_ctrl_io.load_wake_up_predict_failed || 
                 pipe_ctrl_io.load_wake_up_failed_stall || 
                 pipe_ctrl_io.replay_muldiv_stall ) begin
        issue_io.instr_valid <= 0;
        issue_io.instr_pack <= 0;
    end else if (pipe_ctrl_io.load_depend_replay) begin
        issue_io.instr_valid <= replay_issue_instr_valid;
        issue_io.instr_pack <= replay_issue_instr_pack;
    end else begin
        issue_io.instr_valid <= issue_slot_idx_valid;
        issue_io.instr_pack <= issue_slot[issue_slot_idx];
    end
end

always_ff @( posedge clk ) begin
    if (rst) begin
        issue_io.BCAST_ld_spec.valid <= 0;
        issue_io.BCAST_ld_spec.prf_addr <= 0;
        ld_spec_rob_tag <= 0;
    end else if (pipe_ctrl_io.recovery_stall) begin
        issue_io.BCAST_ld_spec.valid <= IsBrROBKill(pipe_ctrl_io.recovery_flush_BCAST, ld_spec_rob_tag) ?
                                 0 : issue_io.BCAST_ld_spec.valid;
        issue_io.BCAST_ld_spec.prf_addr <= issue_io.BCAST_ld_spec.prf_addr;
        ld_spec_rob_tag <= ld_spec_rob_tag;
    end else if (pipe_ctrl_io.load_wake_up_predict_failed) begin
        issue_io.BCAST_ld_spec.valid <= 0;
        issue_io.BCAST_ld_spec.prf_addr <= 0;
        ld_spec_rob_tag <= 0;
    end else begin
        issue_io.BCAST_ld_spec.valid <= issue_io.instr_valid && ~issue_io.instr_pack.mem_is_store;
        issue_io.BCAST_ld_spec.prf_addr <= issue_io.instr_pack.rd_addr;
        ld_spec_rob_tag <= issue_io.instr_pack.rob_tag;
    end
end

assign rnds_io.issue_store = issue_lock || ~issue_slot_idx_valid ? 0 : issue_slot[issue_slot_idx].mem_is_store;
assign rnds_io.issue_store_id = issue_lock || ~issue_slot_idx_valid ? 0 : issue_slot[issue_slot_idx].store_id;
assign rnds_io.issue_store_set_id = issue_lock || ~issue_slot_idx_valid ? 0 : issue_slot[issue_slot_idx].store_set_id;

function logic bypass_network(
    input prf_specifier_t prf_addr,
    input logic ready 
);
    begin
        if ((prf_addr == broadcast_io.BCAST_muldiv.prf_addr && broadcast_io.BCAST_muldiv.valid) || 
            (prf_addr == issue_io.BCAST_ld_spec.prf_addr && issue_io.BCAST_ld_spec.valid) ||
            (grant_io.instr0_valid && prf_addr == grant_io.issue_alu0_rd &&
             grant_io.issue_alu0_rd_valid) ||
            (grant_io.instr1_valid && prf_addr == grant_io.issue_alu1_rd &&
            grant_io.issue_alu1_rd_valid) ) begin //TODO: memory broadcast port
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
            (prf_addr == issue_io.BCAST_ld_spec.prf_addr && issue_io.BCAST_ld_spec.valid) ||
            (grant_io.instr0_valid && prf_addr == grant_io.issue_alu0_rd &&
            grant_io.issue_alu0_rd_valid) ||
            (grant_io.instr1_valid && prf_addr == grant_io.issue_alu1_rd &&
           grant_io.issue_alu1_rd_valid) ) begin
            wakeup_network = 1;    
        end else begin
            wakeup_network = 0;
        end
    end
endfunction

`ifdef FALCO_SIM_DEBUG
    function [31:0] ver_MEM_IQ_pc;
        /*verilator public*/
        input [31:0] index;
        ver_MEM_IQ_pc = issue_slot[index[2:0]].pc;
    endfunction

    function [31:0] ver_MEM_IQ_rd_addr;
        /*verilator public*/
        input [31:0] index;
        ver_MEM_IQ_rd_addr = issue_slot[index[2:0]].rd_addr;
    endfunction
    
    function [31:0] ver_MEM_IQ_rs1_addr;
        /*verilator public*/
        input [31:0] index;
        ver_MEM_IQ_rs1_addr = issue_slot[index[2:0]].rs1_addr;
    endfunction
    
    function [31:0] ver_MEM_IQ_rs2_addr;
        /*verilator public*/
        input [31:0] index;
        ver_MEM_IQ_rs2_addr = issue_slot[index[2:0]].rs2_addr;
    endfunction

    function [31:0] ver_MEM_IQ_rob_tag;
        /*verilator public*/
        input [31:0] index;
        ver_MEM_IQ_rob_tag = issue_slot[index[2:0]].rob_tag;
    endfunction

    function [31:0] ver_MEM_IQ_store_set_id;
        /*verilator public*/
        input [31:0] index;
        ver_MEM_IQ_store_set_id = issue_slot[index[2:0]].store_set_id;
    endfunction

    function [31:0] ver_MEM_IQ_store_id;
        /*verilator public*/
        input [31:0] index;
        ver_MEM_IQ_store_id = issue_slot[index[2:0]].store_id;
    endfunction

`endif

endmodule
