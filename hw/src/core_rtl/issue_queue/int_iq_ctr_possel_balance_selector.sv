`timescale 1ns/1ps
// =============================================================================
//  Program : int_iq_ctr_possel_balance_selector.sv
//  Author  : Hon-Chou Dai
//  Date    : May/15/2021
// -----------------------------------------------------------------------------
//  Description:
//  Not use in Falco in this version.
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

//============================================
// Algorithm: add aged counter to every entry
//            when 1 issue apply, add all valid entry's counter
//            it would cause aged counter saturated sometimes.
//            balance selector
//============================================

module int_iq_ctr_possel_balance_selector(
    input                               clk,
    input                               rst,
    // int issue queue slot status
    input   logic                       entry_valid[Falco_pkg::INT_IQ_NUM],
    input   logic                       entry_ready[Falco_pkg::INT_IQ_NUM],
    input   logic                       entry_is_ctrl_instr[Falco_pkg::INT_IQ_NUM],
    input   logic                       entry_is_csr_instr[Falco_pkg::INT_IQ_NUM],
    input   logic                       entry_is_muldiv[Falco_pkg::INT_IQ_NUM],
    input   logic                       ld_poison_wake_up[Falco_pkg::INT_IQ_NUM],
    input   logic                       iq_poison_wake_up[Falco_pkg::INT_IQ_NUM],
    // issue queue dispatch to update age counter
    input   logic [INT_IQ_WIDTH-1:0]    dispatch_slot_idx0,
    input   logic [INT_IQ_WIDTH-1:0]    dispatch_slot_idx1,
    input   logic                       dispatch_instr0_valid,
    input   logic                       dispatch_instr1_valid,
    // issue type select signal
    input   logic                       muldiv_busy,
    input   logic                       icache_miss,
    input   logic                       recovery_flush,
    input   logic                       non_posion_issue,
    input   logic                       replay_issue_first,
    input   logic                       issue_replay,
    input   logic                       branch_miss_flush,
    input   logic                       load_wake_up_kill,
    input   logic                       replay_issue_muldiv,

    output  logic                       muldiv_issuable,
    output  logic [INT_IQ_WIDTH-1:0]    issue_slot_idx0,
    output  logic [INT_IQ_WIDTH-1:0]    issue_slot_idx1,
    output  logic                       issue_slot_idx0_is_poison, 
    output  logic                       issue_slot_idx1_is_poison, 
    output  logic                       issue_slot_idx0_valid,
    output  logic                       issue_slot_idx1_valid
);

logic issue_lock;

always_comb 
    issue_lock = branch_miss_flush ||
                 load_wake_up_kill ||
                 issue_replay || 
                 (non_posion_issue && replay_issue_first);

logic [INT_IQ_WIDTH-1:0] issue_alu_op_slot_idx0; //alu only
logic [INT_IQ_WIDTH-1:0] issue_alu_op_slot_idx1;
logic [INT_IQ_WIDTH-1:0] issue_control_op_slot; //branch,csr
logic [INT_IQ_WIDTH-1:0] issue_muldiv_op_slot; // mul

logic issue_alu_op_slot_idx0_valid;
logic issue_alu_op_slot_idx1_valid;
logic issue_control_op_slot_valid;
logic issue_muldiv_op_slot_valid;

logic [1:0] issue_muldiv_counter;
logic [INT_IQ_WIDTH:0] age_counter [INT_IQ_NUM] /*verilator public*/;

logic entry_is_poison [INT_IQ_NUM];
logic entry_wake_up_next_is_poison[INT_IQ_NUM];
logic [1:0] poison_count_down_counter [INT_IQ_NUM];
logic [1:0] poison_pos [INT_IQ_NUM]; 
//We don't care who wake up this entry,we just record time
//When spec kill time, only can issue non posion instruction to EXE
//When spec kill time, it cannot be waked up by load instruction due 
//to mem issue queue lock. 

//modified ver. position based replay
//use position based selective replay algorithm to identify load spec
//dependant instruction
//
//algorithm reference: Understanding scheduling replay schemes (HPCA'04)
//
//only indentify which instruction is poison and don't issue when load
//spec kill stall time

//=====================================================================
// issue unit
//=====================================================================

always_comb 
    muldiv_issuable = issue_muldiv_counter == 0 && ~muldiv_busy;

genvar geni;


//TODO: age counter increment determine which types of current issue instruction
//ex : if current entry is muldiv instr, than only when muldiv instruction issue then it can incement
generate
for (geni = 0 ; geni < INT_IQ_NUM ; geni = geni + 1) begin
    always_ff @(posedge clk) begin
        if (rst) begin
            age_counter[geni] <= 0;
        end else if ( (geni == dispatch_slot_idx0 && dispatch_instr0_valid) ||
                      (geni == dispatch_slot_idx1 && dispatch_instr1_valid) )begin
            age_counter[geni] <= 0;
        end else if ( issue_slot_idx1_valid &&
                      issue_slot_idx1 == issue_muldiv_op_slot && 
                      entry_is_muldiv[geni]
            ) begin
            age_counter[geni] <= age_counter[geni] == INT_IQ_NUM ? INT_IQ_NUM : age_counter[geni] + 1; //saturate
        end else if ( issue_slot_idx0_valid || 
                      (issue_slot_idx1_valid && 
                       issue_slot_idx1 != issue_muldiv_op_slot) 
            )begin // TODO: change to when issue instr then increment
            age_counter[geni] <= age_counter[geni] == INT_IQ_NUM ? INT_IQ_NUM : age_counter[geni] + 1; //saturate
        end else begin
            age_counter[geni] <= age_counter[geni];
        end

    end
end
endgenerate

generate
for (geni = 0 ; geni < INT_IQ_NUM ; geni = geni + 1) begin
    always_ff @(posedge clk) begin
        if (rst) begin
            poison_count_down_counter[geni] <= 2;
        end else if ((geni == dispatch_slot_idx0 && dispatch_instr0_valid) ||
                     (geni == dispatch_slot_idx1 && dispatch_instr1_valid)) begin
            poison_count_down_counter[geni] <= 2;
        end else if ( ~recovery_flush && ~non_posion_issue && ~load_wake_up_kill
                        /*(~issue_lock && ~non_posion_issue) ||
                      (issue_lock && issue_replay && ~branch_miss_flush)*/
                     ) begin
            poison_count_down_counter[geni] <= poison_count_down_counter[geni] == 0 ? 
                                               poison_count_down_counter[geni] : 
                                               poison_count_down_counter[geni] - 1;
        end else begin
            poison_count_down_counter[geni] <= poison_count_down_counter[geni];
        end
    end
end
endgenerate

generate
for (geni = 0 ; geni < INT_IQ_NUM ; geni = geni + 1) begin
    always_ff @( posedge clk ) begin
        if (rst) begin
            poison_pos[geni][0] <= 0;
        end else if ((geni == dispatch_slot_idx0 && dispatch_instr0_valid) ||
                     (geni == dispatch_slot_idx1 && dispatch_instr1_valid)) begin
            poison_pos[geni][0] <= 0;
        end else if (~recovery_flush && ~non_posion_issue && ~load_wake_up_kill) begin
            poison_pos[geni][0] <= ld_poison_wake_up[geni];
        end else begin
            poison_pos[geni][0] <= poison_pos[geni][0];    
        end
    end
end
for (geni = 0 ; geni < INT_IQ_NUM ; geni = geni + 1) begin
    always_ff @( posedge clk ) begin
        if (rst) begin
            poison_pos[geni][1] <= 0;
        end else if ((geni == dispatch_slot_idx0 && dispatch_instr0_valid) ||
                     (geni == dispatch_slot_idx1 && dispatch_instr1_valid)) begin
            poison_pos[geni][1] <= 0;
        end else if (~recovery_flush && ~non_posion_issue && ~load_wake_up_kill) begin
            poison_pos[geni][1] <= poison_pos[geni][0] | iq_poison_wake_up[geni];
        end else begin
            poison_pos[geni][1] <= poison_pos[geni][1];    
        end
    end
end
endgenerate

//integer i;

//super unefficency oldest finder algorithm for logic proof only
logic entry_alu_valid[INT_IQ_NUM];
logic entry_muldiv_valid[INT_IQ_NUM];
logic entry_ctrl_valid[INT_IQ_NUM];

always_comb
    for (int i = 0 ; i < INT_IQ_NUM ; i = i + 1)
        entry_alu_valid[i] = entry_valid[i] && entry_ready[i] &&
            ~(entry_is_ctrl_instr[i] || entry_is_csr_instr[i] || entry_is_muldiv[i]) &&
            ~(non_posion_issue && entry_is_poison[i]);

always_comb
    for (int i = 0 ; i < INT_IQ_NUM ; i = i + 1)
        entry_muldiv_valid[i] = entry_valid[i] && entry_ready[i] && entry_is_muldiv[i] &&
                                ~(non_posion_issue && entry_is_poison[i]);

always_comb
    for (int i = 0 ; i < INT_IQ_NUM ; i = i + 1)
        entry_ctrl_valid[i] = entry_valid[i] && entry_ready[i] &&
            (entry_is_ctrl_instr[i] || entry_is_csr_instr[i]) &&
            ~(non_posion_issue && entry_is_poison[i]);

always_comb
    for (int i = 0 ; i < INT_IQ_NUM ; i = i + 1)
        entry_is_poison[i] = poison_count_down_counter[i] != 0 || poison_pos[i] !=0;
always_comb
    for (int i = 0 ; i < INT_IQ_NUM ; i = i + 1)
        entry_wake_up_next_is_poison[i] = poison_count_down_counter[i] != 0 || poison_pos[i][0] !=0;

Picker8_2 alu_picker(
    .in_age(age_counter),
    .in_valid(entry_alu_valid),
    .out_id_0(issue_alu_op_slot_idx0),
    .out_id_1(issue_alu_op_slot_idx1),
    .out_valid_0(issue_alu_op_slot_idx0_valid),
    .out_valid_1(issue_alu_op_slot_idx1_valid)
);

Picker8_1 control_picker(
    .in_age(age_counter),
    .in_valid(entry_ctrl_valid),
    .out_id(issue_control_op_slot),
    .out_valid(issue_control_op_slot_valid)
);

Picker8_1 muldiv_picker(
    .in_age(age_counter),
    .in_valid(entry_muldiv_valid),
    .out_id(issue_muldiv_op_slot),
    .out_valid(issue_muldiv_op_slot_valid)
);

always_comb begin
    if (rst) begin
        issue_slot_idx0 = 0;
        issue_slot_idx1 = 0;
        issue_slot_idx0_valid = 0;
        issue_slot_idx1_valid = 0;
        issue_slot_idx0_is_poison = 0;
        issue_slot_idx1_is_poison = 0;
    end else begin
        issue_slot_idx0 = (issue_control_op_slot_valid ? issue_control_op_slot : issue_alu_op_slot_idx0);
        issue_slot_idx1 = (issue_muldiv_op_slot_valid && muldiv_issuable ?
                            issue_muldiv_op_slot
                          : (issue_control_op_slot_valid ? issue_alu_op_slot_idx0 : issue_alu_op_slot_idx1));
        issue_slot_idx0_valid = ~issue_lock && (issue_control_op_slot_valid ? 1 : issue_alu_op_slot_idx0_valid);
        issue_slot_idx1_valid = ~issue_lock && (issue_muldiv_op_slot_valid && muldiv_issuable?
                                1 :
                                (issue_control_op_slot_valid ? issue_alu_op_slot_idx0_valid : issue_alu_op_slot_idx1_valid));
        issue_slot_idx0_is_poison = entry_wake_up_next_is_poison[issue_slot_idx0] && ~non_posion_issue;
        issue_slot_idx1_is_poison = entry_wake_up_next_is_poison[issue_slot_idx1] && ~non_posion_issue;
    end
end



always_ff @(posedge clk) begin
    if (rst)
        issue_muldiv_counter <= 0;
    else if (issue_muldiv_counter == 3 && ~muldiv_busy)
        issue_muldiv_counter <= 0;
    else if (load_wake_up_kill)
        issue_muldiv_counter <= issue_muldiv_counter ? 1 : 0;
    else if (recovery_flush || (non_posion_issue && replay_issue_first))
        issue_muldiv_counter <= issue_muldiv_counter;
    else if (issue_muldiv_counter)
        issue_muldiv_counter <= (issue_muldiv_counter == 3 ? 3 : issue_muldiv_counter + 1);
    else if ((issue_muldiv_op_slot_valid && ~muldiv_busy && ~issue_lock) ||
             (replay_issue_muldiv && issue_replay))
        issue_muldiv_counter <= 1;
    else
        issue_muldiv_counter <= issue_muldiv_counter;
end

endmodule

