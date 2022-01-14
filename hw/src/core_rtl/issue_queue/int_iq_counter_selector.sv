`timescale 1ns/1ps
// =============================================================================
//  Program : int_iq_counter_selector.sv
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
//============================================

module int_iq_counter_selector(
    input   clk,
    input   rst,

    // int issue queue slot status
    input   logic                       entry_valid[Falco_pkg::INT_IQ_NUM],
    input   logic                       entry_ready[Falco_pkg::INT_IQ_NUM],
    input   logic                       entry_is_ctrl_instr[Falco_pkg::INT_IQ_NUM],
    input   logic                       entry_is_csr_instr[Falco_pkg::INT_IQ_NUM],
    input   logic                       entry_is_muldiv[Falco_pkg::INT_IQ_NUM],

    //dispatch to clear age counter
    input   logic [INT_IQ_WIDTH-1:0]    dispatch_slot_idx0,
    input   logic [INT_IQ_WIDTH-1:0]    dispatch_slot_idx1,
    input   logic                       dispatch_instr0_valid,
    input   logic                       dispatch_instr1_valid,

    // issue type select
    input   logic                       issue_lock,
    input   logic                       muldiv_busy,
    input   logic                       icache_miss,
    input   logic                       recovery_flush,

    // select result
    output  logic [INT_IQ_WIDTH-1:0]    issue_slot_idx0,
    output  logic [INT_IQ_WIDTH-1:0]    issue_slot_idx1,
    output  logic                       issue_slot_idx0_valid,
    output  logic                       issue_slot_idx1_valid
);


logic [INT_IQ_WIDTH-1:0] issue_alu_op_slot_idx0; //alu only
logic [INT_IQ_WIDTH-1:0] issue_alu_op_slot_idx1;
logic [INT_IQ_WIDTH-1:0] issue_control_op_slot; //branch,csr
logic [INT_IQ_WIDTH-1:0] issue_muldiv_op_slot; // mul

logic issue_alu_op_slot_idx0_valid;
logic issue_alu_op_slot_idx1_valid;
logic issue_control_op_slot_valid;
logic issue_muldiv_op_slot_valid;

logic [1:0] issue_muldiv_counter;
logic muldiv_issuable;
logic [INT_IQ_WIDTH:0] age_counter [INT_IQ_NUM] /*verilator public*/;
//=====================================================================
// issue unit
//=====================================================================

assign muldiv_issuable = issue_muldiv_counter == 0 && ~muldiv_busy;

genvar geni;

generate
for (geni = 0 ; geni < INT_IQ_NUM ; geni = geni + 1) begin
    always_ff @(posedge clk) begin
        if (rst) begin
            age_counter[geni] <= 0;
        end else if ( (geni == dispatch_slot_idx0 && dispatch_instr0_valid) ||
                      (geni == dispatch_slot_idx1 && dispatch_instr1_valid) )begin
            age_counter[geni] <= 0;
        end else if (issue_slot_idx0_valid || issue_slot_idx1_valid)begin // TODO: change to when issue instr then increment
            age_counter[geni] <= age_counter[geni] == INT_IQ_NUM ? INT_IQ_NUM : age_counter[geni] + 1; //saturate
        end else begin
            age_counter[geni] <= age_counter[geni];
        end

    end
end
endgenerate

integer i;

//super unefficency oldest finder algorithm for logic proof only
logic [INT_IQ_WIDTH:0] issue_alu_cur_age_cnt;
always_comb begin
    issue_alu_cur_age_cnt = 0;
    issue_alu_op_slot_idx0_valid = 0;
    issue_alu_op_slot_idx1_valid = 0;
    for (i = 0 ; i < INT_IQ_NUM ; i = i + 1) begin
        if (issue_alu_cur_age_cnt <= age_counter[i] && entry_valid[i] && entry_ready[i] && 
        ~(
            entry_is_ctrl_instr[i]||
            entry_is_csr_instr[i] || 
            entry_is_muldiv[i]
        )) begin //TODO: < or <= ?
            issue_alu_op_slot_idx1_valid = issue_alu_op_slot_idx0_valid;
            issue_alu_op_slot_idx0_valid = 1;
            issue_alu_op_slot_idx1 = issue_alu_op_slot_idx0;
            issue_alu_op_slot_idx0 = i;
            issue_alu_cur_age_cnt = age_counter[i];
        end
    end
end

logic [INT_IQ_WIDTH:0] issue_control_cur_age_cnt;
always_comb begin
    issue_control_cur_age_cnt = 0;
    issue_control_op_slot_valid = 0;
    for (i = 0 ; i < INT_IQ_NUM ; i = i + 1) begin
        if (issue_control_cur_age_cnt <= age_counter[i] && entry_valid[i] && entry_ready[i] && 
            ( entry_is_ctrl_instr[i] || entry_is_csr_instr[i])
           ) begin //TODO: < or <= ?
            issue_control_op_slot_valid = 1;
            issue_control_op_slot = i;
            issue_control_cur_age_cnt = age_counter[i];
        end
    end
end

logic [INT_IQ_WIDTH-1:0] issue_muldiv_cur_age_cnt;
always_comb begin
    issue_muldiv_cur_age_cnt = 0;
    issue_muldiv_op_slot_valid = 0;
    for (i = 0 ; i < INT_IQ_NUM ; i = i + 1) begin
        if (issue_muldiv_cur_age_cnt <= age_counter[i] && entry_valid[i] && entry_ready[i] &&
        (entry_is_muldiv[i])
        ) begin //TODO: < or <= ?
            issue_muldiv_op_slot_valid = 1;
            issue_muldiv_op_slot = i;
            issue_muldiv_cur_age_cnt = age_counter[i];
        end
    end
end

always_comb begin
    if (rst) begin
        issue_slot_idx0 = 0; 
        issue_slot_idx1 = 0; 
        issue_slot_idx0_valid = 0; 
        issue_slot_idx1_valid = 0; 
    end else begin
        issue_slot_idx0 = (issue_control_op_slot_valid ? issue_control_op_slot : issue_alu_op_slot_idx0);
        issue_slot_idx1 = (issue_muldiv_op_slot_valid && muldiv_issuable ? 
                            issue_muldiv_op_slot 
                          : (issue_control_op_slot_valid ? issue_alu_op_slot_idx0 : issue_alu_op_slot_idx1));
        issue_slot_idx0_valid = ~issue_lock && (issue_control_op_slot_valid ? 1 : issue_alu_op_slot_idx0_valid);
        issue_slot_idx1_valid = ~issue_lock && (issue_muldiv_op_slot_valid && muldiv_issuable? 
                                1 : 
                                (issue_control_op_slot_valid ? issue_alu_op_slot_idx0_valid : issue_alu_op_slot_idx1_valid));
    end
end



always_ff @(posedge clk) begin
    if (rst)
        issue_muldiv_counter <= 0;
    else if (issue_muldiv_counter == 3 && ~muldiv_busy)
        issue_muldiv_counter <= 0;
    else if (recovery_flush)
        issue_muldiv_counter <= issue_muldiv_counter;
    else if (issue_muldiv_counter)
        issue_muldiv_counter <= (issue_muldiv_counter == 3 ? 3 : issue_muldiv_counter + 1);
    else if (issue_muldiv_op_slot_valid && ~muldiv_busy && ~issue_lock)
        issue_muldiv_counter <= 1;
    else
        issue_muldiv_counter <= issue_muldiv_counter;
end

endmodule
