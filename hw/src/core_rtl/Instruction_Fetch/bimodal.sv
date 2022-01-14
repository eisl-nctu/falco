`timescale 1ns/1ps
// =============================================================================
//  Program : bimodal.sv
//  Author  : Hon-Chou Dai
//  Date    : May/15/2021
// -----------------------------------------------------------------------------
//  Description:
//  Bimodal branch predictor for Falco.
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

// 2 predict 1 update

module bimodal_branch_predictor(
    input           clk,
    input           rst,

    // PC -> IF stage siganl
    input   pc_t    IF_instr0_pc,
    input   pc_t    IF_instr1_pc,
    input   logic   IF_instr0_btb_hit,
    input   logic   IF_instr1_btb_hit,
    input   logic   cache_hit, //check whether current IF_instr0_pc is valid

    //EXE branch result update siganl
    input   logic   exe_cond_valid,
    input   logic   exe_cond_miss_prediction,
    input   logic   exe_cond_branch_taken,
    input   pc_t    exe_cond_branch_addr,

    //Prediction result to IF stage
    output  logic instr0_branch_predict_taken,
    output  logic instr1_branch_predict_taken
);

/*
Software should be optimized such that the sequential code path is the most common path, with
less-frequently taken code paths placed out of line. Software should also assume that backward
branches will be predicted taken and forward branches as not taken, at least the first time they are
encountered. Dynamic predictors should quickly learn any predictable branch behavior.
*/

// =======================================================
// main structure
// =======================================================
// Pattern history Table
logic [1:0] pht_table [GSHARE_PHT_SIZE-1:0];

// EXE branch result compute signal
logic [GSHARE_PHT_WIDTH-1:0] update_hash_addr;
logic [GSHARE_PHT_WIDTH-1:0] instr0_hash_addr;
logic [GSHARE_PHT_WIDTH-1:0] instr1_hash_addr;
logic pht_update;


always_comb pht_update = exe_cond_valid;

//BHSR shift left
always_comb update_hash_addr = bimodal_hash(exe_cond_branch_addr);

always_comb instr0_hash_addr = bimodal_hash(IF_instr0_pc);
always_comb instr1_hash_addr = bimodal_hash(IF_instr1_pc);


genvar i;
generate
for (i = 0 ; i < GSHARE_PHT_SIZE ; i = i + 1) begin
    always_ff @(posedge clk) begin
        if (rst) begin
            pht_table[i] <= 0;//2'b10;
        end else if (pht_update && update_hash_addr == i) begin
            case  (pht_table[i])
                2'b00: pht_table[i] = (exe_cond_branch_taken ? 2'b01 : 2'b00); //strongly not taken
                2'b01: pht_table[i] = (exe_cond_branch_taken ? 2'b10 : 2'b00); //weakly not taken
                2'b10: pht_table[i] = (exe_cond_branch_taken ? 2'b11 : 2'b01); //weakly taken
                2'b11: pht_table[i] = (exe_cond_branch_taken ? 2'b11 : 2'b10); //strongly taken
            endcase
        end else begin
            pht_table[i] <= pht_table[i];
        end
    end
end
endgenerate

always_comb instr0_branch_predict_taken = pht_table[instr0_hash_addr][1];
always_comb instr1_branch_predict_taken = pht_table[instr1_hash_addr][1];

function [GSHARE_PHT_WIDTH-1:0] bimodal_hash(
    input pc_t branch_pc
);
    bimodal_hash = branch_pc[GSHARE_PHT_WIDTH+1:2]; //bimodal
endfunction



endmodule
