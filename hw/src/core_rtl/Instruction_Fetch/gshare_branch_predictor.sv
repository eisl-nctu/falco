`timescale 1ns/1ps
// =============================================================================
//  Program : gshare_branch_predictor.sv
//  Author  : Hon-Chou Dai
//  Date    : May/15/2021
// -----------------------------------------------------------------------------
//  Description:
//  Gshare branch predictor for Falco.
// -----------------------------------------------------------------------------
//  Revision information:
//
//    August/07/2021, by Chun-Wei Chao:
//      Add branch_miss_first & store_set_violation to input.
//      If both trigger simultaneously, 
//      we should update BHSR to appropriate instruction BHSR.
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

module gshare_branch_predictor(
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
    input   BHSR_t  exe_cond_bhsr, //recovey BHSR if branch misprediction
    input   logic   exe_cond_branch_taken,
    input   pc_t    exe_cond_branch_addr,

    input   logic   store_set_violation,
    input   logic   branch_miss_first,
    input   BHSR_t  lsu_bhsr,

    //Prediction result to IF stage
    output  logic instr0_branch_predict_taken,
    output  logic instr1_branch_predict_taken,
    output  BHSR_t current_instr0_BHSR,
    output  BHSR_t current_instr1_BHSR
);

/*
Software should be optimized such that the sequential code path is the most common path, with
less-frequently taken code paths placed out of line. Software should also assume that backward
branches will be predicted taken and forward branches as not taken, at least the first time they are
encountered. Dynamic predictors should quickly learn any predictable branch behavior.
*/

// =======================================================
// Gshare main structure
// =======================================================
// Gshare Pattern history Table
logic [1:0] gshare_pht_table [GSHARE_PHT_SIZE-1:0];
// Gshare BHSR
BHSR_t gshare_BHSR;

// BHSR xor result
logic [GSHARE_PHT_WIDTH-1:0] gshare_instr0_hash_addr,gshare_instr1_hash_addr, gshare_i0_taken_i1_hash_addr;

// EXE branch result compute signal
BHSR_t gshare_recovery_BHSR;
BHSR_t gshare_update_BHSR;
BHSR_t gshare_update_hash_addr;
logic gshare_pht_update;

// Gshare predition if instr0 hit but not taken scenario
//BHSR_t gshare_instr1_BHSR; //assume instr0 btb hit but not taken
logic gshare_instr1_BHSR;

`ifdef FALCO_SIM_DEBUG
BHSR_t IF_debug_inst0_bhsr /*verilator public*/;
BHSR_t IF_debug_inst1_bhsr /*verilator public*/;
pc_t IF_debug_inst0_pc /*verilator public*/;
pc_t IF_debug_inst1_pc /*verilator public*/;

always_comb IF_debug_inst0_bhsr = current_instr0_BHSR;
always_comb IF_debug_inst1_bhsr = current_instr1_BHSR;
always_comb IF_debug_inst0_pc = IF_instr0_pc;
always_comb IF_debug_inst1_pc = IF_instr1_pc;

`endif

always_comb gshare_pht_update = exe_cond_valid;

//BHSR shift left
always_comb gshare_recovery_BHSR = {exe_cond_bhsr[GSHARE_BHSR_WIDTH-2:0] , exe_cond_branch_taken};
always_comb gshare_update_hash_addr = gshare_hash(exe_cond_bhsr,exe_cond_branch_addr);
always_comb gshare_instr1_BHSR = {gshare_BHSR[GSHARE_BHSR_WIDTH-2:0] , 1'b0};

always_comb gshare_instr0_hash_addr = gshare_hash(gshare_BHSR,IF_instr0_pc);
always_comb gshare_instr1_hash_addr = gshare_hash(gshare_BHSR,IF_instr1_pc);
always_comb gshare_i0_taken_i1_hash_addr = gshare_hash(gshare_instr1_BHSR, IF_instr1_pc);
always_comb
    if (instr0_branch_predict_taken && IF_instr0_btb_hit)
        gshare_update_BHSR = {gshare_BHSR[GSHARE_BHSR_WIDTH-2:0] ,instr0_branch_predict_taken};
    else if (instr1_branch_predict_taken && IF_instr1_btb_hit && IF_instr0_btb_hit)
        gshare_update_BHSR = {gshare_BHSR[GSHARE_BHSR_WIDTH-3:0] , 1'b0 ,instr1_branch_predict_taken};
    else if (instr1_branch_predict_taken && IF_instr1_btb_hit)
        gshare_update_BHSR = {gshare_BHSR[GSHARE_BHSR_WIDTH-2:0], instr1_branch_predict_taken};
    else if (IF_instr0_btb_hit)
        gshare_update_BHSR = {gshare_BHSR[GSHARE_BHSR_WIDTH-2:0] , 1'b0};
    else
        gshare_update_BHSR = gshare_BHSR;

always_ff @(posedge clk) begin
    if (rst)
        gshare_BHSR <= 0; //?
     else if (store_set_violation && !branch_miss_first)
         gshare_BHSR <= lsu_bhsr;
    else if (exe_cond_miss_prediction) //update BHSR
        gshare_BHSR <= gshare_recovery_BHSR;
    else
        gshare_BHSR <= cache_hit ? gshare_update_BHSR : gshare_BHSR;
end

always_comb begin
    current_instr0_BHSR = gshare_BHSR;
end
always_comb begin
    if (IF_instr0_btb_hit && instr0_branch_predict_taken == 0)
        current_instr1_BHSR = {gshare_BHSR[GSHARE_BHSR_WIDTH-2:0],1'b0};
    else
        current_instr1_BHSR = gshare_BHSR;
end

genvar i;
generate
for (i = 0 ; i < GSHARE_PHT_SIZE ; i = i + 1) begin
    always_ff @(posedge clk) begin
        if (rst) begin
            gshare_pht_table[i] <= 2'b10;
        end else if (gshare_pht_update && gshare_update_hash_addr == i) begin
            case  (gshare_pht_table[i])
                2'b00: gshare_pht_table[i] = (exe_cond_branch_taken ? 2'b01 : 2'b00); //strongly not taken
                2'b01: gshare_pht_table[i] = (exe_cond_branch_taken ? 2'b10 : 2'b00); //weakly not taken
                2'b10: gshare_pht_table[i] = (exe_cond_branch_taken ? 2'b11 : 2'b01); //weakly taken
                2'b11: gshare_pht_table[i] = (exe_cond_branch_taken ? 2'b11 : 2'b10); //strongly taken
            endcase
        end else begin
            gshare_pht_table[i] <= gshare_pht_table[i];
        end
    end
end
endgenerate

always_comb instr0_branch_predict_taken = gshare_pht_table[gshare_instr0_hash_addr][1];
always_comb
    if (instr0_branch_predict_taken == 0 && IF_instr0_btb_hit)
        instr1_branch_predict_taken = gshare_pht_table[gshare_instr1_hash_addr][1];
    else
        instr1_branch_predict_taken = gshare_pht_table[gshare_i0_taken_i1_hash_addr][1];

function [GSHARE_PHT_WIDTH-1:0] gshare_hash(
    input logic [GSHARE_BHSR_WIDTH-1:0] bhsr,
    input pc_t branch_pc
);
    gshare_hash = bhsr ^ branch_pc[GSHARE_PHT_WIDTH+1:2]; //drop low 2 bit
endfunction



endmodule
