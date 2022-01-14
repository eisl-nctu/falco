`timescale 1ns/1ps
// =============================================================================
//  Program : last_fetch_store_table.sv
//  Author  : Chun-Wei Chao
//  Date    : August/07/2021
// -----------------------------------------------------------------------------
//  Description:
//  Last fetch store table for Falco.
//  It can check whether the load instrction can OOO execute or not.
//  There is a input from mem_issue_stage, because store instruction which has issued 
//  should not influence load instruction OOO issue.
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

module last_fetch_store_table(
    input clk,
    input rst,

    input [SSIT_WIDTH-1 : 0]            instr0_pc/*verilator public*/, 
    input [SSIT_WIDTH-1 : 0]            instr1_pc/*verilator public*/,
    input                               instr0_is_store/*verilator public*/,
    input                               instr1_is_store/*verilator public*/,
    input [STORE_ID_WIDTH-1 : 0]        instr0_store_id/*verilator public*/,
    input [STORE_ID_WIDTH-1 : 0]        instr1_store_id/*verilator public*/,
    input [LFST_WIDTH-1 : 0]            instr0_store_set_id/*verilator public*/,
    input [LFST_WIDTH-1 : 0]            instr1_store_set_id/*verilator public*/,
    output                              instr0_predict_result/*verilator public*/,
    output                              instr1_predict_result/*verilator public*/,

    // from mem_issue
    input                               issue_store,
    input [STORE_ID_WIDTH-1 : 0]        issue_store_id,
    input [LFST_WIDTH-1 : 0]            issue_store_set_id
);

// assign instr0_predict_result = 0;
// assign instr1_predict_result = 0;

logic [STORE_ID_WIDTH-1 : 0] last_fetch_store_table[0:LFST_SIZE];
logic check;
logic [STORE_ID_WIDTH-1 : 0] issue_store_id_ori;

assign check = (instr0_store_set_id == instr1_store_set_id) && instr0_is_store && (instr1_store_set_id != 0);
assign instr0_predict_result = (instr0_store_set_id == 0) || ((last_fetch_store_table[instr0_store_set_id] == 0) && ~(instr0_store_set_id == 1));
assign instr1_predict_result = ~check && 
                ((instr1_store_set_id == 0) || ((last_fetch_store_table[instr1_store_set_id] == 0) && ~(instr1_store_set_id == 1)));
assign issue_store_id_ori = last_fetch_store_table[issue_store_set_id];

genvar geni;
generate
    for(geni = 0; geni < LFST_SIZE; geni = geni + 1) begin
        always_ff @(posedge clk) begin
            if(rst) last_fetch_store_table[geni] <= 0;
            else if(instr0_is_store && geni == instr0_store_set_id) last_fetch_store_table[geni] <= instr0_store_id;
            else if(instr1_is_store && geni == instr1_store_set_id) last_fetch_store_table[geni] <= instr1_store_id;
            else if(issue_store && geni == issue_store_set_id && (issue_store_id_ori == issue_store_id))
                last_fetch_store_table[geni] <= 0;
        end
    end
endgenerate

endmodule
