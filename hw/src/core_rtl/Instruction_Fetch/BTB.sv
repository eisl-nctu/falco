`timescale 1ns/1ps
// =============================================================================
//  Program : BTB.sv
//  Author  : Hon-Chou Dai
//  Date    : May/15/2021
// -----------------------------------------------------------------------------
//  Description:
//  Branch Target Buffer for Falco, entry size is defined in falco_pkg.sv
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

module BTB(
    input           clk,
    input           rst,

    //PC stage -> IF stage pc
    input   pc_t    IF_instr0_pc,
    input   pc_t    IF_instr1_pc,

    // From EXE branch unit , use to update btb entry and status
    input   logic   branch_valid,
    input   logic   branch_taken,
    input   pc_t    branch_addr,
    input   pc_t    branch_target_addr,

    //BTB hit signal to IF stage
    output  logic   instr0_btb_hit,
    output  pc_t    instr0_btb_target_addr,
    output  logic   instr1_btb_hit,
    output  pc_t    instr1_btb_target_addr
);

/***********************************************************************
*   BTB structure
*   PC[INDEX_WIDTH+1:2] -> BTB Table
*
*   | TAG (PC [31:INDEX_WIDTH+1] ) | Branch Target address (30 bits) |
*
************************************************************************/

localparam WORD_WIDTH = 2; //4 byte
localparam INDEX_WIDTH = $clog2(BTB_ENTRY_NUM);
localparam TAG_WIDTH = XLEN_WIDTH - WORD_WIDTH - INDEX_WIDTH; 
localparam BTB_TARGET_ADDR_WIDTH = XLEN_WIDTH - WORD_WIDTH; //30 bits
localparam BTB_SINGLE_ENTRY_WIDTH = TAG_WIDTH + BTB_TARGET_ADDR_WIDTH; 
//Should we save tag in LUT?,may be it could casue critcal path

// Branch Target Valid bits array (Whether this entry contain valid Target address.)
logic   btb_entry_valids [0 : BTB_ENTRY_NUM-1];

// EXE branch result update signal
logic [INDEX_WIDTH-1:0]                 update_entry_addr;
logic                                   update_btb_entry;
logic [BTB_TARGET_ADDR_WIDTH - 1: 0]    new_btb_cache_target_addr;
logic [TAG_WIDTH - 1: 0]                new_btb_cache_tag;

// IF stage branch target buffer hit signal
logic [INDEX_WIDTH-1:0]                 instr0_read_addr,instr1_read_addr;
logic [TAG_WIDTH - 1 : 0]               instr0_read_btb_tag;
logic [TAG_WIDTH - 1 : 0]               instr1_read_btb_tag;
logic [TAG_WIDTH - 1 :0 ]               instr0_input_pc_tag;
logic [TAG_WIDTH - 1 :0 ]               instr1_input_pc_tag;

//internal compute siganl
pc_t instr0_btb_trunc_target_addr;
pc_t instr1_btb_trunc_target_addr;


assign update_btb_entry = branch_valid && branch_taken;
assign update_entry_addr = branch_addr[INDEX_WIDTH + WORD_WIDTH - 1: WORD_WIDTH];
assign new_btb_cache_target_addr = branch_target_addr >> WORD_WIDTH;
assign new_btb_cache_tag = branch_addr[XLEN_WIDTH-1:INDEX_WIDTH+WORD_WIDTH];

assign instr0_read_addr = IF_instr0_pc[INDEX_WIDTH + WORD_WIDTH - 1: WORD_WIDTH];
assign instr0_input_pc_tag = IF_instr0_pc[XLEN_WIDTH-1:INDEX_WIDTH+WORD_WIDTH];
assign instr0_btb_hit = btb_entry_valids[instr0_read_addr] &&
    instr0_input_pc_tag == instr0_read_btb_tag;

assign instr1_read_addr = IF_instr1_pc[INDEX_WIDTH + WORD_WIDTH - 1: WORD_WIDTH];
assign instr1_input_pc_tag = IF_instr1_pc[XLEN_WIDTH-1:INDEX_WIDTH+WORD_WIDTH];
assign instr1_btb_hit = btb_entry_valids[instr1_read_addr] && 
    instr1_input_pc_tag == instr1_read_btb_tag;

assign instr0_btb_target_addr = instr0_btb_trunc_target_addr << WORD_WIDTH;
assign instr1_btb_target_addr = instr1_btb_trunc_target_addr << WORD_WIDTH;

distri_ram2r1w #( .ENTRY_NUM(BTB_ENTRY_NUM), .ADDR_WIDTH(INDEX_WIDTH) ,.DATA_WIDTH(BTB_TARGET_ADDR_WIDTH) )
    direct_mapped_cache_target(
        .clk(clk),
        .we(update_btb_entry),
        .data_i(new_btb_cache_target_addr),
        .write_addr(update_entry_addr),
        .read_0_addr(instr0_read_addr),
        .read_1_addr(instr1_read_addr),
        .data_0_o(instr0_btb_trunc_target_addr),
        .data_1_o(instr1_btb_trunc_target_addr)
    );

distri_ram2r1w #(. ENTRY_NUM(BTB_ENTRY_NUM), .ADDR_WIDTH(INDEX_WIDTH), .DATA_WIDTH(TAG_WIDTH))
    direct_mapped_cache_tag(
        .clk(clk),
        .we(update_btb_entry),
        .data_i(new_btb_cache_tag),
        .write_addr(update_entry_addr),
        .read_0_addr(instr0_read_addr),
        .read_1_addr(instr1_read_addr),
        .data_0_o(instr0_read_btb_tag),
        .data_1_o(instr1_read_btb_tag)
    );

integer i;


always_ff @(posedge clk) begin
    if (rst)
        for (i = 0 ; i < BTB_ENTRY_NUM ; i = i + 1)
            btb_entry_valids[i] <= 0;
    else if (update_btb_entry)
        btb_entry_valids[update_entry_addr] <= 1;
    else
        for (i = 0 ; i < BTB_ENTRY_NUM ; i = i + 1)
            btb_entry_valids[i] <= btb_entry_valids[i];
end

endmodule
