`timescale 1ns/1ps
// =============================================================================
//  Program : icache.v
//  Author  : Jin-you Wu
//  Date    : Oct/31/2018
// -----------------------------------------------------------------------------
//  Description:
//  This module implements the L1 Instruction Cache with the following
//  properties:
//      4-way
//      Read-only
//      FIFO replacement policy
//      Block size: 8 32-bit words
// -----------------------------------------------------------------------------
//  Revision information:
//
//  None.
// -----------------------------------------------------------------------------
//  License information:
//
//  This software is released under the BSD-3-Clause Licence,
//  see https://opensource.org/licenses/BSD-3-Clause for details.
//  In the following license statements, "software" refers to the
//  "source code" of the complete hardware/software system.
//
//  Copyright 2019,
//                    Embedded Intelligent Systems Lab (EISL)
//                    Deparment of Computer Science
//                    National Chiao Tung Uniersity
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
import L1_cache_pkg::*;

module L1_icache
(
    /////////// System signals   ////////////////////////////////////////////////////
    input                         clk, rst,

    L1_icache_io.icache           io,
    output icache_mem_req_t       icache_req,
    input icache_mem_resp_t       mem_resp
);

// Parameter        ////////////////////////////////////////////////////////////////
localparam N_WAYS          = 4;
localparam WORDS_PER_LINE  = 8;
localparam N_LINES         = (ICACHE_SIZE*1024*8) / (N_WAYS*ICACHE_LINE_SIZE);

localparam WAY_BITS        = $clog2(N_WAYS);
localparam BYTE_BITS       = 2;
localparam WORD_BITS       = $clog2(WORDS_PER_LINE);
localparam LINE_BITS       = $clog2(N_LINES);
localparam NONTAG_BITS     = LINE_BITS + WORD_BITS + BYTE_BITS;
localparam TAG_BITS        = ADDR_WIDTH - NONTAG_BITS;

// Input information ////////////////////////////////////////////////////////////////
logic [WORD_BITS-1 : 0] line_offset;
logic [WORD_BITS-1 : 0] instr1_line_offset;
logic [LINE_BITS-1 : 0] line_index;
logic [TAG_BITS-1  : 0] tag;
mem_addr_t instr1_addr;
logic is_cross_line_fetch;
always_comb instr1_addr = io.core_req.instr1_addr;
always_comb line_offset = io.core_req.instr0_addr[WORD_BITS + BYTE_BITS - 1 : BYTE_BITS];
always_comb instr1_line_offset = instr1_addr[WORD_BITS + BYTE_BITS - 1 : BYTE_BITS];
always_comb line_index  = io.core_req.instr0_addr[NONTAG_BITS - 1 : WORD_BITS + BYTE_BITS];
always_comb tag         = io.core_req.instr0_addr[ADDR_WIDTH - 1 : NONTAG_BITS];
always_comb
    is_cross_line_fetch = (io.core_req.instr0_addr[ADDR_WIDTH-1: WORD_BITS + BYTE_BITS]
                            != instr1_addr[ADDR_WIDTH-1: WORD_BITS+BYTE_BITS] ? 1 : 0);
//=======================================================
// 4-way associative cache signals
//=======================================================
logic way_hit[0 : N_WAYS-1];
logic cache_hit;
logic [WORDS_PER_LINE*32-1 : 0] c_instr_o [0 : N_WAYS-1]; // the data from 4-way associative cache
logic  [WORDS_PER_LINE*32-1 : 0] c_instr;
logic  cache_write [0 : N_WAYS-1];

//=======================================================
//  Valid and Tag store in LUT
//=======================================================
logic VALID_ [0 : N_LINES-1][0 : N_WAYS-1];
logic [TAG_BITS-1 : 0] TAG_ [0 : N_LINES-1][0 : N_WAYS-1];

//=======================================================
// FIFO replace policy signals
//=======================================================
logic  [WAY_BITS-1 : 0]  FIFO_cnt [0 : N_LINES-1]; // Replace policy counter.
logic  [WAY_BITS-1 : 0]  victim_sel;               // The victim cache select.
logic                    is_diff_index;
logic                    is_diff_hit_index;
logic  [LINE_BITS-1 : 0] index_prev;
logic  [LINE_BITS-1 : 0] hit_index_prev;

//=======================================================
//  I-cache Finite State Machine
//=======================================================
typedef enum {Idle,Next,RdfromMem,RdfromMemFinish} state_t;
state_t  S, S_nxt;

always_ff @(posedge clk)
begin
    if (rst)
        S <= Idle;
    else
        S <= S_nxt;
end

always_comb
begin
    case (S)
        Idle:
            if (io.core_req.p_strobe)
                S_nxt = Next;
            else
                S_nxt = Idle;
        Next:
            /*if(p_stop) S_nxt = Idle;
            else */
            if (!cache_hit)
                S_nxt =  RdfromMem;
            else
                S_nxt = Next;
        RdfromMem:
            if (mem_resp.m_ready)
                S_nxt = RdfromMemFinish;
            else
                S_nxt = RdfromMem;
        RdfromMemFinish:
            S_nxt = Next;
        default:
            S_nxt = Idle;
    endcase
end

always_comb way_hit[0] = ( VALID_[line_index][0] && (TAG_[line_index][0] == tag) ) ? 1 : 0;
always_comb way_hit[1] = ( VALID_[line_index][1] && (TAG_[line_index][1] == tag) ) ? 1 : 0;
always_comb way_hit[2] = ( VALID_[line_index][2] && (TAG_[line_index][2] == tag) ) ? 1 : 0;
always_comb way_hit[3] = ( VALID_[line_index][3] && (TAG_[line_index][3] == tag) ) ? 1 : 0;
always_comb cache_hit = (way_hit[0] || way_hit[1] || way_hit[2] || way_hit[3]);
// Bug: Don't Update PC when ICache miss, or it could casue ICache multiple way hit and get wrong Instruction value
always_comb
begin
    case ( {way_hit[0], way_hit[1], way_hit[2], way_hit[3]} )
        4'b1000: c_instr = c_instr_o[0];
        4'b0100: c_instr = c_instr_o[1];
        4'b0010: c_instr = c_instr_o[2];
        4'b0001: c_instr = c_instr_o[3];
        default: c_instr = 0;    // error: the same line_index and tag in the cache!!!!!!!!!!
    endcase
end

/* valid */
integer idx, jdx;

always_comb
begin
    if (mem_resp.m_ready)
        for (idx = 0; idx < N_WAYS; idx = idx + 1)
            cache_write[idx] = (idx == victim_sel);
    else
        for (idx = 0; idx < N_WAYS; idx = idx + 1)
            cache_write[idx] = 0;
end

always_ff @(posedge clk)
begin
    if (rst)
        for (idx = 0; idx < N_WAYS; idx = idx + 1)
            for (jdx = 0; jdx < N_LINES; jdx = jdx + 1)
                VALID_[jdx][idx] <= 1'b0;
    else if (S == RdfromMem && mem_resp.m_ready)
        VALID_[index_prev][victim_sel] <= 1'b1;
end

/* tag */
always_ff @(posedge clk)
begin
    if (rst)
        for (idx = 0; idx < N_WAYS; idx = idx + 1)
            for (jdx = 0; jdx < N_LINES; jdx = jdx + 1)
                TAG_[jdx][idx] <= 0;
    else if (S == RdfromMem && mem_resp.m_ready)
        TAG_[index_prev][victim_sel] <= tag;
end

// always_comb victim_sel = FIFO_cnt[line_index];
always_ff @(posedge clk)
begin
    if (S == Next && !cache_hit)
        victim_sel <= FIFO_cnt[line_index];
    else
        victim_sel <= victim_sel;
end

always_ff @(posedge clk)
begin
    if (rst)
        for (idx = 0; idx < N_LINES; idx = idx + 1)
            FIFO_cnt[idx] <= 0;
    else if (S == RdfromMemFinish)
        FIFO_cnt[line_index] <= FIFO_cnt[line_index] + 1;
end

/* block ram delay debug!!!! */
always_comb is_diff_index = (index_prev != line_index);
always_comb is_diff_hit_index = (hit_index_prev != line_index);
always_ff @(posedge clk)
begin
    if (S == Next && !cache_hit)
        index_prev <= line_index;
    else
        index_prev <= index_prev;
end

always_ff @(posedge clk)
begin
    hit_index_prev <= line_index;
end

//------------------------------------------------------------------------
// Plz modify here to get the correct instruction according to line_offset
//------------------------------------------------------------------------
logic [DATA_WIDTH-1 : 0] instr0_fromCache, instr0_fromMem; // Get the specific word
logic [DATA_WIDTH-1 : 0] instr1_fromCache, instr1_fromMem; // Get the specific word

always_comb
begin
    case (line_offset)
        3'b111: instr0_fromCache = c_instr[ 31: 0];     // [255:224]
        3'b110: instr0_fromCache = c_instr[ 63: 32];    // [223:192]
        3'b101: instr0_fromCache = c_instr[ 95: 64];    // [191:160]
        3'b100: instr0_fromCache = c_instr[127: 96];    // [159:128]
        3'b011: instr0_fromCache = c_instr[159: 128];   // [127: 96]
        3'b010: instr0_fromCache = c_instr[191: 160];   // [ 95: 64]
        3'b001: instr0_fromCache = c_instr[223: 192];   // [ 63: 32]
        3'b000: instr0_fromCache = c_instr[255: 224];   // [ 31:  0]
    endcase
end

always_comb
begin
    case (line_offset)
        3'b111: instr0_fromMem = mem_resp.m_dout[ 31: 0];        // [255:224]
        3'b110: instr0_fromMem = mem_resp.m_dout[ 63: 32];       // [223:192]
        3'b101: instr0_fromMem = mem_resp.m_dout[ 95: 64];       // [191:160]
        3'b100: instr0_fromMem = mem_resp.m_dout[127: 96];       // [159:128]
        3'b011: instr0_fromMem = mem_resp.m_dout[159: 128];      // [127: 96]
        3'b010: instr0_fromMem = mem_resp.m_dout[191: 160];      // [ 95: 64]
        3'b001: instr0_fromMem = mem_resp.m_dout[223: 192];      // [ 63: 32]
        3'b000: instr0_fromMem = mem_resp.m_dout[255: 224];      // [ 31:  0]
    endcase
end

always_comb
begin
    case (instr1_line_offset)
        3'b111: instr1_fromCache = c_instr[ 31: 0];     // [255:224]
        3'b110: instr1_fromCache = c_instr[ 63: 32];    // [223:192]
        3'b101: instr1_fromCache = c_instr[ 95: 64];    // [191:160]
        3'b100: instr1_fromCache = c_instr[127: 96];    // [159:128]
        3'b011: instr1_fromCache = c_instr[159: 128];   // [127: 96]
        3'b010: instr1_fromCache = c_instr[191: 160];   // [ 95: 64]
        3'b001: instr1_fromCache = c_instr[223: 192];   // [ 63: 32]
        3'b000: instr1_fromCache = c_instr[255: 224];   // [ 31:  0]
    endcase
end

always_comb
begin
    case (instr1_line_offset)
        3'b111: instr1_fromMem = mem_resp.m_dout[ 31: 0];        // [255:224]
        3'b110: instr1_fromMem = mem_resp.m_dout[ 63: 32];       // [223:192]
        3'b101: instr1_fromMem = mem_resp.m_dout[ 95: 64];       // [191:160]
        3'b100: instr1_fromMem = mem_resp.m_dout[127: 96];       // [159:128]
        3'b011: instr1_fromMem = mem_resp.m_dout[159: 128];      // [127: 96]
        3'b010: instr1_fromMem = mem_resp.m_dout[191: 160];      // [ 95: 64]
        3'b001: instr1_fromMem = mem_resp.m_dout[223: 192];      // [ 63: 32]
        3'b000: instr1_fromMem = mem_resp.m_dout[255: 224];      // [ 31:  0]
    endcase
end

// Output signals   /////////////////////////////////////////////////////////////
always_comb io.icache_resp.raw_instr0 = ((S == Next) && cache_hit) ? instr0_fromCache : (mem_resp.m_ready) ? instr0_fromMem : 0;
always_comb io.icache_resp.raw_instr1 = ((S == Next) && cache_hit) ? instr1_fromCache : (mem_resp.m_ready) ? instr1_fromMem : 0;
always_comb 
    io.icache_resp.instr0_valid = ( ( (S == Next) && cache_hit && !is_diff_hit_index ) ||  //Hit and diffent index (next instruction)
                                       (mem_resp.m_ready && !is_diff_index) ) ? 1 : 0;     //Get From Mem finished and index unchange 
always_comb 
    io.icache_resp.instr1_valid = io.icache_resp.instr0_valid && !is_cross_line_fetch; //no cross line

always_ff @(posedge clk)
begin
    if (rst)
        icache_req.m_strobe <= 0;
    else if (S == RdfromMem && !mem_resp.m_ready)
        icache_req.m_strobe <= 1;
    else
        icache_req.m_strobe <= 0;
end

always_ff @(posedge clk)
begin
    if (rst)
        icache_req.m_addr <= 0;
    else if (S == RdfromMem)
        icache_req.m_addr <= {io.core_req.instr0_addr[ADDR_WIDTH-1 : 5], 3'b0, 2'b0}; // read 8 words
    else
        icache_req.m_addr <= 0;
end

// Storages /////////////////////////////////////////////////////////////////////

//=======================================================
//  Instructions store in Block RAM
//=======================================================
genvar i;
generate
    for (i = 0; i < N_WAYS; i = i + 1)
    begin
        sram #( .DATA_WIDTH(WORDS_PER_LINE * 32), .N_ENTRIES(N_LINES) )
             DATA_BRAM(
                 .clk(clk),
                 .en(1'b1),
                 .we(cache_write[i]),
                 .addr(line_index),
                 .data_i(mem_resp.m_dout),   // Instructions are read-only.
                 .data_o(c_instr_o[i])
             );
    end
endgenerate
endmodule
