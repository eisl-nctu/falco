`timescale 1ns/1ps
// =============================================================================
//  Program : Falco_top.sv
//  Author  : Hon-Chou Dai
//  Date    : May/15/2021
// -----------------------------------------------------------------------------
//  Description:
//  This is the top-level Falco module.
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

module Falco_top
(
    input                                clk,
    input                                rst,

    input  [XLEN_WIDTH-1:0]              init_pc,

    //debug performance bus
    output  [1:0]                         M_CORE_commit_count,

    output                                M_DEVICE_load_access_stall,
    output                                M_DEVICE_store_req,
    output  [ XLEN_WIDTH - 1 : 0]         M_DEVICE_store_addr,
    output  [ XLEN_WIDTH/8-1 : 0]         M_DEVICE_store_byte_enable,
    output  [ XLEN_WIDTH - 1 : 0]         M_DEVICE_store_data,
    output                                M_DEVICE_load_ck_hit_load_req,
    output                                M_DEVICE_load_ck_hit_load_kill,
    output  [ XLEN_WIDTH - 1 : 0]         M_DEVICE_load_addr,

    input                                 M_DEVICE_load_hit,
    input                                 M_DEVICE_load_miss,
    input                                 M_DEVICE_load_data_ready,
    input                                 M_DEVICE_store_finished,
    input [ XLEN_WIDTH - 1 : 0]           M_DEVICE_load_data
);


// ----------- System Memory Map: DDRx DRAM, Devices, or CLINT --------------
//       [0] 0x0000_0000 - 0x0FFF_FFFF : Instruction Tightly-Coupled Memory (ITCM)
//       [0] 0x1000_0000 - 0x1FFF_FFFF : DATA Tightly-Coupled Memory (DTCM)
//       [1] 0x8000_0000 - 0xBFFF_FFFF : DDRx DRAM memory (cached)
//       [2] 0xC000_0000 - 0xCFFF_FFFF : device memory (uncached)
//       [3] 0xF000_0000 - 0xF000_0010 : CLINT I/O registers (uncached)
//

localparam [3:0] ITCM_SEG       = 4'h0;
localparam [3:0] DTCM_SEG       = 4'h1;
localparam [3:0] DRAM_SEG_START = 4'h8;
localparam [3:0] DRAM_SEG_END   = 4'hB; //?
localparam [3:0] DEVICE_SEG     = 4'hC;
localparam [3:0] CLINT_SEG      = 4'hF;

instruction_req_t instruction_req;
instruction_resp_t instruction_resp;

core_load_ck_hit_req_t core_load_ck_hit_req; //LSU stage
core_load_hit_resp_t   core_load_hit_resp;
core_load_data_req_t   core_load_data_req;   //MEM access stage
core_load_data_resp_t  core_load_data_resp;
core_store_req_t       core_store_req;
core_store_resp_t      core_store_resp;

logic DMEM_access_stall;
core_load_ck_hit_req_t DTCM_load_ck_hit_req; //LSU stage
core_load_hit_resp_t   DTCM_load_hit_resp;
core_load_data_req_t   DTCM_load_data_req;   //MEM access stage
core_load_data_resp_t  DTCM_load_data_resp;
core_store_req_t       DTCM_store_req;
core_store_resp_t      DTCM_store_resp;

core_load_ck_hit_req_t DEVICE_load_ck_hit_req;//LSU stage
core_load_hit_resp_t   DEVICE_load_hit_resp;
core_load_data_req_t   DEVICE_load_data_req;  //MEM access stage
core_load_data_resp_t  DEVICE_load_data_resp;
core_store_req_t       DEVICE_store_req;
core_store_resp_t      DEVICE_store_resp;

logic [3:0] core_load_data_segment;     //LSU stage
logic [3:0] core_load_data_segment_dly; //MEM access stage
logic [3:0] core_load_data_segment_wb_dly; //MEM wb stage
logic [3:0] core_store_data_segment;    //LSU stage


// =====================================================
// DEVICE port convert to Systemverilog package
// =====================================================

assign M_DEVICE_load_access_stall = DMEM_access_stall;

// output
assign M_DEVICE_load_ck_hit_load_req = DEVICE_load_ck_hit_req.load_req;
assign M_DEVICE_load_ck_hit_load_kill = DEVICE_load_ck_hit_req.load_kill;
assign M_DEVICE_load_addr = DEVICE_load_ck_hit_req.load_addr;

assign M_DEVICE_store_req = DEVICE_store_req.store_req;
assign M_DEVICE_store_addr = DEVICE_store_req.store_addr;
assign M_DEVICE_store_byte_enable = DEVICE_store_req.store_mask;
assign M_DEVICE_store_data = DEVICE_store_req.store_data;

// input
assign DEVICE_load_hit_resp.load_hit = M_DEVICE_load_hit;
assign DEVICE_load_hit_resp.load_miss = M_DEVICE_load_miss;
assign DEVICE_load_data_resp.load_finished = M_DEVICE_load_data_ready;
assign DEVICE_load_data_resp.load_miss = M_DEVICE_load_miss;
assign DEVICE_load_data_resp.load_data = M_DEVICE_load_data;

assign DEVICE_store_resp.store_finished = M_DEVICE_store_finished;
assign DEVICE_store_resp.store_miss = ~M_DEVICE_store_finished;


// =====================================================
//  arbiter segment
// =====================================================

always_comb
    core_load_data_segment = core_load_ck_hit_req.load_addr[19:16] == 4'b1 ? 4'b1 : core_load_ck_hit_req.load_addr[XLEN_WIDTH-1:XLEN_WIDTH-4];
always_comb
    core_store_data_segment = core_store_req.store_addr[19:16] == 4'b1 ? 4'b1 : core_store_req.store_addr[XLEN_WIDTH-1:XLEN_WIDTH-4];

always_ff @( posedge clk )
    if (rst)
        core_load_data_segment_dly <= 0;
    else if (DMEM_access_stall)
        core_load_data_segment_dly <= core_load_data_segment_dly;
    else
        core_load_data_segment_dly <= core_load_data_segment;

always_ff @( posedge clk )
    if (rst)
        core_load_data_segment_wb_dly <= 0;
    else if (DMEM_access_stall)
        core_load_data_segment_wb_dly <= core_load_data_segment_wb_dly;
    else
        core_load_data_segment_wb_dly <= core_load_data_segment_dly;
// =====================================================
// Load arbiter
// =====================================================

// load request channel
// DTCM 
always_comb begin
    DTCM_load_ck_hit_req.load_req = core_load_ck_hit_req.load_req &&
                                    core_load_data_segment == DTCM_SEG;
    DTCM_load_ck_hit_req.load_addr = core_load_ck_hit_req.load_addr;
end
always_comb begin
    DTCM_load_data_req.load_req = 0;
    DTCM_load_data_req.load_addr = 0;
end

// DEVICE
always_comb begin
    DEVICE_load_ck_hit_req.load_req = core_load_ck_hit_req.load_req &&
                                    core_load_data_segment == DEVICE_SEG;
    DEVICE_load_ck_hit_req.load_kill = core_load_ck_hit_req.load_kill &&
                                    core_load_data_segment == DEVICE_SEG;
    DEVICE_load_ck_hit_req.load_addr = core_load_ck_hit_req.load_addr;
end

// load response channel
always_comb
    if (core_load_data_segment == DTCM_SEG)
        core_load_hit_resp = DTCM_load_hit_resp;
    else if (core_load_data_segment == DEVICE_SEG)
        core_load_hit_resp = DEVICE_load_hit_resp; 
    else
        core_load_hit_resp = DTCM_load_hit_resp;

always_comb
    if (core_load_data_segment_wb_dly == DTCM_SEG)
        core_load_data_resp = DTCM_load_data_resp;
    else if (core_load_data_segment_wb_dly == DEVICE_SEG)
        core_load_data_resp = DEVICE_load_data_resp;
    else
        core_load_data_resp = DTCM_load_data_resp;

// =====================================================
// store arbiter
// =====================================================

// store request channel
// DTCM
always_comb begin
    DTCM_store_req.store_req = core_store_req.store_req &&
                               core_store_data_segment == DTCM_SEG;
    DTCM_store_req.store_addr = core_store_req.store_addr;
    DTCM_store_req.store_data = core_store_req.store_data;
    DTCM_store_req.store_mask = core_store_req.store_mask;
end

// DEVICE
always_comb begin
    DEVICE_store_req.store_req = core_store_req.store_req &&
                               core_store_data_segment == DEVICE_SEG;
    DEVICE_store_req.store_addr = core_store_req.store_addr;
    DEVICE_store_req.store_data = core_store_req.store_data;
    DEVICE_store_req.store_mask = core_store_req.store_mask;
end
// store response channel
always_comb
    if (core_store_data_segment == DTCM_SEG)
        core_store_resp = DTCM_store_resp;
    else if (core_store_data_segment == DEVICE_SEG)
        core_store_resp = DEVICE_store_resp;
    else
        core_store_resp = DTCM_store_resp;

TCM my_tcm(
    .clk                (clk),
    .rst                (rst),
    .stall_i            (DMEM_access_stall),

    .core_req           (instruction_req),
    .itcm_resp          (instruction_resp),

    .load_ck_hit_req    (DTCM_load_ck_hit_req),
    .load_hit_resp      (DTCM_load_hit_resp),
    .load_data_req      (DTCM_load_data_req),
    .load_data_resp     (DTCM_load_data_resp),

    .store_req          (DTCM_store_req),
    .store_resp         (DTCM_store_resp)
); /*verilator no_inline_module*/

core_top core(
    .clk                (clk),
    .rst                (rst),
    .init_pc            (init_pc),

    //debug bus
    .core_commit_count  (M_CORE_commit_count),

    .instruction_req    (instruction_req),
    .instruction_resp   (instruction_resp),

    .load_dmem_stall    (DMEM_access_stall),
    //check data hit
    .load_ck_hit_req    (core_load_ck_hit_req),
    .load_hit_resp      (core_load_hit_resp),
    //get data
    .load_data_req      (core_load_data_req),
    .load_data_resp     (core_load_data_resp),
    .store_req          (core_store_req),
    .store_resp         (core_store_resp)
);

`ifdef FALCO_SIM_DEBUG
logic [31:0] cycle_counter;

always_ff @(posedge clk) begin
    if (rst)
        cycle_counter <= 0;
    else
        cycle_counter <= cycle_counter + 1;
end
`endif
endmodule
