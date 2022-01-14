`timescale 1ns/1ps
// =============================================================================
//  Program : busy_list.sv
//  Author  : Hon-Chou Dai
//  Date    : May/15/2021
// -----------------------------------------------------------------------------
//  Description:
//  Tell Falco instructions' operand is busy or not.
//  If an instruction's operand is busy, it should stay in issue_queue until operand ready.
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

//When prf pop freelist,set busy list entry to 1
//When write back to prf update busy list entry to 0
//must handle bypass issue (wb to busy list and update rnds stage issue entry 
// due to one cycle latency)

module busy_list(
    input                       clk,
    input                       rst,

    //RNDS stage rs1,rs2 prf look up siganl
    input   prf_specifier_t     instr0_rs1_prf,
    input   prf_specifier_t     instr0_rs2_prf,
    input   prf_specifier_t     instr1_rs1_prf,
    input   prf_specifier_t     instr1_rs2_prf,

    //RNDS stage rs1,rs2 status
    output  logic               instr0_rs1_ready,
    output  logic               instr0_rs2_ready,
    output  logic               instr1_rs1_ready,
    output  logic               instr1_rs2_ready,

    // RNDS stage rd use (set prf to busy request)
    input   prf_specifier_t     instr0_rd_prf,
    input   prf_specifier_t     instr1_rd_prf,
    input   logic               instr0_rd_valid,
    input   logic               instr1_rd_valid,

    // EXE stage and LSU stage to clear prf busy event
    // (if back to back issue enable then some signal (ALU0,ALU1) must depreciated)
    input   exe_broadcast_t     BCAST_alu_csr_bc,
    input   exe_broadcast_t     BCAST_ALU_1,
    input   exe_broadcast_t     BCAST_muldiv,
    // LSU stage
    input   exe_broadcast_t     BCAST_load,
    // Issue queue stage (back to back issue signal)
    input   logic               int_issue0_rd_valid,
    input   prf_specifier_t     int_issue0_rd_prf,
    input   logic               int_issue1_rd_valid,
    input   prf_specifier_t     int_issue1_rd_prf
);

logic busy_list_bits[PRF_NUM-1:0] /*verilator public*/;

always_comb instr0_rs1_ready = bypass_network(instr0_rs1_prf,busy_list_bits[instr0_rs1_prf]);
always_comb instr0_rs2_ready = bypass_network(instr0_rs2_prf,busy_list_bits[instr0_rs2_prf]);
always_comb instr1_rs1_ready = bypass_next_network(instr1_rs1_prf,busy_list_bits[instr1_rs1_prf]);
always_comb instr1_rs2_ready = bypass_next_network(instr1_rs2_prf,busy_list_bits[instr1_rs2_prf]);
integer i;

always_ff @(posedge clk) begin
    if (rst) begin
        for (i = 0 ; i < 32 ; i = i + 1)
            busy_list_bits[i] <= 1;
        for (i = 32 ; i < PRF_NUM ; i = i + 1)
            busy_list_bits[i] <= 1;
    end

    if (instr0_rd_prf && instr0_rd_valid)
        busy_list_bits[instr0_rd_prf] <= 0;
    if (instr1_rd_prf && instr1_rd_valid)
        busy_list_bits[instr1_rd_prf] <= 0;

    if (BCAST_alu_csr_bc.valid)
        busy_list_bits[BCAST_alu_csr_bc.prf_addr] <= 1;
    if (BCAST_ALU_1.valid)
        busy_list_bits[BCAST_ALU_1.prf_addr] <= 1;
    if (BCAST_muldiv.valid)
        busy_list_bits[BCAST_muldiv.prf_addr] <= 1;
    if (BCAST_load.valid)
        busy_list_bits[BCAST_load.prf_addr] <= 1;
    if (int_issue0_rd_valid)
        busy_list_bits[int_issue0_rd_prf] <= 1;
    if (int_issue1_rd_valid)
        busy_list_bits[int_issue1_rd_prf] <= 1;
    
end

function logic bypass_network(
    input prf_specifier_t prf_addr,
    input logic stale_bit
);
    begin

        if ( (BCAST_alu_csr_bc.valid && BCAST_alu_csr_bc.prf_addr == prf_addr) ||
             (BCAST_ALU_1.valid && BCAST_ALU_1.prf_addr == prf_addr) ||
             (BCAST_muldiv.valid && BCAST_muldiv.prf_addr == prf_addr) ||
             (BCAST_load.valid && BCAST_load.prf_addr == prf_addr) ||
             (int_issue0_rd_valid && int_issue0_rd_prf == prf_addr) ||
             (int_issue1_rd_valid && int_issue1_rd_prf == prf_addr))
            bypass_network = 1;
        else
            bypass_network = stale_bit;
    end
endfunction

function logic bypass_next_network(
    input prf_specifier_t prf_addr,
    input logic stale_bit
);
    begin
        if (instr0_rd_prf == prf_addr && instr0_rd_valid)
            bypass_next_network = 0;
        else if ((BCAST_alu_csr_bc.valid && BCAST_alu_csr_bc.prf_addr == prf_addr) ||
            (BCAST_ALU_1.valid && BCAST_ALU_1.prf_addr == prf_addr) ||
            (BCAST_muldiv.valid && BCAST_muldiv.prf_addr == prf_addr) ||
            (BCAST_load.valid && BCAST_load.prf_addr == prf_addr) ||
            (int_issue0_rd_valid && int_issue0_rd_prf == prf_addr) ||
            (int_issue1_rd_valid && int_issue1_rd_prf == prf_addr)
            )
            bypass_next_network = 1;
        else
            bypass_next_network = stale_bit;
    end
endfunction

endmodule
