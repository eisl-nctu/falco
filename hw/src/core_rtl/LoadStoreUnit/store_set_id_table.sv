`timescale 1ns/1ps
// =============================================================================
//  Program : store_set_id_table.sv
//  Author  : Chun-Wei Chao
//  Date    : August/07/2021
// -----------------------------------------------------------------------------
//  Description:
//  Store set ID table for Falco.
//  If a load instruction and a store instructino is in same set,
//  it means the load instrcution can not execute before the store instrction execute.
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

module store_set_id_table(
    input clk,
    input rst,

    input [SSIT_WIDTH-1 : 0]            instr0_pc /*verilator public*/, 
    input [SSIT_WIDTH-1 : 0]            instr1_pc /*verilator public*/,
    output [LFST_WIDTH-1 : 0]           instr0_store_set_id /*verilator public*/,
    output [LFST_WIDTH-1 : 0]           instr1_store_set_id /*verilator public*/,

    //recovery signal
    input                               violation /*verilator public*/,
    input [SSIT_WIDTH-1 : 0]            recovery_insrt0_pc /*verilator public*/,
    input [SSIT_WIDTH-1 : 0]            recovery_insrt1_pc /*verilator public*/,
    input [LFST_WIDTH-1 : 0]            recovery_insrt0_id /*verilator public*/,
    input [LFST_WIDTH-1 : 0]            recovery_insrt1_id /*verilator public*/,
    input                               device_violation /*verilator public*/,
    input [SSIT_WIDTH-1 : 0]            device_violation_pc /*verilator public*/
);

logic [SSIT_WIDTH-1 : 0]        my_addr_A /*verilator public*/;
logic [SSIT_WIDTH-1 : 0]        my_addr_B /*verilator public*/;

logic [LFST_WIDTH-1 : 0]        data_iA /*verilator public*/;
logic [LFST_WIDTH-1 : 0]        data_iB /*verilator public*/;
logic                           we_A /*verilator public*/;
logic                           we_B /*verilator public*/;

logic [SSIT_WIDTH-1 : 0]        recovery_change_pc;
logic [SSIT_WIDTH-1 : 0]        recovery_change_pc2;
logic [LFST_WIDTH-1 : 0]        recovery_change_id;

logic [LFST_WIDTH-1 : 0]        allocate_new_id /*verilator public*/;
logic                           all_zero /*verilator public*/;

assign recovery_change_pc = recovery_insrt0_id > recovery_insrt1_id ? recovery_insrt1_pc : recovery_insrt0_pc;
assign recovery_change_pc2 = recovery_insrt0_id > recovery_insrt1_id ? recovery_insrt0_pc : recovery_insrt1_pc;
assign recovery_change_id = recovery_insrt0_id > recovery_insrt1_id ? recovery_insrt0_id : recovery_insrt1_id;
assign all_zero = (recovery_insrt0_id == 0 && recovery_insrt1_id == 0);

always_comb begin
    if(device_violation) my_addr_A = device_violation_pc;
    else if(violation) my_addr_A = recovery_change_pc;
    else my_addr_A = instr0_pc;
end

always_comb begin
    if(violation) my_addr_B = recovery_change_pc2;
    else my_addr_B = instr1_pc;
end

assign data_iA = device_violation ? 1 : all_zero ? allocate_new_id : recovery_change_id;  // 1 for device, 0 is no dependency
assign data_iB = allocate_new_id;

always_ff @(posedge clk) begin
    if(rst) allocate_new_id <= 2;
    else if(allocate_new_id == 2047 && violation && all_zero && ~device_violation) allocate_new_id <= 2;
    else if(violation && all_zero && ~device_violation) allocate_new_id <= allocate_new_id + 1;
    else allocate_new_id <= allocate_new_id;
end


sram_dual_port #( .RAM_WIDTH(LFST_WIDTH), .RAM_ADDR_BITS(SSIT_WIDTH))
    store_set_id(
        .clka(clk),
        .clkb(clk),
        .we_A(violation | device_violation),
        .we_B(violation && all_zero && ~device_violation),
        .en_A(1'b1),
        .en_B(1'b1),
        .addr_A(my_addr_A),
        .addr_B(my_addr_B),
        .data_iA(data_iA),
        .data_iB(data_iB),
        .data_oA(instr0_store_set_id),
        .data_oB(instr1_store_set_id)
    );

endmodule
