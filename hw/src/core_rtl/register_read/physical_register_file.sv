`timescale 1ns/1ps
// =============================================================================
//  Program : physical_register_file.sv
//  Author  : Hon-Chou Dai
//  Date    : May/15/2021
// -----------------------------------------------------------------------------
//  Description:
//  Physical register file for Falco.
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

// 6 Read port 4 write port

module physical_register_file(
    input clk,
    input rst,
    //INT IQ
    input   prf_specifier_t     IQ0_rs1_addr,
    input   prf_specifier_t     IQ0_rs2_addr,
    input   prf_specifier_t     IQ1_rs1_addr,
    input   prf_specifier_t     IQ1_rs2_addr,
    //MEM IQ
    input   prf_specifier_t     MEM_rs1_addr,
    input   prf_specifier_t     MEM_rs2_addr, //for store
    //INT data
    output  xlen_data_t         IQ0_rs1_data,
    output  xlen_data_t         IQ0_rs2_data,
    output  xlen_data_t         IQ1_rs1_data,
    output  xlen_data_t         IQ1_rs2_data,
    //MEM data
    output  xlen_data_t         MEM_rs1_data,
    output  xlen_data_t         MEM_rs2_data,

    input   exe_fu_wb_t         alu_csr_bc_wb,
    input   exe_fu_wb_t         alu1_wb,
    input   exe_fu_wb_t         muldiv_wb,
    input   exe_fu_wb_t         mem_wb
);

xlen_data_t prf_file[PRF_NUM-1:0] /* verilator public */;

integer i;

always_ff @(posedge clk) begin
    if (rst) begin
        for (i = 0 ; i < PRF_NUM ; i = i + 1) begin
            prf_file[i] <= 0;
        end
    end

    if (alu_csr_bc_wb.valid && alu_csr_bc_wb.wb_addr != 0)
        prf_file[alu_csr_bc_wb.wb_addr] <= alu_csr_bc_wb.wb_data;
    if (alu1_wb.valid && alu1_wb.wb_addr != 0)
        prf_file[alu1_wb.wb_addr] <= alu1_wb.wb_data;
    if (muldiv_wb.valid && muldiv_wb.wb_addr != 0)
        prf_file[muldiv_wb.wb_addr] <= muldiv_wb.wb_data;
    if (mem_wb.valid && mem_wb.wb_addr != 0 )
        prf_file[mem_wb.wb_addr] <= mem_wb.wb_data;
end
//super big net
assign IQ0_rs1_data = prf_file[IQ0_rs1_addr];
assign IQ0_rs2_data = prf_file[IQ0_rs2_addr];
assign IQ1_rs1_data = prf_file[IQ1_rs1_addr];
assign IQ1_rs2_data = prf_file[IQ1_rs2_addr];
assign MEM_rs1_data = prf_file[MEM_rs1_addr];
assign MEM_rs2_data = prf_file[MEM_rs2_addr];

endmodule
