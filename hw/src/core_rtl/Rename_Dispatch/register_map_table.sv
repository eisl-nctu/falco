`timescale 1ns/1ps
// =============================================================================
//  Program : register_map_table.sv
//  Author  : Hon-Chou Dai
//  Date    : May/15/2021
// -----------------------------------------------------------------------------
//  Description:
//  Map the architected register to physical register.
// -----------------------------------------------------------------------------
//  Revision information:
//
//    November/18/2021, by Chun-Wei Chao:
//      Revise to checkpoint recovery version.
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

`timescale 1ns/1ps
// =============================================================================
//  Program : register_map_table.sv
//  Author  : Hon-Chou Dai
//  Date    :
// -----------------------------------------------------------------------------
//  Description:
//
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

module register_map_table(
    input                   clk,
    input                   rst,

//branch tag signal (deprecated)
    input   logic           recovery_mode,

//  ROB branch recovery signal
    input   arf_specifier_t recovery_map_arf_0, //0 older than 1
    input   prf_specifier_t recovery_map_prf_0,
    input   arf_specifier_t recovery_map_arf_1,
    input   prf_specifier_t recovery_map_prf_1,
    input   logic           recovery_map_0_valid,
    input   logic           recovery_map_1_valid,

// RNDS: instr0 register look up signal
    input   arf_specifier_t instr0_rd/*verilator public*/,
    input   arf_specifier_t instr0_rs1,
    input   arf_specifier_t instr0_rs2,
    input   prf_specifier_t instr0_rd_new_prf,
    input   logic           instr0_rd_new_map_valid/*verilator public*/,
// instr0 look up result
    output  prf_specifier_t instr0_rd_stale_prf,
    output  prf_specifier_t instr0_rs1_prf,
    output  prf_specifier_t instr0_rs2_prf,

// RNDS: instr1 register look up signal
    input   arf_specifier_t instr1_rd/*verilator public*/,
    input   arf_specifier_t instr1_rs1,
    input   arf_specifier_t instr1_rs2,
    input   prf_specifier_t instr1_rd_new_prf,
    input   logic           instr1_rd_new_map_valid/*verilator public*/,
// instr1 look up result
    output  prf_specifier_t instr1_rd_stale_prf,
    output  prf_specifier_t instr1_rs1_prf,
    output  prf_specifier_t instr1_rs2_prf,
//Recovery
    input   logic               recovery_flush,
    input   logic               recovery_no_copy,
    input   rob_tag_t           recovery_target_rob_tag,
    input   rob_tag_t           instr0_rob_tag,
    input   rob_tag_t           instr1_rob_tag,
    input   logic               instr0_valid,
    input   logic               instr1_valid
);

prf_specifier_t reg_map_table [31:0]/*verilator public*/;
prf_specifier_t copy_table [7:0][31:0]/*verilator public*/;
logic [2:0]     recovery_target/*verilator public*/;

assign instr0_rd_stale_prf = reg_map_table[instr0_rd];
assign instr0_rs1_prf = reg_map_table[instr0_rs1];
assign instr0_rs2_prf = reg_map_table[instr0_rs2];

assign instr1_rd_stale_prf = (instr0_rd == instr1_rd && instr0_rd_new_map_valid) ? instr0_rd_new_prf : reg_map_table[instr1_rd];
assign instr1_rs1_prf = (instr0_rd == instr1_rs1 && instr0_rd_new_map_valid) ? instr0_rd_new_prf : reg_map_table[instr1_rs1];
assign instr1_rs2_prf = (instr0_rd == instr1_rs2 && instr0_rd_new_map_valid) ? instr0_rd_new_prf : reg_map_table[instr1_rs2];

assign recovery_target = recovery_target_rob_tag[2:0] == 0 ? recovery_target_rob_tag[5:3] : recovery_target_rob_tag[5:3] + 1;

//internal instruction prf specifier bypassing
//because instr0 is older than instr1, only instr1 need bypass.
integer i;

//TODO: roll back and check it synthesis two write port and 4 read port only
//ISSUE: if 2 branch occur in one instruction group, how to backup reg map
always_ff @(posedge clk) begin
    if (rst) begin
        for (i = 0 ; i < 32 ; i = i + 1) begin
            reg_map_table[i] <= i; //init map to architected register specifiers
        end
    end

    if (recovery_flush && ~recovery_no_copy) begin
        for (i = 0 ; i < 32 ; i = i + 1) begin
            reg_map_table[i] <= copy_table[recovery_target][i];
        end
    end else if (recovery_mode) begin
        if (recovery_map_0_valid && 
            recovery_map_1_valid && 
            recovery_map_arf_0 == recovery_map_arf_1) begin
            reg_map_table[recovery_map_arf_0] <= recovery_map_prf_0; //choose older mapping
        end else begin
            if (recovery_map_0_valid)
                reg_map_table[recovery_map_arf_0] <= recovery_map_prf_0;
            if (recovery_map_1_valid)
                reg_map_table[recovery_map_arf_1] <= recovery_map_prf_1;
        end
    end else begin
        if (instr0_rd_new_map_valid && instr1_rd_new_map_valid && instr0_rd == instr1_rd) begin
            reg_map_table[instr0_rd] <= instr1_rd_new_prf; //choose latest mapping
        end else begin
            if (instr0_rd_new_map_valid && instr0_rd != 0)
                reg_map_table[instr0_rd] <= (instr0_rd_new_map_valid ? instr0_rd_new_prf : reg_map_table[instr0_rd]);
            if (instr1_rd_new_map_valid && instr1_rd != 0)
                reg_map_table[instr1_rd] <= (instr1_rd_new_map_valid ? instr1_rd_new_prf : reg_map_table[instr1_rd]);
        end
    end
end

// copy table
logic       instr0_copy, instr1_copy;
logic [2:0] copy_target;

assign instr0_copy = (instr0_rob_tag[2:0] == 0);
assign instr1_copy = (instr1_rob_tag[2:0] == 0);
assign copy_target = instr0_copy ? instr0_rob_tag[5:3] : instr1_rob_tag[5:3];

genvar geni, genj;
generate
    for (geni = 0; geni < 8; geni = geni + 1) begin
        for (genj = 0; genj < 32; genj = genj + 1) begin
            always_ff @(posedge clk) begin
                if (rst) begin
                    copy_table[geni][genj] <= 0;
                end
                else if (~instr0_copy && ~instr1_copy) begin
                    copy_table[geni][genj] <= copy_table[geni][genj];
                end
                else if (copy_target == geni) begin
                    if (genj == instr0_rd && instr0_rd_new_map_valid && instr1_copy)
                        copy_table[geni][genj] <= instr0_rd_new_prf;
                    else
                        copy_table[geni][genj] <= reg_map_table[genj];
                end
                else begin
                    copy_table[geni][genj] <= copy_table[geni][genj];
                end
            end
        end
    end
endgenerate

`ifdef FALCO_SIM_DEBUG
function [31:0] ver_copy_table;
        /*verilator public*/
        input [31:0] index, index2;
        ver_copy_table = copy_table[index][index2];
endfunction
`endif

endmodule
