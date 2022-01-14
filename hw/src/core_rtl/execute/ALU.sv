`timescale 1ns/1ps
// =============================================================================
//  Program : ALU.sv
//  Author  : Hon-Chou Dai
//  Date    : May/15/2021
// -----------------------------------------------------------------------------
//  Description:
//  ALU for Falco.
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

module ALU(
    input   xlen_data_t a,
    input   xlen_data_t b,
    input   logic [2:0] operation_sel,
    input   logic       shift_sel,
    output  xlen_data_t alu_result
);

xlen_data_t result_add, result_sll, result_slt, result_sltu;
xlen_data_t result_xor, result_sr, result_or, result_and;

alu_op_t operation;

assign operation = alu_op_t'(operation_sel);
assign result_add = a + b;                                    // add
assign result_sll = a << b[4:0];                              // sll
assign result_slt = ($signed(a) < $signed(b)) ? 1 : 0;        // slt
assign result_sltu = (a < b) ? 1 : 0;                         // sltu
assign result_xor = a ^ b;                                    // xor
assign result_sr = shift_sel? ($signed(a) >>> b[4:0]) : ($signed(a) >> b[4:0]); // sra, srl
assign result_or = a | b;                                     // or
assign result_and = a & b;                                    // and

always_comb begin
    case (operation)
        ALU_OP_ADD:  alu_result = result_add;
        ALU_OP_SLL:  alu_result = result_sll;
        ALU_OP_SLT:  alu_result = result_slt;
        ALU_OP_SLTU: alu_result = result_sltu;
        ALU_OP_XOR:  alu_result = result_xor;
        ALU_OP_SR:   alu_result = result_sr;
        ALU_OP_OR:   alu_result = result_or;
        ALU_OP_AND:  alu_result = result_and;
    endcase
end

endmodule
