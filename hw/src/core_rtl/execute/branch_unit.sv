`timescale 1ns/1ps
// =============================================================================
//  Program : branch_unit.sv
//  Author  : Hon-Chou Dai
//  Date    : May/15/2021
// -----------------------------------------------------------------------------
//  Description:
//  Branch result caculation unit for Falco.
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

module branch_unit(
    input   xlen_data_t a,
    input   xlen_data_t b,
    input   logic [2:0] operation_sel,
    output  logic       compare_result
);

branch_op_t branch_op;

assign branch_op = branch_op_t'(operation_sel);

wire signed [DATA_WIDTH-1 : 0] signed_a = a;
wire signed [DATA_WIDTH-1 : 0] signed_b = b;

always_comb begin
    case (branch_op)
        BRANCH_OP_BEQ:   compare_result = (a == b) ? 1 : 0;
        BRANCH_OP_BNE:   compare_result = (a != b) ? 1 : 0;
        BRANCH_OP_DUMMY0:compare_result = 0; //should not happen
        BRANCH_OP_DUMMY1:compare_result = 0; //should not happen
        BRANCH_OP_BLT:   compare_result = (signed_a < signed_b) ? 1 : 0;
        BRANCH_OP_BGE:   compare_result = (signed_a >= signed_b) ? 1 : 0;
        BRANCH_OP_BLTU:  compare_result = (a < b) ? 1 : 0;
        BRANCH_OP_BGEU:  compare_result = (a >= b) ? 1 : 0;
    endcase
end

endmodule
