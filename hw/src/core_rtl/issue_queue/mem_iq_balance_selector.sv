`timescale 1ns/1ps
// =============================================================================
//  Program : mem_iq_balance_selector.sv
//  Author  : Hon-Chou Dai
//  Date    : May/15/2021
// -----------------------------------------------------------------------------
//  Description:
//  Not use in Falco in this version.
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

//============================================
// Algorithm: add aged counter to every entry
//            when 1 issue apply, add all valid entry's counter
//            it would cause aged counter saturated sometimes.
//            balance selector
//============================================

module mem_iq_balance_selector(
    input           clk,
    input           rst,
    input   logic                       issue_ready[Falco_pkg::MEM_IQ_NUM],
    input   logic                       entry_is_store[Falco_pkg::MEM_IQ_NUM],
    input   logic                       issue_lock,
    output  logic [MEM_IQ_WIDTH-1:0]    issue_slot_idx,
    output  logic                       issue_slot_idx_valid
);

logic [MEM_IQ_WIDTH-1:0] issue_slot_idx_pre_valid;

LS_Picker8_1 mem_picker(
    .in_is_store(entry_is_store),
    .in_valid(issue_ready),
    .out_id(issue_slot_idx),
    .out_valid(issue_slot_idx_pre_valid)
);
always_comb begin
    if (rst) begin
        issue_slot_idx_valid = 0;
    end else begin
        issue_slot_idx_valid = issue_lock ? 0 : issue_slot_idx_pre_valid;
    end
end
//TODO: memory sda full only issue load instruction only (if no dcache miss)
endmodule

module LS_Picker2_1 (
    input logic [MEM_IQ_WIDTH-1:0] in_id[2],
    input logic in_is_store[2],
    input logic in_valid[2],
    output logic [MEM_IQ_WIDTH-1:0] out_id,
    output logic out_valid,
    output logic out_is_store
);
    always_comb begin
        if ( in_valid[0] && (in_is_store[0] || ~in_valid[1])) begin
            out_valid = 1;
            out_id = in_id[0];
            out_is_store = 1;
        end else begin
            out_valid = in_valid[1];
            out_is_store = in_is_store[1];
            out_id = in_id[1];
        end
    end
endmodule

module LS_Picker8_1 (
    input logic in_is_store[8],
    input logic in_valid[8],
    output logic [MEM_IQ_WIDTH-1:0] out_id,
    output logic out_valid
);
    integer i;
    logic [MEM_IQ_WIDTH-1:0] id[8];

    always_comb begin
        for (i = 0 ; i < 8 ; i = i + 1)
            id[i] = i;
    end
    logic l1_is_store[4];
    logic [MEM_IQ_WIDTH-1:0] l1_id[4];
    logic l1_valid[4];

    genvar j;
    generate
    for (j = 0 ; j < 8 ; j = j + 2) begin: gen_Picker_l1
        LS_Picker2_1 l1_picker(
            .in_id(id[j : j+1]),
            .in_is_store(in_is_store[j : j+1]),
            .in_valid(in_valid[j : j+1]),
            .out_id(l1_id[j/2]),
            .out_is_store(l1_is_store[j/2]),
            .out_valid(l1_valid[j/2])
        );
    end
    endgenerate

    logic l2_is_store[2];
    logic [MEM_IQ_WIDTH-1:0] l2_id[2];
    logic l2_valid[2];

    generate
    for (j = 0 ; j < 4 ; j = j + 2) begin: gen_Picker_l2
        LS_Picker2_1 l2_picker(
            .in_id(l1_id[j : j+1]),
            .in_is_store(l1_is_store[j : j+1]),
            .in_valid(l1_valid[j : j+1]),
            .out_id(l2_id[j/2]),
            .out_is_store(l2_is_store[j/2]),
            .out_valid(l2_valid[j/2])
        );
    end
    endgenerate

    LS_Picker2_1 l3_picker(
        .in_id(l2_id[0:1]),
        .in_is_store(l2_is_store[0:1]),
        .in_valid(l2_valid[0:1]),
        .out_id(out_id),
        .out_is_store(),
        .out_valid(out_valid)
    );

endmodule
