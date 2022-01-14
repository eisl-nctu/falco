`timescale 1ns/1ps
// =============================================================================
//  Program : int_balance_picker.sv
//  Author  : Hon-Chou Dai
//  Date    : May/15/2021
// -----------------------------------------------------------------------------
//  Description:
//  Pick one ready instruction which is the oldest instruction in issue queue.
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

module Picker2_1 (
    input logic [INT_IQ_WIDTH-1:0] in_id[2],
    input logic [INT_IQ_WIDTH:0] in_age[2],
    input logic in_valid[2],
    output logic [INT_IQ_WIDTH-1:0] out_id,
    output logic [INT_IQ_WIDTH:0] out_age,
    output logic out_valid
);
    always_comb begin
        if ( (in_age[1] >= in_age[0] || ~in_valid[0]) && in_valid[1]) begin
            out_valid = 1;
            out_age = in_age[1];
            out_id = in_id[1];
        end else begin
            out_valid = in_valid[0];
            out_age = in_age[0];
            out_id = in_id[0];
        end
    end
endmodule

module Picker8_1 (
    input logic [INT_IQ_WIDTH:0] in_age[8],
    input logic in_valid[8],
    output logic [INT_IQ_WIDTH-1:0] out_id,
    output logic out_valid
);
    integer i;
    logic [INT_IQ_WIDTH-1:0] id[8];

    always_comb begin
        for (i = 0 ; i < 8 ; i = i + 1)
            id[i] = i;
    end
    logic [INT_IQ_WIDTH:0] l1_age[4];
    logic [INT_IQ_WIDTH-1:0] l1_id[4];
    logic l1_valid[4];

    genvar j;
    generate
    for (j = 0 ; j < 8 ; j = j + 2) begin: gen_Picker_l1
        Picker2_1 l1_picker(
            .in_id(id[j : j+1]),
            .in_age(in_age[j : j+1]),
            .in_valid(in_valid[j : j+1]),
            .out_id(l1_id[j/2]),
            .out_age(l1_age[j/2]),
            .out_valid(l1_valid[j/2])
        );
    end
    endgenerate

    logic [INT_IQ_WIDTH:0] l2_age[2];
    logic [INT_IQ_WIDTH-1:0] l2_id[2];
    logic l2_valid[2];

    generate
    for (j = 0 ; j < 4 ; j = j + 2) begin: gen_Picker_l2
        Picker2_1 l2_picker(
            .in_id(l1_id[j : j+1]),
            .in_age(l1_age[j : j+1]),
            .in_valid(l1_valid[j : j+1]),
            .out_id(l2_id[j/2]),
            .out_age(l2_age[j/2]),
            .out_valid(l2_valid[j/2])
        );
    end
    endgenerate

    Picker2_1 l3_picker(
        .in_id(l2_id[0:1]),
        .in_age(l2_age[0:1]),
        .in_valid(l2_valid[0:1]),
        .out_id(out_id),
        .out_age(),
        .out_valid(out_valid)
    );

endmodule

module Picker2_2 (
    input logic [INT_IQ_WIDTH-1:0] in_id[2],
    input logic [INT_IQ_WIDTH:0] in_age[2],
    input logic in_valid[2],
    output logic [INT_IQ_WIDTH-1:0] out_big_id,
    output logic [INT_IQ_WIDTH-1:0] out_small_id,
    output logic [INT_IQ_WIDTH:0] out_big_age,
    output logic [INT_IQ_WIDTH:0] out_small_age,
    output logic out_big_valid,
    output logic out_small_valid
);
    always_comb begin
        if ( (in_age[1] >= in_age[0] || ~in_valid[0]) && in_valid[1]) begin
            out_big_valid = in_valid[1];
            out_big_age = in_age[1];
            out_big_id = in_id[1];
            out_small_valid = in_valid[0];
            out_small_age = in_age[0];
            out_small_id = in_id[0];
        end else begin
            out_big_valid = in_valid[0];
            out_big_age = in_age[0];
            out_big_id = in_id[0];
            out_small_valid = in_valid[1];
            out_small_age = in_age[1];
            out_small_id = in_id[1];
        end
    end
endmodule


module Picker4_2 (
    input logic [INT_IQ_WIDTH-1:0] in_big_id[2],
    input logic [INT_IQ_WIDTH:0] in_big_age[2],
    input logic in_big_valid[2],
    input logic [INT_IQ_WIDTH-1:0] in_small_id[2],
    input logic [INT_IQ_WIDTH:0] in_small_age[2],
    input logic in_small_valid[2],
    output logic [INT_IQ_WIDTH-1:0] out_big_id,
    output logic [INT_IQ_WIDTH-1:0] out_small_id,
    output logic [INT_IQ_WIDTH:0] out_big_age,
    output logic [INT_IQ_WIDTH:0] out_small_age,
    output logic out_big_valid,
    output logic out_small_valid
);
    logic [INT_IQ_WIDTH+1:0] big_cmp[2];
    logic [INT_IQ_WIDTH+1:0] small_cmp[2];

    always_comb begin
        big_cmp[0] = {in_big_valid[0],in_big_age[0]};
        big_cmp[1] = {in_big_valid[1],in_big_age[1]};
        small_cmp[0] = {in_small_valid[0],in_small_age[0]};
        small_cmp[1] = {in_small_valid[1],in_small_age[1]};
    end
    // It's impossible to happen (small small) case
    always_comb begin
        if ( big_cmp[0] >= big_cmp[1] && small_cmp[0] >= big_cmp[1]) begin
            out_big_valid = in_big_valid[0];
            out_big_age = in_big_age[0];
            out_big_id = in_big_id[0];
            out_small_valid = in_small_valid[0];
            out_small_age = in_small_age[0];
            out_small_id = in_small_id[0];
        end else if (big_cmp[0] >= big_cmp[1] && big_cmp[1] > small_cmp[0])begin
            out_big_valid = in_big_valid[0];
            out_big_age = in_big_age[0];
            out_big_id = in_big_id[0];
            out_small_valid = in_big_valid[1];
            out_small_age = in_big_age[1];
            out_small_id = in_big_id[1];
        end else if (big_cmp[1] > big_cmp[0] && big_cmp[0] >= small_cmp[1]) begin
            out_big_valid = in_big_valid[1];
            out_big_age = in_big_age[1];
            out_big_id = in_big_id[1];
            out_small_valid = in_big_valid[0];
            out_small_age = in_big_age[0];
            out_small_id = in_big_id[0]; 
        end else begin
            out_big_valid = in_big_valid[1];
            out_big_age = in_big_age[1];
            out_big_id = in_big_id[1];
            out_small_valid = in_small_valid[1];
            out_small_age = in_small_age[1];
            out_small_id = in_small_id[1]; 
        end
    end
endmodule


module Picker8_2 (
    input logic [INT_IQ_WIDTH:0] in_age[8],
    input logic in_valid[8],
    output logic [INT_IQ_WIDTH-1:0] out_id_1,
    output logic [INT_IQ_WIDTH-1:0] out_id_0,
    output logic out_valid_0,
    output logic out_valid_1
);
    integer i;
    logic [INT_IQ_WIDTH-1:0] id[8];

    always_comb begin
        for (i = 0 ; i < 8 ; i = i + 1)
            id[i] = i;
    end
    logic [INT_IQ_WIDTH:0] l1_big_age[4];
    logic [INT_IQ_WIDTH-1:0] l1_big_id[4];
    logic l1_big_valid[4];
    logic [INT_IQ_WIDTH:0] l1_small_age[4];
    logic [INT_IQ_WIDTH-1:0] l1_small_id[4];
    logic l1_small_valid[4];

    genvar j;
    generate
    for (j = 0 ; j < 8 ; j = j + 2) begin: gen_Picker_l1
        Picker2_2 l1_picker(
            .in_id(id[j : j+1]),
            .in_age(in_age[j : j+1]),
            .in_valid(in_valid[j : j+1]),
            .out_big_id(l1_big_id[j/2]),
            .out_small_id(l1_small_id[j/2]),
            .out_big_age(l1_big_age[j/2]),
            .out_small_age(l1_small_age[j/2]),
            .out_big_valid(l1_big_valid[j/2]),
            .out_small_valid(l1_small_valid[j/2])
        );
    end
    endgenerate

    logic [INT_IQ_WIDTH:0] l2_big_age[2];
    logic [INT_IQ_WIDTH-1:0] l2_big_id[2];
    logic l2_big_valid[2];
    logic [INT_IQ_WIDTH:0] l2_small_age[2];
    logic [INT_IQ_WIDTH-1:0] l2_small_id[2];
    logic l2_small_valid[2];

    generate
    for (j = 0 ; j < 4 ; j = j + 2) begin: gen_Picker_l2
        Picker4_2 l2_picker(
            .in_big_id(l1_big_id[j : j+1]),
            .in_big_age(l1_big_age[j : j+1]),
            .in_big_valid(l1_big_valid[j : j+1]),
            .in_small_id(l1_small_id[j : j+1]),
            .in_small_age(l1_small_age[j : j+1]),
            .in_small_valid(l1_small_valid[j : j+1]),
            .out_big_id(l2_big_id[j/2]),
            .out_small_id(l2_small_id[j/2]),
            .out_big_age(l2_big_age[j/2]),
            .out_small_age(l2_small_age[j/2]),
            .out_big_valid(l2_big_valid[j/2]),
            .out_small_valid(l2_small_valid[j/2])
        );
    end
    endgenerate

    Picker4_2 l3_picker(
        .in_big_id(l2_big_id),
        .in_big_age(l2_big_age),
        .in_big_valid(l2_big_valid),
        .in_small_id(l2_small_id),
        .in_small_age(l2_small_age),
        .in_small_valid(l2_small_valid),
        .out_big_id(out_id_0),
        .out_small_id(out_id_1),
        .out_big_age(),
        .out_small_age(),
        .out_big_valid(out_valid_0),
        .out_small_valid(out_valid_1)
    );


endmodule
