`timescale 1ns/1ps
// =============================================================================
//  Program : load_forward_unit.sv
//  Author  : Hon-Chou Dai
//  Date    : May/15/2021
// -----------------------------------------------------------------------------
//  Description:
//  Used in store buffer to select forward data.
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

//TODO: full generate and parameterization

module load_forward_unit(
    input logic match[SDB_NUM],
    input logic [SDB_WIDTH - 1:0] push_head,
    output logic hit,
    output logic [SDB_WIDTH-1:0] hit_index
);

integer i;
logic [SDB_WIDTH-1:0] id[SDB_NUM];
always_comb begin
    for (i = 0 ; i < SDB_NUM ; i = i + 1)
        id[i] = i;
end
logic l1_hit[SDB_NUM/2];
logic [SDB_WIDTH-1:0] l1_hit_index[SDB_NUM/2];

genvar j;
generate
for (j = 0 ; j < SDB_NUM ; j = j + 2) begin: gen_matcher_l1
    Latest_Picker_2_1 picker(
        .match(match[j : j +1]),
        .index(id[j : j + 1]),
        .push_head(push_head),
        .hit(l1_hit[j/2]),
        .hit_index(l1_hit_index[j/2])
    );
end
endgenerate

logic l2_hit[SDB_NUM/4];
logic [SDB_WIDTH-1:0] l2_hit_index[SDB_NUM/4];

generate
for (j = 0 ; j < SDB_NUM/2 ; j = j + 2) begin: gen_matcher_l2
    Latest_Picker_2_1 picker(
        .match(l1_hit[j : j +1]),
        .index(l1_hit_index[j : j + 1]),
        .push_head(push_head),
        .hit(l2_hit[j/2]),
        .hit_index(l2_hit_index[j/2])
    );
end
endgenerate

logic l3_hit[SDB_NUM/8];
logic [SDB_WIDTH-1:0] l3_hit_index[SDB_NUM/8];

generate
for (j = 0 ; j < SDB_NUM/4 ; j = j + 2) begin: gen_matcher_l3
    Latest_Picker_2_1 picker(
        .match(l2_hit[j : j +1]),
        .index(l2_hit_index[j : j + 1]),
        .push_head(push_head),
        .hit(l3_hit[j/2]),
        .hit_index(l3_hit_index[j/2])
    );
end
endgenerate

logic l4_hit[SDB_NUM/16];
logic [SDB_WIDTH-1:0] l4_hit_index[SDB_NUM/16];

generate
for (j = 0 ; j < SDB_NUM/8 ; j = j + 2) begin: gen_matcher_l4
    Latest_Picker_2_1 picker(
        .match(l3_hit[j : j +1]),
        .index(l3_hit_index[j : j + 1]),
        .push_head(push_head),
        .hit(l4_hit[j/2]),
        .hit_index(l4_hit_index[j/2])
    );
end
endgenerate

always_comb hit = l4_hit[0];
always_comb hit_index = l4_hit_index[0];

endmodule

module Latest_Picker_2_1(
    input logic match[2],
    input [SDB_WIDTH - 1:0] index[2],
    input logic [SDB_WIDTH - 1:0] push_head,
    output logic hit,
    output logic [SDB_WIDTH-1:0] hit_index
);
    always_comb begin
        if ( (((push_head > index[0]) ^ (push_head > index[1]) ^ (index[0] > index[1])) &&
            match[0]) || ~match[1] ) begin
            hit = match[0];
            hit_index = index[0];
        end else begin
            hit = match[1];
            hit_index = index[1];
        end
    end
endmodule
