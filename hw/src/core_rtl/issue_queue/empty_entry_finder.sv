`timescale 1ns/1ps
// =============================================================================
//  Program : empty_entry_finder.sv
//  Author  : Hon-Chou Dai
//  Date    : May/15/2021
// -----------------------------------------------------------------------------
//  Description:
//  Find the empty slot in issue queue.
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


module emptry_entry_finder2
#(parameter IO_ID_WIDTH = 3)
(
    input logic [IO_ID_WIDTH-1:0] in_id[2],
    input logic in_alloc_valid[2],
    output logic [IO_ID_WIDTH-1:0] out_id_0,
    output logic [IO_ID_WIDTH-1:0] out_id_1,
    output logic out_alloc_valid_0,
    output logic out_alloc_valid_1
);
always_comb begin
    if (in_alloc_valid[0]) begin
        out_id_0 = in_id[0];
        out_id_1 = in_id[1];
        out_alloc_valid_0 = in_alloc_valid[0];
        out_alloc_valid_1 = in_alloc_valid[1];
    end else begin
        out_id_0 = in_id[1];
        out_id_1 = in_id[0];
        out_alloc_valid_0 = in_alloc_valid[1];
        out_alloc_valid_1 = in_alloc_valid[0];
    end
end
endmodule

module emptry_entry_finder4
#(parameter IO_ID_WIDTH = 3)
(
    input logic [IO_ID_WIDTH-1:0] in_id_0[2],
    input logic [IO_ID_WIDTH-1:0] in_id_1[2],
    input logic in_alloc_valid_0[2],
    input logic in_alloc_valid_1[2],
    output logic [IO_ID_WIDTH-1:0] out_id_0,
    output logic [IO_ID_WIDTH-1:0] out_id_1,
    output logic out_alloc_valid_0,
    output logic out_alloc_valid_1
);

always_comb begin
    if (in_alloc_valid_0[0] && in_alloc_valid_1[0]) begin  //from one emptry_entry_finder2 group
        out_id_0 = in_id_0[0];
        out_id_1 = in_id_1[0];
        out_alloc_valid_0 = in_alloc_valid_0[0];
        out_alloc_valid_1 = in_alloc_valid_1[0];
    end else if  (in_alloc_valid_0[1] && in_alloc_valid_1[1]) begin  //from one emptry_entry_finder2 group
        out_id_0 = in_id_0[1];
        out_id_1 = in_id_1[1];
        out_alloc_valid_0 = in_alloc_valid_0[1];
        out_alloc_valid_1 = in_alloc_valid_1[1];
    end else if (in_alloc_valid_0[0])begin //Nor f2 g1 nor f2 g2 can give 2 empty slot, so picker one
        out_id_0 = in_id_0[0];
        out_id_1 = in_id_0[1];
        out_alloc_valid_0 = in_alloc_valid_0[0];
        out_alloc_valid_1 = in_alloc_valid_0[1];
    end else begin
        out_id_0 = in_id_0[1];
        out_id_1 = in_id_0[0];
        out_alloc_valid_0 = in_alloc_valid_0[1];
        out_alloc_valid_1 = in_alloc_valid_0[0];
    end
end
endmodule

module empty_entry_finder8
#(parameter IO_ID_WIDTH = 3)
(
    input logic [IO_ID_WIDTH-1:0] in_id[8],
    input logic in_alloc_valid[8],
    output logic [IO_ID_WIDTH-1:0] out_id_0,
    output logic [IO_ID_WIDTH-1:0] out_id_1,
    output logic out_alloc_valid_0,
    output logic out_alloc_valid_1
);

logic [IO_ID_WIDTH-1:0] l1_out_id_0[4];
logic [IO_ID_WIDTH-1:0] l1_out_id_1[4];
logic l1_out_valid_0[4]; // The empty probabilty of 0 is bigger than 1
logic l1_out_valid_1[4];

genvar i;
generate
for (i = 0 ; i < 8 ; i = i + 2) begin: l1_finder
    emptry_entry_finder2
    #(.IO_ID_WIDTH(IO_ID_WIDTH))
    l2_finder_2
    (
        .in_id(in_id[i:i+1]),
        .in_alloc_valid(in_alloc_valid[i:i+1]),
        .out_id_0(l1_out_id_0[i/2]),
        .out_id_1(l1_out_id_1[i/2]),
        .out_alloc_valid_0(l1_out_valid_0[i/2]),
        .out_alloc_valid_1(l1_out_valid_1[i/2])
    );
end
endgenerate


logic [IO_ID_WIDTH-1:0] l2_out_id_0[2];
logic [IO_ID_WIDTH-1:0] l2_out_id_1[2];
logic l2_out_valid_0[2]; // The empty probabilty of 0 is bigger than 1
logic l2_out_valid_1[2];


generate
for (i = 0 ; i < 4 ; i = i + 2) begin: l2_finder
    emptry_entry_finder4
    #(.IO_ID_WIDTH(IO_ID_WIDTH))
    l2_finder_4
    (
        .in_id_0(l1_out_id_0[i:i+1]),
        .in_id_1(l1_out_id_1[i:i+1]),
        .in_alloc_valid_0(l1_out_valid_0[i:i+1]),
        .in_alloc_valid_1(l1_out_valid_1[i:i+1]),
        .out_id_0(l2_out_id_0[i/2]),
        .out_id_1(l2_out_id_1[i/2]),
        .out_alloc_valid_0(l2_out_valid_0[i/2]),
        .out_alloc_valid_1(l2_out_valid_1[i/2])
    );
end
endgenerate

emptry_entry_finder4
#(.IO_ID_WIDTH(IO_ID_WIDTH))
l3_finder_4
(
    .in_id_0(l2_out_id_0),
    .in_id_1(l2_out_id_1),
    .in_alloc_valid_0(l2_out_valid_0),
    .in_alloc_valid_1(l2_out_valid_1),
    .out_id_0(out_id_0),
    .out_id_1(out_id_1),
    .out_alloc_valid_0(out_alloc_valid_0),
    .out_alloc_valid_1(out_alloc_valid_1)
);

endmodule
