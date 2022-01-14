`timescale 1ns/1ps
// =============================================================================
//  Program : empty_entry_finder8_wrapper.sv
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

module empty_entry_finder8_wrapper(
    input  logic [2:0] dispatch_slot_idx0,
    input  logic [2:0] dispatch_slot_idx1,
    input  logic       dispatch_slot_idx0_valid,
    input  logic       dispatch_slot_idx1_valid,
    input  logic       slot_valid[8],

    output logic [2:0] allocatable_slot_idx0,
    output logic [2:0] allocatable_slot_idx1,
    output logic       allocatable_slot_idx0_valid,
    output logic       allocatable_slot_idx1_valid
);

logic allocate_valid[8];
logic [2:0] entry_id[8];

genvar geni;
generate
    
    for (geni = 0 ; geni < 8 ; geni = geni + 1)
        always_comb entry_id[geni] = geni;

    for (geni = 0 ; geni < 8 ; geni = geni + 1) begin
        always_comb begin
            allocate_valid[geni] = ~slot_valid[geni] &&
                                ~(geni == dispatch_slot_idx0 && dispatch_slot_idx0_valid) &&
                                ~(geni == dispatch_slot_idx1 && dispatch_slot_idx1_valid);
        end
    end
endgenerate
// integer i;

// always_comb
// 	for (i = 0 ; i < 8 ; i = i + 1)
// 		entry_id[i] = i;

// always_comb begin
//     for (i = 0 ; i < 8 ; i = i + 1) begin
//         allocate_valid[i] = ~slot_valid[i] &&
//                             ~(i == dispatch_slot_idx0 && dispatch_slot_idx0_valid) &&
//                             ~(i == dispatch_slot_idx1 && dispatch_slot_idx1_valid);
//     end
// end



empty_entry_finder8 #(.IO_ID_WIDTH(3)) empty_entry_finder8_inst
(
    .in_id(entry_id),
    .in_alloc_valid(allocate_valid),
    .out_id_0(allocatable_slot_idx0),
    .out_id_1(allocatable_slot_idx1),
    .out_alloc_valid_0(allocatable_slot_idx0_valid),
    .out_alloc_valid_1(allocatable_slot_idx1_valid)
);

endmodule
