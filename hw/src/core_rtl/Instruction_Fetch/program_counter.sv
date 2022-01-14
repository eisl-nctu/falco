`timescale 1ns/1ps
// =============================================================================
//  Program : program_counter.sv
//  Author  : Hon-Chou Dai
//  Date    : May/15/2021
// -----------------------------------------------------------------------------
//  Description:
//  Program counter for Falco.
// -----------------------------------------------------------------------------
//  Revision information:
//
//    August/07/2021, by Chun-Wei Chao:
//      Add store_set_violation signal in input, when recovery start, 
//      according store_set_violation or exe_branch_PrMiss signal 
//      jump to appropriate PC 
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

module program_counter(
    input                       clk,
    input                       rst,

    // pipeline stall signal
    input                       stall,

    // Core reset initial pc
    input   pc_t                init_pc,

    //EXE branch result update signal (if misprediction)
    input   pc_t                exe_branch_target_addr,
    input                       exe_branch_PrMiss, //miss , need update pc

    //IF stage branch prediction
    input   pc_t                predict_pc,
    input                       predict_taken,

    // current unused signal
    input   jump_direct_t       jump_direct,
    input   jump_relative_t     jump_relative,
    input   sys_pc_operation_t  sys_pc_oper,

    input                       store_set_violation,
    input                       branch_miss_first,
    input   pc_t                store_set_pc,

    // PC stage pipeline register output
    output  pc_t pc /*verilator public*/,
    output  logic instr1_valid
);

logic cross_line;
assign cross_line = CrossICacheLineFetch(pc);


always_ff@(posedge clk) begin
    if (rst) begin
        pc <= init_pc;
    end else if (store_set_violation && ~branch_miss_first) begin
        pc <= store_set_pc;
    end else if (exe_branch_PrMiss) begin
        pc <= exe_branch_target_addr; //update pc to correct 1
    end else if (stall) begin
        pc <= pc;
    end else if (predict_taken) begin
        pc <= predict_pc;
    end else if (cross_line)begin
        pc <= pc + 4;
    end else begin
        pc <= pc + 8;
    end
end

always_comb
    instr1_valid = cross_line ? 0 : 1;

endmodule
