`timescale 1ns/1ps
// =============================================================================
//  Program : mem_replay_unit.sv
//  Author  : Hon-Chou Dai
//  Date    : May/15/2021
// -----------------------------------------------------------------------------
//  Description:
//  Memory replay unit for Falco.
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

module  mem_replay_unit (
    input  logic clk,
    input  logic rst,

    input  branch_flush_t      recovery_flush_BCAST,
    input  logic               recovery_stall,
    input  logic               load_wake_up_failed_stall,
    input  logic               load_wake_up_predict_failed,
    input  logic               load_depend_replay,
    input  logic               replay_muldiv_stall,

    input  logic               issue_instr_valid,
    input  mem_dispatch_pack_t issue_instr_pack,

    output logic               replay_issue_instr_valid,
    output mem_dispatch_pack_t replay_issue_instr_pack
);

//========================================
// Replay Buffer
//========================================
logic buffer_p0_valid;
logic buffer_p1_valid;
mem_dispatch_pack_t replay_buffer_p0;
mem_dispatch_pack_t replay_buffer_p1;

//===========================================
// replay FIFO valid
//===========================================
always_ff @( posedge clk )
    if (rst)
        buffer_p0_valid <= 0;
    else if (recovery_stall || replay_muldiv_stall)
        buffer_p0_valid <= IsBrROBKill(recovery_flush_BCAST, 
                                            replay_buffer_p0.rob_tag) ?
                                            0 : buffer_p0_valid;
    else if (load_wake_up_failed_stall)
        buffer_p0_valid <= buffer_p0_valid;
    //else if (load_depend_replay)
    //    buffer_p0_valid <= 0;
    else
        buffer_p0_valid <= issue_instr_valid;


always_ff @( posedge clk )
    if (rst)
        buffer_p1_valid <= 0;
    else if (recovery_stall || replay_muldiv_stall)
        buffer_p1_valid <= IsBrROBKill(recovery_flush_BCAST, 
                                            replay_buffer_p1.rob_tag) ?
                                            0 : buffer_p1_valid;
    else if (load_wake_up_failed_stall)
        buffer_p1_valid <= buffer_p1_valid;
    else
        buffer_p1_valid <= buffer_p0_valid;

//===========================================
// replay FIFO instruction payload
//===========================================
always_ff @( posedge clk )
    if (rst)
        replay_buffer_p0 <= 0;
    else if (load_wake_up_failed_stall || recovery_stall || replay_muldiv_stall)
        replay_buffer_p0 <= replay_buffer_p0;
    else
        replay_buffer_p0 <= issue_instr_pack;


always_ff @( posedge clk )
    if (rst)
        replay_buffer_p1 <= 0;
    else if (load_wake_up_failed_stall || recovery_stall || replay_muldiv_stall)
        replay_buffer_p1 <= replay_buffer_p1;
    else
        replay_buffer_p1 <= replay_buffer_p0;


//========================================
// Replay output logic signal
//========================================
always_comb replay_issue_instr_valid = buffer_p1_valid;
always_comb replay_issue_instr_pack = replay_buffer_p1;

endmodule
