`timescale 1ns/1ps
// =============================================================================
//  Program : int_possel_replay_unit.sv
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

module int_possel_replay_unit(
    input  logic clk,
    input  logic rst,

    input  branch_flush_t      recovery_flush_BCAST,
    input  logic               load_wake_up_failed_stall,
    input  logic               load_wake_up_predict_failed,
    input  logic               load_depend_replay,
    input  logic               replay_muldiv_stall,

    input  logic               issue_instr0_valid,
    input  logic               issue_instr1_valid,
    input  logic               issue_instr0_poison,
    input  logic               issue_instr1_poison,
    input  int_dispatch_pack_t issue_instr0_pack,
    input  int_dispatch_pack_t issue_instr1_pack,

    output logic               replay_issue_first,
    output logic               wake_up_stall_issue_instr0_valid,
    output logic               wake_up_stall_issue_instr1_valid,
    output int_dispatch_pack_t wake_up_stall_issue_instr0_pack,
    output int_dispatch_pack_t wake_up_stall_issue_instr1_pack,
    output logic               replay_issue_instr0_valid,
    output logic               replay_issue_instr1_valid,
    output int_dispatch_pack_t replay_issue_instr0_pack,
    output int_dispatch_pack_t replay_issue_instr1_pack
);

//========================================
// Replay Buffer
//========================================
logic buffer_idx0_p0_valid;
logic buffer_idx0_p1_valid;
int_dispatch_pack_t replay_buffer_idx0_p0;
int_dispatch_pack_t replay_buffer_idx0_p1;
logic buffer_idx1_p0_valid;
logic buffer_idx1_p1_valid;
int_dispatch_pack_t replay_buffer_idx1_p0;
int_dispatch_pack_t replay_buffer_idx1_p1;
logic buffer_idx0_p0_poison;
logic buffer_idx0_p1_poison;
logic buffer_idx1_p0_poison;
logic buffer_idx1_p1_poison;

//========================================
// Replay Issue logic signal
//========================================
logic replay_idx0_issue_valid;
logic replay_idx1_issue_valid;
int_dispatch_pack_t replay_issue_buffer_idx0;
int_dispatch_pack_t replay_issue_buffer_idx1;
logic wake_up_replay_select;
//===========================================
// replay FIFO logic (issue -> p0 -> p1 -> issue)
//===========================================

always_comb
    replay_issue_first = (buffer_idx0_p0_valid && ~buffer_idx0_p0_poison) ||
                         (buffer_idx0_p1_valid && ~buffer_idx0_p1_poison) ||
                         (buffer_idx1_p0_valid && ~buffer_idx1_p0_poison) ||
                         (buffer_idx1_p1_valid && ~buffer_idx1_p1_poison);

always_ff @(posedge clk)
    if (rst)
        wake_up_replay_select <= 0;
    else if (load_wake_up_predict_failed)
        wake_up_replay_select <= 0;
    else if (recovery_flush_BCAST.recovery_flush)
        wake_up_replay_select <= wake_up_replay_select;
    else if (load_wake_up_failed_stall)
        wake_up_replay_select <= 1;
    else
        wake_up_replay_select <= wake_up_replay_select;

// recovery first

//===========================================
// replay FIFO valid
//===========================================
always_ff @( posedge clk )
    if (rst)
        buffer_idx0_p0_valid <= 0;
    else if (recovery_flush_BCAST.recovery_flush || replay_muldiv_stall)
        buffer_idx0_p0_valid <= IsBrROBKill(recovery_flush_BCAST, 
                                            replay_buffer_idx0_p0.rob_tag) ?
                                            0 : buffer_idx0_p0_valid;
    else if (load_wake_up_failed_stall)
        buffer_idx0_p0_valid <= (wake_up_replay_select == 1 && ~buffer_idx0_p0_poison ) ? 
                                0 : buffer_idx0_p0_valid;
    else if (load_depend_replay)
        buffer_idx0_p0_valid <= 0;
    else
        buffer_idx0_p0_valid <= issue_instr0_valid;

always_ff @( posedge clk )
    if (rst)
        buffer_idx1_p0_valid <= 0;
    else if (recovery_flush_BCAST.recovery_flush || replay_muldiv_stall)
        buffer_idx1_p0_valid <= IsBrROBKill(recovery_flush_BCAST, 
                                            replay_buffer_idx1_p0.rob_tag) ?
                                            0 : buffer_idx1_p0_valid;
    else if (load_wake_up_failed_stall)
        buffer_idx1_p0_valid <= (wake_up_replay_select == 1 && ~buffer_idx1_p0_poison ) ? 
                                0 : buffer_idx1_p0_valid;
    else if (load_depend_replay)
        buffer_idx1_p0_valid <= 0;
    else
        buffer_idx1_p0_valid <= issue_instr1_valid;

always_ff @( posedge clk )
    if (rst)
        buffer_idx0_p1_valid <= 0;
    else if (recovery_flush_BCAST.recovery_flush || replay_muldiv_stall)
        buffer_idx0_p1_valid <= IsBrROBKill(recovery_flush_BCAST, 
                                            replay_buffer_idx0_p1.rob_tag) ?
                                            0 : buffer_idx0_p1_valid;
    else if (load_wake_up_failed_stall)
        buffer_idx0_p1_valid <= (wake_up_replay_select == 0 && ~buffer_idx0_p1_poison ) ?
                                0 : buffer_idx0_p1_valid;
    else
        buffer_idx0_p1_valid <= buffer_idx0_p0_valid;

always_ff @( posedge clk )
    if (rst)
        buffer_idx1_p1_valid <= 0;
    else if (recovery_flush_BCAST.recovery_flush || replay_muldiv_stall)
        buffer_idx1_p1_valid <= IsBrROBKill(recovery_flush_BCAST, 
                                            replay_buffer_idx1_p0.rob_tag) ?
                                            0 : buffer_idx1_p1_valid;
    else if (load_wake_up_failed_stall)
        buffer_idx1_p1_valid <= (wake_up_replay_select == 0 && ~buffer_idx1_p1_poison ) ?
                                0 : buffer_idx1_p1_valid;
    else
        buffer_idx1_p1_valid <= buffer_idx1_p0_valid;

//===========================================
// replay FIFO poison
//===========================================

always_ff @( posedge clk )
    if (rst)
        buffer_idx0_p0_poison <= 0;
    else if (load_wake_up_failed_stall || recovery_flush_BCAST.recovery_flush || replay_muldiv_stall) 
        buffer_idx0_p0_poison <= buffer_idx0_p0_poison;
    else
        buffer_idx0_p0_poison <= issue_instr0_poison;

always_ff @( posedge clk )
    if (rst)
        buffer_idx1_p0_poison <= 0;
    else if (load_wake_up_failed_stall || recovery_flush_BCAST.recovery_flush || replay_muldiv_stall)
        buffer_idx1_p0_poison <= buffer_idx1_p0_poison;
    else
        buffer_idx1_p0_poison <= issue_instr1_poison;

always_ff @( posedge clk )
    if (rst)
        buffer_idx0_p1_poison <= 0;
    else if (load_wake_up_failed_stall || recovery_flush_BCAST.recovery_flush || replay_muldiv_stall)
        buffer_idx0_p1_poison <= buffer_idx0_p1_poison;
    else
        buffer_idx0_p1_poison <= buffer_idx0_p0_poison;


always_ff @( posedge clk )
    if (rst)
        buffer_idx1_p1_poison <= 0;
    else if (load_wake_up_failed_stall || recovery_flush_BCAST.recovery_flush || replay_muldiv_stall)
        buffer_idx1_p1_poison <= buffer_idx1_p1_poison;
    else
        buffer_idx1_p1_poison <= buffer_idx1_p0_poison;

//===========================================
// replay FIFO instruction payload
//===========================================
always_ff @( posedge clk )
    if (rst)
        replay_buffer_idx0_p0 <= 0;
    else if (load_wake_up_failed_stall || recovery_flush_BCAST.recovery_flush || replay_muldiv_stall)
        replay_buffer_idx0_p0 <= replay_buffer_idx0_p0;
    else
        replay_buffer_idx0_p0 <= issue_instr0_pack;

always_ff @( posedge clk )
    if (rst)
        replay_buffer_idx1_p0 <= 0;
    else if (load_wake_up_failed_stall || recovery_flush_BCAST.recovery_flush || replay_muldiv_stall)
        replay_buffer_idx1_p0 <= replay_buffer_idx1_p0;
    else
        replay_buffer_idx1_p0 <= issue_instr1_pack;

always_ff @( posedge clk )
    if (rst)
        replay_buffer_idx0_p1 <= 0;
    else if (load_wake_up_failed_stall || recovery_flush_BCAST.recovery_flush || replay_muldiv_stall)
        replay_buffer_idx0_p1 <= replay_buffer_idx0_p1;
    else
        replay_buffer_idx0_p1 <= replay_buffer_idx0_p0;

always_ff @( posedge clk )
    if (rst)
        replay_buffer_idx1_p1 <= 0;
    else if (load_wake_up_failed_stall || recovery_flush_BCAST.recovery_flush || replay_muldiv_stall)
        replay_buffer_idx1_p1 <= replay_buffer_idx1_p1;
    else
        replay_buffer_idx1_p1 <= replay_buffer_idx1_p0;

//========================================
// Replay output logic signal
//========================================
always_comb replay_issue_instr0_valid = buffer_idx0_p1_valid;
always_comb replay_issue_instr1_valid = buffer_idx1_p1_valid;
always_comb replay_issue_instr0_pack = replay_buffer_idx0_p1;
always_comb replay_issue_instr1_pack = replay_buffer_idx1_p1;
//========================================
// wake up stall issue logic signal
//========================================
// TODO: should we care muldiv busy signal?

always_comb begin 
    if (wake_up_replay_select == 0) begin
        wake_up_stall_issue_instr0_pack = replay_buffer_idx0_p1;
    end else begin
        wake_up_stall_issue_instr0_pack = replay_buffer_idx0_p0;
    end
end

always_comb begin 
    if (wake_up_replay_select == 0) begin
        wake_up_stall_issue_instr1_pack = replay_buffer_idx1_p1;
    end else begin
        wake_up_stall_issue_instr1_pack = replay_buffer_idx1_p0;
    end
end

always_comb begin 
    if (wake_up_replay_select == 0) begin
        wake_up_stall_issue_instr0_valid = buffer_idx0_p1_valid && ~buffer_idx0_p1_poison;
    end else begin
        wake_up_stall_issue_instr0_valid = buffer_idx0_p0_valid && ~buffer_idx0_p0_poison;
    end
end

always_comb begin 
    if (wake_up_replay_select == 0) begin
        wake_up_stall_issue_instr1_valid = buffer_idx1_p1_valid && ~buffer_idx1_p1_poison;
    end else begin
        wake_up_stall_issue_instr1_valid = buffer_idx1_p0_valid && ~buffer_idx1_p0_poison;
    end
end

endmodule
