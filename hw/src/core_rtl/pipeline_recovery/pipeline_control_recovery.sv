`timescale 1ns/1ps
// =============================================================================
//  Program : pipeline_control_recovery.sv
//  Author  : Hon-Chou Dai
//  Date    : May/15/2021
// -----------------------------------------------------------------------------
//  Description:
//  Pipeline controll for Falco
// -----------------------------------------------------------------------------
//  Revision information:
//
//    August/07/2021, by Chun-Wei Chao:
//      Add load/store violation stall & csr instruction stall signal.
//    November/18/2021, by Chun-Wei Chao:
//      Revise recovery mechnism, first cycle flush all instructions need to be flushed and
//      jump to checkpoint, then do rollback recovery.
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

module pipeline_control_recovery(
    input clk,
    input rst,
    output  logic DMEM_access_stall,
    pipeline_control_recovery_io.controller io,
    csr_io.Recovery csr_io
);

logic spec_tag_full;

typedef enum logic [1:0] {  
    RECOVERY_IDLE = 2'b00,
    RECOVERY_FLUSH = 2'b01,
    RECOVERY_RN_ROLL_BACK = 2'b10,
    RECOVERY_FINISH = 2'b11
} Recovery_state_t;
Recovery_state_t recovery_state/*verilator public*/;
Recovery_state_t recovery_next_state;
logic rollback_finished;

typedef enum logic [1:0] {
    REPLAY_IDLE = 2'b00,
    REPLAY_KILL = 2'b01,
    REPLAY_STALL = 2'b10,
    REPLAY_ROUTINE = 2'b11
} Replay_state_t;

Replay_state_t replay_state;
Replay_state_t replay_next_state;

logic [1:0] load_replay_counter;
logic load_consumer_spec_replay;
logic load_consumer_spec_kill;
logic load_consumer_spec_kill_done;

logic SDA_full_delay;

// =========================================================
//  Branch Recovery FSM
// =========================================================

always_ff @(posedge clk) begin
    if (rst)
        recovery_state <= RECOVERY_IDLE;
    else
        recovery_state <= recovery_next_state;
end

always_comb rollback_finished = io.ROB_recovery_finished;

always_comb begin
    case (recovery_state)
        RECOVERY_IDLE: recovery_next_state = ((io.bp_PrMiss || io.store_set_violation) ? RECOVERY_FLUSH : RECOVERY_IDLE);
        RECOVERY_FLUSH: recovery_next_state = RECOVERY_RN_ROLL_BACK;
        RECOVERY_RN_ROLL_BACK: recovery_next_state = (rollback_finished ? RECOVERY_IDLE : RECOVERY_RN_ROLL_BACK);
        default: recovery_next_state = RECOVERY_IDLE;
    endcase
end

// =========================================================
//  Load replay
// =========================================================
always_comb
    load_consumer_spec_kill = io.load_wait_stall ||
                              io.wait_for_non_idempotent;
                              //non_idempotent event would cancell due to replay kill



always_ff @( posedge clk ) begin
    if (rst)
        load_consumer_spec_kill_done <= 0;
    else if (io.recovery_stall)
        load_consumer_spec_kill_done <= load_consumer_spec_kill_done;
    else if (load_consumer_spec_replay)
        load_consumer_spec_kill_done <= 0;
    else if (load_consumer_spec_kill)
        load_consumer_spec_kill_done <= 1;
    else
        load_consumer_spec_kill_done <= load_consumer_spec_kill_done;
end

always_ff @( posedge clk ) begin
    if (rst)
        load_replay_counter <= 3;
    else if (recovery_state == RECOVERY_FLUSH ||
             recovery_state == RECOVERY_RN_ROLL_BACK) //recovery first
        load_replay_counter <= load_replay_counter;
    else if (load_consumer_spec_kill_done) 
        load_replay_counter <= 2;
    else if (load_replay_counter < 3 && ~load_consumer_spec_kill && ~io.replay_muldiv_stall)
        load_replay_counter <= load_replay_counter - 1;
    else if (load_replay_counter == 0 || load_consumer_spec_kill)
        load_replay_counter <= 3;
    else
        load_replay_counter <= load_replay_counter;
end

always_comb
    load_consumer_spec_replay = (load_replay_counter == 2 && ~load_consumer_spec_kill) ||
                                load_replay_counter == 1;

// =========================================================
// load replay control signal
// =========================================================
always_comb
    io.load_depend_replay = load_consumer_spec_replay && ~load_consumer_spec_kill;
always_comb
    io.load_wake_up_predict_failed = load_consumer_spec_kill &&
                                    ~load_consumer_spec_kill_done;
always_comb
    io.load_wake_up_failed_stall = (load_consumer_spec_kill || load_replay_counter == 3) &&
                                   load_consumer_spec_kill_done ;

always_comb
    io.replay_muldiv_stall = io.replay_failed_to_issue_muldiv && io.load_depend_replay;
// =========================================================
// pipeline control signal
// =========================================================
always_comb
    io.pc_stall = io.icache_miss || io.freelist_empty || io.int_iq_full ||
                  io.mem_iq_full || io.rob_full || io.recovery_procedure || io.csr_stall; 

always_comb
    io.IF_stall = io.freelist_empty || io.int_iq_full || io.mem_iq_full ||
                  io.rob_full || io.recovery_procedure || io.csr_stall;
always_comb
    io.IF_flush = (io.bp_PrMiss || io.store_set_violation); // branch miss

always_comb
    io.ID_stall = io.freelist_empty || io.int_iq_full || io.mem_iq_full ||
                  io.rob_full || io.recovery_procedure || io.csr_stall;

always_comb
    io.ID_flush = (io.bp_PrMiss || io.store_set_violation); // branch miss 

always_comb
    io.RNDS_flush = (io.bp_PrMiss || io.store_set_violation);
always_comb
    io.RNDS_stall = io.freelist_empty || io.int_iq_full || io.mem_iq_full ||
                    io.rob_full || io.recovery_procedure;

always_comb
    io.icache_miss_stall_branch = io.icache_miss;

always_ff @(posedge clk) SDA_full_delay <= io.SDA_full;

always_comb
    io.INT_IQ_stall = SDA_full_delay;
always_comb
    io.MEM_IQ_stall = io.SDA_full || io.load_wait_stall || io.wait_for_non_idempotent;
always_comb
    io.rr_mem_stall = io.SDA_full || io.load_wait_stall || io.wait_for_non_idempotent;
always_comb
    io.AGU_stall = io.SDA_full || io.load_wait_stall || io.wait_for_non_idempotent;
always_comb
    io.LSU_stall = io.SDA_full || io.load_wait_stall || io.wait_for_non_idempotent;
always_comb
    io.LSU_non_idempotent_lock = io.wait_for_non_idempotent;
always_comb
    io.MA_stall = io.load_wait_stall || io.wait_for_non_idempotent;


always_comb
    io.recovery_start = (io.bp_PrMiss || io.store_set_violation);
always_comb
    io.recovery_flush = recovery_state == RECOVERY_FLUSH;
 always_comb
    io.recovery_stall = io.recovery_procedure || (io.bp_PrMiss || io.store_set_violation);
always_comb
    io.recovery_procedure = recovery_state == RECOVERY_RN_ROLL_BACK ||
                            recovery_state == RECOVERY_FLUSH;
always_comb
    io.recovery_rollback = recovery_state == RECOVERY_RN_ROLL_BACK;                     

always_comb csr_io.start_recovery = (recovery_next_state == RECOVERY_FLUSH);                            
always_comb csr_io.recovery_procedure = io.recovery_procedure;

always_comb begin
    io.recovery_flush_BCAST.flush_tag_0 = io.flush_rob_tag_0;
    io.recovery_flush_BCAST.flush_tag_1 = io.flush_rob_tag_1;
    io.recovery_flush_BCAST.flush_tag_0_valid = io.flush_rob_tag_0_valid;
    io.recovery_flush_BCAST.flush_tag_1_valid = io.flush_rob_tag_1_valid;
    // io.recovery_flush_BCAST.recovery_flush = io.recovery_procedure || (io.bp_PrMiss || io.store_set_violation);
    io.recovery_flush_BCAST.recovery_flush = io.recovery_flush;
end

always_comb
    DMEM_access_stall = io.recovery_stall || io.SDA_full ||
                        io.load_wait_stall || io.wait_for_non_idempotent;



endmodule

