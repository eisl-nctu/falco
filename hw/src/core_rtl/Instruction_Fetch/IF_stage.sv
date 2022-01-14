`timescale 1ns/1ps
// =============================================================================
//  Program : IF_stage.sv
//  Author  : Hon-Chou Dai
//  Date    : May/15/2021
// -----------------------------------------------------------------------------
//  Description:
//  Instruction Fetch stage for Falco.
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
import L1_cache_pkg::*;

module IF_stage(
    input                       clk,
    input                       rst,

    //From Core top I/O port
    input   [XLEN_WIDTH-1:0]    init_pc,
    output  instruction_req_t   instruction_req,
    input   instruction_resp_t  instruction_resp,

    // To ID stage
    output  pc_t                instr0_predict_pc,      //combinational logic for ID stage predict pc
    output  pc_t                instr1_predict_pc,

    IF_stage_io.IF_stage        if_stage_io,
    exe_stage_io.IF_stage       exe_io,
    load_store_unit_io.IF_stage lsu_io,
    pipeline_control_recovery_io.IF_stage pipe_ctrl_io,
    commit_stage_io.IF_stage commit_io
);

// PC stage program_counter
(* mark_debug = "true"*) pc_t instr0_pc;
(* mark_debug = "true"*) pc_t instr1_pc;

// PC stage: handle cross cache line fetch event
logic pc_instr1_valid;

//Branch predict logic
logic instr0_btb_hit;
logic instr1_btb_hit;
pc_t instr0_btb_target_addr;
pc_t instr1_btb_target_addr;
pc_t next_predict_pc;
logic predict_taken;
logic instr0_predict_taken;
logic instr1_predict_taken;
// BHSR register: pass to pipeline and use to recovery BHSR signal
BHSR_t current_instr0_BHSR;
BHSR_t current_instr1_BHSR;


logic PrMiss_dly;

program_counter PC(
    .clk(clk),
    .rst(rst),

    .stall(pipe_ctrl_io.pc_stall),
    .init_pc(init_pc),

    .exe_branch_target_addr(exe_io.branch_result.target_addr),
    .exe_branch_PrMiss(exe_io.branch_result.is_misprediction && exe_io.branch_result.valid),
    .predict_pc(next_predict_pc),
    .predict_taken(predict_taken && ~PrMiss_dly),
    .jump_direct(),
    .jump_relative(),
    .sys_pc_oper(),

    .store_set_violation(lsu_io.store_set_violation),
    .store_set_pc(lsu_io.violation_pc),
    .branch_miss_first(commit_io.branch_miss_first),

    .pc(instr0_pc),
    .instr1_valid(pc_instr1_valid)
);

// handle branch misprediction in branch compute next cycle to prevent critcial path
always_ff @(posedge clk) begin
    if (rst) begin
        PrMiss_dly <= 0;
    end else begin
        PrMiss_dly <= exe_io.branch_result.is_misprediction;
    end
end

always_comb
    instr1_pc = instr0_pc + 4;

always_comb
    pipe_ctrl_io.icache_miss = ~instruction_resp.ready;

/////
BTB Branch_target_buffer(
    .clk(clk),
    .rst(rst),

    .IF_instr0_pc(instr0_pc),
    .IF_instr1_pc(instr1_pc),
    .branch_valid(exe_io.branch_result.valid),
    .branch_taken(exe_io.branch_result.branch_taken),
    .branch_addr(exe_io.branch_result.branch_addr),
    .branch_target_addr(exe_io.branch_result.target_addr),

    .instr0_btb_hit(instr0_btb_hit),
    .instr0_btb_target_addr(instr0_btb_target_addr),
    .instr1_btb_hit(instr1_btb_hit),
    .instr1_btb_target_addr(instr1_btb_target_addr)
);

// `define USE_BIMODAL 1

`ifdef USE_BIMODAL

bimodal_branch_predictor branch_predictor(
    .clk(clk),
    .rst(rst),

    .IF_instr0_pc(instr0_pc),
    .IF_instr1_pc(instr1_pc),
    .IF_instr0_btb_hit(instr0_btb_hit ),
    .IF_instr1_btb_hit(instr1_btb_hit && pc_instr1_valid), //critcial path??
    .cache_hit(instruction_resp.instr0_valid),
    .exe_cond_valid(exe_io.branch_result.valid),
    .exe_cond_branch_taken(exe_io.branch_result.branch_taken),
    .exe_cond_branch_addr(exe_io.branch_result.branch_addr),
    .exe_cond_miss_prediction(exe_io.branch_result.is_misprediction && exe_io.branch_result.valid),
    .instr0_branch_predict_taken(instr0_predict_taken),
    .instr1_branch_predict_taken(instr1_predict_taken)
);

`else
gshare_branch_predictor branch_predictor(
    .clk(clk),
    .rst(rst),

    .IF_instr0_pc(instr0_pc),
    .IF_instr1_pc(instr1_pc),
    .IF_instr0_btb_hit(instr0_btb_hit ),
    .IF_instr1_btb_hit(instr1_btb_hit && pc_instr1_valid), //critcial path??
    .cache_hit(instruction_resp.instr0_valid),
    .exe_cond_valid(exe_io.branch_result.valid),
    .exe_cond_bhsr(exe_io.branch_result.bhsr),
    .exe_cond_branch_taken(exe_io.branch_result.branch_taken),
    .exe_cond_branch_addr(exe_io.branch_result.branch_addr),
    .exe_cond_miss_prediction(exe_io.branch_result.is_misprediction && exe_io.branch_result.valid),
    .instr0_branch_predict_taken(instr0_predict_taken),
    .instr1_branch_predict_taken(instr1_predict_taken),
    .current_instr0_BHSR(current_instr0_BHSR),
    .current_instr1_BHSR(current_instr1_BHSR),

    .store_set_violation(lsu_io.store_set_violation),
    .branch_miss_first(commit_io.branch_miss_first),
    .lsu_bhsr(lsu_io.violation_bhsr)
);
`endif

always_comb
    predict_taken = (instr0_btb_hit && instr0_predict_taken) ||
                    (instr1_btb_hit && instr1_predict_taken && pc_instr1_valid);

always_comb
    if (instr0_btb_hit && instr0_predict_taken)
        next_predict_pc = instr0_btb_target_addr;
    else
        next_predict_pc = instr1_btb_target_addr;

/*
instr0 hit and taken -> instr1 flush and pc go to instr0 target addr
instr0 hit and not taken -> instr1 hit and taken -> pc go to instr1 target addr
instr0 hit and not taken -> instr1 hit and taken -> pc go to instr0 + 8
instr0 not hit -> instr1 hit and taken -> pc go to instr1 taget addr
instr0 not hit -> instr1 hit and not taken -> pc go to instr0 + 8
instr0 not hit -> instr1 not hit -> pc got to instr0 + 8
*/

// ==================================================
// IF stage stall logic
// ==================================================
raw_instruction_t instr0_delay;
raw_instruction_t instr1_delay;
logic IF_stall_delay;
logic IF_flush_delay;

always_ff @(posedge clk)
    if (rst)
        instruction_req.p_strobe <= 1; // better way to start fetch ?
    else if (pipe_ctrl_io.pc_stall /*~instruction_resp.ready*/)
        instruction_req.p_strobe <= 0;
    else
        instruction_req.p_strobe <= 1;

always_comb
    instruction_req.instr0_addr = instr0_pc;

always_comb
    instruction_req.instr1_addr = instr1_pc;

always_comb
    instr1_predict_pc = instr0_pc;

always_ff @(posedge clk) begin
    if (rst)
        instr0_predict_pc <= 0;
    else if (pipe_ctrl_io.IF_flush)
        instr0_predict_pc <= 0;
    else if (pipe_ctrl_io.IF_stall)
        instr0_predict_pc <= instr0_predict_pc;
    else
        instr0_predict_pc <= (instr0_btb_hit && instr0_predict_taken) ?
                        instr0_btb_target_addr :
                        instr1_pc;
end

always_ff @(posedge clk) begin
    if (rst) begin
        IF_stall_delay <= 0;
        IF_flush_delay <= 0;
    end else begin
        IF_stall_delay <= pipe_ctrl_io.IF_stall;
        IF_flush_delay <= pipe_ctrl_io.IF_flush;
    end
end

always_ff @(posedge clk) begin
    if (rst) begin
        instr0_delay <= 0;
        instr1_delay <= 0;
    end else if (IF_stall_delay) begin
        instr0_delay <= instr0_delay;
        instr1_delay <= instr1_delay;
    end else begin
        instr0_delay <= instruction_resp.raw_instr0;
        instr1_delay <= instruction_resp.raw_instr1;
    end
end

always_comb begin
    if (rst) begin
        if_stage_io.raw_instr0 = INSTRUCTION_NOP;
        if_stage_io.raw_instr1 = INSTRUCTION_NOP;
    end else if (IF_flush_delay) begin
        if_stage_io.raw_instr0 = INSTRUCTION_NOP;
        if_stage_io.raw_instr1 = INSTRUCTION_NOP;
    end else if (IF_stall_delay) begin
        if_stage_io.raw_instr0 = instr0_delay;
        if_stage_io.raw_instr1 = instr1_delay;
    end else begin
        if_stage_io.raw_instr0 = instruction_resp.raw_instr0;
        if_stage_io.raw_instr1 = instruction_resp.raw_instr1;
    end
end

always_ff @(posedge clk) begin
    if (rst) begin
        if_stage_io.instr0_valid <= 0;
        if_stage_io.instr0_pc <= 32'h00000000;
        if_stage_io.current_instr0_BHSR <= 0;
    end else if (pipe_ctrl_io.IF_flush) begin
        if_stage_io.instr0_valid <= 0;
        if_stage_io.instr0_pc <= instr0_pc;
        if_stage_io.current_instr0_BHSR <= 0;
    end else if (pipe_ctrl_io.IF_stall) begin
        if_stage_io.instr0_valid <= if_stage_io.instr0_valid;
        if_stage_io.instr0_pc <= if_stage_io.instr0_pc;
        if_stage_io.current_instr0_BHSR <= if_stage_io.current_instr0_BHSR;
    end else begin
        if_stage_io.instr0_valid <= instruction_resp.instr0_valid;
        if_stage_io.instr0_pc <= instr0_pc;
        if_stage_io.current_instr0_BHSR <= current_instr0_BHSR;
    end
end


always_ff @(posedge clk) begin
    if (rst) begin
        if_stage_io.instr1_valid <= 0;
        if_stage_io.instr1_pc <= 32'h00000000;
        if_stage_io.current_instr1_BHSR <= 0;
    end else if (pipe_ctrl_io.IF_flush) begin
        if_stage_io.instr1_valid <= 0;
        if_stage_io.instr1_pc <= instr1_pc;
        if_stage_io.current_instr1_BHSR <= 0;
    end else if (pipe_ctrl_io.IF_stall) begin
        if_stage_io.instr1_valid <= if_stage_io.instr1_valid;
        if_stage_io.instr1_pc <= if_stage_io.instr1_pc;
        if_stage_io.current_instr1_BHSR <= if_stage_io.current_instr0_BHSR;
    end else begin
        if_stage_io.instr1_valid <= instruction_resp.instr1_valid && ~(instr0_btb_hit && instr0_predict_taken);
        if_stage_io.instr1_pc <= instr1_pc;
        if_stage_io.current_instr1_BHSR <= current_instr0_BHSR;
    end
end


endmodule
