`timescale 1ns/1ps
// =============================================================================
//  Program : load_store_unit.sv
//  Author  : Hon-Chou Dai
//  Date    : May/15/2021
// -----------------------------------------------------------------------------
//  Description:
//  Load instruction execute in this stage.
//  Store instruction pass through this stage and store in store_buffer.
// -----------------------------------------------------------------------------
//  Revision information:
//
//    August/07/2021, by Chun-Wei Chao:
//      Tranfer load/store violation signal to other stage.
//      There are two situation need to recovery, one is load/store violation,
//      other is speculative load and it's address prefix is 'C',
//      because load address which is 0xCXXXXXXX, should not execute speculatively.
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

// ==========================================
// load_store_unit stage (Disambiguation  stage)
// ==========================================
// Load operation:
//      1. trigger load forward checking
//      2. trigger L1 DCache Tag Access (TODO)
// Store operation:
//      1. allocate store data to store buffer
//      2. store commit

module load_store_unit(
    input       clk,
    input       rst,

    // Load check data hit port
    output  core_load_ck_hit_req_t  load_ck_hit_req,
    input   core_load_hit_resp_t    load_hit_resp,

    AGU_io.LSU agu_io,
    pipeline_control_recovery_io.LSU pipe_ctrl_io,
    store_buffer_io.LSU sdb_io,
    load_buffer_io.LSU ldb_io,
    load_store_unit_io.LSU lsu_io
);

//stall memory pipeline if SDB full, so it dosen't need to check allocatable
logic store_commit_valid;
logic load_commit_valid;

xlen_data_t align_access_addr /*verilator public*/;
logic [1:0] addr_alignment;

logic LSU_kill;
logic LSU_stall;

logic violation /*verilator public*/;

`ifdef FALCO_SIM_DEBUG
logic dviolation /*verilator public*/; // use to debug
always_comb dviolation = lsu_io.device_violation;

BHSR_t LSU_debug_bhsr /*verilator public*/;
always_comb LSU_debug_bhsr = agu_io.agu_pack.bhsr;
`endif

always_comb LSU_kill = IsBrROBKill(pipe_ctrl_io.recovery_flush_BCAST, agu_io.agu_pack.rob_tag);


always_comb store_commit_valid = agu_io.instr_valid &&
                                 agu_io.agu_pack.mem_is_store && 
                                 ~pipe_ctrl_io.LSU_stall;
always_comb load_commit_valid = agu_io.instr_valid &&
                                ~agu_io.agu_pack.mem_is_store &&
                                ~pipe_ctrl_io.LSU_stall &&
                                (sdb_io.sb_load_forward_hit || load_hit_resp.load_hit);

always_comb align_access_addr = {agu_io.agu_pack.access_addr[XLEN_WIDTH-1:2],2'b0};
always_comb addr_alignment = agu_io.agu_pack.access_addr[2:0];

always_comb violation = 
    (lsu_io.device_violation || (ldb_io.violation_detect && agu_io.agu_pack.mem_is_store)) 
    && ~pipe_ctrl_io.recovery_procedure && agu_io.instr_valid;

always_comb lsu_io.store_set_rob_tag = agu_io.agu_pack.rob_tag;
always_comb lsu_io.store_set_violation = violation;

always_comb lsu_io.violation_pc = agu_io.agu_pack.pc;
always_comb lsu_io.violation_bhsr = agu_io.agu_pack.bhsr;
always_comb lsu_io.violation_store_pc = agu_io.agu_pack.store_set_pc;
always_comb lsu_io.violation_load_pc = ldb_io.violation_load_pc;
always_comb lsu_io.violation_store_id = agu_io.agu_pack.store_set_id;
always_comb lsu_io.violation_load_id = ldb_io.violation_load_id;
always_comb lsu_io.device_violation = 
    ((agu_io.agu_pack.access_addr[XLEN_WIDTH-1:XLEN_WIDTH-4] == 4'hC) && agu_io.agu_pack.predict_no_vilation && ~agu_io.agu_pack.mem_is_store) 
    && ~pipe_ctrl_io.recovery_procedure && agu_io.instr_valid;


always_comb begin
    sdb_io.instr_valid = agu_io.instr_valid && ~pipe_ctrl_io.LSU_stall &&
                         ~LSU_kill &&
                          ~pipe_ctrl_io.recovery_stall;
    sdb_io.store_allocate = agu_io.agu_pack.mem_is_store;
    sdb_io.store_allocate_addr = align_access_addr;
    sdb_io.store_allocate_data = agu_io.agu_pack.store_data;
    sdb_io.store_allocate_mask = agu_io.agu_pack.byte_sel;
    sdb_io.store_non_idempotent_region = agu_io.agu_pack.is_non_idempotent;
    sdb_io.store_allocate_rob_tag = agu_io.agu_pack.rob_tag;
`ifdef FALCO_SIM_DEBUG
    sdb_io.store_pc = agu_io.agu_pack.pc;
`endif
    //load forward logic
    sdb_io.sb_load_find_addr = align_access_addr;
    sdb_io.sb_load_find_mask = agu_io.agu_pack.byte_sel;

    //load_io logic
    ldb_io.instr_valid = agu_io.instr_valid && ~pipe_ctrl_io.LSU_stall &&
                         ~LSU_kill &&
                          ~pipe_ctrl_io.recovery_stall;
    ldb_io.load_allocate = ~agu_io.agu_pack.mem_is_store;
    ldb_io.load_allocate_addr = align_access_addr;
    ldb_io.load_allocate_mask = agu_io.agu_pack.byte_sel;
    ldb_io.load_allocate_rob_tag = agu_io.agu_pack.rob_tag;
    ldb_io.load_store_set_pc = agu_io.agu_pack.store_set_pc;
    ldb_io.load_store_set_id = agu_io.agu_pack.store_set_id;
    ldb_io.store_instruction = agu_io.agu_pack.mem_is_store;
    ldb_io.store_addr = align_access_addr;
    ldb_io.store_rob_tag = agu_io.agu_pack.rob_tag;
`ifdef FALCO_SIM_DEBUG
    ldb_io.load_pc = agu_io.agu_pack.pc;
`endif
end

// Load req logic
always_comb begin
    load_ck_hit_req.load_req  = agu_io.instr_valid && (~agu_io.agu_pack.mem_is_store && ~lsu_io.device_violation) &&
                                 ~pipe_ctrl_io.LSU_non_idempotent_lock;
    load_ck_hit_req.load_kill = agu_io.instr_valid && ~agu_io.agu_pack.mem_is_store &&
                                LSU_kill;
    load_ck_hit_req.load_addr = align_access_addr;
end


// Load pipeline register
always_ff @(posedge clk) begin
    if (rst) begin
        lsu_io.lsu_pack.load_instr_valid <= 0;
        lsu_io.lsu_pack.load_forward_data_valid <= 0;
        lsu_io.lsu_pack.load_forward_data <= 0;
        lsu_io.lsu_pack.load_mem_ready <= 0;
        lsu_io.lsu_pack.load_addr_alignment <= 0;
        lsu_io.lsu_pack.load_byte_sel <= 0;
        lsu_io.lsu_pack.load_mem_input_sel <= MEM_OP_BYTE;
        lsu_io.lsu_pack.rd_addr <= 0;
        lsu_io.lsu_pack.mem_load_ext_sel <= 0;
        lsu_io.lsu_pack.rob_tag <= 0;
`ifdef FALCO_SIM_DEBUG
        lsu_io.lsu_pack.pc <= 0;
`endif
    end else if (pipe_ctrl_io.LSU_stall || pipe_ctrl_io.recovery_stall) begin
        lsu_io.lsu_pack.load_instr_valid <= 
            IsBrROBKill(pipe_ctrl_io.recovery_flush_BCAST, lsu_io.lsu_pack.rob_tag) 
                ? 0 : lsu_io.lsu_pack.load_instr_valid;
        lsu_io.lsu_pack.load_forward_data_valid <= lsu_io.lsu_pack.load_forward_data_valid;
        lsu_io.lsu_pack.load_forward_data <= lsu_io.lsu_pack.load_forward_data;
        lsu_io.lsu_pack.load_mem_ready <= lsu_io.lsu_pack.load_mem_ready;
        lsu_io.lsu_pack.load_addr_alignment <= lsu_io.lsu_pack.load_addr_alignment;
        lsu_io.lsu_pack.load_byte_sel <= lsu_io.lsu_pack.load_byte_sel;
        lsu_io.lsu_pack.mem_load_ext_sel <= lsu_io.lsu_pack.mem_load_ext_sel;
        lsu_io.lsu_pack.load_mem_input_sel <= lsu_io.lsu_pack.load_mem_input_sel;
        lsu_io.lsu_pack.rd_addr <= lsu_io.lsu_pack.rd_addr;
        lsu_io.lsu_pack.rob_tag <= lsu_io.lsu_pack.rob_tag;
`ifdef FALCO_SIM_DEBUG
        lsu_io.lsu_pack.pc <= lsu_io.lsu_pack.pc;
`endif
    end else begin
        lsu_io.lsu_pack.load_instr_valid <= agu_io.instr_valid &&
                                            ~agu_io.agu_pack.mem_is_store;
        lsu_io.lsu_pack.load_forward_data_valid <= sdb_io.sb_load_forward_hit;
        lsu_io.lsu_pack.load_forward_data <= sdb_io.sb_load_forward_data;
        lsu_io.lsu_pack.load_forward_hit_mask <= sdb_io.sb_load_forward_mask;
        lsu_io.lsu_pack.load_mem_ready <= load_hit_resp.load_hit;
        lsu_io.lsu_pack.load_addr_alignment <= addr_alignment;
        lsu_io.lsu_pack.load_byte_sel <= agu_io.agu_pack.byte_sel;
        lsu_io.lsu_pack.mem_load_ext_sel <= agu_io.agu_pack.mem_load_ext_sel;
        lsu_io.lsu_pack.load_mem_input_sel <= agu_io.agu_pack.mem_input_sel;
        lsu_io.lsu_pack.rd_addr <= agu_io.agu_pack.rd_addr;
        lsu_io.lsu_pack.rob_tag <= agu_io.agu_pack.rob_tag;
`ifdef FALCO_SIM_DEBUG
        lsu_io.lsu_pack.pc <= agu_io.agu_pack.pc;
`endif
    end
end

always_comb lsu_io.cur_lsu_rob_tag = agu_io.agu_pack.rob_tag;

// Load consumer wakeup logic
always_ff @(posedge clk) begin
    if (rst) begin
        lsu_io.BCAST_load.valid <= 0;
        lsu_io.BCAST_load.prf_addr <= 0;
    end else if (pipe_ctrl_io.recovery_stall) begin //force redo
        lsu_io.BCAST_load.valid <= 0;
        lsu_io.BCAST_load.prf_addr <= 0;
    end else begin
        lsu_io.BCAST_load.valid <= load_commit_valid; //wake up early
        lsu_io.BCAST_load.prf_addr <= agu_io.agu_pack.rd_addr;
    end
end

//commit logic
always_ff @(posedge clk) begin
    if (rst) begin
        lsu_io.LSU_commit.valid <= 0;
        lsu_io.LSU_commit.commit_addr <= 0;
    end else if (pipe_ctrl_io.recovery_stall) begin
        lsu_io.LSU_commit.valid <= 0;
        lsu_io.LSU_commit.commit_addr <= 0;
    end else begin
        lsu_io.LSU_commit.valid <= store_commit_valid;
        lsu_io.LSU_commit.commit_addr <= agu_io.agu_pack.rob_tag;
    end
end

//pipeline ctrl logic
always_comb
    pipe_ctrl_io.load_wait_stall = agu_io.instr_valid &&
                                   ~agu_io.agu_pack.mem_is_store &&
                                   (load_hit_resp.load_miss && ~sdb_io.sb_load_forward_hit);

always_comb pipe_ctrl_io.store_set_violation = violation;

`ifdef FALCO_SIM_DEBUG
    function logic [7:0] ver_lsu_is_store;
        /*verilator public*/
        ver_lsu_is_store = agu_io.agu_pack.mem_is_store ? 1 : 0;
    endfunction
    function logic [31:0] ver_lsu_pc;
        /*verilator public*/
        ver_lsu_pc = lsu_io.lsu_pack.pc;
    endfunction
    function logic [7:0] ver_lsu_commit;
        /*verilator public*/
        ver_lsu_commit = lsu_io.LSU_commit.valid;
    endfunction
    function logic [7:0] ver_lsu_load_mask;
        /*verilator public*/
        ver_lsu_load_mask = lsu_io.lsu_pack.load_byte_sel;
    endfunction
    function logic [31:0] ver_lsu_load_forward_data;
        /*verilator public*/
        ver_lsu_load_forward_data = lsu_io.lsu_pack.load_forward_data;
    endfunction
    
    function logic [7:0] ver_lsu_load_req;
        /*verilator public*/
        ver_lsu_load_req = load_ck_hit_req.load_req ? 1 : 0;
    endfunction
    function logic [31:0] ver_lsu_load_finished;
        /*verilator public*/
        /*if (sdb_io.sb_load_forward_hit_but_mask_fail)
            ver_lsu_load_finished = 0;
        else */if (sdb_io.sb_load_forward_hit || load_hit_resp.load_hit)
            ver_lsu_load_finished = 1;
        else
            ver_lsu_load_finished = 0;
    endfunction
`endif
endmodule
