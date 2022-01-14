`timescale 1ns/1ps
// =============================================================================
//  Program : AGU.sv
//  Author  : Hon-Chou Dai
//  Date    : May/15/2021
// -----------------------------------------------------------------------------
//  Description:
//  Compute memory operation's address in this stage,
//  and check memory address unaligned exception happen or not.
// -----------------------------------------------------------------------------
//  Revision information:
//
//    August/07/2021, by Chun-Wei Chao:
//      Transfer store set id information from rr_stage to LSU_stage
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

module AGU(
    input clk,
    input rst,
    register_read_io.mem rr_io,
    exe_stage_io.AGU exe_io,
    AGU_io.AGU agu_io,
    store_buffer_io.AGU sdb_io,
    mem_access_io.AGU ma_io,
    pipeline_control_recovery_io.AGU pipe_ctrl_io
);

xlen_data_t rs1_data;
xlen_data_t rs2_data;
xlen_data_t unalign_store_data;
xlen_data_t store_data;
xlen_data_t access_addr;
byte_mask_t byte_sel;
logic memory_align_exception;
logic non_idempotent_region;

always_comb rs1_data = bypass_network(rr_io.mem_issue_pack.rs1_addr,
                                      rr_io.mem_issue_pack.rs1_data,
                                      ma_io.load_wb,
                                      exe_io.alu_csr_bc_wb,
                                      exe_io.alu1_wb,
                                      exe_io.muldiv_wb
                                     );
always_comb rs2_data = bypass_network(rr_io.mem_issue_pack.rs2_addr,
                                      rr_io.mem_issue_pack.rs2_data,
                                      ma_io.load_wb,
                                      exe_io.alu_csr_bc_wb,
                                      exe_io.alu1_wb,
                                      exe_io.muldiv_wb
                                     );
always_comb access_addr = rr_io.mem_issue_pack.immediate + rs1_data;
always_comb unalign_store_data = rs2_data;
always_comb
begin
    case (access_addr[1:0])
        2'b00:
        begin
            case (rr_io.mem_issue_pack.mem_input_sel)
                MEM_OP_BYTE:
                begin   // byte
                    store_data = {24'b0, unalign_store_data[7: 0]};
                    byte_sel = 4'b0001;
                    memory_align_exception = 0;
                end
                MEM_OP_HALF_WORD:
                begin   // half-word
                    store_data = {16'b0, unalign_store_data[15: 0]};
                    byte_sel = 4'b0011;
                    memory_align_exception = 0;
                end
                MEM_OP_WORD:
                begin   // word
                    store_data = unalign_store_data;
                    byte_sel = 4'b1111;
                    memory_align_exception = 0;
                end
                default:
                begin
                    store_data = 0;
                    byte_sel = 4'b0000;
                    memory_align_exception = 1;
                end
            endcase
        end
        2'b01:
        begin
            case (rr_io.mem_issue_pack.mem_input_sel)
                MEM_OP_BYTE:
                begin   // byte
                    store_data = {16'b0, unalign_store_data[7: 0], 8'b0};
                    byte_sel = 4'b0010;
                    memory_align_exception = 0;
                end
                default:
                begin
                    store_data = 0;
                    byte_sel = 4'b0000;
                    memory_align_exception = 1;
                end
            endcase
        end
        2'b10:
        begin
            case (rr_io.mem_issue_pack.mem_input_sel)
                MEM_OP_BYTE:
                begin
                    store_data = {8'b0, unalign_store_data[7: 0], 16'b0};
                    byte_sel = 4'b0100;
                    memory_align_exception = 0;
                end
                MEM_OP_HALF_WORD:
                begin
                    store_data = {unalign_store_data[15: 0], 16'b0};
                    byte_sel = 4'b1100;
                    memory_align_exception = 0;
                end
                default:
                begin
                    store_data = 0;
                    byte_sel = 4'b0000;
                    memory_align_exception = 1;
                end
            endcase
        end
        2'b11:
        begin
            case (rr_io.mem_issue_pack.mem_input_sel)
                2'b00:
                begin
                    store_data = {unalign_store_data[7: 0], 24'b0};
                    byte_sel = 4'b1000;
                    memory_align_exception = 0;
                end
                default:
                begin
                    store_data = 0;
                    byte_sel = 4'b0000;
                    memory_align_exception = 1;
                end
            endcase
        end
        default:
        begin
            store_data = 0;
            byte_sel = 4'b0000;
            memory_align_exception = 0;
        end
    endcase
end

// non idempotent region checking
// current hardwire to 4'hC
// This must change to PMP csr if the core support PMP and PMA

always_comb
    if (access_addr[31:28] == 4'hC)
        non_idempotent_region = 1;
    else
        non_idempotent_region = 0;

always_ff @(posedge clk) begin
    if (rst) begin
        agu_io.instr_valid <= 0;
        agu_io.agu_pack <= 0;
        agu_io.memory_align_exception <= 0;
    end else if (pipe_ctrl_io.recovery_stall || pipe_ctrl_io.AGU_stall) begin
        agu_io.instr_valid <= IsBrROBKill(pipe_ctrl_io.recovery_flush_BCAST, agu_io.agu_pack.rob_tag)
                                    ? 0 : agu_io.instr_valid;
        agu_io.agu_pack <= agu_io.agu_pack;
        agu_io.memory_align_exception <= agu_io.memory_align_exception;
    end else if (pipe_ctrl_io.load_wake_up_predict_failed || pipe_ctrl_io.load_wake_up_failed_stall) begin
        agu_io.instr_valid <= 0;
        agu_io.agu_pack <= 0;
        agu_io.memory_align_exception <= 0;
    end else begin
        agu_io.instr_valid <= rr_io.mem_issue_pack_valid; //TODO: BrKill ?
        agu_io.agu_pack.store_set_pc <= rr_io.mem_issue_pack.store_set_pc;
        agu_io.agu_pack.store_set_id <= rr_io.mem_issue_pack.store_set_id;
        agu_io.agu_pack.predict_no_vilation <= rr_io.mem_issue_pack.predict_no_vilation;
        agu_io.agu_pack.store_data <= store_data;
        agu_io.agu_pack.access_addr <= access_addr;
        agu_io.agu_pack.mem_load_ext_sel <= rr_io.mem_issue_pack.mem_load_ext_sel;
        agu_io.agu_pack.mem_is_store <= rr_io.mem_issue_pack.mem_is_store;
        agu_io.agu_pack.mem_input_sel <= rr_io.mem_issue_pack.mem_input_sel;
        agu_io.agu_pack.byte_sel <= byte_sel;
        agu_io.agu_pack.rd_addr <= rr_io.mem_issue_pack.rd_addr;
        agu_io.agu_pack.rob_tag <= rr_io.mem_issue_pack.rob_tag;
        agu_io.agu_pack.is_non_idempotent <= non_idempotent_region;
        agu_io.agu_pack.bhsr <= rr_io.mem_issue_pack.bhsr;
// `ifdef FALCO_SIM_DEBUG
        agu_io.agu_pack.pc <= rr_io.mem_issue_pack.pc;
// `endif
        agu_io.memory_align_exception <= memory_align_exception;
    end
end

always_comb begin
    pipe_ctrl_io.wait_for_non_idempotent = 
        agu_io.agu_pack.is_non_idempotent &&
        agu_io.instr_valid &&
        sdb_io.non_idempotent_instr_exists;
end

function xlen_data_t bypass_network(
    input prf_specifier_t rs_addr,
    input xlen_data_t prf_register_data,
    input exe_fu_wb_t mem_wb,
    input exe_fu_wb_t alu_csr_wb,
    input exe_fu_wb_t alu1_wb,
    input exe_fu_wb_t muldiv_wb
);
    begin
        if (rs_addr == 0)
            bypass_network = 32'h00000000;
        else if (rs_addr == alu_csr_wb.wb_addr && alu_csr_wb.valid)
            bypass_network = alu_csr_wb.wb_data;
        else if (rs_addr == alu1_wb.wb_addr && alu1_wb.valid)
            bypass_network = alu1_wb.wb_data;
        else if (rs_addr == muldiv_wb.wb_addr && muldiv_wb.valid)
            bypass_network = muldiv_wb.wb_data;
        else if (rs_addr == mem_wb.wb_addr && mem_wb.valid)
            bypass_network = mem_wb.wb_data;
        else
            bypass_network = prf_register_data;
    end
endfunction

endmodule
