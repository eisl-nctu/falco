`timescale 1ns/1ps
// =============================================================================
//  Program : alu_csr_bc_execute_group.sv
//  Author  : Hon-Chou Dai
//  Date    : May/15/2021
// -----------------------------------------------------------------------------
//  Description:
//  Execute group for ALU & CSR & BRANCH instruction.
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

module alu_csr_bc_execute_group(
    input clk,
    input rst,
    register_read_io.alu_bc_csr_exe rr_io,
    exe_stage_io.alu_csr_bc_group exe_io,
    mem_access_io.alu_csr_bc_group ma_io,
    csr_io.EXE_stage csr_io,
    pipeline_control_recovery_io.alu_csr_bc_exe pipe_ctrl_io
);

logic instruction_valid;
// bypass rs result
xlen_data_t rs1_data,rs2_data;
xlen_data_t operand_A,operand_B;
xlen_data_t alu_result;
// xlen_data_t csr_result;
xlen_data_t csr_inputA;
xlen_data_t csr_inputB;
xlen_data_t csr_update_data;

logic csr_result_valid;
logic alu_result_valid;
logic is_control_instruction;

logic [2:0] alu_operation;
pc_t branch_target_addr;
logic branch_result_valid;
logic branch_taken;
logic compare_result;
logic branch_misprediction;
logic is_branch_instruction;
logic invalidate_current_instr;
pc_t branch_restore_pc;

always_comb instruction_valid = rr_io.alu_bc_csr_pack_valid && ~pipe_ctrl_io.load_wake_up_predict_failed;
always_comb invalidate_current_instr = IsBrROBKill(pipe_ctrl_io.recovery_flush_BCAST, rr_io.alu_bc_csr_pack.rob_tag);
always_comb is_control_instruction = invalidate_current_instr ? 0 :
                                    ((rr_io.alu_bc_csr_pack.is_branch ||
                                    rr_io.alu_bc_csr_pack.is_csr_instr) &&
                                    instruction_valid); //TODO: csr relocate
always_comb is_branch_instruction = invalidate_current_instr ? 0 : 
                                    ((rr_io.alu_bc_csr_pack.is_branch ||
                                    rr_io.alu_bc_csr_pack.is_jal ||
                                    rr_io.alu_bc_csr_pack.is_jalr) &&
                                    instruction_valid);
always_comb alu_result_valid = invalidate_current_instr ? 
                               0 : 
                               (~is_control_instruction && 
                                instruction_valid);
always_comb alu_operation = (is_control_instruction ? ALU_OP_ADD : rr_io.alu_bc_csr_pack.ctrl_signal);
always_comb branch_restore_pc = rr_io.alu_bc_csr_pack.pc + 4;

always_comb rs1_data = bypass_network(rr_io.alu_bc_csr_pack.rs1_addr,
                                      rr_io.alu_bc_csr_pack.rs1_data,
                                      ma_io.load_wb,
                                      exe_io.alu_csr_bc_wb,
                                      exe_io.alu1_wb,
                                      exe_io.muldiv_wb
                                      );
always_comb rs2_data = bypass_network(rr_io.alu_bc_csr_pack.rs2_addr,
                                      rr_io.alu_bc_csr_pack.rs2_data,
                                      ma_io.load_wb,
                                      exe_io.alu_csr_bc_wb,
                                      exe_io.alu1_wb,
                                      exe_io.muldiv_wb
                                      );

always_comb begin
    case(rr_io.alu_bc_csr_pack.operand0_sel)
        OPERAND0_SEL_LUI:    operand_A = 0;
        OPERAND0_SEL_PC:     operand_A = rr_io.alu_bc_csr_pack.pc;
        OPERAND0_SEL_RS1:    operand_A = rs1_data;
        OPERAND0_SEL_DUMMY0: operand_A = 0; //should not happen
    endcase
end

always_comb begin
    case(rr_io.alu_bc_csr_pack.operand1_sel)
        OPERAND1_SEL_IMM:       operand_B = rr_io.alu_bc_csr_pack.immediate;
        OPERAND1_SEL_RS2:        operand_B = rs2_data;
        OPERAND1_SEL_NEG_RS2:   operand_B = ~rs2_data + 1'b1;
        OPERAND1_SEL_DUMMY0:    operand_B = 0; //should not happen
    endcase
end

ALU ALU0(
    .a(operand_A),
    .b(operand_B),
    .operation_sel(alu_operation),
    .shift_sel(rr_io.alu_bc_csr_pack.shift_sel),
    .alu_result(alu_result)
);


always_comb branch_target_addr = branch_taken ? alu_result : branch_restore_pc;

branch_unit BCU(
    .a(rs1_data),
    .b(rs2_data),
    .operation_sel(rr_io.alu_bc_csr_pack.ctrl_signal),
    .compare_result(compare_result)
);

always_comb branch_taken = (rr_io.alu_bc_csr_pack.is_branch & compare_result ) ||
                            rr_io.alu_bc_csr_pack.is_jal ||
                            rr_io.alu_bc_csr_pack.is_jalr;

always_comb branch_misprediction = branch_target_addr != rr_io.alu_bc_csr_pack.predict_pc;


//TO pipeline controller
always_ff @(posedge clk) begin
    if (rst) begin
        //pipe_ctrl_io.bp_result_tag <= 0;
        //pipe_ctrl_io.bp_result_mask <= 0;
        pipe_ctrl_io.bp_PrSuccess <= 0;
        pipe_ctrl_io.bp_PrMiss <= 0;
    end else if (is_branch_instruction && ~pipe_ctrl_io.recovery_start && ~pipe_ctrl_io.recovery_procedure) begin
        //pipe_ctrl_io.bp_result_mask <= rr_io.alu_bc_csr_br_info.br_mask;
        //pipe_ctrl_io.bp_result_tag <= rr_io.alu_bc_csr_br_info.br_tag;
        pipe_ctrl_io.bp_PrSuccess <= (!branch_misprediction  && is_branch_instruction && rr_io.alu_bc_csr_pack_valid);
        pipe_ctrl_io.bp_PrMiss <= (branch_misprediction && is_branch_instruction && rr_io.alu_bc_csr_pack_valid);
    end else begin
        //pipe_ctrl_io.bp_result_tag <= 0;
        //pipe_ctrl_io.bp_result_mask <= 0;
        pipe_ctrl_io.bp_PrSuccess <= 0;
        pipe_ctrl_io.bp_PrMiss <= 0;
    end
end

//===================================================================================
// To Register Read stage and Commit stage
//===================================================================================

always_ff @(posedge clk) begin
    if (rst) begin
        exe_io.alu_csr_bc_wb.valid <= 0;
        exe_io.alu_csr_bc_wb.wb_addr <= 0;
        exe_io.alu_csr_bc_wb.wb_data <= 0;
`ifdef FALCO_SIM_DEBUG
        exe_io.alu_csr_bc_wb.pc <= 0;
`endif
    end else if (rr_io.alu_bc_csr_pack.is_csr_instr)begin // And CSR write operation
        exe_io.alu_csr_bc_wb.valid <= instruction_valid;
        exe_io.alu_csr_bc_wb.wb_addr <= rr_io.alu_bc_csr_pack.rd_addr;
        // exe_io.alu_csr_bc_wb.wb_data <= csr_result;
        exe_io.alu_csr_bc_wb.wb_data <= csr_io.csr_data_o;//rr_io.alu_bc_csr_pack.csr_data;
`ifdef FALCO_SIM_DEBUG
        exe_io.alu_csr_bc_wb.pc <= rr_io.alu_bc_csr_pack.pc;
`endif
    end else if (/*rr_io.alu_bc_csr_pack.is_branch ||*/
                 pipe_ctrl_io.recovery_stall) begin
        exe_io.alu_csr_bc_wb.valid <= exe_io.alu_csr_bc_wb.valid;
        exe_io.alu_csr_bc_wb.wb_addr <= exe_io.alu_csr_bc_wb.wb_addr;
        exe_io.alu_csr_bc_wb.wb_data <= exe_io.alu_csr_bc_wb.wb_data;
`ifdef FALCO_SIM_DEBUG
        exe_io.alu_csr_bc_wb.pc <= exe_io.alu_csr_bc_wb.pc;
`endif
    end else begin //jalr write back
        exe_io.alu_csr_bc_wb.valid <= alu_result_valid;
        exe_io.alu_csr_bc_wb.wb_addr <= rr_io.alu_bc_csr_pack.rd_addr;
        exe_io.alu_csr_bc_wb.wb_data <= (rr_io.alu_bc_csr_pack.is_jalr || rr_io.alu_bc_csr_pack.is_jal ? 
                                        branch_restore_pc : alu_result);
`ifdef FALCO_SIM_DEBUG
        exe_io.alu_csr_bc_wb.pc <= rr_io.alu_bc_csr_pack.pc;
`endif
    end
end

always_ff @(posedge clk) begin
    if (rst) begin
        exe_io.exe_alu_csr_bc.valid <= 0;
        exe_io.exe_alu_csr_bc.is_branch <= 0;
        exe_io.exe_alu_csr_bc.commit_addr <= 0;
    end else if (pipe_ctrl_io.recovery_stall) begin
        exe_io.exe_alu_csr_bc.valid <= 0;
        exe_io.exe_alu_csr_bc.is_branch <= 0;
        exe_io.exe_alu_csr_bc.commit_addr <= 0;
    end else begin
        exe_io.exe_alu_csr_bc.valid <= (alu_result_valid || is_branch_instruction || rr_io.alu_bc_csr_pack.is_csr_instr) &&
                                     ~pipe_ctrl_io.recovery_start && ~pipe_ctrl_io.recovery_procedure;
        // exe_io.exe_alu_csr_bc.valid <= (alu_result_valid || is_branch_instruction) &&
        //                              ~pipe_ctrl_io.recovery_start && ~pipe_ctrl_io.recovery_procedure;
        exe_io.exe_alu_csr_bc.is_branch <= is_branch_instruction;
        exe_io.exe_alu_csr_bc.commit_addr <= rr_io.alu_bc_csr_pack.rob_tag;
    end 
end

//===================================================================================
// Branch to IF stage
//===================================================================================

always_ff @(posedge clk) begin
    if (rst) begin
        exe_io.branch_result.valid <= 0;
        exe_io.branch_result.branch_taken <= 0;
        exe_io.branch_result.target_addr <= 0;
        exe_io.branch_result.is_misprediction <= 0;
        exe_io.branch_result.branch_addr <= 0;
        exe_io.branch_result.bhsr <= 0;
    end else if (pipe_ctrl_io.recovery_stall) begin
        exe_io.branch_result.valid <= 0;
        exe_io.branch_result.branch_taken <= 0;
        exe_io.branch_result.target_addr <= 0;
        exe_io.branch_result.is_misprediction <= 0;
        exe_io.branch_result.branch_addr <= 0;
        exe_io.branch_result.bhsr <= 0;
    end else if (is_branch_instruction && ~pipe_ctrl_io.recovery_start && ~pipe_ctrl_io.recovery_procedure) begin
        exe_io.branch_result.valid <= instruction_valid;
        exe_io.branch_result.branch_taken <= branch_taken;
        exe_io.branch_result.target_addr <= branch_target_addr;
        exe_io.branch_result.is_misprediction <= branch_misprediction;
        exe_io.branch_result.branch_addr <= rr_io.alu_bc_csr_pack.pc;
        exe_io.branch_result.bhsr <= rr_io.alu_bc_csr_pack.bhsr;
    end else begin
        exe_io.branch_result.valid <= 0;
        exe_io.branch_result.branch_taken <= 0;
        exe_io.branch_result.target_addr <= 0;
        exe_io.branch_result.is_misprediction <= 0;
        exe_io.branch_result.branch_addr <= 0;
        exe_io.branch_result.bhsr <= 0;
    end
end

//===================================================================================
// Exe result to csr
//===================================================================================
always_comb csr_inputA = csr_io.csr_data_o;
always_comb csr_inputB = rr_io.alu_bc_csr_pack.ctrl_signal[2] ? {27'b0, rr_io.alu_bc_csr_pack.csr_imm} 
                        : rr_io.alu_bc_csr_pack.rs1_data;

always_comb begin
    case (rr_io.alu_bc_csr_pack.ctrl_signal[1: 0])
        2'b01:
            csr_update_data = csr_inputB;
        2'b10:
            csr_update_data = csr_inputA | csr_inputB;
        2'b11:
            csr_update_data = csr_inputA & ~csr_inputB;
        default:
            csr_update_data = csr_inputA;
    endcase
end

always_ff @(posedge clk) begin
    if (rst) begin
        csr_io.csr_we_i <= 0;
        csr_io.csr_waddr_i <= 0;
        csr_io.csr_wdata_i <= 0;
    end else begin
        csr_io.csr_we_i <= rr_io.alu_bc_csr_pack.is_csr_instr;
        csr_io.csr_waddr_i <= rr_io.alu_bc_csr_pack.csr_addr;
        csr_io.csr_wdata_i <= csr_update_data;
    end
end

always_comb csr_io.csr_raddr_i = rr_io.alu_bc_csr_pack.csr_addr;
always_comb csr_io.exe_stall =  pipe_ctrl_io.recovery_start || pipe_ctrl_io.recovery_procedure;
always_comb csr_io.exe_is_branch = is_branch_instruction;
always_comb csr_io.exe_misspredict = branch_misprediction;

`ifdef FALCO_SIM_DEBUG
always_ff @(posedge clk) begin
    if (rst)
        exe_io.branch_result.pc <= 0;
    else
        exe_io.branch_result.pc <= rr_io.alu_bc_csr_pack.pc;
end

logic t_branch_misprediction/*verilator public*/;
logic t_is_branch_instruction/*verilator public*/;
logic t_branch_taken/*verilator public*/;
logic [31:0] t_branch_target_addr/*verilator public*/;
logic [31:0] t_rob_tag/*verilator public*/;

always_comb t_branch_misprediction = exe_io.branch_result.is_misprediction;
always_comb t_is_branch_instruction = exe_io.exe_alu_csr_bc.is_branch;
always_comb t_branch_taken = exe_io.branch_result.branch_taken;
always_comb t_branch_target_addr = exe_io.branch_result.target_addr;
always_comb t_rob_tag = exe_io.exe_alu_csr_bc.commit_addr;
`endif
//===================================================================================
// Broadcast to RNDS (busy_list) ,INT_IQ,MEM_IQ
//===================================================================================

always_ff @(posedge clk) begin
    if (rst) begin
        exe_io.BCAST_alu_csr_bc.valid <= 0;
        exe_io.BCAST_alu_csr_bc.prf_addr <= 0;
    end else if (pipe_ctrl_io.recovery_stall) begin
        exe_io.BCAST_alu_csr_bc.valid <= 0;
        exe_io.BCAST_alu_csr_bc.prf_addr <= 0;
    end else if(rr_io.alu_bc_csr_pack.is_csr_instr) begin
        exe_io.BCAST_alu_csr_bc.valid <= 1;
        exe_io.BCAST_alu_csr_bc.prf_addr <= rr_io.alu_bc_csr_pack.rd_addr;
    end else if (alu_result_valid)begin //TODO: csr valid
        exe_io.BCAST_alu_csr_bc.valid <= alu_result_valid;
        exe_io.BCAST_alu_csr_bc.prf_addr <= rr_io.alu_bc_csr_pack.rd_addr;
    end else if (rr_io.alu_bc_csr_pack_valid &&
                 rr_io.alu_bc_csr_pack.rd_addr != 0 &&
                 (rr_io.alu_bc_csr_pack.is_jal || rr_io.alu_bc_csr_pack.is_jalr)) begin
        exe_io.BCAST_alu_csr_bc.valid <= is_branch_instruction;
        exe_io.BCAST_alu_csr_bc.prf_addr <= rr_io.alu_bc_csr_pack.rd_addr;
    end else begin
        exe_io.BCAST_alu_csr_bc.valid <= 0;
        exe_io.BCAST_alu_csr_bc.prf_addr <= 0;
    end
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
