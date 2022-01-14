`timescale 1ns/1ps
// =============================================================================
//  Program : register_read.sv
//  Author  : Hon-Chou Dai
//  Date    : May/15/2021
// -----------------------------------------------------------------------------
//  Description:
//  Register Read stage for Falco.
// -----------------------------------------------------------------------------
//  Revision information:
//
//    August/07/2021, by Chun-Wei Chao:
//      Tranfer store set id information from mem_issue_queue stage to AGU stage.
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

//TODO: execute unit busy detect


module register_read(
    input clk,
    input rst,

    int_issue_queue_io.register_read int_iq_io,
    mem_issue_queue_io.register_read mem_iq_io,
    exe_stage_io.write_back wb_io,
    mem_access_io.register_read ma_io,
    pipeline_control_recovery_io.register_read pipe_ctrl_io,
    register_read_io.register_read rr_io
);

xlen_data_t IQ0_rs1_data;
xlen_data_t IQ0_rs2_data;
xlen_data_t IQ1_rs1_data;
xlen_data_t IQ1_rs2_data;
xlen_data_t MEM_rs1_data;
xlen_data_t MEM_rs2_data;

physical_register_file prf(
    .clk(clk),
    .rst(rst),
    //INT IQ
    .IQ0_rs1_addr(int_iq_io.instr0_pack.rs1_addr),
    .IQ0_rs2_addr(int_iq_io.instr0_pack.rs2_addr),
    .IQ1_rs1_addr(int_iq_io.instr1_pack.rs1_addr),
    .IQ1_rs2_addr(int_iq_io.instr1_pack.rs2_addr),
    //MEM IQ
    .MEM_rs1_addr(mem_iq_io.instr_pack.rs1_addr),
    .MEM_rs2_addr(mem_iq_io.instr_pack.rs2_addr), //for store
    //INT data
    .IQ0_rs1_data(IQ0_rs1_data),
    .IQ0_rs2_data(IQ0_rs2_data),
    .IQ1_rs1_data(IQ1_rs1_data),
    .IQ1_rs2_data(IQ1_rs2_data),
    //MEM data
    .MEM_rs1_data(MEM_rs1_data),
    .MEM_rs2_data(MEM_rs2_data),

    .alu_csr_bc_wb(wb_io.alu_csr_bc_wb),
    .alu1_wb(wb_io.alu1_wb),
    .muldiv_wb(wb_io.muldiv_wb),
    .mem_wb(ma_io.load_wb)
);


always_ff @(posedge clk) begin
    if (rst) begin
        rr_io.alu_bc_csr_pack <= 0;
    end else if (pipe_ctrl_io.recovery_stall) begin
        rr_io.alu_bc_csr_pack.predict_pc <= rr_io.alu_bc_csr_pack.predict_pc;
        rr_io.alu_bc_csr_pack.pc <= rr_io.alu_bc_csr_pack.pc;
        rr_io.alu_bc_csr_pack.immediate <= rr_io.alu_bc_csr_pack.immediate;
        rr_io.alu_bc_csr_pack.operand0_sel <= rr_io.alu_bc_csr_pack.operand0_sel;
        rr_io.alu_bc_csr_pack.operand1_sel <= rr_io.alu_bc_csr_pack.operand1_sel;
        rr_io.alu_bc_csr_pack.ctrl_signal <= rr_io.alu_bc_csr_pack.ctrl_signal;
        rr_io.alu_bc_csr_pack.shift_sel <= rr_io.alu_bc_csr_pack.shift_sel;
        rr_io.alu_bc_csr_pack.is_branch <= rr_io.alu_bc_csr_pack.is_branch;
        rr_io.alu_bc_csr_pack.is_jal <= rr_io.alu_bc_csr_pack.is_jal;
        rr_io.alu_bc_csr_pack.is_jalr <= rr_io.alu_bc_csr_pack.is_jalr;
        rr_io.alu_bc_csr_pack.is_csr_instr <= rr_io.alu_bc_csr_pack.is_csr_instr;
        rr_io.alu_bc_csr_pack.csr_addr <= rr_io.alu_bc_csr_pack.csr_addr;
        rr_io.alu_bc_csr_pack.csr_imm <= rr_io.alu_bc_csr_pack.csr_imm;
        rr_io.alu_bc_csr_pack.csr_data <= rr_io.alu_bc_csr_pack.csr_data;
        rr_io.alu_bc_csr_pack.sys_jump <= rr_io.alu_bc_csr_pack.sys_jump;
        rr_io.alu_bc_csr_pack.sys_jump_csr_addr <= rr_io.alu_bc_csr_pack.sys_jump_csr_addr;
        rr_io.alu_bc_csr_pack.rd_addr <= rr_io.alu_bc_csr_pack.rd_addr;
        rr_io.alu_bc_csr_pack.rs1_data <= rr_io.alu_bc_csr_pack.rs1_data;
        rr_io.alu_bc_csr_pack.rs2_data <= rr_io.alu_bc_csr_pack.rs2_data;
        rr_io.alu_bc_csr_pack.rs1_addr <= rr_io.alu_bc_csr_pack.rs1_addr;
        rr_io.alu_bc_csr_pack.rs2_addr <= rr_io.alu_bc_csr_pack.rs2_addr;
        rr_io.alu_bc_csr_pack.rob_tag <= rr_io.alu_bc_csr_pack.rob_tag;
        rr_io.alu_bc_csr_pack.bhsr <= rr_io.alu_bc_csr_pack.bhsr;
    end else begin
        rr_io.alu_bc_csr_pack.predict_pc <= int_iq_io.instr0_pack.predict_pc;
        rr_io.alu_bc_csr_pack.pc <= int_iq_io.instr0_pack.pc;
        rr_io.alu_bc_csr_pack.immediate <= int_iq_io.instr0_pack.immediate;
        rr_io.alu_bc_csr_pack.operand0_sel <= int_iq_io.instr0_pack.operand0_sel;
        rr_io.alu_bc_csr_pack.operand1_sel <= int_iq_io.instr0_pack.operand1_sel;
        rr_io.alu_bc_csr_pack.ctrl_signal <= int_iq_io.instr0_pack.ctrl_signal;
        rr_io.alu_bc_csr_pack.shift_sel <= int_iq_io.instr0_pack.shift_sel;
        rr_io.alu_bc_csr_pack.is_branch <= int_iq_io.instr0_pack.is_branch;
        rr_io.alu_bc_csr_pack.is_jal <= int_iq_io.instr0_pack.is_jal;
        rr_io.alu_bc_csr_pack.is_jalr <= int_iq_io.instr0_pack.is_jalr;
        rr_io.alu_bc_csr_pack.is_csr_instr <= int_iq_io.instr0_pack.is_csr_instr;
        rr_io.alu_bc_csr_pack.csr_addr <= int_iq_io.instr0_pack.csr_addr;
        rr_io.alu_bc_csr_pack.csr_imm <= int_iq_io.instr0_pack.csr_imm;
        rr_io.alu_bc_csr_pack.csr_data <= int_iq_io.instr0_pack.csr_data;
        rr_io.alu_bc_csr_pack.sys_jump <= int_iq_io.instr0_pack.sys_jump;
        rr_io.alu_bc_csr_pack.sys_jump_csr_addr <= int_iq_io.instr0_pack.sys_jump_csr_addr;
        rr_io.alu_bc_csr_pack.rd_addr <= int_iq_io.instr0_pack.rd_addr;
        rr_io.alu_bc_csr_pack.rs1_data <= bypass_network(
                                                int_iq_io.instr0_pack.rs1_addr,
                                                IQ0_rs1_data,
                                                ma_io.load_wb,
                                                wb_io.alu_csr_bc_wb,
                                                wb_io.alu1_wb,
                                                wb_io.muldiv_wb
                                            );
        rr_io.alu_bc_csr_pack.rs2_data <= bypass_network(
                                                int_iq_io.instr0_pack.rs2_addr,
                                                IQ0_rs2_data,
                                                ma_io.load_wb,
                                                wb_io.alu_csr_bc_wb,
                                                wb_io.alu1_wb,
                                                wb_io.muldiv_wb
                                            );
        rr_io.alu_bc_csr_pack.rs1_addr <= int_iq_io.instr0_pack.rs1_addr;
        rr_io.alu_bc_csr_pack.rs2_addr <= int_iq_io.instr0_pack.rs2_addr;
        rr_io.alu_bc_csr_pack.rob_tag <= int_iq_io.instr0_pack.rob_tag;
        rr_io.alu_bc_csr_pack.bhsr <= int_iq_io.instr0_pack.bhsr;
        //rr_io.alu_bc_csr_br_info <= int_iq_io.instr0_br_info;
    end
end

always_ff @(posedge clk) begin
    if (rst) begin
        rr_io.alu_muldiv_pack <= 0;
    end else if (pipe_ctrl_io.recovery_stall) begin
        rr_io.alu_muldiv_pack.pc <= rr_io.alu_muldiv_pack.pc;
        rr_io.alu_muldiv_pack.immediate <= rr_io.alu_muldiv_pack.immediate;
        rr_io.alu_muldiv_pack.operand0_sel <= rr_io.alu_muldiv_pack.operand0_sel;
        rr_io.alu_muldiv_pack.operand1_sel <= rr_io.alu_muldiv_pack.operand1_sel;
        rr_io.alu_muldiv_pack.ctrl_signal <= rr_io.alu_muldiv_pack.ctrl_signal;
        rr_io.alu_muldiv_pack.shift_sel <= rr_io.alu_muldiv_pack.shift_sel;
        rr_io.alu_muldiv_pack.alu_muldiv_sel <= rr_io.alu_muldiv_pack.alu_muldiv_sel;
        rr_io.alu_muldiv_pack.rd_addr <= rr_io.alu_muldiv_pack.rd_addr;
        rr_io.alu_muldiv_pack.rs1_data <= rr_io.alu_muldiv_pack.rs1_data;
        rr_io.alu_muldiv_pack.rs2_data <= rr_io.alu_muldiv_pack.rs2_data;
        rr_io.alu_muldiv_pack.rs1_addr <= rr_io.alu_muldiv_pack.rs1_addr;
        rr_io.alu_muldiv_pack.rs2_addr <= rr_io.alu_muldiv_pack.rs2_addr;
        rr_io.alu_muldiv_pack.rob_tag <= rr_io.alu_muldiv_pack.rob_tag;
    end else begin
        rr_io.alu_muldiv_pack.pc <= int_iq_io.instr1_pack.pc;
        rr_io.alu_muldiv_pack.immediate <= int_iq_io.instr1_pack.immediate;
        rr_io.alu_muldiv_pack.operand0_sel <= int_iq_io.instr1_pack.operand0_sel;
        rr_io.alu_muldiv_pack.operand1_sel <= int_iq_io.instr1_pack.operand1_sel;
        rr_io.alu_muldiv_pack.ctrl_signal <= int_iq_io.instr1_pack.ctrl_signal;
        rr_io.alu_muldiv_pack.shift_sel <= int_iq_io.instr1_pack.shift_sel;
        rr_io.alu_muldiv_pack.alu_muldiv_sel <= int_iq_io.instr1_pack.alu_muldiv_sel;
        rr_io.alu_muldiv_pack.rd_addr <= int_iq_io.instr1_pack.rd_addr;
        rr_io.alu_muldiv_pack.rs1_data <= bypass_network(
                                                int_iq_io.instr1_pack.rs1_addr,
                                                IQ1_rs1_data,
                                                ma_io.load_wb,
                                                wb_io.alu_csr_bc_wb,
                                                wb_io.alu1_wb,
                                                wb_io.muldiv_wb
                                            );
        rr_io.alu_muldiv_pack.rs2_data <= bypass_network(
                                                int_iq_io.instr1_pack.rs2_addr,
                                                IQ1_rs2_data,
                                                ma_io.load_wb,
                                                wb_io.alu_csr_bc_wb,
                                                wb_io.alu1_wb,
                                                wb_io.muldiv_wb
                                            );
        rr_io.alu_muldiv_pack.rs1_addr <= int_iq_io.instr1_pack.rs1_addr;
        rr_io.alu_muldiv_pack.rs2_addr <= int_iq_io.instr1_pack.rs2_addr;
        rr_io.alu_muldiv_pack.rob_tag <= int_iq_io.instr1_pack.rob_tag;
        //rr_io.alu_muldiv_br_mask <= int_iq_io.instr1_br_info.br_mask;
    end
end

always_ff @(posedge clk) begin
    if (rst) begin
        rr_io.mem_issue_pack <= 0;
    end else if (pipe_ctrl_io.recovery_stall || pipe_ctrl_io.rr_mem_stall) begin
        rr_io.mem_issue_pack.immediate <= rr_io.mem_issue_pack.immediate;
        rr_io.mem_issue_pack.mem_load_ext_sel <= rr_io.mem_issue_pack.mem_load_ext_sel;
        rr_io.mem_issue_pack.mem_is_store <= rr_io.mem_issue_pack.mem_is_store;
        rr_io.mem_issue_pack.mem_input_sel <= rr_io.mem_issue_pack.mem_input_sel;
        rr_io.mem_issue_pack.store_set_pc <= rr_io.mem_issue_pack.store_set_pc;
        rr_io.mem_issue_pack.store_set_id <= rr_io.mem_issue_pack.store_set_id;
        rr_io.mem_issue_pack.predict_no_vilation <= rr_io.mem_issue_pack.predict_no_vilation;
        rr_io.mem_issue_pack.rd_addr <= rr_io.mem_issue_pack.rd_addr;
        rr_io.mem_issue_pack.rs1_data <= rr_io.mem_issue_pack.rs1_data;
        rr_io.mem_issue_pack.rs2_data <= rr_io.mem_issue_pack.rs2_data;
        rr_io.mem_issue_pack.rs1_addr <= rr_io.mem_issue_pack.rs1_addr;
        rr_io.mem_issue_pack.rs2_addr <= rr_io.mem_issue_pack.rs2_addr;
        rr_io.mem_issue_pack.bhsr <= rr_io.mem_issue_pack.bhsr;
// `ifdef FALCO_SIM_DEBUG
        rr_io.mem_issue_pack.pc <= rr_io.mem_issue_pack.pc;
// `endif
    rr_io.mem_issue_pack.rob_tag <= rr_io.mem_issue_pack.rob_tag;
    end else  begin
        rr_io.mem_issue_pack.immediate <= mem_iq_io.instr_pack.immediate;
        rr_io.mem_issue_pack.mem_load_ext_sel <= mem_iq_io.instr_pack.mem_load_ext_sel;
        rr_io.mem_issue_pack.mem_is_store <= mem_iq_io.instr_pack.mem_is_store;
        rr_io.mem_issue_pack.mem_input_sel <= mem_iq_io.instr_pack.mem_input_sel;
        rr_io.mem_issue_pack.store_set_pc <= mem_iq_io.instr_pack.store_set_pc;
        rr_io.mem_issue_pack.store_set_id <= mem_iq_io.instr_pack.store_set_id;
        rr_io.mem_issue_pack.predict_no_vilation <= mem_iq_io.instr_pack.predict_no_vilation;
        rr_io.mem_issue_pack.rd_addr <= mem_iq_io.instr_pack.rd_addr;
        rr_io.mem_issue_pack.rs1_data <= bypass_network(
                                                mem_iq_io.instr_pack.rs1_addr,
                                                MEM_rs1_data,
                                                ma_io.load_wb,
                                                wb_io.alu_csr_bc_wb,
                                                wb_io.alu1_wb,
                                                wb_io.muldiv_wb
                                            );
        rr_io.mem_issue_pack.rs2_data <= bypass_network(
                                                mem_iq_io.instr_pack.rs2_addr,
                                                MEM_rs2_data,
                                                ma_io.load_wb,
                                                wb_io.alu_csr_bc_wb,
                                                wb_io.alu1_wb,
                                                wb_io.muldiv_wb
                                            );;
        rr_io.mem_issue_pack.rs1_addr <= mem_iq_io.instr_pack.rs1_addr;
        rr_io.mem_issue_pack.rs2_addr <= mem_iq_io.instr_pack.rs2_addr;
        rr_io.mem_issue_pack.bhsr <= mem_iq_io.instr_pack.bhsr;
// `ifdef FALCO_SIM_DEBUG
        rr_io.mem_issue_pack.pc <= mem_iq_io.instr_pack.pc;
// `endif
        rr_io.mem_issue_pack.rob_tag <= mem_iq_io.instr_pack.rob_tag;
    end
end

always_ff @(posedge clk) begin
    if (rst)
        rr_io.alu_bc_csr_pack_valid <= 0;
    else if (pipe_ctrl_io.recovery_stall)
        rr_io.alu_bc_csr_pack_valid <= IsBrROBKill(pipe_ctrl_io.recovery_flush_BCAST,
                                                   rr_io.alu_bc_csr_pack.rob_tag) ? 0 : rr_io.alu_bc_csr_pack_valid;
    else if (pipe_ctrl_io.load_wake_up_predict_failed || pipe_ctrl_io.INT_IQ_stall)
        rr_io.alu_bc_csr_pack_valid <= 0;
    else
        rr_io.alu_bc_csr_pack_valid <= int_iq_io.instr0_valid;
end

always_ff @(posedge clk) begin
    if (rst)
        rr_io.alu_muldiv_pack_valid <= 0;
    else if (pipe_ctrl_io.recovery_stall)
        rr_io.alu_muldiv_pack_valid <= IsBrROBKill(pipe_ctrl_io.recovery_flush_BCAST,
                                                   rr_io.alu_muldiv_pack.rob_tag) ? 0 : rr_io.alu_muldiv_pack_valid;
    else if (pipe_ctrl_io.load_wake_up_predict_failed || pipe_ctrl_io.INT_IQ_stall)
        rr_io.alu_muldiv_pack_valid <= 0;
    else
        rr_io.alu_muldiv_pack_valid <= int_iq_io.instr1_valid;
end

always_ff @(posedge clk) begin
    if (rst)
        rr_io.mem_issue_pack_valid <= 0;
    else if (pipe_ctrl_io.recovery_stall /*|| pipe_ctrl_io.rr_mem_stall*/)
        rr_io.mem_issue_pack_valid <= IsBrROBKill(pipe_ctrl_io.recovery_flush_BCAST,
                                                  rr_io.mem_issue_pack.rob_tag) ? 0 : rr_io.mem_issue_pack_valid;
    else if (pipe_ctrl_io.load_wake_up_predict_failed)
        rr_io.mem_issue_pack_valid <= 0;
    else
        rr_io.mem_issue_pack_valid <= mem_iq_io.instr_valid;
end
function xlen_data_t bypass_network(
    input prf_specifier_t rs_addr,
    input xlen_data_t prf_register_data,
    input exe_fu_wb_t load_wb,
    input exe_fu_wb_t alu_csr_wb,
    input exe_fu_wb_t alu1_wb,
    input exe_fu_wb_t muldiv_wb
);
//Don't need bypass load write back in this stage
    begin
        if (rs_addr == 0)
            bypass_network = 32'h00000000;
        else if (rs_addr == alu_csr_wb.wb_addr && alu_csr_wb.valid)
            bypass_network = alu_csr_wb.wb_data;
        else if (rs_addr == alu1_wb.wb_addr && alu1_wb.valid)
            bypass_network = alu1_wb.wb_data;
        else if (rs_addr == muldiv_wb.wb_addr && muldiv_wb.valid)
            bypass_network = muldiv_wb.wb_data;
        else if (rs_addr == load_wb.wb_addr && load_wb.valid)
            bypass_network = load_wb.wb_data;
        else
            bypass_network = prf_register_data;
    end
endfunction

endmodule
