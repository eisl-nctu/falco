`timescale 1ns/1ps
// =============================================================================
//  Program : alu_muldiv_execute_group.sv
//  Author  : Hon-Chou Dai
//  Date    : May/15/2021
// -----------------------------------------------------------------------------
//  Description:
//  Execute group for ALU & MUL & DIV instructions.
//  When MUL or DIV instruction is executing, for no waste the resource of ALU,
//  this stage still can execute other ALU instruction,
//  so the write back port has two, one for ALU instruction,
//  the other for MUL or DIV instrction.
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

module alu_muldiv_execute_group(
    input clk,
    input rst,
    register_read_io.alu_muldiv_exe rr_io,
    exe_stage_io.alu_muldiv_group exe_io,
    mem_access_io.alu_muldiv_group ma_io,
    pipeline_control_recovery_io.alu_muldiv_exe pipe_ctrl_io,
    //To issue queue
    output logic muldiv_busy/*verilator public*/
);


logic instruction_valid;
// bypass rs result
xlen_data_t rs1_data,rs2_data;

// operand MUX
xlen_data_t operand_A,operand_B;
xlen_data_t alu_result;

// mul/div group logic
logic muldiv_ready /*verilator public*/;
logic muldiv_req /*verilator public*/;
//mulipltier combination intput
xlen_data_t mul_result;
logic mul_ready /*verilator public*/;
logic mul_result_valid;
logic is_mul_instr;
logic [1:0] mul_operation_sel;
logic mul_early_wake_up;

//divider ff input
xlen_data_t div_result;
logic div_req;
logic div_ready /*verilator public*/;
logic div_result_valid;
logic is_div_instr;
xlen_data_t div_op_A,div_op_B;
logic [2:0] div_operation_sel;
logic div_req_reg;

logic muldiv_running;
xlen_data_t muldiv_result;
logic muldiv_result_valid;
logic muldiv_BCAST_valid; //for early wake up
logic alu_result_valid;
logic invalidate_cur_instr;

always_comb instruction_valid = rr_io.alu_muldiv_pack_valid && ~pipe_ctrl_io.load_wake_up_predict_failed;

always_comb invalidate_cur_instr = IsBrROBKill(pipe_ctrl_io.recovery_flush_BCAST, rr_io.alu_muldiv_pack.rob_tag); 

always_comb muldiv_req = invalidate_cur_instr ?
                        0 : 
                        (rr_io.alu_muldiv_pack.alu_muldiv_sel && instruction_valid);
always_comb is_mul_instr = muldiv_req && (rr_io.alu_muldiv_pack.ctrl_signal[2] == 0);
always_comb is_div_instr = muldiv_req && (rr_io.alu_muldiv_pack.ctrl_signal[2] == 1);

always_comb alu_result_valid = invalidate_cur_instr ? 0 : 
                                (!rr_io.alu_muldiv_pack.alu_muldiv_sel && instruction_valid);

always_comb rs1_data = bypass_network(rr_io.alu_muldiv_pack.rs1_addr,
                                      rr_io.alu_muldiv_pack.rs1_data,
                                      ma_io.load_wb,
                                      exe_io.alu_csr_bc_wb,
                                      exe_io.alu1_wb,
                                      exe_io.muldiv_wb
                                      );
always_comb rs2_data = bypass_network(rr_io.alu_muldiv_pack.rs2_addr,
                                      rr_io.alu_muldiv_pack.rs2_data,
                                      ma_io.load_wb,
                                      exe_io.alu_csr_bc_wb,
                                      exe_io.alu1_wb,
                                      exe_io.muldiv_wb
                                      );

always_comb begin
    case(rr_io.alu_muldiv_pack.operand0_sel)
        OPERAND0_SEL_LUI: operand_A = 0;
        OPERAND0_SEL_PC:  operand_A = rr_io.alu_muldiv_pack.pc;
        OPERAND0_SEL_RS1: operand_A = rs1_data;
        OPERAND0_SEL_DUMMY0: operand_A = 0; //should not happen
    endcase
end

always_comb begin
    case(rr_io.alu_muldiv_pack.operand1_sel)
        OPERAND1_SEL_IMM: operand_B = rr_io.alu_muldiv_pack.immediate;
        OPERAND1_SEL_RS2:  operand_B = rs2_data;
        OPERAND1_SEL_NEG_RS2: operand_B = ~rs2_data + 1'b1;
        OPERAND1_SEL_DUMMY0: operand_B = 0; //should not happen
    endcase
end

//TODO: kill
always_ff @(posedge clk) begin
    if (rst) begin
        div_op_A <= 0;
        div_op_B <= 0;
        div_operation_sel <= 0;
        div_req_reg <= 0;
    end else if (muldiv_req && ~pipe_ctrl_io.recovery_stall) begin
        div_op_A <= operand_A;
        div_op_B <= operand_B;
        div_operation_sel <= rr_io.alu_muldiv_pack.ctrl_signal[1:0];
        div_req_reg <= is_div_instr;
    end else begin
        div_op_A <= div_op_A;
        div_op_B <= div_op_B;
        div_operation_sel <= div_operation_sel;
        div_req_reg <= 0;
    end
end

always_ff @(posedge clk) begin
    if (rst)
        muldiv_running <= 0;
    else if (muldiv_req)
        muldiv_running <= 1;
    else if (muldiv_result_valid)
        muldiv_running <= 0;
    else
        muldiv_running <= muldiv_running;
end

ALU ALU1(
    .a(operand_A),
    .b(operand_B),
    .operation_sel(rr_io.alu_muldiv_pack.ctrl_signal),
    .shift_sel(rr_io.alu_muldiv_pack.shift_sel),
    .alu_result(alu_result)
);

divider idiv(
    .clk_i(clk),
    .rst_i(rst),
    .stall_i(pipe_ctrl_io.recovery_stall || pipe_ctrl_io.load_wake_up_predict_failed), //stall result when recovery
    .a_i(div_op_A),
    .b_i(div_op_B),
    .req_i(div_req_reg),
    .kill_i(IsBrROBKill(pipe_ctrl_io.recovery_flush_BCAST, exe_io.exe_muldiv.commit_addr)),
    .operation_sel_i(div_operation_sel),
    .div_result_o(div_result),
    .ready_o(div_ready),
    .result_valid_o(div_result_valid)
);

fast_mul imul(
    .clk_i(clk),
    .rst_i(rst),
    .stall_i(pipe_ctrl_io.recovery_stall || pipe_ctrl_io.load_wake_up_predict_failed), //stall result when recovery
    .a_i(operand_A),
    .b_i(operand_B),
    .req_i(muldiv_req && is_mul_instr),
    .kill_i(IsBrROBKill(pipe_ctrl_io.recovery_flush_BCAST, exe_io.exe_muldiv.commit_addr)),
    .operation_sel_i(rr_io.alu_muldiv_pack.ctrl_signal),
    .mul_result_o(mul_result),
    .ready_o(mul_ready),
    .result_valid_o(mul_result_valid),
    .early_wake_up_o(mul_early_wake_up)
);


always_ff @(posedge clk) begin
    if (rst)
        muldiv_busy <= 0;
    else
        muldiv_busy <= ~muldiv_ready || muldiv_req;
end

//TODO: spec flush
always_ff @(posedge clk) begin
    if (rst) begin
        exe_io.alu1_wb.valid <= 0;
        exe_io.alu1_wb.wb_addr <= 0;
        exe_io.alu1_wb.wb_data <= 0;
`ifdef FALCO_SIM_DEBUG
        exe_io.alu1_wb.pc <= 0;
`endif
    end else if (pipe_ctrl_io.recovery_stall) begin
        exe_io.alu1_wb.valid <= exe_io.alu1_wb.valid;
        exe_io.alu1_wb.wb_addr <= exe_io.alu1_wb.wb_addr;
        exe_io.alu1_wb.wb_data <= exe_io.alu1_wb.wb_data;
`ifdef FALCO_SIM_DEBUG
        exe_io.alu1_wb.pc <= exe_io.alu1_wb.pc;
`endif
    end else begin
        exe_io.alu1_wb.valid <= alu_result_valid;
        exe_io.alu1_wb.wb_addr <= rr_io.alu_muldiv_pack.rd_addr;
        exe_io.alu1_wb.wb_data <= alu_result;
`ifdef FALCO_SIM_DEBUG
        exe_io.alu1_wb.pc <= rr_io.alu_muldiv_pack.pc;
`endif
    end
end

always_comb begin
    if (mul_result_valid)
        muldiv_result = mul_result;
    else
        muldiv_result = div_result;
end

always_comb muldiv_result_valid = mul_result_valid | div_result_valid;
always_comb muldiv_BCAST_valid = mul_early_wake_up | div_result_valid;

always_comb muldiv_ready = div_ready && mul_ready;

always_ff @(posedge clk) begin
    if (rst) begin
        exe_io.muldiv_wb.wb_data <= 0;
    end else if (pipe_ctrl_io.recovery_stall) begin
        exe_io.muldiv_wb.wb_data <= exe_io.muldiv_wb.wb_data;
    end else begin
        exe_io.muldiv_wb.wb_data <= muldiv_result;
    end
end

always_ff @(posedge clk) begin
    if (rst) begin
        exe_io.muldiv_wb.wb_addr <= 0;
`ifdef FALCO_SIM_DEBUG
        exe_io.muldiv_wb.pc <= 0;
`endif
    end else if (muldiv_req && ~pipe_ctrl_io.recovery_stall) begin
        exe_io.muldiv_wb.wb_addr <= rr_io.alu_muldiv_pack.rd_addr;
`ifdef FALCO_SIM_DEBUG
        exe_io.muldiv_wb.pc <= rr_io.alu_muldiv_pack.pc;
`endif
    end else begin
        exe_io.muldiv_wb.wb_addr <= exe_io.muldiv_wb.wb_addr;
`ifdef FALCO_SIM_DEBUG
        exe_io.muldiv_wb.pc <= exe_io.muldiv_wb.pc;
`endif
    end
end

always_ff @(posedge clk) begin
    if (rst) begin
        exe_io.muldiv_wb.valid <= 0;
    end else if (pipe_ctrl_io.recovery_stall) begin
        exe_io.muldiv_wb.valid <= exe_io.muldiv_wb.valid;
    end else begin
        exe_io.muldiv_wb.valid <= muldiv_result_valid;
    end
end

always_ff @(posedge clk) begin
    if (rst) begin
        exe_io.exe_ALU_1.valid <= 0;
        exe_io.exe_ALU_1.commit_addr <= 0;
    end else if (pipe_ctrl_io.recovery_stall) begin
        exe_io.exe_ALU_1.valid <= 0;
        exe_io.exe_ALU_1.commit_addr <= 0;
    end else begin
        exe_io.exe_ALU_1.valid <= alu_result_valid;
        exe_io.exe_ALU_1.commit_addr <= rr_io.alu_muldiv_pack.rob_tag;
    end
end

always_ff @(posedge clk) begin
    if (rst) begin
        exe_io.exe_muldiv.valid <= 0;
    end else if (pipe_ctrl_io.recovery_stall) begin //don't finish instruction when recovery
        exe_io.exe_muldiv.valid <= 0;
    end else begin
        exe_io.exe_muldiv.valid <= IsBrROBKill(pipe_ctrl_io.recovery_flush_BCAST, exe_io.exe_muldiv.commit_addr)
                                    ? 0 : muldiv_result_valid;
    end
end

`ifdef FALCO_SIM_DEBUG
rob_tag_t mul_div_tag /*verilator public*/;
rob_tag_t mul_div_rr_tag /*verilator public*/;

always_comb mul_div_tag = exe_io.exe_muldiv.commit_addr;
always_comb mul_div_rr_tag = rr_io.alu_muldiv_pack.rob_tag;
`endif

always_ff @(posedge clk) begin
    if (rst) begin
        exe_io.exe_muldiv.commit_addr <= 0;
    end else if (muldiv_req) begin
        exe_io.exe_muldiv.commit_addr <= rr_io.alu_muldiv_pack.rob_tag;
    end else begin
        exe_io.exe_muldiv.commit_addr <= exe_io.exe_muldiv.commit_addr;
    end
end
//===================================================================================
// Broadcast to RNDS (busy_list) ,INT_IQ,MEM_IQ
//===================================================================================

always_ff @(posedge clk) begin
    if (rst) begin
        exe_io.BCAST_ALU_1.valid <= 0;
        exe_io.BCAST_ALU_1.prf_addr <= 0;
    end else if (alu_result_valid)begin //TODO: csr valid
        exe_io.BCAST_ALU_1.valid <= alu_result_valid;
        exe_io.BCAST_ALU_1.prf_addr <= rr_io.alu_muldiv_pack.rd_addr;
    end else begin
        exe_io.BCAST_ALU_1.valid <= 0;
        exe_io.BCAST_ALU_1.prf_addr <= 0;
    end
end

always_ff @(posedge clk) begin
    if (rst) begin
        exe_io.BCAST_muldiv.valid <= 0;
    end else if (pipe_ctrl_io.recovery_stall || pipe_ctrl_io.load_wake_up_predict_failed) begin
        exe_io.BCAST_muldiv.valid <= exe_io.BCAST_muldiv.valid;
    end else begin
        exe_io.BCAST_muldiv.valid <= muldiv_result_valid;//muldiv_BCAST_valid;
    end
end

always_ff @(posedge clk) begin
    if (rst) begin
        exe_io.BCAST_muldiv.prf_addr <= 0;
    end else if (muldiv_req && ~pipe_ctrl_io.recovery_stall) begin
        exe_io.BCAST_muldiv.prf_addr <= rr_io.alu_muldiv_pack.rd_addr;
    end else begin
        exe_io.BCAST_muldiv.prf_addr <= exe_io.BCAST_muldiv.prf_addr;
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
