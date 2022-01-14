`timescale 1ns/1ps
// =============================================================================
//  Program : ID_stage.sv
//  Author  : Hon-Chou Dai
//  Date    : May/15/2021
// -----------------------------------------------------------------------------
//  Description:
//  Decode Stage, has two decoders.
// -----------------------------------------------------------------------------
//  Revision information:
//
//    August/07/2021, by Chun-Wei Chao:
//      Add Store Set ID table in this stage.
//      This information will used in next stage(RNDS), 
//      for predict whether the load instruction will have violation 
//      with other store instruction or not.
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

module ID_stage(
    input                   clk,
    input                   rst,

    // IF stage combination path for predict_pc
    input   pc_t            instr0_predict_pc,
    input   pc_t            instr1_predict_pc,
    IF_stage_io.ID_stage    if_stage_io,
    ID_stage_io.ID_stage    id_stage_io,
    load_store_unit_io.ID_stage lsu_io,
    pipeline_control_recovery_io.ID_stage pipe_ctrl_io
);

logic instr0_illegal_instr;
logic instr1_illegal_instr;

decoded_op_t decoded_instr0;
decoded_op_t decoded_instr1;

logic [LFST_WIDTH-1 : 0] instr0_store_set_id;
logic [LFST_WIDTH-1 : 0] instr1_store_set_id;

decoder instr0_decoder(
    .pc(if_stage_io.instr0_pc),
    .instruction(if_stage_io.raw_instr0),
    .predict_pc(instr0_predict_pc),

    .illegal_instr_o(instr0_illegal_instr),
    .decoded_instr(decoded_instr0)
);

decoder instr1_decoder(
    .pc(if_stage_io.instr1_pc),
    .instruction(if_stage_io.raw_instr1),
    .predict_pc(instr1_predict_pc),

    .illegal_instr_o(instr1_illegal_instr),
    .decoded_instr(decoded_instr1)
);

store_set_id_table SSIT(
    .clk(clk),
    .rst(rst),

    .instr0_pc(if_stage_io.instr0_pc[SSIT_WIDTH-1+2 : 2]), 
    .instr1_pc(if_stage_io.instr1_pc[SSIT_WIDTH-1+2 : 2]),
    .instr0_store_set_id(instr0_store_set_id),
    .instr1_store_set_id(instr1_store_set_id),

    //recovery signal
    .violation(lsu_io.store_set_violation), //from io
    .recovery_insrt0_pc(lsu_io.violation_load_pc), //from io
    .recovery_insrt1_pc(lsu_io.violation_store_pc), //from io
    .recovery_insrt0_id(lsu_io.violation_load_id), //from io
    .recovery_insrt1_id(lsu_io.violation_store_id), //from io
    .device_violation(lsu_io.device_violation), //from io
    .device_violation_pc(lsu_io.violation_store_pc) //from io
);

always_comb id_stage_io.instr0_store_set_id = instr0_store_set_id;
always_comb id_stage_io.instr1_store_set_id = instr1_store_set_id;

always_ff @(posedge clk) begin
    if (rst) begin
        id_stage_io.decoded_instr0 <= 0;
        id_stage_io.instr0_BHSR <= 0;
    end else if (pipe_ctrl_io.ID_flush) begin
        id_stage_io.decoded_instr0 <= 0;
        id_stage_io.instr0_BHSR <= 0;
    end else if (pipe_ctrl_io.ID_stall) begin
        id_stage_io.decoded_instr0 <= id_stage_io.decoded_instr0;
        id_stage_io.instr0_BHSR <= id_stage_io.instr0_BHSR;
    end else begin
        id_stage_io.decoded_instr0 <= decoded_instr0;
        id_stage_io.instr0_BHSR <= if_stage_io.current_instr0_BHSR;
    end
end

always_ff @(posedge clk) begin
    if (rst) begin
        id_stage_io.decoded_instr1 <= 0;
        id_stage_io.instr1_BHSR <= 0;
    end else if (pipe_ctrl_io.ID_flush) begin
        id_stage_io.decoded_instr1 <= 0;
        id_stage_io.instr1_BHSR <= 0;
    end else if (pipe_ctrl_io.ID_stall) begin
        id_stage_io.decoded_instr1 <= id_stage_io.decoded_instr1;
        id_stage_io.instr1_BHSR <= id_stage_io.instr1_BHSR;
    end else begin
        id_stage_io.decoded_instr1 <= decoded_instr1;
        id_stage_io.instr1_BHSR <= if_stage_io.current_instr1_BHSR;
    end 
end

always_ff @(posedge clk) begin
    if (rst) begin
        id_stage_io.instr0_valid <= 0;
    end else if (pipe_ctrl_io.ID_flush) begin
        id_stage_io.instr0_valid <= 0;
    end else if (pipe_ctrl_io.ID_stall && id_stage_io.instr0_issue) begin
        id_stage_io.instr0_valid <= 0;
    end else if (pipe_ctrl_io.ID_stall) begin
        id_stage_io.instr0_valid <= id_stage_io.instr0_valid;
    end else begin
        id_stage_io.instr0_valid <= if_stage_io.instr0_valid;
    end 
end 

always_ff @(posedge clk) begin
    if (rst) begin
        id_stage_io.instr1_valid <= 0;
    end else if (pipe_ctrl_io.ID_flush) begin
        id_stage_io.instr1_valid <= 0;
    end else if (pipe_ctrl_io.ID_stall && id_stage_io.instr1_issue) begin
        id_stage_io.instr1_valid <= 0;
    end else if (pipe_ctrl_io.ID_stall) begin
        id_stage_io.instr1_valid <= id_stage_io.instr1_valid;
    end else begin
        id_stage_io.instr1_valid <= if_stage_io.instr1_valid;
    end 
end 
endmodule
