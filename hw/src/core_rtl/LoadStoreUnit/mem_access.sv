`timescale 1ns/1ps
// =============================================================================
//  Program : mem_access.sv
//  Author  : Hon-Chou Dai
//  Date    : May/15/2021
// -----------------------------------------------------------------------------
//  Description:
//  Get data from TCM then broadcast to other stage.
// -----------------------------------------------------------------------------
//  Revision information:
//
//    August/07/2021, by Chun-Wei Chao:
//      Combine the data from TCM and store buffer.
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

// ==========================================
// Mem Access stage
// ==========================================
// Load operation: 
//      1. Get Load Data From DCache/DTCM
//      2. commit load operation
//      3. load data write back and broad cast
// Store operation:
//      nop

module mem_access(
    input clk,
    input rst,
    input core_load_data_resp_t load_data_resp,
    mem_access_io.MemAccess ma_io,
    load_store_unit_io.MemAccess lsu_io,
    pipeline_control_recovery_io.MemAccess pipe_ctrl_io
);

logic load_data_valid;
xlen_data_t load_forward_data_reg;
xlen_data_t load_primitive_data;
xlen_data_t load_alignment_data;
xlen_data_t load_data/*verilator public*/;

xlen_data_t load_data_delay;
logic mem_access_stall;
logic mem_access_stall_delay;

//WB reg
logic load_forward_data_valid_reg;
byte_mask_t load_forward_hit_mask_reg;
logic [1:0] load_addr_alignment_reg;
logic mem_load_ext_sel_reg;
mem_op_size_t load_mem_input_sel_reg;
byte_mask_t load_byte_sel_reg;
rob_tag_t load_rob_tag_reg;


always_comb
    mem_access_stall =
        pipe_ctrl_io.recovery_stall ;

always_ff @( posedge clk )
    if (rst)
        mem_access_stall_delay <= 0;
    else
        mem_access_stall_delay <= mem_access_stall;

always_ff @( posedge clk ) begin
    if (rst) begin
        load_forward_data_reg <= 0;
        load_addr_alignment_reg <= 0;
        mem_load_ext_sel_reg <= 0;
        load_mem_input_sel_reg <= MEM_OP_DUMMY0;
        load_byte_sel_reg <= 4'b0;
        load_forward_data_valid_reg <= 0;
        load_rob_tag_reg <= 0;
    end else if (mem_access_stall) begin
        load_forward_data_reg <= load_forward_data_reg;
        load_addr_alignment_reg <= load_addr_alignment_reg;
        mem_load_ext_sel_reg <= mem_load_ext_sel_reg;
        load_mem_input_sel_reg <= load_mem_input_sel_reg;
        load_byte_sel_reg <= load_byte_sel_reg;
        load_forward_data_valid_reg <= load_forward_data_valid_reg;
        load_rob_tag_reg <= load_rob_tag_reg;
    end else begin
        load_forward_data_reg <= lsu_io.lsu_pack.load_forward_data;
        load_forward_hit_mask_reg <= lsu_io.lsu_pack.load_forward_hit_mask;
        load_addr_alignment_reg <= lsu_io.lsu_pack.load_addr_alignment;
        mem_load_ext_sel_reg <= lsu_io.lsu_pack.mem_load_ext_sel;
        load_mem_input_sel_reg <= lsu_io.lsu_pack.load_mem_input_sel;
        load_byte_sel_reg <= lsu_io.lsu_pack.load_byte_sel;
        load_forward_data_valid_reg <= lsu_io.lsu_pack.load_forward_data_valid;
        load_rob_tag_reg <= lsu_io.lsu_pack.rob_tag;
    end
end

always_ff @( posedge clk )
    if (rst)
        load_data_delay <= 0;
    else if (mem_access_stall_delay)
        load_data_delay <= load_data_delay;
    else if (load_forward_data_valid_reg)
        load_data_delay <= load_primitive_data;
    else
        load_data_delay <= load_data_resp.load_data;
// Load data src MUX
always_comb begin
    if (mem_access_stall_delay)
        load_primitive_data = load_data_delay;
    else if (load_forward_data_valid_reg) begin
        case(load_forward_hit_mask_reg)
        4'b0001: load_primitive_data = {load_data_resp.load_data[31:8], load_forward_data_reg[7:0]};
        4'b0011: load_primitive_data = {load_data_resp.load_data[31:16], load_forward_data_reg[15:0]};
        4'b1111: load_primitive_data = load_forward_data_reg;
        4'b0010: load_primitive_data = {load_data_resp.load_data[31:16], load_forward_data_reg[15:8], load_data_resp.load_data[7:0]};
        4'b0100: load_primitive_data = {load_data_resp.load_data[31:24], load_forward_data_reg[23:16], load_data_resp.load_data[15:0]};
        4'b1100: load_primitive_data = {load_forward_data_reg[31:16], load_data_resp.load_data[15:0]};
        4'b1000: load_primitive_data = {load_forward_data_reg[31:24], load_data_resp.load_data[23:0]};
        4'b0101: load_primitive_data = {load_data_resp.load_data[31:24], load_forward_data_reg[23:16], load_data_resp.load_data[15:8], load_forward_data_reg[7:0]};
        4'b0110: load_primitive_data = {load_data_resp.load_data[31:24], load_forward_data_reg[23:8], load_data_resp.load_data[7:0]};
        4'b0111: load_primitive_data = {load_data_resp.load_data[31:24], load_forward_data_reg[23:0]};
        4'b1001: load_primitive_data = {load_forward_data_reg[31:24], load_data_resp.load_data[23:8], load_forward_data_reg[7:0]};
        4'b1010: load_primitive_data = {load_forward_data_reg[31:24], load_data_resp.load_data[23:16], load_forward_data_reg[15:8], load_data_resp.load_data[7:0]};
        4'b1011: load_primitive_data = {load_forward_data_reg[31:24], load_data_resp.load_data[23:16], load_forward_data_reg[15:0]};
        4'b1101: load_primitive_data = {load_forward_data_reg[31:16], load_data_resp.load_data[15:8], load_forward_data_reg[7:0]};
        4'b1110: load_primitive_data = {load_forward_data_reg[31:8], load_data_resp.load_data[7:0]};
        default: load_primitive_data = 0;  
        endcase
    end else 
        load_primitive_data = load_data_resp.load_data;
end

always_comb
begin
    case (load_addr_alignment_reg)
        2'b00: load_alignment_data = load_primitive_data;
        2'b01: load_alignment_data = {load_primitive_data[ 7: 0], load_primitive_data[31: 8]};
        2'b10: load_alignment_data = {load_primitive_data[15: 0], load_primitive_data[31: 16]};
        2'b11: load_alignment_data = {load_primitive_data[23: 0], load_primitive_data[31: 24]};
        default: load_alignment_data = load_primitive_data;
    endcase
end

always_comb
begin
    case (load_mem_input_sel_reg)
        MEM_OP_BYTE: load_data = mem_load_ext_sel_reg ?
                          {24'b0, load_alignment_data[7 : 0]} :
                          {{24{load_alignment_data[7]}}, load_alignment_data[7 : 0]};   // load byte
        MEM_OP_HALF_WORD: load_data = mem_load_ext_sel_reg ?
                          {16'b0, load_alignment_data[15 : 0]} :
                          {{16{load_alignment_data[15]}}, load_alignment_data[15 : 0]}; // load half word
        MEM_OP_WORD: load_data = load_alignment_data;                                   // load word
        MEM_OP_DUMMY0: load_data = 0;
    endcase
end

always_ff @(posedge clk) begin
    if (rst)
        load_data_valid <= 0;
    else if (mem_access_stall)
        load_data_valid <= load_data_valid;
    else if (lsu_io.lsu_pack.load_forward_data_valid || lsu_io.lsu_pack.load_mem_ready)
        load_data_valid <= lsu_io.lsu_pack.load_instr_valid;
    else
        load_data_valid <= 0;
end

always_comb begin
    if (rst) begin
        ma_io.load_wb.wb_data = load_data;
        ma_io.load_wb.valid = load_data_valid;
    end else begin
        ma_io.load_wb.wb_data = load_data;
        ma_io.load_wb.valid = load_data_valid;
    end
end
always_ff @(posedge clk) begin
    if (rst) begin
        ma_io.load_wb.wb_addr <= 0;
`ifdef FALCO_SIM_DEBUG
        ma_io.load_wb.pc <= 0;
`endif
    end else if (mem_access_stall) begin
        ma_io.load_wb.wb_addr <= ma_io.load_wb.wb_addr;
`ifdef FALCO_SIM_DEBUG
        ma_io.load_wb.pc <= ma_io.load_wb.wb_addr;
`endif
    end else begin
        ma_io.load_wb.wb_addr <= lsu_io.lsu_pack.rd_addr;
`ifdef FALCO_SIM_DEBUG
        ma_io.load_wb.pc <= lsu_io.lsu_pack.pc;
`endif
    end
end

always_ff @( posedge clk ) begin
    if (rst) begin
        ma_io.load_commit.valid <= 0;
        ma_io.load_commit.commit_addr <= 0;
    end else begin
        ma_io.load_commit.valid <= load_data_valid;
        ma_io.load_commit.commit_addr <= load_rob_tag_reg;
    end
end

`ifdef FALCO_SIM_DEBUG
    rob_tag_t ma_rob_tag;
    rob_tag_t ma_rob_tag_dly;
    xlen_data_t ma_wb_data;
    prf_specifier_t ma_wb_prf;
    logic ma_wb_valid;
    always_ff @(posedge clk) begin
        if (rst)
            ma_rob_tag <= 0;
        else if (mem_access_stall)
            ma_rob_tag <= ma_rob_tag;
        else
            ma_rob_tag <= lsu_io.lsu_pack.rob_tag;
    end
    always_ff@(posedge clk) begin
        ma_rob_tag_dly <= ma_rob_tag;
        ma_wb_data <= ma_io.load_wb.wb_data;
        ma_wb_prf <= ma_io.load_wb.wb_addr;
        ma_wb_valid <= ma_io.load_wb.valid;
    end
    function logic [7:0] ver_ma_rob_tag;
        /*verilator public*/
        ver_ma_rob_tag = {2'b0,ma_rob_tag};
    endfunction
    function logic [31:0] ver_lsu_load_dmem_data;
        /*verilator public*/
        ver_lsu_load_dmem_data = load_data_resp.load_data;
    endfunction
    function logic [31:0] ver_lsu_load_wb_prf_addr;
        /*verilator public*/
        ver_lsu_load_wb_prf_addr = ma_io.load_wb.wb_addr;
    endfunction
    function logic [31:0] ver_lsu_load_wb_prf_valid;
        /*verilator public*/
        ver_lsu_load_wb_prf_valid = ma_io.load_wb.valid;
    endfunction
    function logic [31:0] ver_lsu_load_wb_prf_data;
        /*verilator public*/
        ver_lsu_load_wb_prf_data = ma_io.load_wb.wb_data;
    endfunction
    function logic [31:0] ver_lsu_load_wb_pc;
        /*verilator public*/
        ver_lsu_load_wb_pc = ma_io.load_wb.pc;
    endfunction
    
    function logic [31:0] ver_ma_load_wb_prf_valid;
        /*verilator public*/
        ver_ma_load_wb_prf_valid = ma_wb_valid;
    endfunction
    function logic [31:0] ver_ma_load_wb_prf_data;
        /*verilator public*/
        ver_ma_load_wb_prf_data = ma_wb_data;
    endfunction
    function logic [31:0] ver_ma_load_wb_prf_addr;
        /*verilator public*/
        ver_ma_load_wb_prf_addr = ma_wb_prf;
    endfunction

    /////////////////////////////////////////////////////////
   
    function logic [31:0] ver_from_lsu_instr_valid;
        /*verilator public*/
        ver_from_lsu_instr_valid =  lsu_io.lsu_pack.load_instr_valid;
    endfunction
    function logic [31:0] ver_from_lsu_tag;
        /*verilator public*/
        ver_from_lsu_tag = lsu_io.lsu_pack.rob_tag;
    endfunction
    function logic [31:0] ver_from_lsu_forward_hit;
        /*verilator public*/
        ver_from_lsu_forward_hit = lsu_io.lsu_pack.load_forward_data_valid;
    endfunction
    function logic [31:0] ver_from_lsu_forward_hit_mask;
        /*verilator public*/
        ver_from_lsu_forward_hit_mask = lsu_io.lsu_pack.load_forward_hit_mask;
    endfunction
    function logic [31:0] ver_from_lsu_forward_hit_data;
        /*verilator public*/
        ver_from_lsu_forward_hit_data = lsu_io.lsu_pack.load_forward_data;
    endfunction
    function logic [31:0] ver_from_lsu_load_data;
        /*verilator public*/
        ver_from_lsu_load_data = load_data_resp.load_data;
    endfunction
    function logic [31:0] ver_from_lsu_load_mask;
        /*verilator public*/
        ver_from_lsu_load_mask = lsu_io.lsu_pack.load_byte_sel;
    endfunction
    function logic [31:0] ver_from_lsu_combine_data;
        /*verilator public*/
        ver_from_lsu_combine_data = load_primitive_data;
    endfunction
`endif

endmodule
