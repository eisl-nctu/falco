`timescale 1ns/1ps
// =============================================================================
//  Program : TCM.sv
//  Author  : Chun-Wei Chao
//  Date    : November/18/2021
// -----------------------------------------------------------------------------
//  Description:
//  Combine DTCM & ITCM, copy same memory file to two BRAM.
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

module TCM
#(
    parameter int MEM_SIZE /*verilator public*/= TCM_SIZE * 1024, //small memory for verilator
    parameter int MEM_OFFSET /*verilator public*/= TCM_MEM_OFFSET
)
(
    input  logic                      clk,
    input  logic                      rst,
    input  logic                      stall_i,

    input core_load_ck_hit_req_t      load_ck_hit_req,
    output core_load_hit_resp_t       load_hit_resp,
    input core_load_data_req_t        load_data_req,
    output core_load_data_resp_t      load_data_resp,
    input core_store_req_t            store_req,
    output core_store_resp_t          store_resp,

    input instruction_req_t core_req,
    output instruction_resp_t itcm_resp
);

function integer clogb2 (input integer bit_depth);
    begin
        for(clogb2=0; bit_depth>0; clogb2=clogb2+1)
            bit_depth = bit_depth >> 1;
    end
endfunction

localparam DTCM_WORD_NUM = TCM_SIZE * 1024 / 4;
localparam PART_ADDR_WIDTH = clogb2(DTCM_WORD_NUM);
localparam DTCM_N_ENTRYS = DTCM_WORD_NUM;
localparam DTCM_ADDRW = clogb2(DTCM_N_ENTRYS);

localparam int ITCM_N_SIZE = (TCM_SIZE) * 1024; //byte
localparam int ITCM_ENTRY = ITCM_N_SIZE/4; //WORD
localparam int ITCM_BIT_WIDTH = $clog2(ITCM_ENTRY);
// Parameter        ////////////////////////////////////////////////////////////////
// same paramter with ICache to make itcm behavior same with ICache
// to prevent cross line fetch
localparam N_WAYS          = 4;
localparam WORDS_PER_LINE  = 8;
localparam N_LINES         = (ICACHE_SIZE*1024*8) / (N_WAYS*ICACHE_LINE_SIZE);

localparam WAY_BITS        = $clog2(N_WAYS);
localparam BYTE_BITS       = 2;
localparam WORD_BITS       = $clog2(WORDS_PER_LINE);
localparam LINE_BITS       = $clog2(N_LINES);
localparam NONTAG_BITS     = LINE_BITS + WORD_BITS + BYTE_BITS;
localparam TAG_BITS        = ADDR_WIDTH - NONTAG_BITS;
// Parameter        ////////////////////////////////////////////////////////////////

// DTCM logic
logic load_ready;
logic store_ready;

// ITCM logic
logic is_cross_line_fetch;
logic insrt0_ready;
logic insrt1_ready;
logic [ITCM_BIT_WIDTH-1:0] instr0_sram_addr /*verilator public*/;
logic [ITCM_BIT_WIDTH-1:0] instr1_sram_addr /*verilator public*/;
Falco_pkg::raw_instruction_t instr0_instr /*verilator public*/;
Falco_pkg::raw_instruction_t instr1_instr /*verilator public*/;

`ifdef FALCO_SIM_DEBUG

// logic load_req /*verilator public*/;
// always_comb load_req = load_ck_hit_req.load_req;
// logic store_req /*verilator public*/;
// always_comb store_req = store_req.store_req;
logic [PART_ADDR_WIDTH-1:0] load_total_addr /*verilator public*/;
always_comb load_total_addr = load_ck_hit_req.load_addr[PART_ADDR_WIDTH-1:0];
logic [PART_ADDR_WIDTH-1:0] store_total_addr /*verilator public*/;
always_comb store_total_addr = store_req.store_addr[PART_ADDR_WIDTH-1:0];
logic [31:0] store_data /*verilator public*/;
always_comb store_data = store_req.store_data;


`endif

always_comb begin
    store_resp.store_miss = 0;
    store_resp.store_finished = store_ready && store_req.store_req;
    load_hit_resp.load_hit = load_ck_hit_req.load_req; //always hit
    load_hit_resp.load_miss = 0;
end

integer i;

logic [PART_ADDR_WIDTH-1:0] store_part_addr /*verilator public*/;
logic [PART_ADDR_WIDTH-1:0] load_part_addr /*verilator public*/;
xlen_data_t load_data /*verilator public*/;

always_comb store_part_addr = store_req.store_addr[PART_ADDR_WIDTH-1:2];
always_comb load_part_addr = load_ck_hit_req.load_addr[PART_ADDR_WIDTH-1:2];

always_comb begin
        itcm_resp.raw_instr0 = instr0_instr;
        itcm_resp.raw_instr1 = instr1_instr;
end

always_comb
    is_cross_line_fetch = (core_req.instr0_addr[ADDR_WIDTH-1: WORD_BITS + BYTE_BITS]
                            != core_req.instr1_addr[ADDR_WIDTH-1: WORD_BITS+BYTE_BITS] ? 1 : 0);

typedef enum logic [1:0] {IDLE = 2'b0, READ} state_t;
state_t itcm_cur_state, itcm_next_state;

always_ff @(posedge clk) begin
	if (rst)
		itcm_cur_state <= IDLE;
	else
		itcm_cur_state <= itcm_next_state;
end
always_comb begin
	case (itcm_cur_state)
	  	IDLE: if (core_req.p_strobe) itcm_next_state = READ;
			else itcm_next_state = IDLE;
	  	READ: itcm_next_state = READ;
	  	default: itcm_next_state = IDLE;
	endcase
end

always_comb instr0_sram_addr = core_req.instr0_addr[ITCM_BIT_WIDTH+2:2];
always_comb instr1_sram_addr = core_req.instr1_addr[ITCM_BIT_WIDTH+2:2];

always_comb itcm_resp.instr0_valid = (itcm_cur_state == READ) ? 1 : 0;
always_comb itcm_resp.instr1_valid = itcm_resp.instr0_valid && ~is_cross_line_fetch;
always_comb itcm_resp.ready = itcm_resp.instr0_valid; //always ready

sram_dp
#(
    .XLEN(32),
    .N_ENTRIES(DTCM_N_ENTRYS),
`ifdef USE_PRELOAD_TCM
    .MEM_NAME("total_coremark.mem")
`else
    .MEM_NAME("total_dhry.mem")
//    .MEM_NAME("total_coremark.mem")
`endif
) DTCM(
	//load port
    .clk1_i(clk),
    .en1_i(1'b1),
    .we1_i(1'b0),
    .be1_i(4'b1111),
    .addr1_i(load_part_addr),
    .data1_i(32'b0),
    .data1_o(load_data),
    .ready1_o(load_ready),

	//store port
    .clk2_i(clk),
    .en2_i(1'b1),
    .we2_i(store_req.store_req),
    .be2_i(store_req.store_mask),
    .addr2_i(store_part_addr),
    .data2_i(store_req.store_data),
    .data2_o(),
    .ready2_o(store_ready)
); /*verilator  no_inline_module*/

sram_dp 
#(
    .XLEN(32),
    .N_ENTRIES(ITCM_ENTRY),
`ifdef USE_PRELOAD_TCM
    .MEM_NAME("total_coremark.mem")
`else
    .MEM_NAME("total_dhry.mem")
//    .MEM_NAME("total_coremark.mem")
`endif
) ITCM(
	//load port
    .clk1_i(clk),
    .en1_i(1'b1),
    .we1_i(1'b0),
    .be1_i(4'b1111),
    .addr1_i(instr0_sram_addr),
    .data1_i(32'b0),
    .data1_o(instr0_instr),
    .ready1_o(insrt0_ready),

	//store port
    .clk2_i(clk),
    .en2_i(1'b1),
    .we2_i(1'b0),
    .be2_i(4'b1111),
    .addr2_i(instr1_sram_addr),
    .data2_i(32'b0),
    .data2_o(instr1_instr),
    .ready2_o(insrt1_ready)
); /*verilator  no_inline_module*/

logic stall_delay;

always_ff @( posedge clk )
    if (rst)
        stall_delay <= 0;
    else
        stall_delay <= stall_i;

always_ff @( posedge clk ) begin
    if (rst) begin
        load_data_resp.load_data <= 0;
        load_data_resp.load_finished <= 0;
        load_data_resp.load_miss <= 1;
    end else if (stall_delay) begin
        load_data_resp.load_data <= load_data_resp.load_data;
        load_data_resp.load_finished <= load_data_resp.load_finished;
        load_data_resp.load_miss <= load_data_resp.load_miss;
    end else begin
        load_data_resp.load_data <= load_data;
        load_data_resp.load_finished <= load_ready;
        load_data_resp.load_miss <= ~load_ready;
    end
end

`ifdef FALCO_SIM_DEBUG
// function [31:0] readWord;
//     /*verilator public*/
//     input integer byte_addr;
//     begin
//         reg [31:0] tmp_addr;
//         tmp_addr = byte_addr - MEM_OFFSET;
//         if (byte_addr >= MEM_OFFSET &&  byte_addr < MEM_OFFSET + MEM_SIZE)
//             readWord = DTCM.RAM[tmp_addr[PART_ADDR_WIDTH-1:2]];
//         else
//             readWord = 32'hdeadbeef;
//     end
// endfunction

task writeWord;
    /*verilator public*/
    input integer byte_addr;
    input [31:0] val;
    begin
        reg [31:0] tmp_addr;
        tmp_addr = byte_addr - MEM_OFFSET;
        // if (byte_addr >= MEM_OFFSET &&  byte_addr < MEM_OFFSET + MEM_SIZE) begin
            DTCM.RAM[tmp_addr[PART_ADDR_WIDTH-1:2]] = val;
            ITCM.RAM[tmp_addr[PART_ADDR_WIDTH-1:2]] = val;
        // end
    end
endtask

// function [31:0] readWord;
//     /*verilator public*/
//     input integer byte_addr;
//     begin
//         reg [31:0] tmp_addr;
//         tmp_addr = byte_addr - MEM_OFFSET;
//         if (byte_addr >= MEM_OFFSET &&  byte_addr < MEM_OFFSET + MEM_SIZE) begin
//             if (tmp_addr[2] == 0) begin
//                 readWord = ITCM_BANK0.RAM[tmp_addr[ITCM_BIT_WIDTH+2:3]];
//             end else begin
//                 readWord = ITCM_BANK1.RAM[tmp_addr[ITCM_BIT_WIDTH+2:3]];
//             end
//         end else begin
//             readWord = 32'hdeadbeef;
//         end
//     end
//   endfunction

// task writeWord;
//     /*verilator public*/
//     input integer byte_addr;
//     input [31:0] val;
//     begin
//         reg [31:0] tmp_addr;
//         tmp_addr = byte_addr - MEM_OFFSET;
//         if (byte_addr >= MEM_OFFSET &&  byte_addr < MEM_OFFSET + MEM_SIZE) begin
//             if (tmp_addr[2] == 0)
//                 ITCM_BANK0.RAM[tmp_addr[ITCM_BIT_WIDTH+2:3]] = val;
//             else
//                 ITCM_BANK1.RAM[tmp_addr[ITCM_BIT_WIDTH+2:3]] = val;
//         end
//     end
// endtask
`endif

endmodule
