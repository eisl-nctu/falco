`timescale 1ns/1ps
// =============================================================================
//  Program : fast_mul.sv
//  Author  : Hon-Chou Dai
//  Date    : May/15/2021
// -----------------------------------------------------------------------------
//  Description:
//  MUL unit for falco.
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

// IO is combinational
// input should be reg
// stall handle by outside
// no pipeline support

module fast_mul(
    input                   clk_i,
    input                   rst_i,
    input                   stall_i,
    input  xlen_data_t      a_i/*verilator public*/,
    input  xlen_data_t      b_i/*verilator public*/,
    input  logic            req_i,
    input  logic            kill_i,
    input  logic [ 1 : 0]   operation_sel_i,
    output xlen_data_t      mul_result_o,
    output logic            ready_o,
    output logic            result_valid_o,
    output logic            early_wake_up_o
);

typedef struct packed {
    logic [1:0] operation_sel;
    logic is_a_neg;
    logic is_b_neg;
    logic mul_overflow;
} mul_stat_t;

function [XLEN_WIDTH-1 : 0] abs;
    input [XLEN_WIDTH-1 : 0] num;

    abs = num[XLEN_WIDTH-1] ? -num : num;
endfunction

// ================================================================================
// pipeline signal
mul_stat_t mul_p0;
mul_stat_t mul_p1;
mul_stat_t mul_p2;
logic valid0,valid1;

// ================================================================================
// Operation signals
//
wire operation_mul_in, operation_mulh_in, operation_mulhsu_in, operation_mulhu_in;

assign operation_mul_in    =   (operation_sel_i == 2'b00);
assign operation_mulh_in   =   (operation_sel_i == 2'b01);
assign operation_mulhsu_in =   (operation_sel_i == 2'b10);
assign operation_mulhu_in  =   (operation_sel_i == 2'b11);

wire is_a_neg, is_b_neg, mul_overflow;

assign is_a_neg  = a_i[31] & (operation_mulh_in | operation_mulhsu_in);
assign is_b_neg  = b_i[31] & (operation_mulh_in);
assign mul_overflow   = (is_a_neg & (a_i[30: 0] == 31'b0));

wire [XLEN_WIDTH-1 : 0] op_a, op_b;

assign op_a = ( | is_a_neg)? abs(a_i) : a_i;
assign op_b = ( | is_b_neg)? abs(b_i) : b_i;



// For the fast multiplier using Xilinx FPGA IPs.
reg  [XLEN_WIDTH*2-1 : 0] fast_result, mul0, mul1;
wire [XLEN_WIDTH*2-1 : 0] result;
wire [XLEN_WIDTH-1 : 0] mull, mulh;

assign mull = result[31 : 0];
assign mulh = result[63 : 32];


// Here, we use Xilinx synthesizer coding style to infer a
// 3-cycel multiplier. On KC-705@100MHz, you can even infer
// a single-cycle 32-bit x 32-bit multipler.
//
// 1 latency stage on operands
// 3 latency stage after the multiplication
always @(posedge clk_i)
begin
    if (rst_i) begin
        fast_result <= 0;
    end else if (stall_i) begin
        mul0 <= mul0;
        mul1 <= mul1;
        fast_result <= fast_result;
    end else begin
        mul0 <= op_a * op_b;
        mul1 <= mul0;
        fast_result <= mul1;
    end
end

wire operation_mul_final, 
     operation_mulh_final,
     operation_mulhsu_final,
     operation_mulhu_final;
wire signed_adjust_final;

assign signed_adjust_final = mul_p2.is_a_neg ^ mul_p2.is_b_neg;
assign operation_mul_final    =   (mul_p2.operation_sel == 2'b00);
assign operation_mulh_final   =   (mul_p2.operation_sel == 2'b01);
assign operation_mulhsu_final =   (mul_p2.operation_sel == 2'b10);
assign operation_mulhu_final  =   (mul_p2.operation_sel == 2'b11);
assign result =  (operation_mulh_final | operation_mulhu_final)? (signed_adjust_final)? -fast_result : fast_result
                    : (operation_mulhsu_final)? (mul_p2.is_a_neg)? -fast_result : fast_result : fast_result;

// ================================================================================
// Output signals
//

// always_comb begin
//     ready_o = 1; //always ready
// end

always @(posedge clk_i)
begin
    if ( rst_i || kill_i) begin
        valid0 <= 0;
        valid1 <= 0;
        result_valid_o <= 0;
        // state pipeline
        mul_p0 <= 0;
        mul_p1 <= 0;
        mul_p2 <= 0;
    end else if (stall_i) begin
        valid0 <= valid0;
        valid1 <= valid1;
        result_valid_o <= result_valid_o;

        mul_p0 <= mul_p0;
        mul_p1 <= mul_p1;
        mul_p2 <= mul_p2;
    end else begin
        valid0 <= req_i;
        valid1 <= valid0;
        result_valid_o <= valid1;
        //state pipeline
        mul_p0.operation_sel <= operation_sel_i;
        mul_p0.is_a_neg <= is_a_neg;
        mul_p0.is_b_neg <= is_b_neg;
        mul_p0.mul_overflow <= mul_overflow;

        mul_p1 <= mul_p0;
        mul_p2 <= mul_p1;
    end
end

assign mul_result_o = ( {32{operation_mul_final}} & mull)
                      | ( {32{operation_mulh_final | operation_mulhsu_final | operation_mulhu_final}} & mulh);
                      ;


assign ready_o = ~(valid0 || valid1 || result_valid_o);
assign early_wake_up_o = valid1;

endmodule
