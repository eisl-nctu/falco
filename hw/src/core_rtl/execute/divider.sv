`timescale 1ns/1ps
// =============================================================================
//  Program : divider.sv
//  Author  : Hon-Chou Dai
//  Date    : May/15/2021
// -----------------------------------------------------------------------------
//  Description:
//  Division unit for Falco.
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
//base on aquila muldiv.v
module divider(
    input                   clk_i,
    input                   rst_i,
    input                   stall_i,
    input  xlen_data_t      a_i,
    input  xlen_data_t      b_i,
    input  logic            req_i,
    input  logic            kill_i,
    input  logic [ 1 : 0]   operation_sel_i,
    output xlen_data_t      div_result_o,
    output logic            ready_o,
    output logic            result_valid_o
);


function xlen_data_t abs;
    input xlen_data_t num;

    abs = num[XLEN_WIDTH-1] ? -num : num;
endfunction

// ================================================================================
// Operation signals
//
wire operation_div, operation_divu, operation_rem, operation_remu;

assign operation_div  = (operation_sel_i == 2'b00);
assign operation_divu = (operation_sel_i == 2'b01);
assign operation_rem  = (operation_sel_i == 2'b10);
assign operation_remu = (operation_sel_i == 2'b11);

wire is_a_zero, is_b_zero,
     is_a_neg, is_b_neg, signed_adjust, mul_overflow;


assign is_a_zero = (a_i == 32'b0);
assign is_b_zero = (b_i == 32'b0);
assign is_a_neg  = a_i[31] & (operation_div | operation_rem);
assign is_b_neg  = b_i[31] & (operation_div | operation_rem);
assign signed_adjust = is_a_neg ^ is_b_neg;

wire [XLEN_WIDTH-1 : 0] op_a, op_b;

assign op_a = ( | is_a_neg)? abs(a_i) : a_i;
assign op_b = ( | is_b_neg)? abs(b_i) : b_i;

wire              is_calc_done;
reg  [XLEN_WIDTH-1 : 0] reg32;
reg  [XLEN_WIDTH*2 : 0] result;       // 65-bit, 1 extra bit
reg  [ 5 : 0]     cnt;


// For the slow shift-add binary multiplier.
assign is_calc_done = ~|cnt;

wire [XLEN_WIDTH-1 : 0]  quotient, remainder;


assign quotient = result[31 : 0];
assign remainder = result[64 : 33];

// ================================================================================
// Finite State Machine
//
localparam S_IDLE        = 3'b000,
           S_CALC        = 3'b001,
           S_SIGN_ADJUST = 3'b010,
           S_DONE        = 3'b011,
           S_STALL       = 3'b100,
           S_KILL        = 3'b101;
reg [2 : 0] S, S_nxt;

always @(posedge clk_i)
begin
    if (rst_i)
        S <= S_IDLE;
    else
        S <= S_nxt;
end

always_comb
begin
    case (S)
        S_IDLE:
            if (req_i) begin //kill should not happen in req_i
                if ((is_a_zero | is_b_zero))
                    S_nxt = S_DONE;
                else
                    S_nxt = S_CALC;
            end else begin
                S_nxt = S_IDLE;
            end
        S_CALC:
            if (kill_i)
                S_nxt = S_KILL;
            else
                S_nxt = (is_calc_done)? S_SIGN_ADJUST : S_CALC;
        S_SIGN_ADJUST:
            S_nxt = (kill_i) ? S_KILL : S_DONE;
        S_DONE:
            S_nxt = (stall_i)? S_STALL : S_IDLE;
        S_STALL:
            S_nxt = (stall_i)? S_STALL : S_IDLE;
        S_KILL:
            S_nxt = S_IDLE;
        default:
            S_nxt = S_IDLE;
    endcase
end

// ================================================================================
// Computation
//

wire div_sub = (result[63: 32] >= reg32);

wire [32 : 0] adder_opa, adder_opb;
assign adder_opa = -reg32;
assign adder_opb = result[63: 32];

wire [32 : 0] adder_tmp = adder_opa + adder_opb;
wire [64 : 0] result_tmp = {adder_tmp, result[31: 0]};



// Here, we use the slowest shift-add and shift-sub algorithms to
// implement the hardware divider and the multiplier. By default,
// the fast multiplier and the slow divider will be used for calculation.
//
always @(posedge clk_i) begin
    if ( (S == S_IDLE) & req_i) begin
        // divisor
        if (is_b_zero) begin  // divide by zero, generate special value
            result[64: 33] <= a_i;
            result[32]  <= 1'b0;
            result[31: 0] <= {32{1'b1}};
        end else begin
            cnt <= 'd32;
            reg32 <= op_b;
            result <= {1'b0, 32'b0, op_a};  // ext_bit | remainder | quotient
        end
    end else if (S == S_CALC) begin
        cnt <= cnt - 'd1;
        // slow divider: shift left
        result <= (div_sub)? {result_tmp[63 : 0], 1'b1} : {result[63 : 0], 1'b0};
    end else if (S == S_SIGN_ADJUST) begin
        result[64: 33] <= (is_a_neg)? -remainder : remainder;
        result[31: 0] <= (signed_adjust)? -quotient : quotient;
    end else begin
        result <= result;
    end
end

// ================================================================================
// Output signals
//
always_ff @(posedge clk_i) begin
    if (S == S_DONE || S == S_STALL || S == S_KILL || (S == S_IDLE && ~req_i))
        ready_o <= 1;
    else
        ready_o <= 0;
end

always_ff @(posedge clk_i) begin
    if ( S == S_DONE && ~kill_i)
        result_valid_o <= 1;
    else if ( (S == S_STALL || stall_i) && ~kill_i)
        result_valid_o <= result_valid_o;
    else
        result_valid_o <= 0;
end

always_ff @(posedge clk_i) begin
    if (S == S_DONE)
        div_result_o <= ( {32{operation_div | operation_divu}} & quotient)
                      | ( {32{operation_rem | operation_remu}} & remainder)
                      ;
    else
        div_result_o <= div_result_o;
end

endmodule
