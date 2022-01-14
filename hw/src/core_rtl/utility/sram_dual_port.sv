`timescale 1ns/1ps
// =============================================================================
//  Program : sram_dual_port.sv
//  Author  : Chun-Wei Chao
//  Date    : August/07/2021
// -----------------------------------------------------------------------------
//  Description:
//  This module is a dual port BRAM with initial zero data.
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

module sram_dual_port #(parameter RAM_WIDTH = 40, RAM_ADDR_BITS = 16)(
input clka, 
input clkb, 
input we_A,
input we_B,
input en_A,  
input en_B,
input  [RAM_ADDR_BITS-1 : 0] addr_A, 
input  [RAM_ADDR_BITS-1 : 0] addr_B,
input  [RAM_WIDTH-1 : 0] data_iA,
input  [RAM_WIDTH-1 : 0] data_iB,
output reg [RAM_WIDTH-1 : 0] data_oA,
output reg [RAM_WIDTH-1 : 0] data_oB);


reg [0:RAM_WIDTH-1] RAM [(2**RAM_ADDR_BITS) -1:0];

initial
begin
    $readmemh("initial_table.mem", RAM);//given zero initial data
end

// ------------------------------------
// BRAM Port-A read operation
// ------------------------------------
always@(posedge clka)
begin
  if (en_A & we_A)
    data_oA <= data_iA;
  else
    data_oA <= RAM[addr_A];
end

// ------------------------------------
// BRAM Port-B read operation
// ------------------------------------
always@(posedge clkb)
begin
  if (en_B & we_B)
    data_oB <= data_iB;
  else
    data_oB <= RAM[addr_B];
end

// ------------------------------------
// BRAM Port-A write operation
// ------------------------------------
always@(posedge clka)
begin
  if (en_A & we_A)
    RAM[addr_A] <= data_iA;
end

// ------------------------------------
// BRAM Port-B write operation
// ------------------------------------
always@(posedge clkb)
begin
  if (en_B & we_B)
    RAM[addr_B] <= data_iB;
end


endmodule
