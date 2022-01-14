// =============================================================================
//  Program : sram.v
//  Author  : Jin-you Wu
//  Date    : Oct/22/2018
// -----------------------------------------------------------------------------
//  Description:
//  This module synthesizes BRAM for cache storage.
// -----------------------------------------------------------------------------
//  Revision information:
//
//  None.
// -----------------------------------------------------------------------------
//  License information:
//
//  This software is released under the BSD-3-Clause Licence,
//  see https://opensource.org/licenses/BSD-3-Clause for details.
//  In the following license statements, "software" refers to the
//  "source code" of the complete hardware/software system.
//
//  Copyright 2019,
//                    Embedded Intelligent Systems Lab (EISL)
//                    Deparment of Computer Science
//                    National Chiao Tung Uniersity
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

module sram
#(parameter N_ENTRIES = 1024, DATA_WIDTH = 256, MEM_NAME = "bootrom.mem")
(
    input                           clk,
    input                           en,
    input                           we,
    input  [$clog2(N_ENTRIES)-1: 0] addr,
    input  [DATA_WIDTH-1: 0]        data_i,
    output reg [DATA_WIDTH-1: 0]    data_o
);

(*ram_style = "block" *) reg [DATA_WIDTH-1 : 0] RAM [N_ENTRIES-1: 0];

`ifdef USE_PRELOAD_TCM
    initial begin
        $display("load %s...",MEM_NAME);
        $readmemh(MEM_NAME,RAM);
    end
`elsif FALCO_SIM_DEBUG
`else
    initial begin
        $readmemh(MEM_NAME,RAM);
    end
`endif
// ------------------------------------
// Read operation
// ------------------------------------
always@(posedge clk)
begin
    if (en)
    begin
        data_o <= RAM[addr];
    end
end

// ------------------------------------
// Write operation
// ------------------------------------
always@(posedge clk)
begin
    if (en & we)
    begin
        RAM[addr] <= data_i;
    end
end

endmodule   // sram
