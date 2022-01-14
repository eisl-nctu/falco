`timescale 1ns/1ps
// =============================================================================
//  Program : load_buffer_select.sv
//  Author  : Chun-Wei Chao
//  Date    : August/07/2021
// -----------------------------------------------------------------------------
//  Description:
//  Not use in Falco in this version.
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

module load_buffer_select (
    input                   valid[LDB_NUM],
    output [LDB_WIDTH-1:0]  select_index,
    output                  select_index_valid
);

always_comb begin
    if(~valid[0]) begin
            select_index = 0;
            select_index_valid = 1;
    end
    else if(~valid[1]) begin
            select_index = 1;
            select_index_valid = 1;
    end
    else if(~valid[2]) begin
            select_index = 2;
            select_index_valid = 1;
    end
    else if(~valid[3]) begin
            select_index = 3;
            select_index_valid = 1;
    end
    else if(~valid[4]) begin
            select_index = 4;
            select_index_valid = 1;
    end
    else if(~valid[5]) begin
            select_index = 5;
            select_index_valid = 1;
    end
    else if(~valid[6]) begin
            select_index = 6;
            select_index_valid = 1;
    end
    else if(~valid[7]) begin
            select_index = 7;
            select_index_valid = 1;
    end
    else if(~valid[8]) begin
            select_index = 8;
            select_index_valid = 1;
    end
    else if(~valid[9]) begin
            select_index = 9;
            select_index_valid = 1;
    end
    else if(~valid[10]) begin
            select_index = 10;
            select_index_valid = 1;
    end
    else if(~valid[11]) begin
            select_index = 11;
            select_index_valid = 1;
    end
    else if(~valid[12]) begin
            select_index = 12;
            select_index_valid = 1;
    end
    else if(~valid[13]) begin
            select_index = 13;
            select_index_valid = 1;
    end
    else if(~valid[14]) begin
            select_index = 14;
            select_index_valid = 1;
    end
    else if(~valid[15]) begin
            select_index = 15;
            select_index_valid = 1;
    end
    else begin
        select_index = 0;
        select_index_valid = 0;
    end
end


endmodule
