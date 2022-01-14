`timescale 1ns/1ps
// =============================================================================
//  Program : AGU_io.sv
//  Author  : Hon-Chou Dai
//  Date    : May/15/2021
// -----------------------------------------------------------------------------
//  Description:
//  The interface between AGU stage and LSU stage.
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

interface AGU_io;
    logic instr_valid;
    agu_pack_t agu_pack;
    logic memory_align_exception;
    
    modport AGU(
        output instr_valid,
        output agu_pack,
        output memory_align_exception
    );

    modport LSU(
        input instr_valid,
        input agu_pack,
        input memory_align_exception
    );

`ifdef FALCO_SIM_DEBUG
    function [31:0] ver_AGU_IO_instr_pc;
        /*verilator public*/
        ver_AGU_IO_instr_pc = (instr_valid ? agu_pack.pc : 32'hFFFFFFFF);
    endfunction
    function [7:0] ver_AGU_IO_is_store;
        /*verilator public*/
        ver_AGU_IO_is_store = (agu_pack.mem_is_store ? 1 : 0);
    endfunction
    function [31:0] ver_AGU_IO_store_data;
        /*verilator public*/
        ver_AGU_IO_store_data = agu_pack.store_data;
    endfunction
    function [31:0] ver_AGU_IO_access_addr;
        /*verilator public*/
        ver_AGU_IO_access_addr = agu_pack.access_addr;
    endfunction
    function [7:0] ver_AGU_IO_byte_sel;
        /*verilator public*/
        ver_AGU_IO_byte_sel = agu_pack.byte_sel;
    endfunction
    function [7:0] ver_AGU_IO_rd_addr;
        /*verilator public*/
        ver_AGU_IO_rd_addr = agu_pack.rd_addr;
    endfunction
`endif
endinterface //AGU_io
