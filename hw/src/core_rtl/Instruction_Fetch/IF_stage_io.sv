`timescale 1ns/1ps
// =============================================================================
//  Program : IF_stage_io.sv
//  Author  : Hon-Chou Dai
//  Date    : May/15/2021
// -----------------------------------------------------------------------------
//  Description:
//  Interface between IF stage and ID stage.
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

interface IF_stage_io;
    raw_instruction_t raw_instr0;
    raw_instruction_t raw_instr1;
    logic instr0_valid;
    logic instr1_valid;
    pc_t instr0_pc;
    pc_t instr1_pc;
    BHSR_t current_instr0_BHSR;
    BHSR_t current_instr1_BHSR;


    modport IF_stage(
        output raw_instr0,
        output raw_instr1,
        output instr0_valid,
        output instr1_valid,
        output instr0_pc,
        output instr1_pc,
        output current_instr0_BHSR,
        output current_instr1_BHSR
    );
    modport ID_stage(
        input raw_instr0,
        input raw_instr1,
        input instr0_valid,
        input instr1_valid,
        input instr0_pc,
        input instr1_pc,
        input current_instr0_BHSR,
        input current_instr1_BHSR
    );

`ifdef FALCO_SIM_DEBUG
    function [7:0] ver_check_IF_stage_valid;
        /*verilator public*/
        begin
            ver_check_IF_stage_valid = (instr0_valid ? 1 : 0);
        end
    endfunction

    function [31:0] ver_IF_IO_instr0_pc;
        /*verilator public*/
        ver_IF_IO_instr0_pc = ( instr0_valid ? instr0_pc : 32'hFFFFFFFF);
    endfunction
    function [31:0] ver_IF_IO_instr1_pc;
        /*verilator public*/
        ver_IF_IO_instr1_pc = ( instr1_valid ? instr1_pc : 32'hFFFFFFFF);
    endfunction
    function [31:0] ver_IF_IO_instr0_FINAL;
        /*verilator public*/
        ver_IF_IO_instr0_FINAL = ( instr0_valid && raw_instr0 == 32'hFFFFFFFF);
    endfunction
    function [31:0] ver_IF_IO_instr1_FINAL;
        /*verilator public*/
        ver_IF_IO_instr1_FINAL = ( instr1_valid && raw_instr1 == 32'hFFFFFFFF);
    endfunction
`endif
endinterface //IF_stage_io
