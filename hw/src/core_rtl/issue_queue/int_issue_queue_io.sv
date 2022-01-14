`timescale 1ns/1ps
// =============================================================================
//  Program : int_issue_queue_io.sv
//  Author  : Hon-Chou Dai
//  Date    : May/15/2021
// -----------------------------------------------------------------------------
//  Description:
//  Interface between int_issue_queue stage and register_read stage, 
//  also tranfer ready register to RNDS stage for early wake up instrucion.
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

interface int_issue_queue_io;
    logic instr0_valid;
    logic instr1_valid;
    //TODO issue queue io remove unnecessary variable (such as, br_tag/branch in instr1)
    int_dispatch_pack_t instr0_pack;
    int_dispatch_pack_t instr1_pack;

    // Wakeup logic (back to back wakeup for INT issue queue)
    logic issue_alu0_rd_valid;
    logic issue_alu1_rd_valid;
    prf_specifier_t issue_alu0_rd;
    prf_specifier_t issue_alu1_rd;

    modport int_iq(
        output instr0_valid,
        output instr1_valid,
        output instr0_pack,
        output instr1_pack,
        output issue_alu0_rd_valid,
        output issue_alu1_rd_valid,
        output issue_alu0_rd,
        output issue_alu1_rd
    );

    modport mem_iq(
        input instr0_valid,
        input instr1_valid,
        input issue_alu0_rd_valid,
        input issue_alu1_rd_valid,
        input issue_alu0_rd,
        input issue_alu1_rd
    );

    modport register_read(
        input instr0_valid,
        input instr1_valid,
        input instr0_pack,
        input instr1_pack
    );

    modport RNDS_stage(
        input issue_alu0_rd_valid,
        input issue_alu1_rd_valid,
        input issue_alu0_rd,
        input issue_alu1_rd
    );
`ifdef FALCO_SIM_DEBUG
    function [31:0] ver_INT_IQ_instr0_pc;
        /*verilator public*/
        ver_INT_IQ_instr0_pc = ( instr0_valid ? instr0_pack.pc : 32'hFFFFFFFF);
    endfunction
    function [31:0] ver_INT_IQ_instr1_pc;
        /*verilator public*/
        ver_INT_IQ_instr1_pc = ( instr1_valid ? instr1_pack.pc : 32'hFFFFFFFF);
    endfunction
`endif
endinterface // int_issue_queue_io
