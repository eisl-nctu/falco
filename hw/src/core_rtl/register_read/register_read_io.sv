`timescale 1ns/1ps
// =============================================================================
//  Program : register_read_io.sv
//  Author  : Hon-Chou Dai
//  Date    : May/15/2021
// -----------------------------------------------------------------------------
//  Description:
//  Interface between register read stage and execute stage.
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

interface register_read_io;
    int_issue_pack_t alu_bc_csr_pack;
    int_issue_no_csr_pack_t alu_muldiv_pack;
    mem_issue_pack_t mem_issue_pack;

    //branch_info_t alu_bc_csr_br_info;
    //spec_tag_t alu_muldiv_br_mask;
    //spec_tag_t mem_issue_br_mask;

    logic alu_bc_csr_pack_valid;
    logic alu_muldiv_pack_valid;
    logic mem_issue_pack_valid;

    modport register_read(
        //output alu_bc_csr_br_info,
        //output alu_muldiv_br_mask,
        //output mem_issue_br_mask,
        output alu_bc_csr_pack,
        output alu_muldiv_pack,
        output mem_issue_pack,
        output alu_bc_csr_pack_valid,
        output alu_muldiv_pack_valid,
        output mem_issue_pack_valid
    );

    modport alu_bc_csr_exe(
        //input alu_bc_csr_br_info,
        input alu_bc_csr_pack,
        input alu_bc_csr_pack_valid
    );

    modport alu_muldiv_exe(
        //input alu_muldiv_br_mask,
        input alu_muldiv_pack,
        input alu_muldiv_pack_valid
    );

    modport mem(
        //input mem_issue_br_mask,
        input mem_issue_pack,
        input mem_issue_pack_valid
    );
`ifdef FALCO_SIM_DEBUG
    function [31:0] ver_RR_IO_alu_bc_csr_pc;
        /*verilator public*/
        ver_RR_IO_alu_bc_csr_pc = (  alu_bc_csr_pack_valid? alu_bc_csr_pack.pc : 32'hFFFFFFFF);
    endfunction

    function [31:0] ver_RR_IO_alu_muldiv_pc;
        /*verilator public*/
        ver_RR_IO_alu_muldiv_pc = ( alu_muldiv_pack_valid ? alu_muldiv_pack.pc : 32'hFFFFFFFF);
    endfunction
    function [31:0] ver_RR_IO_mem_pc;
        /*verilator public*/
        ver_RR_IO_mem_pc = ( mem_issue_pack_valid ? mem_issue_pack.pc : 32'hFFFFFFFF);
    endfunction
`endif
endinterface //register_read_io
