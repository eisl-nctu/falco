`timescale 1ns/1ps
// =============================================================================
//  Program : exe_stage_io.sv
//  Author  : Hon-Chou Dai
//  Date    : May/15/2021
// -----------------------------------------------------------------------------
//  Description:
//  Interface between execute stage & other stage, broadcast execute result to other stage.
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

interface exe_stage_io;
    exe_int_commit_t exe_ALU_1;
    exe_int_commit_t exe_muldiv;
    exe_branch_commit_t exe_alu_csr_bc; //ALU0 , branch , CSR

    exe_fu_wb_t alu_csr_bc_wb;
    exe_fu_wb_t alu1_wb;
    exe_fu_wb_t muldiv_wb;

    exe_broadcast_t BCAST_alu_csr_bc;
    exe_broadcast_t BCAST_ALU_1;
    exe_broadcast_t BCAST_muldiv;
//TODO: branch result port 
    exe_branch_t branch_result;
//direct port connect to IF and branch manager
    modport alu_csr_bc_group(
        output exe_alu_csr_bc,
        output alu_csr_bc_wb,
        output BCAST_alu_csr_bc,
        output branch_result,
        input alu1_wb,
        input muldiv_wb
    );

    modport alu_muldiv_group(
        output exe_ALU_1,
        output exe_muldiv,
        input alu_csr_bc_wb,
        output alu1_wb,
        output muldiv_wb,
        output BCAST_ALU_1,
        output BCAST_muldiv
    );

    modport AGU(
        input alu_csr_bc_wb,
        input alu1_wb,
        input muldiv_wb
    );

    modport IF_stage(
        input branch_result
    );

    modport ROB(
        input exe_alu_csr_bc,
        input exe_ALU_1,
        input exe_muldiv
    );
//TO: register read
    modport write_back(
        input alu_csr_bc_wb,
        input alu1_wb,
        input muldiv_wb
    );

    modport int_iq(
        input BCAST_alu_csr_bc,
        input BCAST_ALU_1,
        input BCAST_muldiv
    );

    modport mem_iq(
        input BCAST_alu_csr_bc,
        input BCAST_ALU_1,
        input BCAST_muldiv
    );

    modport RNDS_stage(
        input BCAST_alu_csr_bc,
        input BCAST_ALU_1,
        input BCAST_muldiv
    );

`ifdef FALCO_SIM_DEBUG
    function [31:0] ver_EXE_IO_alu_csr_pc;
        /*verilator public*/
        ver_EXE_IO_alu_csr_pc = (  alu_csr_bc_wb.valid ? alu_csr_bc_wb.pc : 32'hFFFFFFFF);
    endfunction
    function [31:0] ver_EXE_IO_alu1_pc;
        /*verilator public*/
        ver_EXE_IO_alu1_pc = ( alu1_wb.valid ? alu1_wb.pc : 32'hFFFFFFFF);
    endfunction
    function [31:0] ver_EXE_IO_muldiv_pc;
        /*verilator public*/
        ver_EXE_IO_muldiv_pc = muldiv_wb.pc;
    endfunction
    function [31:0] ver_EXE_IO_muldiv_valid;
        /*verilator public*/
        ver_EXE_IO_muldiv_valid = muldiv_wb.valid ? 1 : 0;
    endfunction

    function [31:0] ver_EXE_IO_alu_csr_data;
        /*verilator public*/
        ver_EXE_IO_alu_csr_data = alu_csr_bc_wb.wb_data;
    endfunction
    function [31:0] ver_EXE_IO_alu1_data;
        /*verilator public*/
        ver_EXE_IO_alu1_data = alu1_wb.wb_data;
    endfunction
    function [31:0] ver_EXE_IO_muldiv_data;
        /*verilator public*/
        ver_EXE_IO_muldiv_data = muldiv_wb.wb_data;
    endfunction

    function [31:0] ver_EXE_IO_alu_csr_prf;
        /*verilator public*/
        ver_EXE_IO_alu_csr_prf = alu_csr_bc_wb.wb_addr;
    endfunction
    function [31:0] ver_EXE_IO_alu1_prf;
        /*verilator public*/
        ver_EXE_IO_alu1_prf = alu1_wb.wb_addr;
    endfunction
    function [31:0] ver_EXE_IO_muldiv_prf;
        /*verilator public*/
        ver_EXE_IO_muldiv_prf = muldiv_wb.wb_addr;
    endfunction
`endif
endinterface //exe_stage_io
