`timescale 1ns/1ps
// =============================================================================
//  Program : csr_io.sv
//  Author  : Chun-Wei Chao
//  Date    : September/05/2021
// -----------------------------------------------------------------------------
//  Description:
//  Interface between CSR module and other module.
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

interface csr_io;

    pc_t            pc; // not use now
    
    // From Decode
    csr_addr_t      csr_raddr_i;
    
    // To Decode
    xlen_data_t     csr_data_o;

    // From commit
    logic           csr_we_i;
    csr_addr_t      csr_waddr_i;
    xlen_data_t     csr_wdata_i;

    // System Jump
    logic           sys_jump_i;
    logic [1:0]     sys_jump_csr_addr_i;
    logic           sys_jump_o;
    xlen_data_t     sys_jump_csr_data_o;

    // Current preivilege level
    logic [1:0]     privilege_level_o;

    // Exception request
    logic           xcpt_valid_i;
    logic [3:0]     xcpt_cause_i;
    xlen_data_t     xcpt_tval_i;

    logic           exe_stall;
    logic           exe_is_branch;
    logic           exe_misspredict;

    logic [ROB_ENTRY_WIDTH:0]           recovery_distance;
    logic                               start_recovery;
    logic                               recovery_procedure;
    xlen_data_t                         start_recovery_tag;

    modport CSR(
        input   csr_raddr_i,
        output  csr_data_o,

        input   csr_we_i,
        input   csr_waddr_i,
        input   csr_wdata_i,

        input   sys_jump_i,
        input   sys_jump_csr_addr_i,
        output  sys_jump_o,
        output  sys_jump_csr_data_o,

        output  privilege_level_o,

        input   xcpt_valid_i,
        input   xcpt_cause_i,
        input   xcpt_tval_i,

        input   exe_stall,
        input   exe_is_branch,
        input   exe_misspredict,
        input   start_recovery,
        input   recovery_procedure,
        input   recovery_distance,
        input   start_recovery_tag,

        input   pc
    );

    modport EXE_stage(
        input   privilege_level_o,
        input   csr_data_o,
        output  csr_raddr_i,

        output  exe_stall,
        output  exe_is_branch,
        output  exe_misspredict,

        output  csr_we_i,
        output  csr_waddr_i,
        output  csr_wdata_i,
        output  sys_jump_i,
        output  sys_jump_csr_addr_i,
        output  xcpt_valid_i,
        output  xcpt_cause_i,
        output  xcpt_tval_i,
        output  pc
    );

    modport Fetch_stage(
        input sys_jump_o,
        input sys_jump_csr_data_o
    );
    
    modport ROB(
        output recovery_distance,
        output start_recovery_tag
    );

    modport Recovery(
        output start_recovery,
        output recovery_procedure
    );

`ifdef FALCO_SIM_DEBUG
    function [31:0] ver_CSR_IO_read_addr_0;
        /*verilator public*/
        ver_CSR_IO_read_addr_0 = csr_raddr_i;
    endfunction
    function [31:0] ver_CSR_IO_read_addr_1;
        /*verilator public*/
        ver_CSR_IO_read_addr_1 = csr_raddr_i;
    endfunction
    function [31:0] ver_CSR_IO_read_data_0;
        /*verilator public*/
        ver_CSR_IO_read_data_0 = csr_data_o;
    endfunction
    function [31:0] ver_CSR_IO_read_data_1;
        /*verilator public*/
        ver_CSR_IO_read_data_1 = csr_data_o;
    endfunction
`endif 

endinterface //csr_io
