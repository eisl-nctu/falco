`timescale 1ns/1ps
// =============================================================================
//  Program : ID_stage_io.sv
//  Author  : Hon-Chou Dai
//  Date    : May/15/2021
// -----------------------------------------------------------------------------
//  Description:
//  Interface between Decode stage & RNDS stage.
// -----------------------------------------------------------------------------
//  Revision information:
//
//    August/07/2021, by Chun-Wei Chao:
//      Add store_set_id in interface.
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

interface ID_stage_io;
    decoded_op_t decoded_instr0;
    decoded_op_t decoded_instr1;
    logic instr0_valid;
    logic instr1_valid;
    logic instr0_issue;
    logic instr1_issue;
    BHSR_t instr0_BHSR;
    BHSR_t instr1_BHSR;

    logic [LFST_WIDTH-1 : 0] instr0_store_set_id/*verilator public*/;
    logic [LFST_WIDTH-1 : 0] instr1_store_set_id/*verilator public*/;


    modport ID_stage(
        output decoded_instr0,
        output decoded_instr1,
        output instr0_valid,
        output instr1_valid,
        input  instr0_issue,
        input  instr1_issue,
        output instr0_BHSR,
        output instr1_BHSR,
        output instr0_store_set_id,
        output instr1_store_set_id
    );
    modport RNDS_stage(
        input decoded_instr0,
        input decoded_instr1,
        input instr0_valid,
        input instr1_valid,
        output instr0_issue,
        output instr1_issue,
        input instr0_BHSR,
        input instr1_BHSR,
        input instr0_store_set_id,
        input instr1_store_set_id
    );

`ifdef FALCO_SIM_DEBUG
    function [7:0] ver_check_ID_stage_valid;
        /*verilator public*/
        begin
            ver_check_ID_stage_valid = (instr0_valid ? 1 : 0);
        end
    endfunction

    task ver_dump_ID_stage;
        /*verilator public*/
        begin
            $display("instr0_valid: %d decoded_instr0: \
{pc: 0x%8h immediate: 0x%8h operand0_sel: %d operand1_sel: %d ctrl_signal: %d \
mem_load_ext_sel: %d alu_muldiv_sel: %d shift_sel: %d is_branch: %d is_jal: %d is_jalr: %d \
is_csr_instr:%d csr_addr: 0x%3h csr_imm: %d\
mem_we: %d mem_re: %d mem_input_sel: %d sys_jump: %d sys_jump_csr_addr: 0x%3h\
rd_addr: %d rs1_addr: %d rs2_addr: %d use_rd_field: %d}",instr0_valid, decoded_instr0.pc, decoded_instr0.immediate, 
decoded_instr0.operand0_sel, decoded_instr0.operand1_sel, decoded_instr0.ctrl_signal, decoded_instr0.mem_load_ext_sel, decoded_instr0.alu_muldiv_sel,
decoded_instr0.shift_sel, decoded_instr0.is_branch, decoded_instr0.is_jal, decoded_instr0.is_jalr, decoded_instr0.is_csr_instr,
decoded_instr0.csr_addr, decoded_instr0.csr_imm, decoded_instr0.mem_we, decoded_instr0.mem_re, decoded_instr0.mem_input_sel,
decoded_instr0.sys_jump, decoded_instr0.sys_jump_csr_addr, decoded_instr0.rd_addr, decoded_instr0.rs1_addr, decoded_instr0.rs2_addr , decoded_instr1.is_use_rd_field);
            $display("instr1_valid: %d decoded_instr1: \
{pc: 0x%8h immediate: 0x%8h operand0_sel: %d operand1_sel: %d ctrl_signal: %d \
mem_load_ext_sel: %d alu_muldiv_sel: %d shift_sel: %d is_branch: %d is_jal: %d is_jalr: %d \
is_csr_instr:%d csr_addr: 0x%3h csr_imm: %d\
mem_we: %d mem_re: %d mem_input_sel: %d sys_jump: %d sys_jump_csr_addr: 0x%3h\
rd_addr: %d rs1_addr: %d rs2_addr: %d use_rd_field: %d}",instr1_valid, decoded_instr1.pc, decoded_instr1.immediate, 
decoded_instr1.operand0_sel, decoded_instr1.operand1_sel, decoded_instr1.ctrl_signal, decoded_instr1.mem_load_ext_sel, decoded_instr1.alu_muldiv_sel,
decoded_instr1.shift_sel, decoded_instr1.is_branch, decoded_instr1.is_jal, decoded_instr1.is_jalr, decoded_instr1.is_csr_instr,
decoded_instr1.csr_addr, decoded_instr1.csr_imm, decoded_instr1.mem_we, decoded_instr1.mem_re, decoded_instr1.mem_input_sel,
decoded_instr1.sys_jump, decoded_instr1.sys_jump_csr_addr, decoded_instr1.rd_addr, decoded_instr1.rs1_addr, decoded_instr1.rs2_addr , decoded_instr1.is_use_rd_field);
        end
    endtask
    function [31:0] ver_ID_IO_instr0_pc;
        /*verilator public*/
        ver_ID_IO_instr0_pc = ( instr0_valid ? decoded_instr0.pc : 32'hFFFFFFFF);
    endfunction
    function [31:0] ver_ID_IO_instr1_pc;
        /*verilator public*/
        ver_ID_IO_instr1_pc = ( instr1_valid ? decoded_instr1.pc : 32'hFFFFFFFF);
    endfunction
    
    function [31:0] ver_ID_IO_instr0_is_jump;
        /*verilator public*/
        ver_ID_IO_instr0_is_jump = ( instr0_valid && 
                (decoded_instr0.is_jal || decoded_instr0.is_jalr || decoded_instr0.is_branch || decoded_instr0.sys_jump )
                ? 1 : 0);
    endfunction
    function [31:0] ver_ID_IO_instr1_is_jump;
        /*verilator public*/
        ver_ID_IO_instr1_is_jump = ( instr1_valid && 
                (decoded_instr1.is_jal || decoded_instr1.is_jalr || decoded_instr1.is_branch || decoded_instr1.sys_jump ) 
                ? 1 : 0);
    endfunction

`endif

endinterface //ID_stage_io
