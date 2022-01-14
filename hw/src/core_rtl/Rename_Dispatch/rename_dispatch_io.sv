`timescale 1ns/1ps
// =============================================================================
//  Program : rename_dispatch_io.sv
//  Author  : Hon-Chou Dai
//  Date    : May/15/2021
// -----------------------------------------------------------------------------
//  Description:
//  Interface between RNSD stage and other stage.
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

`timescale 1ns/1ps
// =============================================================================
//  Program : rename_dispatch_io.sv
//  Author  : Hon-Chou Dai
//  Date    :
// -----------------------------------------------------------------------------
//  Description:
//
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

//rename dispatch no internal pipeline register
//use IQ as pipeline register
interface rename_dispatch_io;
//INT IQ
    int_dispatch_pack_t     int_pack0;
    int_dispatch_pack_t     int_pack1;
    logic                   int_pack0_valid;
    logic                   int_pack1_valid;
    //branch_info_t int_pack0_br_info;
    //branch_info_t int_pack1_br_info;
    logic                   int_iq_instr0_check_top; //0 -> busy,1 -> 1 or more than 1 empty entry
    logic                   int_iq_instr1_check_top; //0 -> may be 1, 1 -> 2 or more than 2 empty entry
    logic                   int_instr0_rs1_ready;
    logic                   int_instr0_rs2_ready;
    logic                   int_instr1_rs1_ready;
    logic                   int_instr1_rs2_ready;
//MEM IQ
    mem_dispatch_pack_t     mem_pack0;
    mem_dispatch_pack_t     mem_pack1;
    logic                   mem_pack0_valid;
    logic                   mem_pack1_valid;
    //spec_tag_t mem_pack0_br_mask; //mask only , because it can not happen branch in mem
    //spec_tag_t mem_pack1_br_mask;
    logic                   mem_iq_instr0_check_top; //0 -> busy,1 -> 1 or more than 1 empty entry
    logic                   mem_iq_instr1_check_top; //0 -> may be 1, 1 -> 2 or more than 2 empty entry
    logic                   mem_instr0_rs1_ready;
    logic                   mem_instr0_rs2_ready;
    logic                   mem_instr1_rs1_ready;
    logic                   mem_instr1_rs2_ready;
    // store_set
    logic                               issue_store; //from io
    logic [STORE_ID_WIDTH-1 : 0]        issue_store_id; //from io
    logic [LFST_WIDTH-1 : 0]            issue_store_set_id; //from io

//ROB allocation
//TO RNDS, RNDS must check ROB have entry to allocate
    logic                   rob_instr0_check_top;
    logic                   rob_instr1_check_top;
    logic                   rob_is_empty;
    logic                   instr0_req;
    logic                   instr1_req;
    rob_tag_t               instr0_rob_tag;
    rob_tag_t               instr1_rob_tag;
    logic                   recovery_no_copy;
    rob_tag_t               recovery_target_rob_tag;
//ROB prf and arf log
    prf_specifier_t         instr0_stale_rd;
    prf_specifier_t         instr1_stale_rd;
    logic                   instr0_stale_rd_valid;
    logic                   instr1_stale_rd_valid;
    arf_specifier_t         instr0_rd_arf;
    arf_specifier_t         instr1_rd_arf;
    prf_specifier_t         instr0_rd_prf;
    prf_specifier_t         instr1_rd_prf;
    pc_t                    instr0_pc;
    pc_t                    instr1_pc;
    logic                   instr0_is_store_op;
    logic                   instr1_is_store_op;

    modport RNDS_stage(
        //to int iq
        output          int_pack0,
        output          int_pack1,
        output          int_pack0_valid,
        output          int_pack1_valid,
        //output int_pack0_br_info,
        //output int_pack1_br_info,
        input           int_iq_instr0_check_top,
        input           int_iq_instr1_check_top,
        output          int_instr0_rs1_ready,
        output          int_instr0_rs2_ready,
        output          int_instr1_rs1_ready,
        output          int_instr1_rs2_ready,
        //to mem iq
        output          mem_pack0,
        output          mem_pack1,
        output          mem_pack0_valid,
        output          mem_pack1_valid,
        //store_set
        input           issue_store,
        input           issue_store_id,
        input           issue_store_set_id,
        //output mem_pack0_br_mask,
        //output mem_pack1_br_mask,
        input           mem_iq_instr0_check_top,
        input           mem_iq_instr1_check_top,
        output          mem_instr0_rs1_ready,
        output          mem_instr0_rs2_ready,
        output          mem_instr1_rs1_ready,
        output          mem_instr1_rs2_ready,
        //TO ROB
        //TO RNDS, RNDS must check ROB have entry to allocate
        input           rob_instr0_check_top,
        input           rob_instr1_check_top,
        input           rob_is_empty,
        output          instr0_req,
        output          instr1_req,
        input           instr0_rob_tag, //use for update ROB entry status in wb and exe
        input           instr1_rob_tag,
        input           recovery_no_copy,
        input           recovery_target_rob_tag,
        output          instr0_stale_rd,
        output          instr1_stale_rd,
        output          instr0_stale_rd_valid,
        output          instr1_stale_rd_valid,
        output          instr0_rd_arf,
        output          instr1_rd_arf,
        output          instr0_rd_prf,
        output          instr1_rd_prf,
        output          instr0_is_store_op,
        output          instr1_is_store_op,
        output          instr0_pc,
        output          instr1_pc
    );

    modport int_iq(
        input           int_pack0,
        input           int_pack1,
        input           int_pack0_valid,
        input           int_pack1_valid,
        //input int_pack0_br_info,
        //input int_pack1_br_info,
        output          int_iq_instr0_check_top,
        output          int_iq_instr1_check_top,
        input           int_instr0_rs1_ready,
        input           int_instr0_rs2_ready,
        input           int_instr1_rs1_ready,
        input           int_instr1_rs2_ready
    );

    modport mem_iq(
        input           mem_pack0,
        input           mem_pack1,
        input           mem_pack0_valid,
        input           mem_pack1_valid,
        //input mem_pack0_br_mask,
        //input mem_pack1_br_mask,
        output          mem_iq_instr0_check_top,
        output          mem_iq_instr1_check_top,
        input           mem_instr0_rs1_ready,
        input           mem_instr0_rs2_ready,
        input           mem_instr1_rs1_ready,
        input           mem_instr1_rs2_ready,
        //store_set
        output          issue_store,
        output          issue_store_id,
        output          issue_store_set_id
    );

    modport ROB(
        //TO RNDS, RNDS must check ROB have entry to allocate
        //allocate logic
        output  rob_instr0_check_top,
        output  rob_instr1_check_top,
        output  rob_is_empty,
        input   instr0_req,
        input   instr1_req,
        //allocate data
        output  instr0_rob_tag,
        output  instr1_rob_tag,
        output  recovery_no_copy,
        output  recovery_target_rob_tag,
        input   instr0_stale_rd,
        input   instr1_stale_rd,
        input   instr0_stale_rd_valid,
        input   instr1_stale_rd_valid,
        input   instr0_rd_arf,
        input   instr1_rd_arf,
        input   instr0_rd_prf,
        input   instr1_rd_prf,
        input   instr0_is_store_op,
        input   instr1_is_store_op,
        input   instr0_pc,
        input   instr1_pc
    );

`ifdef FALCO_SIM_DEBUG
    function [31:0] ver_RNDS_INT_instr0_pc;
        /*verilator public*/
        ver_RNDS_INT_instr0_pc = ( int_pack0_valid ? int_pack0.pc : 32'hFFFFFFFF);
    endfunction

    function [31:0] ver_RNDS_INT_instr1_pc;
        /*verilator public*/
        ver_RNDS_INT_instr1_pc = ( int_pack1_valid ? int_pack1.pc : 32'hFFFFFFFF);
    endfunction

    function [31:0] ver_RNDS_MEM_instr0_pc;
        /*verilator public*/
        ver_RNDS_MEM_instr0_pc = ( mem_pack0_valid ? mem_pack0.pc : 32'hFFFFFFFF);
    endfunction

    function [31:0] ver_RNDS_MEM_instr1_pc;
        /*verilator public*/
        ver_RNDS_MEM_instr1_pc = ( mem_pack1_valid ? mem_pack1.pc : 32'hFFFFFFFF);
    endfunction

    function [31:0] ver_RNDS_MEM_instr0_store_counter;
        /*verilator public*/
        ver_RNDS_MEM_instr0_store_counter = mem_pack0.store_id;
    endfunction

    function [31:0] ver_RNDS_MEM_instr1_store_counter;
        /*verilator public*/
        ver_RNDS_MEM_instr1_store_counter = mem_pack1.store_id;
    endfunction

     function [31:0] ver_RNDS_MEM_instr0_store_set_id;
        /*verilator public*/
        ver_RNDS_MEM_instr0_store_set_id = mem_pack0.store_set_id;
    endfunction

    function [31:0] ver_RNDS_MEM_instr1_store_set_id;
        /*verilator public*/
        ver_RNDS_MEM_instr1_store_set_id = mem_pack1.store_set_id;
    endfunction

     function [31:0] ver_RNDS_MEM_instr0_predict_result;
        /*verilator public*/
        ver_RNDS_MEM_instr0_predict_result = mem_pack0.predict_no_vilation;
    endfunction

    function [31:0] ver_RNDS_MEM_instr1_predict_result;
        /*verilator public*/
        ver_RNDS_MEM_instr1_predict_result = mem_pack1.predict_no_vilation;
    endfunction
`endif
endinterface //rename_dispatch_io
