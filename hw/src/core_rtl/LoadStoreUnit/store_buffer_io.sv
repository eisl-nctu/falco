`timescale 1ns/1ps
// =============================================================================
//  Program : store_buffer_io.sv
//  Author  : Hon-Chou Dai
//  Date    : May/15/2021
// -----------------------------------------------------------------------------
//  Description:
//  Interface between store_buffer & LSU stage.
// -----------------------------------------------------------------------------
//  Revision information:
//
//    August/07/2021, by Chun-Wei Chao:
//      Also transfer allocated store information to load buffer to 
//      check load/store violation.
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

interface store_buffer_io;
    logic           instr_valid;
    logic           store_allocate;
    xlen_data_t     store_allocate_addr;
    xlen_data_t     store_allocate_data;
    byte_mask_t     store_allocate_mask;
    rob_tag_t       store_allocate_rob_tag;
    logic           store_non_idempotent_region;
`ifdef FALCO_SIM_DEBUG
    pc_t            store_pc;
`endif

    // Load forward signal LSU <=> SDB
    logic           sb_load_forward_hit;
    xlen_data_t     sb_load_forward_data;
    byte_mask_t     sb_load_forward_mask;
    xlen_data_t     sb_load_find_addr;
    byte_mask_t     sb_load_find_mask;

    logic non_idempotent_instr_exists;

    modport storebuffer(
        input instr_valid,
        input store_allocate,
        input store_allocate_addr,
        input store_allocate_data,
        input store_allocate_mask,
        input store_allocate_rob_tag,
        input store_non_idempotent_region,
`ifdef FALCO_SIM_DEBUG
        input store_pc,
`endif 

        output sb_load_forward_data,
        output sb_load_forward_hit,
        output sb_load_forward_mask,
        input  sb_load_find_addr,
        input  sb_load_find_mask,

        output non_idempotent_instr_exists
    );

    modport loadbuffer(
        input instr_valid,
        input store_allocate,
        input store_allocate_addr,
        input store_allocate_mask,
        input store_allocate_rob_tag
    );

    modport LSU(
        //SDB allocation
        output instr_valid,
        output store_allocate,
        output store_allocate_addr,
        output store_allocate_data,
        output store_allocate_mask,
        output store_allocate_rob_tag,
        output store_non_idempotent_region,
`ifdef FALCO_SIM_DEBUG
        output store_pc,
`endif

        //SDB load forward
        input  sb_load_forward_hit,
        input  sb_load_forward_data,
        input  sb_load_forward_mask,
        output sb_load_find_addr,
        output sb_load_find_mask
    );

    modport AGU(
        input non_idempotent_instr_exists
    );

endinterface //store_buffer_io
