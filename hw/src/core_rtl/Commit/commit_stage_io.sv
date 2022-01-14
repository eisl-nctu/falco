`timescale 1ns/1ps
// =============================================================================
//  Program : commit_stage_io.sv
//  Author  : Hon-Chou Dai
//  Date    : May/15/2021
// -----------------------------------------------------------------------------
//  Description:
//  Interface between ROB & RNDS_stage (for recovery)
//  Interface between ROB & store_buffer (for commit store instruction)
//  Interface between ROB & load_buffer (for commit load instruction)
//  Interface between ROB & IF stage (for checking which recovery target to use)
// -----------------------------------------------------------------------------
//  Revision information:
//
//    August/07/2021    : Add loadbuffer port & IF stage port
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

interface commit_stage_io;
    //Current debug purpose, when RISCV debug spec impl, this must keep.
    committed_map_update_t committed_update;

    rob_tag_t cur_commit_rob_tag;

    //prf recovery port when branch misprediction
    prf_specifier_t recovery_old_prf_0;
    arf_specifier_t recovery_arf_0;
    logic           recovery_arf_map_0_valid;
    prf_specifier_t recovery_old_prf_1;
    arf_specifier_t recovery_arf_1;
    logic           recovery_arf_map_1_valid;

    //commit instruction and recycle prf
    prf_specifier_t recycle_freelist_prf_0;
    prf_specifier_t recycle_freelist_prf_1;
    logic           recycle_freelist_0_valid;
    logic           recycle_freelist_1_valid;

    //store data buffer status update port
    //When instr commit ,change relative St instr to retirement stage
    logic           store_commit_0_valid;
    logic           store_commit_1_valid;
    //When branch misprediction, recovery SDB entry
    logic           store_flush_0_valid; //if finished(it means allocate to SDB) && is_mem_op
    logic           store_flush_1_valid;

    logic           commit_instr0;
    logic           commit_instr1;

    logic           branch_miss_first;

    modport ROB(
        output committed_update,
        output cur_commit_rob_tag,
        output recovery_old_prf_0,
        output recovery_arf_0,
        output recovery_arf_map_0_valid,
        output recovery_old_prf_1,
        output recovery_arf_1,
        output recovery_arf_map_1_valid,

        output recycle_freelist_prf_0,
        output recycle_freelist_prf_1,
        output recycle_freelist_0_valid,
        output recycle_freelist_1_valid,

        output store_commit_0_valid,
        output store_commit_1_valid,
        output store_flush_0_valid,
        output store_flush_1_valid,

        output commit_instr0,
        output commit_instr1,

        output branch_miss_first
    );

    modport int_iq(
        input cur_commit_rob_tag
    );

    modport Committed_map(
        input committed_update
    );

    modport RNDS_stage(
        input recovery_old_prf_0, //0 older than 1
        input recovery_arf_0,
        input recovery_arf_map_0_valid,
        input recovery_old_prf_1,
        input recovery_arf_1,
        input recovery_arf_map_1_valid,

        input recycle_freelist_prf_0,
        input recycle_freelist_prf_1,
        input recycle_freelist_0_valid,
        input recycle_freelist_1_valid
    );

    modport storebuffer(
        input store_commit_0_valid,
        input store_commit_1_valid,
        input store_flush_0_valid,
        input store_flush_1_valid
    );

    modport loadbuffer(
        input commit_instr0,
        input commit_instr1,
        input cur_commit_rob_tag
    );

    modport IF_stage(
        input branch_miss_first
    );

endinterface //commit_stage_io
