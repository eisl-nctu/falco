`timescale 1ns/1ps
// =============================================================================
//  Program : load_buffer_io.sv
//  Author  : Chun-Wei Chao
//  Date    : August/07/2021
// -----------------------------------------------------------------------------
//  Description:
//  Interface between LSU stage & load_buffer.
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

interface load_buffer_io;

    logic           instr_valid;
    logic           load_allocate;
    xlen_data_t     load_allocate_addr;
    byte_mask_t     load_allocate_mask;
    rob_tag_t       load_allocate_rob_tag;
    logic [SSIT_WIDTH-1 : 0] load_store_set_pc;
    logic [LFST_WIDTH-1 : 0] load_store_set_id;
    logic [SSIT_WIDTH-1 : 0] violation_load_pc;
    logic [LFST_WIDTH-1 : 0] violation_load_id;
    logic store_instruction;
    xlen_data_t store_addr;
    rob_tag_t       store_rob_tag;
    logic violation_detect;
`ifdef FALCO_SIM_DEBUG
    pc_t            load_pc;
`endif

    modport loadbuffer (
        input instr_valid,
        input load_allocate,
        input load_allocate_addr,
        input load_allocate_mask,
        input load_allocate_rob_tag,
        input load_store_set_pc,
        input load_store_set_id,
        input store_instruction,
        input store_addr,
        input store_rob_tag,
        output violation_detect,
`ifdef FALCO_SIM_DEBUG
       input load_pc,
`endif 
        output violation_load_pc,
        output violation_load_id
    );

    modport LSU(
        output instr_valid,
        output load_allocate,
        output load_allocate_addr,
        output load_allocate_mask,
        output load_allocate_rob_tag,
        output load_store_set_pc,
        output load_store_set_id,
        output store_instruction,
        output store_addr,
        output store_rob_tag,
        input violation_detect,
`ifdef FALCO_SIM_DEBUG
       output load_pc,
`endif
        input violation_load_pc,
        input violation_load_id
    );

endinterface //load_buffer_io
