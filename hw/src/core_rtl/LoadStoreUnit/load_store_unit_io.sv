`timescale 1ns/1ps
// =============================================================================
//  Program : load_store_unit_io.sv
//  Author  : Hon-Chou Dai
//  Date    : May/15/2021
// -----------------------------------------------------------------------------
//  Description:
//  Interface between LSU stage & other stage.
// -----------------------------------------------------------------------------
//  Revision information:
//
//    August/07/2021    : 
//      Add load/store violation signal between LSU stage & IF stage & ROB stage.
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

interface load_store_unit_io;
    //Load Store Unit pipeline register
    lsu_load_pack_t lsu_pack;
    rob_tag_t cur_lsu_rob_tag;

    mem_commit_t LSU_commit;
    exe_broadcast_t BCAST_load;

    logic store_set_violation;
    logic device_violation;
    rob_tag_t store_set_rob_tag;

    pc_t violation_pc;
    BHSR_t violation_bhsr;

    logic [SSIT_WIDTH-1 : 0] violation_load_pc /*verilator public*/;
    logic [SSIT_WIDTH-1 : 0] violation_store_pc /*verilator public*/;
    logic [LFST_WIDTH-1 : 0] violation_load_id;
    logic [LFST_WIDTH-1 : 0] violation_store_id;

    modport LSU(
        //To MA stage
        output lsu_pack,
        output cur_lsu_rob_tag,
        output LSU_commit,
        output BCAST_load,
        output store_set_violation,
        output device_violation,
        output store_set_rob_tag,
        output violation_pc,
        output violation_bhsr,
        output violation_load_pc,
        output violation_store_pc,
        output violation_load_id,
        output violation_store_id
    );

    modport IF_stage(
        input store_set_violation,
        input violation_pc,
        input violation_bhsr
    );

    modport MemAccess(
        input lsu_pack
    );

    modport ROB(
        input LSU_commit,
        input store_set_violation,
        input store_set_rob_tag
    );

    modport int_iq(
        input BCAST_load,
        input cur_lsu_rob_tag
    );

    modport mem_iq(
        input BCAST_load
    );

    modport RNDS_stage( //Freelist update
        input BCAST_load
    );

    modport ID_stage(
        input store_set_violation,
        input device_violation,
        input violation_load_pc,
        input violation_store_pc,
        input violation_load_id,
        input violation_store_id
    );

endinterface //load_store_unit_io
