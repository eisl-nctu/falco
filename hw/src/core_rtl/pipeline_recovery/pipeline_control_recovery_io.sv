`timescale 1ns/1ps
// =============================================================================
//  Program : pipeline_control_recovery_io.sv
//  Author  : Hon-Chou Dai
//  Date    : May/15/2021
// -----------------------------------------------------------------------------
//  Description:
//  Interface between pipeline controll module and other module.
// -----------------------------------------------------------------------------
//  Revision information:
//
//    August/07/2021, by Chun-Wei Chao:
//      Add signal to handle load/store violation.
//    November/18/2021, by Chun-Wei Chao:
//      Add signal to handle checkpoint recovery.
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

interface pipeline_control_recovery_io;
//IF stage
    logic icache_miss;
    logic pc_stall;
    logic IF_flush;
    logic IF_stall;
//ID stage
    logic ID_flush;
    logic ID_stall;
//RNDS stage
    logic RNDS_flush;
    logic RNDS_stall;
    logic freelist_empty;
    logic int_iq_full;
    logic mem_iq_full;
    logic rob_full;
    logic csr_stall;
//INT_IQ stage
    logic icache_miss_stall_branch;
    logic control_flow_stall;
    // stall int_iq select slot0 path ,
    // rr slot0 and alu_csr_bc to prevent issue instruction when icache(lock cache) miss
    // or record whether previous branch issue to alu0 flow in latest 2 cycle, if no it can 
    // issue alu instruction to alu0 flow
    logic load_wake_up_predict_failed; //TO: INT_IQ,MEM_IO,RR,EXE,AGU
    logic load_wake_up_failed_stall; //TO: INT_IQ,MEM_IO
    logic load_depend_replay;
    logic replay_muldiv_stall;
    logic replay_failed_to_issue_muldiv;
//EXE-branch
    logic bp_PrSuccess;
    logic bp_PrMiss;

// MEM_IQ stage
    logic memory_store_stall;
    //memory pipeline stall if SDA full 
    logic MEM_IQ_stall;
    logic INT_IQ_stall;
    logic AGU_stall;
    logic LSU_stall;
    logic SDA_full;
    logic LDA_full;
    logic load_wait_stall; //Dcache miss, SDB overlap event
    logic store_set_violation;
    logic MA_stall;
    logic LSU_non_idempotent_lock;
// Register read
    logic rr_mem_stall;
// Address generate 
    logic wait_for_non_idempotent;
//to ID_latch,RN,IQ,EXE,COMMIT,ROB
    branch_flush_t recovery_flush_BCAST; //broadcast invalid rob_tag to invalidate
//to COMMIT(ROB)
    logic commit_stall; //if committed store buffer full
    rob_tag_t flush_rob_tag_0;
    rob_tag_t flush_rob_tag_1;
    logic flush_rob_tag_0_valid;
    logic flush_rob_tag_1_valid;
    logic recovery_start;
    logic recovery_flush;
    logic recovery_stall;
    logic recovery_procedure;
    logic recovery_rollback;
    logic ROB_recovery_finished;
//Load buffer
    logic load_store_violation;
    rob_tag_t load_store_violation_tag;

    modport controller(
        //IF stage
        input icache_miss,
        output pc_stall,
        output IF_stall,
        output IF_flush,

        //ID stage
        output ID_flush,
        output ID_stall,

        //RNDS stage
        output RNDS_stall,
        output RNDS_flush,
        input freelist_empty,
        input int_iq_full,
        input mem_iq_full,
        input rob_full,
        input csr_stall,
        //INT_IQ
        output icache_miss_stall_branch,
        input replay_failed_to_issue_muldiv,
        output replay_muldiv_stall,
        //EXE

        input bp_PrSuccess,
        input bp_PrMiss,
        //register read
        output rr_mem_stall,
        //AGU
        input wait_for_non_idempotent,
        //MEM
        input SDA_full,
        input LDA_full,
        output MEM_IQ_stall,
        output INT_IQ_stall,
        output AGU_stall,
        output LSU_stall,
        input load_wait_stall,
        input store_set_violation,
        output LSU_non_idempotent_lock,
        output MA_stall,
        //to ID_latch,RN,IQ,EXE,COMMIT,ROB
        output recovery_flush_BCAST,
        //to INT_IQ,MEM_IQ,RR,EXE,AGU
        output load_depend_replay,
        output load_wake_up_predict_failed,
        output load_wake_up_failed_stall,
        //Commit
        input flush_rob_tag_0,
        input flush_rob_tag_1,
        input flush_rob_tag_0_valid,
        input flush_rob_tag_1_valid,
        output recovery_start,
        output recovery_flush,
        output recovery_stall,
        output recovery_procedure,
        output recovery_rollback,
        input ROB_recovery_finished,
        //Load buffer
        input load_store_violation
    );
    modport IF_stage(
        output icache_miss,
        input pc_stall,
        input IF_stall,
        input IF_flush
    );

    modport ID_stage(
    
        input ID_stall,
        input ID_flush

    ); 

    modport RNDS_stage(
        input RNDS_stall,
        input RNDS_flush,
        output freelist_empty, 
        output int_iq_full,
        output mem_iq_full,
        output rob_full,
        output csr_stall,
        input recovery_flush_BCAST,
        input recovery_start,
        input recovery_flush,
        input recovery_rollback,
        input recovery_stall,
        input recovery_procedure
    );

    modport int_iq(
        input recovery_flush_BCAST,
        input recovery_stall,
        input icache_miss_stall_branch,
        input load_wake_up_predict_failed,
        input load_wake_up_failed_stall,
        input load_depend_replay,
        output replay_failed_to_issue_muldiv,
        input INT_IQ_stall,
        input replay_muldiv_stall
    );

    modport mem_iq(
        input recovery_flush_BCAST,
        input recovery_stall,
        input MEM_IQ_stall,
        input SDA_full,
        input load_wake_up_predict_failed,
        input load_wake_up_failed_stall,
        input load_depend_replay,
        input replay_muldiv_stall
    );

    modport register_read(
        input icache_miss_stall_branch,
        input INT_IQ_stall,
        input rr_mem_stall,
        input recovery_flush_BCAST,
        input recovery_stall,
        input load_wake_up_predict_failed
    );

    modport alu_csr_bc_exe(
        output bp_PrSuccess,
        output bp_PrMiss,
        input icache_miss_stall_branch,
        input recovery_flush_BCAST,
        input recovery_start,
        input recovery_flush,
        input recovery_stall,
        input recovery_procedure,
        input load_wake_up_failed_stall,
        input load_wake_up_predict_failed
    );
 
    modport alu_muldiv_exe(
        input recovery_flush_BCAST,
        input recovery_stall,
        input load_wake_up_failed_stall,
        input load_wake_up_predict_failed
    );

    modport AGU(
        input recovery_flush_BCAST,
        input recovery_stall,
        input AGU_stall,
        input load_wake_up_predict_failed,
        input load_wake_up_failed_stall,
        output wait_for_non_idempotent
    );

    modport LSU(
        input recovery_flush_BCAST,
        input LSU_stall,
        input LSU_non_idempotent_lock,
        input recovery_flush,
        input recovery_stall,
        input recovery_procedure,
        output load_wait_stall,
        output store_set_violation
    );

    modport storebuffer(
        input recovery_flush_BCAST,
        input recovery_stall,
        output SDA_full
    );

    modport loadbuffer(
        input recovery_flush_BCAST,
        input recovery_stall,
        output LDA_full,
        output load_store_violation,
        output load_store_violation_tag
    );

    modport MemAccess(
        input MA_stall,
        input recovery_flush_BCAST,
        input recovery_stall
    );

    modport ROB(
        output flush_rob_tag_0,
        output flush_rob_tag_1,
        output flush_rob_tag_0_valid,
        output flush_rob_tag_1_valid,
        input bp_PrMiss,
        input bp_PrSuccess,
        input recovery_start,
        input recovery_flush,
        input recovery_stall,
        input recovery_procedure,
        input recovery_rollback,
        output ROB_recovery_finished,
        input load_store_violation,
        input load_store_violation_tag
    );
endinterface //pipeline_control_recovery_io
