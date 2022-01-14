`timescale 1ns/1ps
// =============================================================================
//  Program : load_buffer.sv
//  Author  : Chun-Wei Chao
//  Date    : August/07/2021
// -----------------------------------------------------------------------------
//  Description:
//  Store load information in this buffer until it be committed
//  These load information provide store instruction in LSU stage check whether 
//  load/store violation happen or not.
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

module load_buffer (
    input clk,
    input rst,
    load_buffer_io.loadbuffer ldb_io,
    store_buffer_io.loadbuffer sdb_io,
    commit_stage_io.loadbuffer rob_io,
    pipeline_control_recovery_io.loadbuffer pipe_ctrl_io
);

xlen_data_t LDB_addr[LDB_NUM]/*verilator public*/;
byte_mask_t LDB_mask[LDB_NUM] /*verilator public*/;
logic       LDB_valid[LDB_NUM] /*verilator public*/;
rob_tag_t   LDB_tag[LDB_NUM] /*verilator public*/;
logic [SSIT_WIDTH-1 : 0] LDB_store_set_pc[LDB_NUM] /*verilator public*/;
logic [LFST_WIDTH-1 : 0] LDB_store_set_id[LDB_NUM] /*verilator public*/;

`ifdef FALCO_SIM_DEBUG
    pc_t    pc[LDB_NUM] /*verilator public*/;
`endif

logic                   alloc_ptr_valid;
logic [LDB_WIDTH-1:0]   alloc_ptr /*verilator public*/;

logic allocate_new_entry;

logic compare_address[LDB_NUM];
logic compare_rob_tag[LDB_NUM];
logic [LDB_NUM-1:0] compare_all;

rob_tag_t next_tag;

always_comb allocate_new_entry = ldb_io.load_allocate &&
                                 ldb_io.instr_valid && alloc_ptr_valid;
                                //  ~pipe_ctrl_io.SDA_full; //lock siganl?

always_comb ldb_io.violation_detect = compare_all != 16'b0;

always_comb next_tag = (rob_io.cur_commit_rob_tag == 63) ? 0 : rob_io.cur_commit_rob_tag + 1;


// load_buffer_select ldb_select(
//     .valid(LDB_valid),
//     .select_index(alloc_ptr),
//     .select_index_valid(alloc_ptr_valid)
// );

always_comb begin
    if (compare_all[0]) ldb_io.violation_load_pc = LDB_store_set_pc[0];
    else if (compare_all[1]) ldb_io.violation_load_pc = LDB_store_set_pc[1];
    else if (compare_all[2]) ldb_io.violation_load_pc = LDB_store_set_pc[2];
    else if (compare_all[3]) ldb_io.violation_load_pc = LDB_store_set_pc[3];
    else if (compare_all[4]) ldb_io.violation_load_pc = LDB_store_set_pc[4];
    else if (compare_all[5]) ldb_io.violation_load_pc = LDB_store_set_pc[5];
    else if (compare_all[6]) ldb_io.violation_load_pc = LDB_store_set_pc[6];
    else if (compare_all[7]) ldb_io.violation_load_pc = LDB_store_set_pc[7];
    else if (compare_all[8]) ldb_io.violation_load_pc = LDB_store_set_pc[8];
    else if (compare_all[9]) ldb_io.violation_load_pc = LDB_store_set_pc[9];
    else if (compare_all[10]) ldb_io.violation_load_pc = LDB_store_set_pc[10];
    else if (compare_all[11]) ldb_io.violation_load_pc = LDB_store_set_pc[11];
    else if (compare_all[12]) ldb_io.violation_load_pc = LDB_store_set_pc[12];
    else if (compare_all[13]) ldb_io.violation_load_pc = LDB_store_set_pc[13];
    else if (compare_all[14]) ldb_io.violation_load_pc = LDB_store_set_pc[14];
    else ldb_io.violation_load_pc = LDB_store_set_pc[15];
end

always_comb begin
    if (compare_all[0]) ldb_io.violation_load_id = LDB_store_set_id[0];
    else if (compare_all[1]) ldb_io.violation_load_id = LDB_store_set_id[1];
    else if (compare_all[2]) ldb_io.violation_load_id = LDB_store_set_id[2];
    else if (compare_all[3]) ldb_io.violation_load_id = LDB_store_set_id[3];
    else if (compare_all[4]) ldb_io.violation_load_id = LDB_store_set_id[4];
    else if (compare_all[5]) ldb_io.violation_load_id = LDB_store_set_id[5];
    else if (compare_all[6]) ldb_io.violation_load_id = LDB_store_set_id[6];
    else if (compare_all[7]) ldb_io.violation_load_id = LDB_store_set_id[7];
    else if (compare_all[8]) ldb_io.violation_load_id = LDB_store_set_id[8];
    else if (compare_all[9]) ldb_io.violation_load_id = LDB_store_set_id[9];
    else if (compare_all[10]) ldb_io.violation_load_id = LDB_store_set_id[10];
    else if (compare_all[11]) ldb_io.violation_load_id = LDB_store_set_id[11];
    else if (compare_all[12]) ldb_io.violation_load_id = LDB_store_set_id[12];
    else if (compare_all[13]) ldb_io.violation_load_id = LDB_store_set_id[13];
    else if (compare_all[14]) ldb_io.violation_load_id = LDB_store_set_id[14];
    else ldb_io.violation_load_id = LDB_store_set_id[15];
end

genvar geni;
generate

    for (geni = 0 ; geni < LDB_NUM ; geni = geni + 1) begin
        always_ff @(posedge clk) begin
            if (rst) LDB_valid[geni] <= 0;
            else if (pipe_ctrl_io.recovery_stall) begin
                LDB_valid[geni] <= IsBrROBKill(pipe_ctrl_io.recovery_flush_BCAST, LDB_tag[geni]) ? 0 : LDB_valid[geni];
            end
            else if (LDB_valid[geni] && 
                ((rob_io.commit_instr0 && rob_io.cur_commit_rob_tag == LDB_tag[geni]) || 
                (rob_io.commit_instr1 && (next_tag) == LDB_tag[geni])))
                LDB_valid[geni] <= 0;
            else if (geni == alloc_ptr && allocate_new_entry) LDB_valid[geni] <= 1;
            else LDB_valid[geni] <= LDB_valid[geni];
        end
    end

    for (geni = 0 ; geni < LDB_NUM ; geni = geni + 1) begin
        always_ff @(posedge clk) begin
            if (rst) LDB_addr[geni] <= 0;
            else if (geni == alloc_ptr && allocate_new_entry) LDB_addr[geni] <= ldb_io.load_allocate_addr;
            else LDB_addr[geni] <= LDB_addr[geni];
        end
    end

    for (geni = 0 ; geni < LDB_NUM ; geni = geni + 1) begin
        always_ff @(posedge clk) begin
            if (rst) LDB_mask[geni] <= 0;
            else if (geni == alloc_ptr && allocate_new_entry) LDB_mask[geni] <= ldb_io.load_allocate_mask;
            else LDB_mask[geni] <= LDB_mask[geni];
        end
    end

    for (geni = 0 ; geni < LDB_NUM ; geni = geni + 1) begin
        always_ff @(posedge clk) begin
            if (rst) LDB_tag[geni] <= 0;
            else if (geni == alloc_ptr && allocate_new_entry) LDB_tag[geni] <= ldb_io.load_allocate_rob_tag;
            else LDB_tag[geni] <= LDB_tag[geni];
        end
    end

    for (geni = 0 ; geni < LDB_NUM ; geni = geni + 1) begin
        always_ff @(posedge clk) begin
            if (rst) LDB_store_set_pc[geni] <= 0;
            else if (geni == alloc_ptr && allocate_new_entry) LDB_store_set_pc[geni] <= ldb_io.load_store_set_pc;
            else LDB_store_set_pc[geni] <= LDB_store_set_pc[geni];
        end
    end

    for (geni = 0 ; geni < LDB_NUM ; geni = geni + 1) begin
        always_ff @(posedge clk) begin
            if (rst) LDB_store_set_id[geni] <= 0;
            else if (geni == alloc_ptr && allocate_new_entry) LDB_store_set_id[geni] <= ldb_io.load_store_set_id;
            else LDB_store_set_id[geni] <= LDB_store_set_id[geni];
        end
    end

`ifdef FALCO_SIM_DEBUG
    for (geni = 0 ; geni < LDB_NUM ; geni = geni + 1) begin
        always_ff @(posedge clk) begin
            if (rst) begin
                pc[geni] <= 0;
            end else if (alloc_ptr == geni && allocate_new_entry) begin
                pc[geni] <= ldb_io.load_pc;
            end else begin
                pc[geni] <= pc[geni];
            end
        end
    end
`endif

    for (geni = 0 ; geni < LDB_NUM ; geni = geni + 1) begin
        always_comb begin
            compare_address[geni] = (LDB_addr[geni] == ldb_io.store_addr);
        end
    end

    for (geni = 0 ; geni < LDB_NUM ; geni = geni + 1) begin
        always_comb begin
            compare_rob_tag[geni] = 
            LDB_valid[geni] & (LDB_tag[geni] != rob_io.cur_commit_rob_tag) & 
            (((LDB_tag[geni] > ldb_io.store_rob_tag) ^
                     (LDB_tag[geni] > rob_io.cur_commit_rob_tag) ^
                     (ldb_io.store_rob_tag > rob_io.cur_commit_rob_tag)) || 
                     (ldb_io.store_rob_tag == rob_io.cur_commit_rob_tag));
        end
    end

    for (geni = 0 ; geni < LDB_NUM ; geni = geni + 1) begin
        always_comb compare_all[geni] = (compare_address[geni] & compare_rob_tag[geni]);
    end

endgenerate

always_comb begin
    if(~LDB_valid[0]) begin
            alloc_ptr = 0;
            alloc_ptr_valid = 1;
    end
    else if(~LDB_valid[1]) begin
            alloc_ptr = 1;
            alloc_ptr_valid = 1;
    end
    else if(~LDB_valid[2]) begin
            alloc_ptr = 2;
            alloc_ptr_valid = 1;
    end
    else if(~LDB_valid[3]) begin
            alloc_ptr = 3;
            alloc_ptr_valid = 1;
    end
    else if(~LDB_valid[4]) begin
            alloc_ptr = 4;
            alloc_ptr_valid = 1;
    end
    else if(~LDB_valid[5]) begin
            alloc_ptr = 5;
            alloc_ptr_valid = 1;
    end
    else if(~LDB_valid[6]) begin
            alloc_ptr = 6;
            alloc_ptr_valid = 1;
    end
    else if(~LDB_valid[7]) begin
            alloc_ptr = 7;
            alloc_ptr_valid = 1;
    end
    else if(~LDB_valid[8]) begin
            alloc_ptr = 8;
            alloc_ptr_valid = 1;
    end
    else if(~LDB_valid[9]) begin
            alloc_ptr = 9;
            alloc_ptr_valid = 1;
    end
    else if(~LDB_valid[10]) begin
            alloc_ptr = 10;
            alloc_ptr_valid = 1;
    end
    else if(~LDB_valid[11]) begin
            alloc_ptr = 11;
            alloc_ptr_valid = 1;
    end
    else if(~LDB_valid[12]) begin
            alloc_ptr = 12;
            alloc_ptr_valid = 1;
    end
    else if(~LDB_valid[13]) begin
            alloc_ptr = 13;
            alloc_ptr_valid = 1;
    end
    else if(~LDB_valid[14]) begin
            alloc_ptr = 14;
            alloc_ptr_valid = 1;
    end
    else if(~LDB_valid[15]) begin
            alloc_ptr = 15;
            alloc_ptr_valid = 1;
    end
    else begin
        alloc_ptr = 0;
        alloc_ptr_valid = 0;
    end
end

`ifdef FALCO_SIM_DEBUG
    function [31:0] ver_LDB_pc;
        /*verilator public*/
        input [31:0] index;
        ver_LDB_pc = pc[index];
    endfunction

    function [31:0] ver_LDB_valid;
        /*verilator public*/
        input [31:0] index;
        ver_LDB_valid = LDB_valid[index];
    endfunction

    function [31:0] ver_LDB_addr;
        /*verilator public*/
        input [31:0] index;
        ver_LDB_addr = LDB_addr[index];
    endfunction
    
    function [31:0] ver_LDB_mask;
        /*verilator public*/
        input [31:0] index;
        ver_LDB_mask = LDB_mask[index];
    endfunction
    
    function [31:0] ver_LDB_tag;
        /*verilator public*/
        input [31:0] index;
        ver_LDB_tag = LDB_tag[index];
    endfunction

`endif

endmodule
