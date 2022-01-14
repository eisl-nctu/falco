`timescale 1ns/1ps
// =============================================================================
//  Program : store_buffer.sv
//  Author  : Hon-Chou Dai
//  Date    : May/15/2021
// -----------------------------------------------------------------------------
//  Description:
//  Store store instruction for in-order commit.
// -----------------------------------------------------------------------------
//  Revision information:
//
//    August/07/2021    : 
//      Combine new allocated store instruction with old store instruction which has same
//      address. So the forward data can be integrated.
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
import L1_cache_pkg::*;

module store_buffer (
    input clk,
    input rst,
    output core_store_req_t store_req,
    input core_store_resp_t store_resp,
    store_buffer_io.storebuffer sdb_io,
    commit_stage_io.storebuffer rob_io,
    pipeline_control_recovery_io.storebuffer pipe_ctrl_io
);

// =============================================
// Store buffer structure
// =============================================
xlen_data_t SDB_data[SDB_NUM]/*verilator public*/;
xlen_data_t SDB_addr[SDB_NUM]/*verilator public*/;
byte_mask_t st_mask[SDB_NUM] /*verilator public*/;
rob_tag_t SDB_rob_tag[SDB_NUM] /*verilator public*/;
`ifdef FALCO_SIM_DEBUG
    pc_t    pc[SDB_NUM] /*verilator public*/;
`endif
logic       retirement[SDB_NUM] /*verilator public*/;
logic       valid[SDB_NUM] /*verilator public*/;
logic       is_non_idempotent[SDB_NUM]/*verilator public*/;

// =============================================
// Store buffer control
// =============================================
logic [SDB_WIDTH-1:0]   alloc_ptr /*verilator public*/;
//ROB must send siganl if recovery flush instruction which finished and is memory op
logic [SDB_WIDTH-1:0]   alloc_pre_0_ptr; //use for clear valid
logic [SDB_WIDTH-1:0]   alloc_pre_1_ptr; //use for clear valid
logic [SDB_WIDTH-1:0]   retire_ptr/*verilator public*/;
//ROB must info that there are store instruction ready to retirement
logic [SDB_WIDTH-1:0]   retire_next_ptr/*verilator public*/;
logic [SDB_WIDTH-1:0]   pop_ptr/*verilator public*/;
// logic [SDB_WIDTH:0]     empty_counter/*verilator public*/;

// =============================================
// Load forward logic
// =============================================
logic                   load_store_addr_match [SDB_NUM];
logic                   load_forward_hit;
logic [SDB_WIDTH-1:0]   load_forward_hit_index;
xlen_data_t             load_forward_data;
byte_mask_t             load_forward_hit_mask;

// =============================================
// Store buffer operation signal (push/pop)
// =============================================
logic pop;
logic store_pop;

logic allocate_new_entry;

// =============================================
// Store buffer non idempotent region checker
// =============================================
logic entry_is_non_idempotent[SDB_NUM];

// =============================================
// Store buffer input data & mask
// =============================================
xlen_data_t             sdb_input_data;
byte_mask_t             sdb_input_mask;
logic                   sdb_input_match[4][SDB_NUM];
logic [SDB_WIDTH-1:0]   temp_ptr[SDB_NUM-1];
logic [7:0]             sdb_input_byte_data[4];
logic                   byte_match[4];
logic [SDB_WIDTH-1:0]   byte_match_index[4];


// =============================================
// Flush signal
// =============================================
logic [SDB_NUM-1:0] flush_vec /*verilator public*/;
logic [SDB_WIDTH-1:0] flush_counter /*verilator public*/;

always_comb allocate_new_entry = sdb_io.store_allocate &&
                                 sdb_io.instr_valid &&
                                 ~pipe_ctrl_io.SDA_full;
always_comb pop = store_pop || pipe_ctrl_io.recovery_stall;
//ROB must info that there are
always_comb pipe_ctrl_io.SDA_full = (alloc_ptr == pop_ptr - 1) ||
                        (alloc_ptr == 15 && pop_ptr == 0);

`ifdef FALCO_SIM_DEBUG
    logic sdbfull /*verilator public*/;
    always_comb sdbfull = pipe_ctrl_io.SDA_full;
`endif 

always_comb alloc_pre_0_ptr = alloc_ptr - 1;
always_comb alloc_pre_1_ptr = alloc_ptr - 2;

always_ff @(posedge clk) begin
    if (rst)
        alloc_ptr <= 0;
    else if (pipe_ctrl_io.recovery_flush_BCAST.recovery_flush)
        alloc_ptr <= alloc_ptr - flush_counter;
    else if (allocate_new_entry)
        alloc_ptr <= alloc_ptr + 1;
    else
        alloc_ptr <= alloc_ptr;
end

always_ff @(posedge clk) begin
    if (rst)
        retire_ptr <= 0;
    else
        retire_ptr <= retire_ptr + rob_io.store_commit_0_valid + rob_io.store_commit_1_valid;
end

always_comb
    flush_counter = flush_vec[0] + flush_vec[1] + flush_vec[2] + flush_vec[3] +
                    flush_vec[4] + flush_vec[5] + flush_vec[6] + flush_vec[7] +
                    flush_vec[8] + flush_vec[9] + flush_vec[10] + flush_vec[11] +
                    flush_vec[12] + flush_vec[13] + flush_vec[14] + flush_vec[15];

always_comb retire_next_ptr = retire_ptr + 1;

always_ff @(posedge clk) begin
    if (rst)
        pop_ptr <= 0;
    else if (store_pop)
        pop_ptr <= pop_ptr + 1; //don't care flush event, because pop_ptr handle "committed" store data
    else
        pop_ptr <= pop_ptr;
end
integer i;
genvar geni, genj;

generate
for (geni = 0 ; geni < SDB_NUM ; geni = geni + 1)
    always_comb
        flush_vec[geni] = IsBrROBKill(pipe_ctrl_io.recovery_flush_BCAST, SDB_rob_tag[geni]) ? (valid[geni]) : 0;
        
for (geni = 0 ; geni < SDB_NUM ; geni = geni + 1) begin
    always_ff @(posedge clk) begin
        if (rst) begin
            SDB_addr[geni] <= 0;
            SDB_data[geni] <= 0;
            st_mask[geni] <= 0;
            SDB_rob_tag[geni] <= 0;
            is_non_idempotent[geni] <= 0;
        end else if (alloc_ptr == geni && allocate_new_entry) begin
            SDB_addr[geni] <= sdb_io.store_allocate_addr;
            SDB_data[geni] <= sdb_input_data; //sdb_io.store_allocate_data;
            st_mask[geni] <= sdb_input_mask; //sdb_io.store_allocate_mask;
            SDB_rob_tag[geni] <= sdb_io.store_allocate_rob_tag;
            is_non_idempotent[geni] <= sdb_io.store_non_idempotent_region;
        end else begin
            SDB_addr[geni] <= SDB_addr[geni];
            SDB_data[geni] <= SDB_data[geni];
            st_mask[geni] <= st_mask[geni];
            SDB_rob_tag[geni] <= SDB_rob_tag[geni];
            is_non_idempotent[geni] <= is_non_idempotent[geni];
        end
    end
end

`ifdef FALCO_SIM_DEBUG
for (geni = 0 ; geni < SDB_NUM ; geni = geni + 1) begin
    always_ff @(posedge clk) begin
        if (rst) begin
            pc[geni] <= 0;
        end else if (alloc_ptr == geni && allocate_new_entry) begin
            pc[geni] <= sdb_io.store_pc;
        end else begin
            pc[geni] <= pc[geni];
        end
    end
end
`endif

for (geni = 0 ; geni < SDB_NUM ; geni = geni + 1) begin
    always_ff @(posedge clk) begin
        if (rst) begin
            valid[geni] <= 0;
        end else if (alloc_ptr == geni && allocate_new_entry) begin
            valid[geni] <= 1;
        end else if ( (pop_ptr == geni && store_pop) || flush_vec[geni])begin
            valid[geni] <= 0;
        end else begin
            valid[geni] <= valid[geni];
        end
    end
end

for (geni = 0 ; geni < SDB_NUM ; geni = geni + 1) begin
    always_ff @(posedge clk) begin
        if (rst) begin
            retirement[geni] <= 0;
        end else if (alloc_ptr == geni && allocate_new_entry) begin
            retirement[geni] <= 0;
        end else if (retire_next_ptr == geni &&
                     rob_io.store_commit_0_valid &&
                     rob_io.store_commit_1_valid) begin
            retirement[geni] <= 1;
        end else if (retire_ptr == geni &&
                     (rob_io.store_commit_0_valid || rob_io.store_commit_1_valid)) begin
            retirement[geni] <= 1;
        end else begin
            retirement[geni] <= retirement[geni];
        end
    end
end

for (geni = 0 ; geni < SDB_NUM ; geni = geni + 1) begin
    always_comb
        load_store_addr_match[geni] = (SDB_addr[geni] == sdb_io.sb_load_find_addr ? 1 : 0) &&
                                valid[geni] == 1 &&
                                (st_mask[geni] & sdb_io.sb_load_find_mask);
end

for (geni = 0 ; geni < SDB_NUM ; geni = geni + 1) begin
    always_comb
        entry_is_non_idempotent[geni] = valid[geni] && is_non_idempotent[geni];
end

for (geni = 0 ; geni < SDB_NUM-1 ; geni = geni + 1) begin
    always_comb
        temp_ptr[geni] = (alloc_ptr > geni) ? 
                alloc_ptr - (1+geni) : (alloc_ptr+8) - (1+geni);
end

for (geni = 0 ; geni < SDB_NUM ; geni = geni + 1) begin
    for (genj = 0 ; genj < 4 ; genj = genj + 1) begin
        always_comb
            sdb_input_match[genj][geni] = valid[geni] && (SDB_addr[geni] == sdb_io.store_allocate_addr) && st_mask[geni][genj];
    end
end

for (geni = 0 ; geni < 4; geni = geni + 1) begin
    combine_data_unit CDU(
        .match(sdb_input_match[geni]),
        .push_head(alloc_ptr),
        .hit(byte_match[geni]),
        .hit_index(byte_match_index[geni])
    );
end
endgenerate

load_forward_unit LFU(
    .match(load_store_addr_match),
    .push_head(alloc_ptr),
    .hit(load_forward_hit),
    .hit_index(load_forward_hit_index)
);

always_comb begin
    sdb_io.non_idempotent_instr_exists =
        entry_is_non_idempotent[0] || entry_is_non_idempotent[1] ||
        entry_is_non_idempotent[2] || entry_is_non_idempotent[3] ||
        entry_is_non_idempotent[4] || entry_is_non_idempotent[5] ||
        entry_is_non_idempotent[6] || entry_is_non_idempotent[7] ||
        entry_is_non_idempotent[8] || entry_is_non_idempotent[9] ||
        entry_is_non_idempotent[10] || entry_is_non_idempotent[11] ||
        entry_is_non_idempotent[12] || entry_is_non_idempotent[13] ||
        entry_is_non_idempotent[14] || entry_is_non_idempotent[15];
end

always_comb begin
    load_forward_hit_mask = st_mask[load_forward_hit_index];
    load_forward_data = SDB_data[load_forward_hit_index];
end

always_comb begin
    sdb_io.sb_load_forward_hit = load_forward_hit;
    sdb_io.sb_load_forward_data = load_forward_data;
    sdb_io.sb_load_forward_mask = load_forward_hit_mask;
end

// Store logic to DCache/DTCM/device
always_comb begin
    store_req.store_req = valid[pop_ptr] & retirement[pop_ptr];
    store_req.store_addr = SDB_addr[pop_ptr];
    store_req.store_data = SDB_data[pop_ptr];
    store_req.store_mask = st_mask[pop_ptr];
end

always_comb store_pop = store_req.store_req && store_resp.store_finished;

always_comb begin
    sdb_input_data[7:0] = sdb_io.store_allocate_mask[0] ? sdb_io.store_allocate_data[7:0] :
                            byte_match[0] ? SDB_data[byte_match_index[0]][7:0] : 8'b0;
    sdb_input_mask[0] = sdb_io.store_allocate_mask[0] | byte_match[0];

    sdb_input_data[15:8] = sdb_io.store_allocate_mask[1] ? sdb_io.store_allocate_data[15:8] :
                            byte_match[1] ? SDB_data[byte_match_index[1]][15:8] : 8'b0;
    sdb_input_mask[1] = sdb_io.store_allocate_mask[1] | byte_match[1];

    sdb_input_data[23:16] = sdb_io.store_allocate_mask[2] ? sdb_io.store_allocate_data[23:16] :
                            byte_match[2] ? SDB_data[byte_match_index[2]][23:16] : 8'b0;
    sdb_input_mask[2] = sdb_io.store_allocate_mask[2] | byte_match[2];

    sdb_input_data[31:24] = sdb_io.store_allocate_mask[3] ? sdb_io.store_allocate_data[31:24] :
                            byte_match[3] ? SDB_data[byte_match_index[3]][31:24] : 8'b0;
    sdb_input_mask[3] = sdb_io.store_allocate_mask[3] | byte_match[3];
    
end


`ifdef FALCO_SIM_DEBUG
    function logic [7:0] ver_store_req;
        /*verilator public*/
        ver_store_req = store_req.store_req ? 1 : 0;
    endfunction
    function logic [31:0] ver_store_addr;
        /*verilator public*/
        ver_store_addr = store_req.store_addr;
    endfunction
    function logic [31:0] ver_store_data;
        /*verilator public*/
        ver_store_data = store_req.store_data;
    endfunction
    function logic [7:0] ver_store_mask;
        /*verilator public*/
        ver_store_mask = store_req.store_mask;
    endfunction
    function logic [7:0] ver_store_finished;
        /*verilator public*/
        ver_store_finished = store_resp.store_finished;
    endfunction
`endif

endmodule
