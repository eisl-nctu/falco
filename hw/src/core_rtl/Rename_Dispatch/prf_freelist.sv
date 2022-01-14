`timescale 1ns/1ps
// =============================================================================
//  Program : prf_freelist.sv
//  Author  : Hon-Chou Dai
//  Date    : May/15/2021
// -----------------------------------------------------------------------------
//  Description:
//  Freelist for Falco, for rename application.
// -----------------------------------------------------------------------------
//  Revision information:
//
//    November/18/2021, by Chun-Wei Chao:
//      Revise to checkpoint recovery version.
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
//  Program : prf_freelist.sv
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

module prf_freelist(
    input                       clk,
    input                       rst,
//TO RNDS,RNDS must check freelist have element to pop
    output  logic               instr0_check_top,
    output  logic               instr1_check_top,
    input   logic               instr0_req,
    input   logic               instr1_req,
    output  prf_specifier_t     instr0_free_prf,
    output  prf_specifier_t     instr1_free_prf,
//From ROB, prf recycle or prf recovery when branch miss
    input   logic               push_stale_rd0_valid,
    input   logic               push_stale_rd1_valid,
    input   prf_specifier_t     push_stale_rd0,
    input   prf_specifier_t     push_stale_rd1,
//Recovery
    input   logic               recovery_flush,
    input   logic               recovery_rollback,
    input   logic               recovery_no_copy,
    input   rob_tag_t           recovery_target_rob_tag,
    input   rob_tag_t           instr0_rob_tag,
    input   rob_tag_t           instr1_rob_tag,
    input   logic               instr0_valid,
    input   logic               instr1_valid
);
//We don't need to check whether there is allocatable space in freelist,
//because it always has space to allocate.

localparam FL_NUM = Falco_pkg::PRF_NUM - Falco_pkg::ARF_NUM; //if not 32 must update head_ptr and tail_ptr counter
localparam FL_WIDTH = $clog2(FL_NUM);
prf_specifier_t freelist [FL_NUM-1:0]/*verilator public*/; //63?
prf_specifier_t copy_list [0:7][FL_NUM-1:0]/*verilator public*/;

logic [FL_WIDTH-1:0] head_ptr/*verilator public*/;
logic [FL_WIDTH-1:0] head_next_ptr/*verilator public*/;
logic [FL_WIDTH-1:0] tail_ptr/*verilator public*/;
logic [FL_WIDTH-1:0] tail_next_ptr/*verilator public*/;
logic [FL_WIDTH:0] counter/*verilator public*/;

logic [FL_WIDTH-1:0] copy_head_ptr [0:7]/*verilator public*/;
logic [FL_WIDTH-1:0] copy_tail_ptr [0:7]/*verilator public*/;
logic [FL_WIDTH-1:0] copy_tail_next_ptr [0:7]/*verilator public*/;
logic [FL_WIDTH:0] copy_counter [0:7]/*verilator public*/;
logic [2:0]     recovery_target;

logic push;
logic pop;

logic [1:0] push_count,pop_count;

always_comb push = push_stale_rd0_valid || push_stale_rd1_valid;
always_comb pop = instr0_req || instr1_req;
always_comb push_count = push_stale_rd0_valid + push_stale_rd1_valid;
always_comb pop_count = instr0_req + instr1_req;
always_comb head_next_ptr = head_ptr + 1;
always_comb tail_next_ptr = tail_ptr + 1;
assign recovery_target = recovery_target_rob_tag[2:0] == 0 ? recovery_target_rob_tag[5:3] : recovery_target_rob_tag[5:3] + 1;

// init: 0~31 occupy by 0
// freelist init 32~63 to array

// sim_verilator unsupport non concurrent assertion
//assert (FL_NUM == 64) 
//else   $display("error: If FL_NUM != 64,free list must use comment out block for computing ptr shift");

always_ff@(posedge clk) begin
    if (rst)
        head_ptr <= 0;
    else if (recovery_flush && ~recovery_no_copy) begin
        head_ptr <= copy_head_ptr[recovery_target];
    end else if (pop)
        /*
        if (head_ptr == FL_NUM - 1)
            head_ptr <= (pop_count == 1 ? 0 : 1);
        else if (head_ptr == FL_NUM - 2 && pop_count == 2) 
            head_ptr <= 0;
        else
            head_ptr <= head_ptr + pop_count;
        */
        head_ptr <= head_ptr + pop_count; //Let it overflow
    else
        head_ptr <= head_ptr;
end

always_ff @(posedge clk) begin
    if (rst)
        tail_ptr <= 0;
    else if (recovery_flush && ~recovery_no_copy)
        tail_ptr <= copy_tail_ptr[recovery_target];
    else if (push)
        /*if (tail_ptr == FL_NUM - 1)
            tail_ptr <= (push_count == 1 ? 0 : 1);
        else if (tail_ptr == FL_NUM - 2 && push_count == 2) 
            tail_ptr <= 0;
        else
            tail_ptr <= tail_ptr + push_count;
        */
        tail_ptr <= tail_ptr + push_count; //let if overflow
    else
        tail_ptr <= tail_ptr;
end

integer i;
always_ff @(posedge clk) begin
    if (rst) begin
        for (i = 0 ; i < 32 ; i = i + 1)
            freelist[i] <= i + 32; //enqueue 32~63 prf to freelist
        //for (i = 32 ; i < FL_NUM ; i = i + 1)
        //    freelist[i] <= 0;
        //for (i = 0 ; i < 63 ; i = i + 1)
        //    freelist[i] <= i+1;
    end else if (recovery_flush && ~recovery_no_copy) begin
        for (i = 0 ; i < FL_NUM ; i = i + 1) begin
            freelist[i] <= copy_list[recovery_target][i];
        end
    end else if (push) begin
        freelist[tail_ptr] <= (push_stale_rd0_valid ? push_stale_rd0 :
                               push_stale_rd1_valid ? push_stale_rd1 : freelist[tail_ptr]);
        freelist[tail_next_ptr] <= (push_stale_rd1_valid && push_stale_rd0_valid ? 
                                        push_stale_rd1 : freelist[tail_next_ptr]);
    end else begin
        for (i = 0 ; i < FL_NUM ; i = i + 1) begin
            freelist[i] <= freelist[i];
        end
    end
end

always_ff @(posedge clk) begin
    if (rst) begin
        counter <= FL_NUM; //init 32 free reg
    end else if (recovery_flush && ~recovery_no_copy) begin
        counter <= copy_counter[recovery_target];
    end else begin
        case ({push,pop})
            2'b00: counter <= counter;
            2'b01: counter <= counter - pop_count;
            2'b10: counter <= counter + push_count;
            2'b11: counter <= counter + push_count - pop_count; //overflow???
            default: counter <= counter;
        endcase
    end
end

// pop queue
always_comb instr0_check_top = (counter >= 1);
always_comb instr1_check_top = (counter >= 2);
always_comb instr0_free_prf = freelist[head_ptr];
always_comb begin
    if (instr0_req)
        instr1_free_prf = freelist[head_next_ptr];
    else
        instr1_free_prf = freelist[head_ptr];
end

// copy list
logic       instr0_copy, instr1_copy;
logic [2:0] copy_target;

assign instr0_copy = (instr0_rob_tag[2:0] == 0);
assign instr1_copy = (instr1_rob_tag[2:0] == 0);
assign copy_target = instr0_copy ? instr0_rob_tag[5:3] : instr1_rob_tag[5:3];

genvar geni, genj;
generate
    for (geni = 0; geni < 8; geni = geni + 1) begin
        for (genj = 0; genj < FL_NUM; genj = genj + 1) begin
            always_ff @(posedge clk) begin
                if (rst)
                    copy_list[geni][genj] <= 0;
                else if (~instr0_copy && ~instr1_copy) begin
                    if (genj == copy_tail_ptr[geni] && ~recovery_rollback)
                        copy_list[geni][genj] <= (push_stale_rd0_valid ? push_stale_rd0 :
                               push_stale_rd1_valid ? push_stale_rd1 : copy_list[geni][genj]);
                    else if (genj == copy_tail_next_ptr[geni] && ~recovery_rollback)
                        copy_list[geni][genj] <= (push_stale_rd1_valid && push_stale_rd0_valid ? 
                                        push_stale_rd1 : copy_list[geni][genj]);
                    else 
                        copy_list[geni][genj] <= copy_list[geni][genj];
                end
                else if (copy_target == geni) begin
                    // if (genj == instr1_rd && instr1_copy)
                    //     copy_list[geni][genj] <= ;
                    // else if (genj == instr0_rd)
                    //     copy_list[geni][genj] <= ;
                    // else 
                    if (genj == tail_ptr && ~recovery_rollback)
                        copy_list[geni][genj] <= (push_stale_rd0_valid ? push_stale_rd0 :
                               push_stale_rd1_valid ? push_stale_rd1 : freelist[genj]);
                    else if (genj == tail_next_ptr && ~recovery_rollback)
                        copy_list[geni][genj] <= (push_stale_rd1_valid && push_stale_rd0_valid ? 
                                        push_stale_rd1 : freelist[genj]);
                    else
                        copy_list[geni][genj] <= freelist[genj];
                end
                else begin
                    if (genj == copy_tail_ptr[geni] && ~recovery_rollback)
                        copy_list[geni][genj] <= (push_stale_rd0_valid ? push_stale_rd0 :
                               push_stale_rd1_valid ? push_stale_rd1 : copy_list[geni][genj]);
                    else if (genj == copy_tail_next_ptr[geni] && ~recovery_rollback)
                        copy_list[geni][genj] <= (push_stale_rd1_valid && push_stale_rd0_valid ? 
                                        push_stale_rd1 : copy_list[geni][genj]);
                    else 
                        copy_list[geni][genj] <= copy_list[geni][genj];
                end
            end
        end
    end

    for (geni = 0; geni < 8; geni = geni + 1) begin
        always_ff @(posedge clk) begin
            if (rst)
                copy_counter[geni] <= 0;
            else if (~instr0_copy && ~instr1_copy)
                copy_counter[geni] <= copy_counter[geni] + (recovery_rollback ? 0 : push_count);
            else if (copy_target == geni && ~recovery_rollback) begin
                case ({push,pop})
                    2'b00: copy_counter[geni] <= counter;
                    2'b01: copy_counter[geni] <= counter - (instr1_copy && instr0_req);
                    2'b10: copy_counter[geni] <= counter + push_count;
                    2'b11: copy_counter[geni] <= counter + push_count - (instr1_copy && instr0_req); //overflow???
                endcase
            end
            else
                copy_counter[geni] <= copy_counter[geni] + (recovery_rollback ? 0 : push_count);
        end
    end

    for (geni = 0; geni < 8; geni = geni + 1) begin
        always_ff @(posedge clk) begin
            if (rst)
                copy_head_ptr[geni] <= 0;
            else if (~instr0_copy && ~instr1_copy)
                copy_head_ptr[geni] <= copy_head_ptr[geni];
            else if (copy_target == geni) begin
                copy_head_ptr[geni] <= head_ptr + (instr1_copy && instr0_req);
            end
            else
                copy_head_ptr[geni] <= copy_head_ptr[geni];
        end
    end

    for (geni = 0; geni < 8; geni = geni + 1) begin
        always_ff @(posedge clk) begin
            if (rst)
                copy_tail_ptr[geni] <= 0;
            else if (~instr0_copy && ~instr1_copy)
                copy_tail_ptr[geni] <= copy_tail_ptr[geni] + (recovery_rollback ? 0 : push_count);
            else if (copy_target == geni && ~recovery_rollback) begin
                copy_tail_ptr[geni] <= tail_ptr + push_count;
            end
            else
                copy_tail_ptr[geni] <= copy_tail_ptr[geni] + (recovery_rollback ? 0 : push_count);
        end
    end

    for (geni = 0; geni < 8; geni = geni + 1) begin
        always_comb copy_tail_next_ptr[geni] = copy_tail_ptr[geni] + 1;
    end

endgenerate

assert property(
    @(posedge clk ) counter <= FL_NUM
)  else  begin
    $display("[%0t] redundant freelist push occur: counter = %d" , $time, counter);
    $display("rd0: %d, valid: %d",push_stale_rd0, push_stale_rd0_valid);
    $display("r10: %d, valid: %d",push_stale_rd1, push_stale_rd1_valid);
end


endmodule
