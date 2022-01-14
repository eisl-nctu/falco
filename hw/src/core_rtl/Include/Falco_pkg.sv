`timescale 1ns/1ps
// =============================================================================
//  Program : Falco_pkg.sv
//  Author  : Hon-Chou Dai
//  Date    : May/15/2021
// -----------------------------------------------------------------------------
//  Description:
//  Parameter & Data structure used in Falco.
// -----------------------------------------------------------------------------
//  Revision information:
//
//    August/07/2021, by Chun-Wei Chao:
//      Add some signal used in store set mechanism
//    November/18/2021, by Chun-Wei Chao:
//      Revise IsROBKill function for range fulsh instructino
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

package Falco_pkg;
//Basic
    localparam int ADDR_WIDTH = 32;
    localparam int XLEN_WIDTH = 32;
    localparam int DATA_WIDTH = 32;
    localparam int MXLEN_WITDH = 32; //RISCV privileged spec terminolgy
//config
    localparam int PRF_NUM = 64;
    localparam int ARF_NUM = 32;
    localparam int GSHARE_BHSR_WIDTH = 10;
    localparam int GSHARE_PHT_SIZE = 1024;
    localparam int GSHARE_PHT_WIDTH = 10; // max ( log2(GSHARE_ENTRY_SIZE) , GSHARE_BHSR_WIDTH)
    localparam int INT_IQ_NUM = 8;
    localparam int INT_IQ_WIDTH = 3;
    localparam int MEM_IQ_NUM = 8; //change must update on hot encode and selector logic
    localparam int MEM_IQ_WIDTH = 3;
    localparam int SDB_NUM = 16;
    localparam int SDB_WIDTH = 4;
    localparam int LDB_NUM = 16;
    localparam int LDB_WIDTH = 4;
    localparam int SSIT_SIZE = 2048;
    localparam int SSIT_WIDTH = 11;
    localparam int LFST_SIZE = 256;
    localparam int LFST_WIDTH = 8;
    localparam int STORE_ID_WIDTH = 7;
    typedef struct packed {
        logic [GSHARE_PHT_WIDTH-1:0] PHT_entry_addr;
        logic branch_taken;
    } gshare_fifo_entry_t ;

    localparam int BTB_ENTRY_NUM = 512;
    localparam int ROB_ENTRY_NUM = 64;
    localparam int ROB_ENTRY_WIDTH = 6; //2 ^ 6
    localparam int CSR_BUFFER_NUM = 4; //buffer csr write data
//RV32 field
    localparam [31:0] INSTRUCTION_NOP = 32'h00000013;
    localparam int CSR_ADDR_WIDTH = 12;
    localparam int RV32_UIMM5 = 5;
    localparam int RV32_SIMM12 = 12;
    localparam int RV32_UIMM32 = 20;
    localparam int RV32_SIMM21 = 21;
    localparam int RV32_SIMM13 = 13;
//ICache settings
    localparam int ICACHE_SIZE = 64;
    localparam int ICACHE_LINE_SIZE = 256;
    localparam ICACHE_LINE_BITS = $clog2(ICACHE_LINE_SIZE);
    localparam ICACHE_BYTE_BITS = 2;
    localparam ICACHE_CARRY_BITS = ICACHE_LINE_BITS - ICACHE_BYTE_BITS*2 - 1;
//ITCM configs
    localparam int ITCM_SIZE = 1024; //KB
    localparam int ITCM_MEM_OFFSET = 32'h00000000;
//DTCM configs
    localparam int DTCM_SIZE = 1024; //KB
    localparam int DTCM_MEM_OFFSET = 32'h10000000;
// TCM configs
    localparam int TCM_SIZE = 1024; //KB
    localparam int TCM_MEM_OFFSET = 32'h00000000;
//Basic types
    typedef logic [XLEN_WIDTH-1:0] raw_instruction_t;
    typedef logic [ADDR_WIDTH-1:0] mem_addr_t;
    typedef logic [ADDR_WIDTH-1:0] pc_t;
    typedef logic [XLEN_WIDTH-1:0] xlen_data_t;
    typedef logic [GSHARE_BHSR_WIDTH-1:0] BHSR_t;
    typedef logic [GSHARE_BHSR_WIDTH-2:0] SBHSR_t; //without lsb bit
    typedef logic [4:0] arf_specifier_t;
    typedef logic [5:0] prf_specifier_t;
    typedef logic [ROB_ENTRY_WIDTH-1:0] rob_tag_t;
    typedef logic [CSR_ADDR_WIDTH-1:0] csr_addr_t;
    typedef logic [3:0] byte_mask_t; //load/store mask
    typedef struct packed {
        logic [MXLEN_WITDH-2:0] exception_code; //same as mcasue field
        logic is_interrupt; //mcasue field
        logic ecall;     //treat ecall and ebreak as exception and handle on retirement csr stage
        logic ebreak;
    } exception_t;
// common types

    // recovery based branch flush signal
    typedef struct packed {
        rob_tag_t flush_tag_0;
        rob_tag_t flush_tag_1;
        logic flush_tag_0_valid;
        logic flush_tag_1_valid;
        logic recovery_flush;
    } branch_flush_t;
//Fetch stage types
    typedef struct packed {
        logic valid;
        pc_t target_addr;
    } jump_direct_t;

    typedef struct packed {
        logic valid;
        pc_t target_addr;
    } jump_relative_t;


    typedef struct packed {
        logic irq_taken;
        pc_t pc_handler;
        logic valid;
        pc_t sys_jump_pc; 
    } sys_pc_operation_t;
//ID stage
    typedef enum logic [2:0] {
        ALU_OP_ADD  = 3'b000,
        ALU_OP_SLL  = 3'b001,
        ALU_OP_SLT  = 3'b010,
        ALU_OP_SLTU = 3'b011,
        ALU_OP_XOR  = 3'b100,
        ALU_OP_SR   = 3'b101,
        ALU_OP_OR   = 3'b110,
        ALU_OP_AND  = 3'b111
    } alu_op_t;

    typedef enum logic [2:0] {
        MULDIV_OP_MUL    = 3'b000,
        MULDIV_OP_MULH   = 3'b001,
        MULDIV_OP_MULHSU = 3'b010,
        MULDIV_OP_MULHU  = 3'b011,
        MULDIV_OP_DIV    = 3'b100,
        MULDIV_OP_DIVU   = 3'b101,
        MULDIV_OP_REM    = 3'b110,
        MULDIV_OP_REMU   = 3'b111
    } muldiv_op_t;

    typedef enum logic [2:0] {
        CSR_OP_DUMMY0   = 3'b000,  //unused
        CSR_OP_CSRRW    = 3'b001,
        CSR_OP_CSRRS    = 3'b010,
        CSR_OP_CSRRC    = 3'b011,
        CSR_OP_DUMMY1   = 3'b100,  //unused 
        CSR_OP_DUMMY2   = 3'b101,  //unused
        CSR_OP_CSRSI    = 3'b110,
        CSR_OP_CSRCI    = 3'b111
    } csr_op_t;

    typedef enum logic [2:0] {
        BRANCH_OP_BEQ   = 3'b000,   //equal
        BRANCH_OP_BNE   = 3'b001,   //not equal
        BRANCH_OP_DUMMY0= 3'b010,   //unused   
        BRANCH_OP_DUMMY1= 3'b011,   //unused
        BRANCH_OP_BLT   = 3'b100,   //less than (signed) (signed_a < signed_b)
        BRANCH_OP_BGE   = 3'b101,   //great than or equal (signed) (signed_a >= signed_b)
        BRANCH_OP_BLTU  = 3'b110,   //less than (unsigned) (a < b)
        BRANCH_OP_BGEU  = 3'b111   //greater than or equal (unsigned) (a >= b)
    } branch_op_t;

    typedef enum logic [1:0] { 
        MEM_OP_BYTE = 2'b00,
        MEM_OP_HALF_WORD = 2'b01,
        MEM_OP_WORD = 2'b10,
        MEM_OP_DUMMY0 = 2'b11 //unused
    } mem_op_size_t;
    
    typedef enum logic [1:0] {
        OPERAND0_SEL_LUI = 2'b00, //load upper immediate
        OPERAND0_SEL_PC  = 2'b01, //AUIPC,JAL,BRANCH
        OPERAND0_SEL_RS1 = 2'b10,
        OPERAND0_SEL_DUMMY0 = 2'b11
    } operand0_sel_t;

    typedef enum logic [1:0] { 
        OPERAND1_SEL_IMM     = 2'b00, //immediate
        OPERAND1_SEL_RS2     = 2'b01, 
        OPERAND1_SEL_NEG_RS2 = 2'b10, //sub
        OPERAND1_SEL_DUMMY0  = 2'b11
    } operand1_sel_t;


    typedef struct packed {
        pc_t predict_pc; //npc for checking branch prediction
        pc_t pc;
        xlen_data_t immediate;
        operand0_sel_t operand0_sel;
        operand1_sel_t operand1_sel;
        logic [2:0] ctrl_signal; //common type , type cast later
        logic mem_load_ext_sel;
        logic alu_muldiv_sel;
        logic shift_sel;
        logic is_branch;
        logic is_jal;
        logic is_jalr;
        logic is_use_rd_field; //mask S-TYPE,B-TYPE and (rd = x0 instruction)
        //csr
        logic is_csr_instr;
        logic [CSR_ADDR_WIDTH-1:0] csr_addr; 
        logic [4:0] csr_imm;
        xlen_data_t csr_data;
        //mem
        logic mem_we;
        logic mem_re;
        mem_op_size_t mem_input_sel;
        // system jump
        logic sys_jump;
        logic [CSR_ADDR_WIDTH-1:0] sys_jump_csr_addr;
        arf_specifier_t rd_addr;
        arf_specifier_t rs1_addr;
        arf_specifier_t rs2_addr;
    } decoded_op_t;

//Rename dispatch
    typedef struct packed {
        pc_t predict_pc;
        pc_t pc;
        xlen_data_t immediate;
        operand0_sel_t operand0_sel;
        operand1_sel_t operand1_sel;
        logic [2:0] ctrl_signal; //common type , type cast later
        logic alu_muldiv_sel;
        logic shift_sel;
        logic is_branch;
        logic is_jal;
        logic is_jalr;
        //csr
        logic is_csr_instr;
        logic [CSR_ADDR_WIDTH-1:0] csr_addr; 
        logic [4:0] csr_imm;
        xlen_data_t csr_data;
        // system jump (TODO : move to exception pack)
        logic sys_jump;
        logic [CSR_ADDR_WIDTH-1:0] sys_jump_csr_addr;
        prf_specifier_t rd_addr;
        prf_specifier_t rs1_addr;
        prf_specifier_t rs2_addr;
        rob_tag_t rob_tag;
        BHSR_t bhsr;
    } int_dispatch_pack_t;

    typedef struct packed {
        xlen_data_t immediate;
        logic [SSIT_WIDTH-1 : 0] store_set_pc;
        logic [LFST_WIDTH-1 : 0] store_set_id;
        logic predict_no_vilation;
        logic [STORE_ID_WIDTH-1 : 0] store_id;
        logic mem_load_ext_sel;
        logic mem_is_store;
        mem_op_size_t mem_input_sel;
        prf_specifier_t rd_addr;
        prf_specifier_t rs1_addr;
        prf_specifier_t rs2_addr;
        rob_tag_t rob_tag;
        BHSR_t bhsr;
// `ifdef FALCO_SIM_DEBUG
        pc_t pc;
// `endif
    } mem_dispatch_pack_t;
// register read
    typedef struct packed {
        pc_t pc;
        xlen_data_t immediate;
        operand0_sel_t operand0_sel;
        operand1_sel_t operand1_sel;
        logic [2:0] ctrl_signal; //common type , type cast later
        logic alu_muldiv_sel;
        logic shift_sel;
        prf_specifier_t rd_addr;
        xlen_data_t rs1_data;
        xlen_data_t rs2_data;
        prf_specifier_t rs1_addr;
        prf_specifier_t rs2_addr;
        rob_tag_t rob_tag;
    } int_issue_no_csr_pack_t;

    typedef struct packed {
        pc_t pc;
        pc_t predict_pc;
        xlen_data_t immediate;
        operand0_sel_t operand0_sel;
        operand1_sel_t operand1_sel;
        logic [2:0] ctrl_signal; //common type , type cast later
        logic shift_sel;
        logic is_branch;
        logic is_jal;
        logic is_jalr;
        //csr
        logic is_csr_instr;
        logic [CSR_ADDR_WIDTH-1:0] csr_addr; 
        logic [4:0] csr_imm;
        xlen_data_t csr_data;
        // system jump (TODO : move to exception pack)
        logic sys_jump;
        logic [CSR_ADDR_WIDTH-1:0] sys_jump_csr_addr;
        prf_specifier_t rd_addr;
        xlen_data_t rs1_data;
        xlen_data_t rs2_data;
        prf_specifier_t rs1_addr;
        prf_specifier_t rs2_addr;
        rob_tag_t rob_tag;
        BHSR_t bhsr;
    } int_issue_pack_t;
    
    typedef struct packed {
        xlen_data_t immediate;
        logic [SSIT_WIDTH-1 : 0] store_set_pc;
        logic [LFST_WIDTH-1 : 0] store_set_id;
        logic predict_no_vilation;
        logic mem_load_ext_sel;
        logic mem_is_store;
        mem_op_size_t mem_input_sel;
        prf_specifier_t rd_addr;
        xlen_data_t rs1_data;
        xlen_data_t rs2_data;
        prf_specifier_t rs1_addr;
        prf_specifier_t rs2_addr;
        rob_tag_t rob_tag;
        BHSR_t bhsr;
// `ifdef FALCO_SIM_DEBUG
        pc_t pc;
// `endif
    } mem_issue_pack_t;
// EXE
    typedef struct packed {
        logic valid;
        rob_tag_t commit_addr;
    } exe_int_commit_t;

    typedef struct packed {
        logic valid;
        rob_tag_t commit_addr;
    } exe_csr_commit_t;

    typedef struct packed {
        logic valid;
        logic is_branch;
        rob_tag_t commit_addr;
    } exe_branch_commit_t;

    typedef struct packed {
        logic valid;
        rob_tag_t commit_addr;
    } mem_commit_t;

    typedef struct packed {
        logic valid;
        prf_specifier_t wb_addr;
        xlen_data_t wb_data;
`ifdef FALCO_SIM_DEBUG
        pc_t pc;
`endif
    } exe_fu_wb_t;

    typedef struct packed {
        logic valid;
        logic branch_taken;
        pc_t target_addr; //branch result pc
        logic is_misprediction;
        pc_t branch_addr; //branch instruction pc
        BHSR_t bhsr;
`ifdef FALCO_SIM_DEBUG
        pc_t pc;
`endif
    } exe_branch_t; //execute branch calculate unit branch result

    typedef struct packed {
        logic valid;
        prf_specifier_t prf_addr;
    } exe_broadcast_t;
// MEM AGU
    typedef struct packed {
        logic [SSIT_WIDTH-1 : 0] store_set_pc;
        logic [LFST_WIDTH-1 : 0] store_set_id;
        logic predict_no_vilation;
        xlen_data_t store_data;
        xlen_data_t access_addr; //word alignment address
        logic mem_load_ext_sel;
        logic mem_is_store;
        byte_mask_t byte_sel;
        mem_op_size_t mem_input_sel;
        prf_specifier_t rd_addr;
    	rob_tag_t rob_tag;
        logic is_non_idempotent;
        BHSR_t bhsr;
// `ifdef FALCO_SIM_DEBUG
        pc_t pc;
// `endif
    } agu_pack_t;

// MEM LSU load
    typedef struct packed {
        logic load_instr_valid;
    	logic load_forward_data_valid;
        logic load_mem_ready;
        xlen_data_t load_forward_data;
        byte_mask_t load_forward_hit_mask;
    	logic [1:0] load_addr_alignment;
    	byte_mask_t load_byte_sel;
    	mem_op_size_t load_mem_input_sel;
        logic mem_load_ext_sel;
    	prf_specifier_t rd_addr;
    	rob_tag_t rob_tag;
        logic is_non_idempotent;
`ifdef FALCO_SIM_DEBUG
    	pc_t pc;
`endif
    } lsu_load_pack_t;

//Commit Stage


    typedef struct packed {
        logic map_update_0;
        logic map_update_1;
        arf_specifier_t map_arf_0;
        arf_specifier_t map_arf_1;
        prf_specifier_t map_prf_0;
        prf_specifier_t map_prf_1;
    } committed_map_update_t;
//============================================================
// Memory Access struct
//============================================================
// Instruction Access (core_top <-> Falco_top)
    typedef struct packed {
        Falco_pkg::mem_addr_t instr0_addr;
        Falco_pkg::mem_addr_t instr1_addr;
        logic p_strobe;
    } instruction_req_t;

    typedef struct packed {
        Falco_pkg::raw_instruction_t raw_instr0;
        logic instr0_valid;
        Falco_pkg::raw_instruction_t raw_instr1;
        logic instr1_valid;
        logic ready; //ICACHE or TCM not in miss state
    } instruction_resp_t;
//Load Memory Access (core_top <-> Falco_top)

    typedef struct packed {
        logic load_req;
        logic load_kill;
        Falco_pkg::mem_addr_t load_addr;
    } core_load_ck_hit_req_t;

    typedef struct packed {
        logic load_req;
        Falco_pkg::mem_addr_t load_addr;
    } core_load_data_req_t;

    typedef struct packed {
        logic load_hit;
        logic load_miss;
    } core_load_hit_resp_t;

    typedef struct packed {
        logic load_finished;
        logic load_miss;
        Falco_pkg::xlen_data_t load_data;
    } core_load_data_resp_t;

    typedef struct packed {
        logic store_req;
        Falco_pkg::mem_addr_t store_addr;
        Falco_pkg::xlen_data_t store_data;
        logic [3:0] store_mask; //byte_mask
    } core_store_req_t;

    typedef struct packed {
        logic store_finished;
        logic store_miss;
    } core_store_resp_t;

// Cross ICache Cache Line Fetch
    function logic CrossICacheLineFetch(
        input pc_t pc
    );
        begin
            //[WORD_BITS+BYTE_BITS - 1 : 0]
            // example: 5'b11100 + 4 = 5'b00000 -> cross line
            CrossICacheLineFetch = (pc[ICACHE_CARRY_BITS+ICACHE_BYTE_BITS-1:0] == 
                                    {{ICACHE_CARRY_BITS{1'b1}},{ICACHE_BYTE_BITS{1'b0}}}); 
            // must change when Cache line size change 
        end
    endfunction

    function logic IsBrROBKill (
        input branch_flush_t branch_recovery_flush,
        input rob_tag_t rob_tag
    );
        begin
            // IsBrROBKill = ((rob_tag == branch_recovery_flush.flush_tag_0 && branch_recovery_flush.flush_tag_0_valid) ||
            //                 (rob_tag == branch_recovery_flush.flush_tag_1 && branch_recovery_flush.flush_tag_1_valid)) &&
            //                branch_recovery_flush.recovery_flush;
            IsBrROBKill = branch_recovery_flush.recovery_flush & 
                                ((branch_recovery_flush.flush_tag_1 >= rob_tag) ^
                                (branch_recovery_flush.flush_tag_1 >= branch_recovery_flush.flush_tag_0) ^
                                (rob_tag >= branch_recovery_flush.flush_tag_0));
        end
    endfunction
`ifdef FALCO_SIM_DEBUG
//Internal simulation Debug
    typedef logic [XLEN_WIDTH-1:0] instruction_sequence_t;
`endif
endpackage
