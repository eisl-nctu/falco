`timescale 1ns/1ps
// =============================================================================
//  Program : decoder.sv
//  Author  : Hon-Chou Dai
//  Date    : May/15/2021
// -----------------------------------------------------------------------------
//  Description:
//  Decode riscv-32 im instruction in this module.
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

module decoder
(
    // Signals from the Fetch Stage.
    input [DATA_WIDTH-1 : 0]  pc,
    input [DATA_WIDTH-1 : 0]  instruction,
    input pc_t                predict_pc,
    // illegal
    output logic               illegal_instr_o,
    output decoded_op_t        decoded_instr
);


/* *******************************************************************************
 * Info Signals Description                                                      *
 * ----------------------------------------------------------------------------- *
 *      There are 2 inputs for the exe:                                          *
 *          # inputA has 4 possible sources, determined by inputA_sel:           *
 *              (0) 0 (LUI)                                                      *
 *              (1) pc (AUIPC, JAL, BRANCH)                                      *
 *              (2) rs1                                                          *
 *          # inputB has 3 possible sources, determined by inputB_sel:           *
 *              (0) imm                                                          *
 *              (1) rs2                                                          *
 *              (2) -rs2 (SUB)                                                   *
 * ----------------------------------------------------------------------------- *
 *      There are 6 possible input sources to the register file, determined by   *
 *      regfile_input_sel:                                                       *
 *              (0) one byte from data memory                                    *
 *              (1) half of word from data memory                                *
 *              (2) word from data memory                                        *
 *              (3) pc + 4 (pc of next instruction)                              *
 *              (4) execute result                                               *
 *              (5) csr value                                                    *
 * ----------------------------------------------------------------------------- *
 *      Usage of operation_sel are listed below:                                 *
 *          # Conditional branch                                                 *
 *              (0) 3'b000 : equal                                               *
 *              (1) 3'b001 : not equal                                           *
 *              (2) 3'b100 : less than                                           *
 *              (3) 3'b101 : greater than or equal to                            *
 *              (4) 3'b110 : less than unsigned                                  *
 *              (5) 3'b111 : greater than or equal to                            *
 *          # RVI, alu_muldiv_sel = 0                                            *
 *              (0) 3'b000 : add (add/sub)                                       *
 *              (1) 3'b001 : sll (shift left logic)                              *
 *              (2) 3'b010 : slt (set less than)                                 *
 *              (3) 3'b011 : sltu (set less than unsigned)                       *
 *              (4) 3'b100 : xor                                                 *
 *              (5) 3'b101 : sr (shift right logic/arithmetic)                   *
 *              (6) 3'b110 : or                                                  *
 *              (7) 3'b111 : and                                                 *
 *          # RVM, alu_muldiv_sel = 1                                            *
 *              (0) 3'b000 : mul                                                 *
 *              (1) 3'b001 : mulh                                                *
 *              (2) 3'b010 : mulhsu                                              *
 *              (3) 3'b011 : mulhu                                               *
 *              (4) 3'b100 : div                                                 *
 *              (5) 3'b101 : divu                                                *
 *              (6) 3'b110 : rem                                                 *
 *              (7) 3'b111 : remu                                                *
 *          # RVI, CSR instructions  (csr_op)                                    *
 *              (0) 3'b001 : csrrw                                               *
 *              (1) 3'b010 : csrrs                                               *
 *              (2) 3'b011 : csrrc                                               *
 *              (3) 3'b101 : csrrwi                                              *
 *              (4) 3'b110 : csrrsi                                              *
 *              (5) 3'b111 : csrrci                                              *
 * ----------------------------------------------------------------------------- *
 *      Bytes of load data or store data, determined by mem_input_sel            *
 *              (0) 2'b00 : byte                                                 *
 *              (1) 2'b01 : half word                                            *
 *              (2) 2'b10 : word                                                 *
 * *******************************************************************************/

logic [DATA_WIDTH-1 : 0] rv32_instr;
always_comb rv32_instr = instruction;
logic [ 6 : 0] opcode;
always_comb opcode = rv32_instr[6: 0];
logic [ 4 : 0] rv32_shamt;
always_comb rv32_shamt = rv32_instr[24: 20];
logic [ 2 : 0] rv32_funct3;
always_comb rv32_funct3 = rv32_instr[14: 12];
logic [ 6 : 0] rv32_funct7;
always_comb rv32_funct7 = rv32_instr[DATA_WIDTH-1 : 25]; 

logic [DATA_WIDTH-1 : 0] immI, immS, immB, immU, immJ;
always_comb immI = { {21{rv32_instr[31]}}, rv32_instr[30: 25],
                 rv32_instr[24: 21], rv32_instr[20] };
always_comb immS = { {21{rv32_instr[31]}}, rv32_instr[30: 25], rv32_instr[11: 7] };
always_comb immB = { {20{rv32_instr[31]}}, rv32_instr[7],
                 rv32_instr[30: 25], rv32_instr[11: 8], 1'b0 };
always_comb immU = { rv32_instr[31: 12], 12'b0 };
always_comb immJ = { {12{rv32_instr[31]}}, rv32_instr[19: 12],
                rv32_instr[20], rv32_instr[30: 25], rv32_instr[24: 21], 1'b0 };

// ================================================================================
//  We generate the signals and reused them as much as possible to save gate counts
//
// wire opcode_1_0_00 = (opcode[1:0] == 2'b00);  // rvc
// wire opcode_1_0_01 = (opcode[1:0] == 2'b01);  // rvc
// wire opcode_1_0_10 = (opcode[1:0] == 2'b10);  // rvc
// wire opcode_1_0_11 = (opcode[1: 0] == 2'b11); // rv32

wire opcode_4_2_000 = (opcode[4: 2] == 3'b000);
wire opcode_4_2_001 = (opcode[4: 2] == 3'b001);
wire opcode_4_2_010 = (opcode[4: 2] == 3'b010);
wire opcode_4_2_011 = (opcode[4: 2] == 3'b011);
wire opcode_4_2_100 = (opcode[4: 2] == 3'b100);
wire opcode_4_2_101 = (opcode[4: 2] == 3'b101);
wire opcode_4_2_110 = (opcode[4: 2] == 3'b110);
wire opcode_4_2_111 = (opcode[4: 2] == 3'b111);

wire opcode_6_5_00 = (opcode[6: 5] == 2'b00);
wire opcode_6_5_01 = (opcode[6: 5] == 2'b01);
wire opcode_6_5_10 = (opcode[6: 5] == 2'b10);
wire opcode_6_5_11 = (opcode[6: 5] == 2'b11);

wire rv32_funct3_000 = (rv32_funct3 == 3'b000);
wire rv32_funct3_001 = (rv32_funct3 == 3'b001);
wire rv32_funct3_010 = (rv32_funct3 == 3'b010);
wire rv32_funct3_011 = (rv32_funct3 == 3'b011);
wire rv32_funct3_100 = (rv32_funct3 == 3'b100);
wire rv32_funct3_101 = (rv32_funct3 == 3'b101);
wire rv32_funct3_110 = (rv32_funct3 == 3'b110);
wire rv32_funct3_111 = (rv32_funct3 == 3'b111);

wire rv32_funct7_0000000 = (rv32_funct7 == 7'b0000000);
wire rv32_funct7_0100000 = (rv32_funct7 == 7'b0100000);
wire rv32_funct7_0000001 = (rv32_funct7 == 7'b0000001);
wire rv32_funct7_0000101 = (rv32_funct7 == 7'b0000101);
wire rv32_funct7_0001001 = (rv32_funct7 == 7'b0001001);
wire rv32_funct7_0001101 = (rv32_funct7 == 7'b0001101);
wire rv32_funct7_0010101 = (rv32_funct7 == 7'b0010101);
wire rv32_funct7_0100001 = (rv32_funct7 == 7'b0100001);
wire rv32_funct7_0010001 = (rv32_funct7 == 7'b0010001);
wire rv32_funct7_0101101 = (rv32_funct7 == 7'b0101101);
wire rv32_funct7_1111111 = (rv32_funct7 == 7'b1111111);
wire rv32_funct7_0000100 = (rv32_funct7 == 7'b0000100);
wire rv32_funct7_0001000 = (rv32_funct7 == 7'b0001000);
wire rv32_funct7_0001100 = (rv32_funct7 == 7'b0001100);
wire rv32_funct7_0101100 = (rv32_funct7 == 7'b0101100);
wire rv32_funct7_0010000 = (rv32_funct7 == 7'b0010000);
wire rv32_funct7_0010100 = (rv32_funct7 == 7'b0010100);
wire rv32_funct7_1100000 = (rv32_funct7 == 7'b1100000);
wire rv32_funct7_1110000 = (rv32_funct7 == 7'b1110000);
wire rv32_funct7_1010000 = (rv32_funct7 == 7'b1010000);
wire rv32_funct7_1101000 = (rv32_funct7 == 7'b1101000);
wire rv32_funct7_1111000 = (rv32_funct7 == 7'b1111000);
wire rv32_funct7_1010001 = (rv32_funct7 == 7'b1010001);
wire rv32_funct7_1110001 = (rv32_funct7 == 7'b1110001);
wire rv32_funct7_1100001 = (rv32_funct7 == 7'b1100001);
wire rv32_funct7_1101001 = (rv32_funct7 == 7'b1101001);

// ================================================================================
//  RV32I Opcode Classification
//
wire rv32_op = opcode_6_5_01 & opcode_4_2_100;      // OP opcode
wire rv32_op_imm = opcode_6_5_00 & opcode_4_2_100;  // OP-IMM opcode
wire rv32_jal = opcode_6_5_11 & opcode_4_2_011;     // JAL opcode
wire rv32_jalr = opcode_6_5_11 & opcode_4_2_001;    // JARL opcode
wire rv32_load = opcode_6_5_00 & opcode_4_2_000;    // LOAD opcode
wire rv32_store = opcode_6_5_01 & opcode_4_2_000;   // STORE opcode
wire rv32_branch = opcode_6_5_11 & opcode_4_2_000;  // BRANCH opcode
wire rv32_lui = opcode_6_5_01 & opcode_4_2_101;     // LUI opcode
wire rv32_auipc = opcode_6_5_00 & opcode_4_2_101;   // AUIPC opcode
wire rv32_miscmem = opcode_6_5_00 & opcode_4_2_011; // MISC-MEM opcode
wire rv32_system = opcode_6_5_11 & opcode_4_2_100;  // SYSTEM opcode
wire rv32_amo = opcode_6_5_01 & opcode_4_2_011;     // AMO opcode

wire rv32m = rv32_op & rv32_funct7_0000001;  // Mul, Div and Rem instructions

wire rv32_imm_seli = rv32_op_imm | rv32_jalr | rv32_load;
wire rv32_imm_sels = rv32_store;
wire rv32_imm_selb = rv32_branch;
wire rv32_imm_selu = rv32_lui | rv32_auipc;
wire rv32_imm_selj = rv32_jal;

wire rv32_sub = rv32_op & rv32_funct3_000 & rv32_funct7_0100000;

// ================================================================================
//  Conditional Branch Instructions
//
wire rv32_beq = rv32_branch & rv32_funct3_000;
wire rv32_bne = rv32_branch & rv32_funct3_001;
wire rv32_blt = rv32_branch & rv32_funct3_100;
wire rv32_bgt = rv32_branch & rv32_funct3_101;
wire rv32_bltu = rv32_branch & rv32_funct3_110;
wire rv32_bgtu = rv32_branch & rv32_funct3_111;

// ================================================================================
//  MISC-MEM
wire rv32_fence = rv32_miscmem & rv32_funct3_000;
wire rv32_fence_i = rv32_miscmem & rv32_funct3_001;

// ================================================================================
//  System Instructions
//
wire rv32_csrrw = rv32_system & rv32_funct3_001;
wire rv32_csrrs = rv32_system & rv32_funct3_010;
wire rv32_csrrc = rv32_system & rv32_funct3_011;
wire rv32_csrrwi = rv32_system & rv32_funct3_101;
wire rv32_csrrsi = rv32_system & rv32_funct3_110;
wire rv32_csrrci = rv32_system & rv32_funct3_111;
wire rv32_csr = rv32_system & (~rv32_funct3_000);

wire rv32_sys_op = rv32_system & rv32_funct3_000;
wire rv32_ecall = rv32_sys_op & (rv32_instr[31: 20] == 12'b0000_0000_0000);
wire rv32_ebreak = rv32_sys_op & (rv32_instr[31: 20] == 12'b0000_0000_0001);
wire rv32_mret = rv32_sys_op & (rv32_instr[31: 20] == 12'b0011_0000_0010);

// ================================================================================
// Load/Store Instructions
//
wire rv32_lb = rv32_load & rv32_funct3_000;
wire rv32_lh = rv32_load & rv32_funct3_001;
wire rv32_lw = rv32_load & rv32_funct3_010;
wire rv32_lbu = rv32_load & rv32_funct3_100;
wire rv32_lhu = rv32_load & rv32_funct3_101;

wire rv32_sb = rv32_store & rv32_funct3_000;
wire rv32_sh = rv32_store & rv32_funct3_001;
wire rv32_sw = rv32_store & rv32_funct3_010;

// ================================================================================
//  Output Signals
//
always_comb 
    decoded_instr.immediate =
       ({32{rv32_imm_seli}} & immI)
       | ({32{rv32_imm_sels}} & immS)
       | ({32{rv32_imm_selb}} & immB)
       | ({32{rv32_imm_selu}} & immU)
       | ({32{rv32_imm_selj}} & immJ)
       ;

// All the RV32IMA need rd except the
//   # BRANCH, STORE,
//   # FENCE, FENCE.I
//   # ECALL, EBREAK

always_comb 
    decoded_instr.mem_re = rv32_load | rv32_amo; // AMO instr. also need to load d-cache data
always_comb 
    decoded_instr.mem_we = rv32_store;

always_comb 
    decoded_instr.is_jal = rv32_jal;
always_comb 
    decoded_instr.is_jalr = rv32_jalr;
always_comb 
    decoded_instr.is_branch = rv32_branch;

always_comb 
    decoded_instr.rd_addr = rv32_instr[11: 7];
always_comb
    decoded_instr.rs1_addr = rv32_instr[19: 15];
always_comb
    decoded_instr.rs2_addr = rv32_instr[24: 20];

always_comb
    decoded_instr.mem_input_sel = mem_op_size_t'(rv32_funct3[1: 0]);         // {00: b}, {01: h}, {10: w}

always_comb 
    decoded_instr.mem_load_ext_sel = rv32_funct3[2];         // {0: signed extension},
                                                             // {1: unsigned extension}
always_comb 
    decoded_instr.alu_muldiv_sel = rv32m;                    // {0: rv32i op}, {1: rv32m op}
always_comb
    decoded_instr.ctrl_signal = (rv32_lui | rv32_auipc | rv32_jal | rv32_jalr) ?  // LUI and AUIPC use alu
                  3'b000 : rv32_funct3;                      //      adder result
                  // Due to CoreFalco share ctrl_signal between ALU and BCU
                  // branch change ALU input in EXE stage by checking is_branch flag
always_comb 
    decoded_instr.shift_sel = rv32_funct7_0100000;           // {0: logic}, {1: arithmetic}

always_comb 
    decoded_instr.is_csr_instr = rv32_csr;
always_comb 
    decoded_instr.csr_addr = rv32_instr[31: 20];
always_comb 
    decoded_instr.csr_imm = rv32_instr[19: 15];
always_comb 
    decoded_instr.csr_data = 0;

always_comb
begin
    if (rv32_auipc | rv32_jal | rv32_branch)
        decoded_instr.operand0_sel = OPERAND0_SEL_PC; // pc
    else if (rv32_lui)
        decoded_instr.operand0_sel = OPERAND0_SEL_LUI; // 0
    else //store must use rs1 field
        decoded_instr.operand0_sel = OPERAND0_SEL_RS1; // rs1
end

always_comb
begin
    if (rv32_sub)
        decoded_instr.operand1_sel = OPERAND1_SEL_NEG_RS2; // -rs2
    else if (rv32_store | rv32_op | rv32_amo)
        decoded_instr.operand1_sel = OPERAND1_SEL_RS2; // rs2
    else
        decoded_instr.operand1_sel = OPERAND1_SEL_IMM; // immediate
end



always_comb decoded_instr.sys_jump = rv32_mret | rv32_ecall; // the instructions that change the pc
always_comb decoded_instr.sys_jump_csr_addr = ( {12{rv32_mret}} & 12'h341)
                         | ( {12{rv32_ecall}} & 12'h305 );

always_comb decoded_instr.predict_pc = predict_pc;
always_comb decoded_instr.pc = pc;

always_comb illegal_instr_o =     // the instructions that are not supported currently
       rv32_fence
       | rv32_fence_i
       | rv32_ebreak;
always_comb decoded_instr.is_use_rd_field = 
    ~( rv32_store || rv32_branch || rv32_instr[11: 7] == 0);
//J-TYPE,B-TYPE and those write to x0 instr

endmodule   // decode
