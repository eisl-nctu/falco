`timescale 1ns/1ps
// =============================================================================
//  Program : falco_wrapper.v
//  Author  : Hon-Chou Dai
//  Date    : May/15/2021
// -----------------------------------------------------------------------------
//  Description:
//  This is the top-level Falco wrapper module.
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

module falco_wrapper #
(
    parameter C_M_DEVICE_PORT_ADDR_WIDTH = 32,
    parameter C_M_DEVICE_PORT_DATA_WIDTH = 32
)
(
    input clk,
    input rst,
    input [31:0] init_pc,

    // Ports of Axi Master Bus Interface M_DEVICE_PORT
    input  device_aclk,
    input  device_aresetn,
    output [C_M_DEVICE_PORT_ADDR_WIDTH-1 : 0] m_device_port_awaddr,
    output [2 : 0] m_device_port_awprot,
    output  m_device_port_awvalid,
    input   m_device_port_awready,
    output [C_M_DEVICE_PORT_DATA_WIDTH-1 : 0] m_device_port_wdata,
    output [C_M_DEVICE_PORT_DATA_WIDTH/8-1 : 0] m_device_port_wstrb,
    output  m_device_port_wvalid,
    input   m_device_port_wready,
    input  [1 : 0] m_device_port_bresp,
    input   m_device_port_bvalid,
    output  m_device_port_bready,
    output [C_M_DEVICE_PORT_ADDR_WIDTH-1 : 0] m_device_port_araddr,
    output [2 : 0] m_device_port_arprot,
    output  m_device_port_arvalid,
    input   m_device_port_arready,
    input  [C_M_DEVICE_PORT_DATA_WIDTH-1 : 0] m_device_port_rdata,
    input  [1 : 0] m_device_port_rresp,
    input   m_device_port_rvalid,
    output  m_device_port_rready
);

localparam integer XLEN_WIDTH = 32;


wire                       M_DEVICE_store_req;
wire [ XLEN_WIDTH - 1 : 0] M_DEVICE_store_addr;
wire [ XLEN_WIDTH/8-1 : 0] M_DEVICE_store_byte_enable;
wire [ XLEN_WIDTH - 1 : 0] M_DEVICE_store_data;
wire                       M_DEVICE_load_ck_hit_load_req;
wire                       M_DEVICE_load_ck_hit_load_kill;
wire [ XLEN_WIDTH - 1 : 0] M_DEVICE_load_addr;

wire                       M_DEVICE_load_hit;
wire                       M_DEVICE_load_miss;
wire                       M_DEVICE_load_data_ready;
wire                       M_DEVICE_store_finished;
wire [ XLEN_WIDTH - 1 : 0] M_DEVICE_load_data;

wire [1:0]                M_CORE_commit_count;

Falco_top Falco_core
(
    .clk(clk),
    .rst(rst),

    .init_pc(init_pc),

    .M_CORE_commit_count            (M_CORE_commit_count),
    .M_DEVICE_store_req             (M_DEVICE_store_req),
    .M_DEVICE_store_addr            (M_DEVICE_store_addr),
    .M_DEVICE_store_byte_enable     (M_DEVICE_store_byte_enable),
    .M_DEVICE_store_data            (M_DEVICE_store_data),
    .M_DEVICE_load_ck_hit_load_req  (M_DEVICE_load_ck_hit_load_req),
    .M_DEVICE_load_ck_hit_load_kill (M_DEVICE_load_ck_hit_load_kill),
    .M_DEVICE_load_addr             (M_DEVICE_load_addr),

    .M_DEVICE_load_hit              (M_DEVICE_load_hit),
    .M_DEVICE_load_miss             (M_DEVICE_load_miss),
    .M_DEVICE_load_data_ready       (M_DEVICE_load_data_ready),
    .M_DEVICE_store_finished        (M_DEVICE_store_finished),
    .M_DEVICE_load_data             (M_DEVICE_load_data)
);


CoreFalco_M_DEVICE_PORT #
(
    .C_M_AXI_ADDR_WIDTH(32), // Width of the AXI addr bus.
    .C_M_AXI_DATA_WIDTH(32)  // Width of the AXI data bus.
) CoreFalco_M_DEVICE_PORT_inst (
    .M_AXI_ACLK                     (clk),       // AXI bus clock signal.
    .M_AXI_ARESETN                  (~rst),    // AXI bus reset singal (Active Low).

    .M_CORE_commit_count            (M_CORE_commit_count),
    .M_DEVICE_store_req             (M_DEVICE_store_req),
    .M_DEVICE_store_addr            (M_DEVICE_store_addr),
    .M_DEVICE_store_byte_enable     (M_DEVICE_store_byte_enable),
    .M_DEVICE_store_data            (M_DEVICE_store_data),
    .M_DEVICE_load_ck_hit_load_req  (M_DEVICE_load_ck_hit_load_req),
    .M_DEVICE_load_ck_hit_load_kill (M_DEVICE_load_ck_hit_load_kill),
    .M_DEVICE_load_addr             (M_DEVICE_load_addr),

    .M_DEVICE_load_hit              (M_DEVICE_load_hit),
    .M_DEVICE_load_miss             (M_DEVICE_load_miss),
    .M_DEVICE_load_data_ready       (M_DEVICE_load_data_ready),
    .M_DEVICE_store_finished        (M_DEVICE_store_finished),
    .M_DEVICE_load_data             (M_DEVICE_load_data),


    .M_AXI_AWADDR(m_device_port_awaddr),
    .M_AXI_AWPROT(m_device_port_awprot),
    .M_AXI_AWVALID(m_device_port_awvalid),
    .M_AXI_AWREADY(m_device_port_awready),
    .M_AXI_WDATA(m_device_port_wdata),
    .M_AXI_WSTRB(m_device_port_wstrb),
    .M_AXI_WVALID(m_device_port_wvalid),
    .M_AXI_WREADY(m_device_port_wready),
    .M_AXI_BRESP(m_device_port_bresp),
    .M_AXI_BVALID(m_device_port_bvalid),
    .M_AXI_BREADY(m_device_port_bready),
    .M_AXI_ARADDR(m_device_port_araddr),
    .M_AXI_ARPROT(m_device_port_arprot),
    .M_AXI_ARVALID(m_device_port_arvalid),
    .M_AXI_ARREADY(m_device_port_arready),
    .M_AXI_RDATA(m_device_port_rdata),
    .M_AXI_RRESP(m_device_port_rresp),
    .M_AXI_RVALID(m_device_port_rvalid),
    .M_AXI_RREADY(m_device_port_rready)
);


endmodule


