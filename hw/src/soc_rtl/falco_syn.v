`timescale 1ns/1ps
// =============================================================================
//  Program : falco_syn.v
//  Author  : Hon-Chou Dai
//  Date    : May/15/2021
// -----------------------------------------------------------------------------
//  Description:
//  This is the top-level Falco simulation module.
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

module falco_syn
(
    input sysclk_p,
    input sysclk_n,
    input reset,

    input rx,
    output tx
);

wire dev_axi_aclk;
wire dev_axi_aresetn;

wire [31 : 0] dev_axi_awaddr;
wire dev_axi_awvalid;
wire dev_axi_awready;
wire [31 : 0] dev_axi_wdata;
wire [3 : 0] dev_axi_wstrb;
wire dev_axi_wvalid;
wire dev_axi_wready;
wire [1 : 0] dev_axi_bresp;
wire dev_axi_bvalid;
wire dev_axi_bready;
wire [31 : 0] dev_axi_araddr;
wire dev_axi_arvalid;
wire dev_axi_arready;
wire [31 : 0] dev_axi_rdata;
wire [1 : 0] dev_axi_rresp;
wire dev_axi_rvalid;
wire dev_axi_rready;

// unused axi port
wire [2:0] dev_axi_awprot;
wire [2:0] dev_axi_arprot;

reg [32:0] rst_counter;
wire RISCV_rst;
always @(posedge clk_out1) begin
    if (reset)
        rst_counter <= 33'd1_000_000;
    else
        rst_counter <= rst_counter - (| rst_counter);
end
assign RISCV_rst = (| rst_counter);
clk_wiz_0 clk_wiz
(
    // Clock out ports
    .clk_out1(clk_out1),     // output clk_out1
    // Status and control signals
    .reset(reset), // input reset
    .locked(locked),       // output locked
   // Clock in ports
    .clk_in1_p(sysclk_p),    // input clk_in1_p
    .clk_in1_n(sysclk_n)
);    // input clk_in1_

/*
proc_sys_reset_0 your_instance_name (
    .slowest_sync_clk(slowest_sync_clk),          // input wire slowest_sync_clk
    .ext_reset_in(ext_reset_in),                  // input wire ext_reset_in
    .aux_reset_in(aux_reset_in),                  // input wire aux_reset_in
    .mb_debug_sys_rst(mb_debug_sys_rst),          // input wire mb_debug_sys_rst
    .dcm_locked(dcm_locked),                      // input wire dcm_locked
    .mb_reset(mb_reset),                          // output wire mb_reset
    .bus_struct_reset(bus_struct_reset),          // output wire [0 : 0] bus_struct_reset
    .peripheral_reset(peripheral_reset),          // output wire [0 : 0] peripheral_reset
    .interconnect_aresetn(interconnect_aresetn),  // output wire [0 : 0] interconnect_aresetn
    .peripheral_aresetn(peripheral_aresetn)      // output wire [0 : 0] peripheral_aresetn
);
*/
assign dev_axi_aclk = clk_out1;
assign device_aresetn = ~reset;

axi_uartlite_0 uart0  (
    .s_axi_aclk(dev_axi_aclk),        // input wire s_axi_aclk
    .s_axi_aresetn(device_aresetn),  // input wire s_axi_aresetn
    //.interrupt(),          // output wire interrupt
    .s_axi_awaddr(dev_axi_awaddr),    // input wire [3 : 0] s_axi_awaddr
    .s_axi_awvalid(dev_axi_awvalid),  // input wire s_axi_awvalid
    .s_axi_awready(dev_axi_awready),  // output wire s_axi_awready
    .s_axi_wdata(dev_axi_wdata),      // input wire [31 : 0] s_axi_wdata
    .s_axi_wstrb(dev_axi_wstrb),      // input wire [3 : 0] s_axi_wstrb
    .s_axi_wvalid(dev_axi_wvalid),    // input wire s_axi_wvalid
    .s_axi_wready(dev_axi_wready),    // output wire s_axi_wready
    .s_axi_bresp(dev_axi_bresp),      // output wire [1 : 0] s_axi_bresp
    .s_axi_bvalid(dev_axi_bvalid),    // output wire s_axi_bvalid
    .s_axi_bready(dev_axi_bready),    // input wire s_axi_bready
    .s_axi_araddr(dev_axi_araddr),    // input wire [3 : 0] s_axi_araddr
    .s_axi_arvalid(dev_axi_arvalid),  // input wire s_axi_arvalid
    .s_axi_arready(dev_axi_arready),  // output wire s_axi_arready
    .s_axi_rdata(dev_axi_rdata),      // output wire [31 : 0] s_axi_rdata
    .s_axi_rresp(dev_axi_rresp),      // output wire [1 : 0] s_axi_rresp
    .s_axi_rvalid(dev_axi_rvalid),    // output wire s_axi_rvalid
    .s_axi_rready(dev_axi_rready),    // input wire s_axi_rready
    .rx(rx),                        // input wire rx
    .tx(tx)                         // output wire tx
);

falco_wrapper #
(
    .C_M_DEVICE_PORT_ADDR_WIDTH(32),
    .C_M_DEVICE_PORT_DATA_WIDTH(32)
)
falco0
(
    .clk (clk_out1),
    .rst (RISCV_rst),
    .init_pc(0),

    // Ports of Axi Master Bus Interface M_DEVICE_PORT
    .device_aclk             (  device_aclk       ),
    .device_aresetn          (  device_aresetn    ),
    .m_device_port_awaddr    (  dev_axi_awaddr    ),
    .m_device_port_awprot    (  dev_axi_awprot    ),
    .m_device_port_awvalid   (  dev_axi_awvalid   ),
    .m_device_port_awready   (  dev_axi_awready   ),
    .m_device_port_wdata     (  dev_axi_wdata     ),
    .m_device_port_wstrb     (  dev_axi_wstrb     ),
    .m_device_port_wvalid    (  dev_axi_wvalid    ),
    .m_device_port_wready    (  dev_axi_wready    ),
    .m_device_port_bresp     (  dev_axi_bresp     ),
    .m_device_port_bvalid    (  dev_axi_bvalid    ),
    .m_device_port_bready    (  dev_axi_bready    ),
    .m_device_port_araddr    (  dev_axi_araddr    ),
    .m_device_port_arprot    (  dev_axi_arprot    ),
    .m_device_port_arvalid   (  dev_axi_arvalid   ),
    .m_device_port_arready   (  dev_axi_arready   ),
    .m_device_port_rdata     (  dev_axi_rdata     ),
    .m_device_port_rresp     (  dev_axi_rresp     ),
    .m_device_port_rvalid    (  dev_axi_rvalid    ),
    .m_device_port_rready    (  dev_axi_rready    )
);

endmodule
