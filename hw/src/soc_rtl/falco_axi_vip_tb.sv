`timescale 1ns/1ps
// =============================================================================
//  Program : falco_axi_vip_tb.sv
//  Author  : Hon-Chou Dai
//  Date    : May/15/2021
// -----------------------------------------------------------------------------
//  Description:
//  This is the top-level Falco wrapper for an AXI-based SoC simulation.
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

import axi_vip_pkg::*;
import axi_vip_0_pkg::*;

module falco_axi_vip_tb();

xil_axi_uint                            error_cnt = 0;
xil_axi_uint                            comparison_cnt = 0;
axi_transaction                         wr_transaction;   
axi_transaction                         rd_transaction;   
axi_monitor_transaction                 mst_monitor_transaction;  
axi_monitor_transaction                 master_moniter_transaction_queue[$];  
xil_axi_uint                            master_moniter_transaction_queue_size =0;  
axi_monitor_transaction                 mst_scb_transaction;  
axi_monitor_transaction                 passthrough_monitor_transaction;  
axi_monitor_transaction                 passthrough_master_moniter_transaction_queue[$];  
xil_axi_uint                            passthrough_master_moniter_transaction_queue_size =0;  
axi_monitor_transaction                 passthrough_mst_scb_transaction;  
axi_monitor_transaction                 passthrough_slave_moniter_transaction_queue[$];  
xil_axi_uint                            passthrough_slave_moniter_transaction_queue_size =0;  
axi_monitor_transaction                 passthrough_slv_scb_transaction;  
axi_monitor_transaction                 slv_monitor_transaction;  
axi_monitor_transaction                 slave_moniter_transaction_queue[$];  
xil_axi_uint                            slave_moniter_transaction_queue_size =0;  
axi_monitor_transaction                 slv_scb_transaction;  
xil_axi_uint                           mst_agent_verbosity = 0;  
xil_axi_uint                           slv_agent_verbosity = 0;  
xil_axi_uint                           passthrough_agent_verbosity = 0;  
bit                                     clock;
bit                                     reset;
xil_axi_ulong                           mem_rd_addr;
xil_axi_ulong                           mem_wr_addr;
bit[32-1:0]                             write_data;
bit                                     write_strb[];
bit[32-1:0]                             read_data;
axi_vip_0_slv_t          slv_agent_0;
bit error_0;
bit done_0;
bit init_0;
bit [8*4096-1:0]                        wr_data;

xil_axi_payload_byte                    data_mem[xil_axi_ulong];


reg clk;
reg reset;

wire [31 : 0] s_axi_awaddr;
wire [2 : 0] s_axi_awprot;
wire s_axi_awvalid;
wire s_axi_awready;
wire [31 : 0] s_axi_wdata;
wire [3 : 0] s_axi_wstrb;
wire s_axi_wvalid;
wire s_axi_wready;
wire [1 : 0] s_axi_bresp;
wire s_axi_bvalid;
wire s_axi_bready;
wire [31 : 0] s_axi_araddr;
wire [2 : 0] s_axi_arprot;
wire s_axi_arvalid;
wire s_axi_arready;
wire [31 : 0] s_axi_rdata;
wire [1 : 0] s_axi_rresp;
wire s_axi_rvalid;
wire s_axi_rready;

initial begin
    clk = 0;
end

initial begin
    reset <= 1'b1;
    #200ns;
    reset <= 1'b0;
    repeat (5) @(negedge clk); 
end
always #5 clk = ~clk;

falco_wrapper #
(
    .C_M_DEVICE_PORT_ADDR_WIDTH(32),
    .C_M_DEVICE_PORT_DATA_WIDTH(32)
) DUT (
    .clk (clk),
    .rst (reset),
    .init_pc(0),

    // Ports of Axi Master Bus Interface M_DEVICE_PORT
    .device_aclk             (  clk       ),
    .device_aresetn          (  ~reset    ),
    .m_device_port_awaddr    (  s_axi_awaddr    ),
    .m_device_port_awprot    (  s_axi_awprot    ),
    .m_device_port_awvalid   (  s_axi_awvalid   ),
    .m_device_port_awready   (  s_axi_awready   ),
    .m_device_port_wdata     (  s_axi_wdata     ),
    .m_device_port_wstrb     (  s_axi_wstrb     ),
    .m_device_port_wvalid    (  s_axi_wvalid    ),
    .m_device_port_wready    (  s_axi_wready    ),
    .m_device_port_bresp     (  s_axi_bresp     ),
    .m_device_port_bvalid    (  s_axi_bvalid    ),
    .m_device_port_bready    (  s_axi_bready    ),
    .m_device_port_araddr    (  s_axi_araddr    ),
    .m_device_port_arprot    (  s_axi_arprot    ),
    .m_device_port_arvalid   (  s_axi_arvalid   ),
    .m_device_port_arready   (  s_axi_arready   ),
    .m_device_port_rdata     (  s_axi_rdata     ),
    .m_device_port_rresp     (  s_axi_rresp     ),
    .m_device_port_rvalid    (  s_axi_rvalid    ),
    .m_device_port_rready    (  s_axi_rready    )
);


axi_vip_0 slave_0 (
  .aclk(clk),                    // input wire aclk
  .aresetn(~reset),              // input wire aresetn
  .s_axi_awaddr(s_axi_awaddr),    // input wire [31 : 0] s_axi_awaddr
  .s_axi_awprot(s_axi_awprot),    // input wire [2 : 0] s_axi_awprot
  .s_axi_awvalid(s_axi_awvalid),  // input wire s_axi_awvalid
  .s_axi_awready(s_axi_awready),  // output wire s_axi_awready
  .s_axi_wdata(s_axi_wdata),      // input wire [31 : 0] s_axi_wdata
  .s_axi_wstrb(s_axi_wstrb),      // input wire [3 : 0] s_axi_wstrb
  .s_axi_wvalid(s_axi_wvalid),    // input wire s_axi_wvalid
  .s_axi_wready(s_axi_wready),    // output wire s_axi_wready
  .s_axi_bresp(s_axi_bresp),      // output wire [1 : 0] s_axi_bresp
  .s_axi_bvalid(s_axi_bvalid),    // output wire s_axi_bvalid
  .s_axi_bready(s_axi_bready),    // input wire s_axi_bready
  .s_axi_araddr(s_axi_araddr),    // input wire [31 : 0] s_axi_araddr
  .s_axi_arprot(s_axi_arprot),    // input wire [2 : 0] s_axi_arprot
  .s_axi_arvalid(s_axi_arvalid),  // input wire s_axi_arvalid
  .s_axi_arready(s_axi_arready),  // output wire s_axi_arready
  .s_axi_rdata(s_axi_rdata),      // output wire [31 : 0] s_axi_rdata
  .s_axi_rresp(s_axi_rresp),      // output wire [1 : 0] s_axi_rresp
  .s_axi_rvalid(s_axi_rvalid),    // output wire s_axi_rvalid
  .s_axi_rready(s_axi_rready)    // input wire s_axi_rready
);

initial begin
    slv_agent_0 = new("slave vip agent",slave_0.inst.IF);
    slv_agent_0.vif_proxy.set_dummy_drive_type(XIL_AXI_VIF_DRIVE_NONE);
    slv_agent_0.set_agent_tag("Slave VIP");
    slv_agent_0.set_verbosity(slv_agent_verbosity);
    slv_agent_0.start_slave();
    $timeformat (-12, 1, " ps", 1);
    fork
        user_gen_awready();
        wr_response();
        rd_response();
    join_none
end
initial begin
    init_0 = 0;
    #200ns;
    init_0 =1'b1;
    #20ns;
    init_0 = 1'b0;
    $display("EXAMPLE TEST M00_AXI:");
    wait( done_0 == 1'b1);
    $display("M00_AXI: PTGEN_TEST_FINISHED!");
    if ( error_0 ) begin
        $display("PTGEN_TEST: FAILED!");
    end else begin
        $display("PTGEN_TEST: PASSED!");
    end
    #1ns;
    $finish;
end
initial begin
    #1;
    forever begin
        slv_agent_0.monitor.item_collected_port.get(slv_monitor_transaction);
        slave_moniter_transaction_queue.push_back(slv_monitor_transaction);
        slave_moniter_transaction_queue_size++;
    end
end
/*************************************************************************************************
  * Task user_gen_awready shows how slave VIP agent generates one customerized awready signal. 
  * Declare axi_ready_gen  awready_gen
  * Call create_ready from agent's write driver to create a new class of axi_ready_gen 
  * Set the poicy of ready generation in this example, it select XIL_AXI_READY_GEN_OSC 
  * Set low time 
  * Set high time
  * Agent's write driver send_awready out
  * Ready generation policy are listed below:
  *  XIL_AXI_READY_GEN_NO_BACKPRESSURE     - Ready stays asserted and will not change. The driver
                                             will still check for policy changes.
  *   XIL_AXI_READY_GEN_SINGLE             - Ready stays 0 for low_time clock cycles and then
                                             dirves 1 until one ready/valid handshake occurs,
                                             the policy repeats until the channel is given
                                             different policy.
  *   XIL_AXI_READY_GEN_EVENTS             - Ready stays 0 for low_time clock cycles and then
                                             dirves 1 until event_count ready/valid handshakes
                                             occur,the policy repeats until the channel is given
                                             different policy.
  *   XIL_AXI_READY_GEN_OSC                - Ready stays 0 for low_time and then goes to 1 and      
                                             stays 1 for high_time,the policy repeats until the
                                             channel is given different policy.
  *   XIL_AXI_READY_GEN_RANDOM             - This policy generate random ready policy and uses
                                             min/max pair of low_time, high_time and event_count to
                                             generate low_time, high_time and event_count.
  *   XIL_AXI_READY_GEN_AFTER_VALID_SINGLE - This policy is active when VALID is detected to be
                                             asserted, Ready stays 0 for low_time clock cycles and
                                             then dirves 1 until one ready/valid handshake occurs,
                                             the policy repeats until the channel is given
                                             different policy.
  *   XIL_AXI_READY_GEN_AFTER_VALID_EVENTS - This policy is active when VALID is detected to be
                                             asserted, Ready stays 0 for low_time clock cycles and
                                             then dirves 1 until event_count ready/valid handshake
                                             occurs,the policy repeats until the channel is given
                                             different policy.
  *   XIL_AXI_READY_GEN_AFTER_VALID_OSC    - This policy is active when VALID is detected to be
                                             asserted, Ready stays 0 for low_time and then goes to
                                             1 and  stays 1 for high_time,the policy repeats until
                                             the channel is given different policy.
 *************************************************************************************************/
  task user_gen_awready();
    axi_ready_gen                           awready_gen;
    awready_gen = slv_agent_0.wr_driver.create_ready("awready");
    awready_gen.set_ready_policy(XIL_AXI_READY_GEN_OSC);
    awready_gen.set_low_time(1);
    awready_gen.set_high_time(2);
    slv_agent_0.wr_driver.send_awready(awready_gen);
  endtask

 
  /*************************************************************************************************
  * wr_response: Task which write driver in slave agent waits till it sees a write transaction
  * and then user enviroment fill in write response, write driver send it over to VIP interface
  * When slave VIP is configured in READ_WRITE/WRITE_ONLY mode,user environment must call this task
  * Otherwise, the simulation will hang there waiting for BRESP from slave till time out.
  *************************************************************************************************/
  task wr_response();
    axi_transaction                         wr_reactive;
    forever begin  :slv_run
      slv_agent_0.wr_driver.get_wr_reactive(wr_reactive);
      wr_data = wr_reactive.get_data_block();
      $display("%t wr_data[0] = %c" ,$time, wr_data[8-1:0]);
      fill_wr_reactive(wr_reactive);
      slv_agent_0.wr_driver.send(wr_reactive);
    end
  endtask

  /*************************************************************************************************
  * rd_response: Task which read driver in slave agent waits till it sees a read transaction
  * and then user enviroment fill in read data channel,read driver send it over to VIP interface
  * When slave VIP is configured in READ_WRITE/READ_ONLY mode,user environment must call this task
  * Otherwise, the simulation will hang there waiting for data channel from slave till time out.
  *************************************************************************************************/
  task rd_response();
    axi_transaction                         rd_reactive;
    forever begin
      slv_agent_0.rd_driver.get_rd_reactive(rd_reactive);
      fill_rd_reactive(rd_reactive);
      slv_agent_0.rd_driver.send(rd_reactive);
    end  
  endtask

  /*************************************************************************************************
  * Fill_rd_reactive: Task fills in data and user(when RUSR_WIDH>0) info for read data channel
  * Fill data into transaction according to related address of the transaction and 
  * existence in memory
  * Fill in user into transaction when RUSER width is bigger than 0
  * When fill in data information, user has to take into account address, burst type, length 
  * and size
  *************************************************************************************************/
function automatic void fill_rd_reactive(inout axi_transaction t);
    longint unsigned                 current_addr;
    longint unsigned                 addr_max;  
    xil_axi_payload_byte             beat[];
    xil_axi_uint                     start_address;
    xil_axi_uint                     number_bytes;
    xil_axi_uint                     aligned_address;
    xil_axi_uint                     burst_length;
    xil_axi_uint                     address_n;
    xil_axi_uint                     wrap_boundary;
    xil_axi_user_beat                ruser;

    current_addr  = t.get_addr();
    start_address  = t.get_addr();
    number_bytes = xil_pow2(t.get_size());
    burst_length = t.get_len() + 1;
    aligned_address = (start_address/number_bytes) * number_bytes;
    wrap_boundary = (start_address/(number_bytes * burst_length)) * (number_bytes * burst_length);
    beat = new[(1<<t.get_size())];
      for (int beat_cnt = 0; beat_cnt <= t.get_len();beat_cnt++) begin
        for (int byte_cnt = 0; byte_cnt < (1<<t.get_size());byte_cnt++) begin
          if (!data_mem.exists(current_addr)) begin
            data_mem[current_addr] = {$random};
          end  
          beat[byte_cnt] = data_mem[current_addr];
          current_addr += 1;
        end
        if(t.get_burst() == XIL_AXI_BURST_TYPE_WRAP) begin
           if (current_addr >= wrap_boundary + (number_bytes*burst_length)) begin
            current_addr = wrap_boundary + (current_addr - wrap_boundary - (number_bytes*burst_length));
          end
        end else if(t.get_burst() == XIL_AXI_BURST_TYPE_FIXED) begin
          current_addr = start_address;
        end else begin
          current_addr = current_addr;
        end  
        t.set_data_beat_unpacked(t.get_beat_index(),beat);
        t.increment_beat_index();
      end
      t.clr_beat_index();

  endfunction: fill_rd_reactive


  /*************************************************************************************************
  * Fill_wr_reactive: Task fills in BREPS,BUSER(when BUSER_WIDTH>0) for write response channel
  * This task show simple example so buser is being set to 0 and bresp is XIL_AXI_RESP_OKAY
  * Fill in all these information into reactive response
  *************************************************************************************************/
  function automatic void fill_wr_reactive(inout axi_transaction t);
    xil_axi_resp_t       bresp;
    xil_axi_user_beat    buser;
    xil_axi_uint         bresp_delay;
    buser = $urandom_range(0,1<< 0 -1);
    if( 1== 0) begin
      bresp = XIL_AXI_RESP_OKAY; 
    end else begin
      if(t.get_all_resp_okay() == XIL_AXI_TRUE) begin
        bresp = XIL_AXI_RESP_OKAY;
      end else begin
        if(t.get_axi_version() != XIL_VERSION_LITE) begin
           if (t.get_exclude_resp_exokay() == XIL_AXI_TRUE) begin
                void'(randomize(bresp)with{bresp inside {XIL_AXI_RESP_OKAY, XIL_AXI_RESP_DECERR, XIL_AXI_RESP_SLVERR };});
           end else begin
                bresp = XIL_AXI_RESP_DECERR;
           end
          end else begin
             void'(randomize(bresp) with {bresp inside {XIL_AXI_RESP_OKAY, XIL_AXI_RESP_DECERR, XIL_AXI_RESP_SLVERR};});
         end
      end
    end
    t.set_buser(buser);
    t.set_bresp(bresp);
  endfunction: fill_wr_reactive

endmodule
