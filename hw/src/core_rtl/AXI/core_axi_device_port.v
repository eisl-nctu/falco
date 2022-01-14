`timescale 1 ns / 1 ps
// =============================================================================
//  Program : core_axi_device_port.v
//  Author  : Hon-Chou Dai
//  Date    : May/15/2021
// -----------------------------------------------------------------------------
//  Description:
//  This code implements the uncached memory access port of Aquila.  This port
//  is usually used for I/O device accesses.
// -----------------------------------------------------------------------------
//  Revision information:
//
//  NONE.
// -----------------------------------------------------------------------------
//  License information:
//
//  This software is released under the BSD-3-Clause Licence,
//  see https://opensource.org/licenses/BSD-3-Clause for details.
//  In the following license statements, "software" refers to the
//  "source code" of the complete hardware/software system.
//
//  Copyright 2019,
//                    Embedded Intelligent Systems Lab (EISL)
//                    Deparment of Computer Science
//                    National Chiao Tung Uniersity
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

module CoreFalco_M_DEVICE_PORT #
(
    parameter integer C_M_AXI_ADDR_WIDTH = 32, // Width of the AXI addr bus.
    parameter integer C_M_AXI_DATA_WIDTH = 32  // Width of the AXI data bus.
)
(
    input wire M_AXI_ACLK,       // AXI bus clock signal.
    input wire M_AXI_ARESETN,    // AXI bus reset singal (Active Low).

    input  wire                               stall_i,

    //debug bus
    input  wire [1:0]                         M_CORE_commit_count,

    input  wire                               M_DEVICE_store_req,
    input  wire [ C_M_AXI_DATA_WIDTH - 1 : 0] M_DEVICE_store_addr,
    input  wire [ C_M_AXI_DATA_WIDTH/8-1 : 0] M_DEVICE_store_byte_enable,
    input  wire [ C_M_AXI_DATA_WIDTH - 1 : 0] M_DEVICE_store_data,
    input  wire                               M_DEVICE_load_ck_hit_load_req,
    input  wire                               M_DEVICE_load_ck_hit_load_kill,
    input  wire [ C_M_AXI_DATA_WIDTH - 1 : 0] M_DEVICE_load_addr,

    output wire                               M_DEVICE_load_hit,
    output wire                               M_DEVICE_load_miss,
    output reg                                M_DEVICE_load_data_ready,
    output wire                               M_DEVICE_store_finished,
    output wire [ C_M_AXI_DATA_WIDTH - 1 : 0] M_DEVICE_load_data,

    // Write Address Channel Signals.
    output wire [C_M_AXI_ADDR_WIDTH - 1 : 0] M_AXI_AWADDR, // Write Address.
    output wire [2 : 0] M_AXI_AWPROT, // Protection type. The privilege level.
    output wire M_AXI_AWVALID,   // Write request, addr/ctrl signals are valid.
    input  wire M_AXI_AWREADY,   // Write ack, slave ready to fetch addr/ctrl.

    // Write Data Channel Signals.
    output wire [C_M_AXI_DATA_WIDTH - 1 : 0] M_AXI_WDATA,  // Write data.
    output wire [C_M_AXI_DATA_WIDTH / 8 - 1 : 0] M_AXI_WSTRB, // Byte-enable.
    output wire M_AXI_WVALID,   // Write data (and byte-enable strobe) is valid.
    input  wire M_AXI_WREADY,   // Write ready. Salve is ready to accept data.

    // Write Response Channel Signals.
    input  wire [1 : 0] M_AXI_BRESP, // Write response.
    input  wire M_AXI_BVALID,        // Slave has the write response valid.
    output wire M_AXI_BREADY,        // Master is ready to accept the response.

    // Read Address Channel Signals.
    output wire [C_M_AXI_ADDR_WIDTH - 1 : 0] M_AXI_ARADDR, // Read Address.
    output wire [2 : 0] M_AXI_ARPROT, // Protection type. The privilege level.
    output wire M_AXI_ARVALID, // Read request, addr/ctrl signals are valid.
    input  wire M_AXI_ARREADY, // Read ack, slave ready to fetch addr/ctrl.

    // Read Data Channel Signals.
    input  wire [C_M_AXI_DATA_WIDTH - 1 : 0] M_AXI_RDATA, // Read data.
    input  wire [1 : 0] M_AXI_RRESP,                      // Read response.
    input  wire M_AXI_RVALID,  // The read data from the slave is valid.
    output wire M_AXI_RREADY   // Master is ready to accept the read data.
);

//       [2] 0xC000_0000 - 0xCFFF_FFFF : device memory (uncached)
//       0xC000_0000 UART RXFIFO
//       0xC000_0004 UART TXFIFO
//       0xC000_0008 UART STAT_REG
//       0xC000_000C UART CTRL_REG

//       0xC100_0000 Cycle counter enable [0:0] enable [1:1] reset
//       0xC100_0004 Cycle lower part
//       0xC100_0008 Cycle higher part
//       0xC100_000C Cycle instret_counter[31:0]
//       0xC100_0010 Cycle instret_counter[63:0]


reg [63:0] cycle_counter; //read only
reg [63:0] instret_counter; //read only
reg [7:0]  cycle_counter_ctrl; //rw

localparam [2:0]
    S_IDLE = 0,
    S_LOAD_INIT = 1,
    S_LOAD = 2,
    S_LOAD_DONE = 3,
    S_STORE_INIT = 4,
    S_STORE = 5,
    S_STORE_DONE = 6;

reg load_kill;

// AXI4Lite internal temp signals.
reg axi_awvalid;
reg axi_wvalid;
reg axi_arvalid;
reg axi_rready;
reg axi_bready;
reg [C_M_AXI_ADDR_WIDTH - 1 : 0] axi_awaddr;
reg [C_M_AXI_ADDR_WIDTH - 1 : 0] axi_araddr;

// Interface response error flags.
wire write_resp_error;
wire read_resp_error;

// user-defined signals
reg [ C_M_AXI_DATA_WIDTH - 1 : 0] fetched_data; // Fetched data from the bus.
reg [ C_M_AXI_DATA_WIDTH - 1 : 0] fetched_data_dly; // Fetched data from the bus.
reg write_done;   // Flags the completion of the write transaction.
reg read_done;    // Flags the completion of the read transaction.

wire store_is_internal;
wire load_is_internal;
wire [7:0] store_internal_offset;
wire [7:0] load_internal_offset;

assign store_is_internal = (M_DEVICE_store_addr[27:24] == 4'h1);
assign load_is_internal  = (M_DEVICE_load_addr[27:24] == 4'h1);
assign store_internal_offset = M_DEVICE_store_addr[7:0];
assign load_internal_offset = M_DEVICE_load_addr[7:0];
// Control FSM
reg [2:0] mst_state, next_mst_state;
always @(posedge M_AXI_ACLK)
    if (M_AXI_ARESETN == 0)
        mst_state <= 0;
    else
        mst_state <= next_mst_state;

always @(*) begin
    case (mst_state)
        S_IDLE:
            if (M_DEVICE_store_req && 
                ~M_DEVICE_store_finished &&
                store_is_internal)
                next_mst_state = S_STORE;
            else if (M_DEVICE_store_req &&
                     ~M_DEVICE_store_finished ) 
                next_mst_state = S_STORE_INIT;
            else if (M_DEVICE_load_ck_hit_load_req && 
                     ~M_DEVICE_load_hit && 
                     ~M_DEVICE_load_ck_hit_load_kill &&
                     load_is_internal) 
                next_mst_state = S_LOAD;
            else if (M_DEVICE_load_ck_hit_load_req && 
                     ~M_DEVICE_load_hit && 
                     ~M_DEVICE_load_ck_hit_load_kill) 
                next_mst_state = S_LOAD_INIT;
            else 
                next_mst_state = S_IDLE;
        S_LOAD_INIT: next_mst_state = S_LOAD;
        S_LOAD:
            if (read_done) next_mst_state = S_LOAD_DONE;
            else next_mst_state = S_LOAD;
        S_LOAD_DONE:
            if (stall_i) next_mst_state = S_LOAD_DONE;
            else next_mst_state = S_IDLE;
        S_STORE_INIT: next_mst_state = S_STORE;
        S_STORE:
            if (write_done) next_mst_state = S_STORE_DONE;
            else next_mst_state = S_STORE;
        S_STORE_DONE:
            next_mst_state = S_IDLE;
        default: next_mst_state = S_IDLE;
    endcase
end

always @(posedge M_AXI_ACLK) begin
    if (M_AXI_ARESETN == 0)
        load_kill <= 0;
    else if (mst_state == S_IDLE || mst_state == S_LOAD_DONE)
        load_kill <= 0;
    else if (M_DEVICE_load_ck_hit_load_kill)
        load_kill <= 1;
    else
        load_kill <= load_kill;
end

// I/O Connections assignments

// Write Address (AW)
assign M_AXI_AWADDR	 = axi_awaddr;
assign M_AXI_AWPROT  = 3'b000;
assign M_AXI_AWVALID = axi_awvalid;

// Write Data(W)
assign M_AXI_WVALID = axi_wvalid;
assign M_AXI_WSTRB  = M_DEVICE_store_byte_enable;

// Write Response (B)
assign M_AXI_BREADY = axi_bready;

// Read Address (AR)
assign M_AXI_ARADDR = axi_araddr;
assign M_AXI_ARVALID = axi_arvalid;
assign M_AXI_ARPROT = 3'b000;

//Read and Read Response (R)
assign M_AXI_RREADY = axi_rready;

// Flag any write errors.
assign write_resp_error = axi_bready & M_AXI_BVALID & M_AXI_BRESP[1];

// Flag any read response errors.
assign read_resp_error = axi_rready & M_AXI_RVALID & M_AXI_RRESP[1];

// Single-cycle signal that flags the completion of a transaction.

// -----------------------
//  Write Address Channel
// -----------------------
// The write address channel requests the address and command information
// for the entire transaction.  It is a single beat of information.
// Typically, the axi_awvalid/axi_wvalid are asserted at the same time,
// and then each is deasserted independent from each other.
// AXI VALID signals must be held active until accepted by the partner.

// A data transfer is accepted by the slave when a master has VALID data
// and the slave acknowledges its READY. A master is allowed to
// generated multiple, back-to-back requests by not deasserting VALID.
always @(posedge M_AXI_ACLK) begin
    // Only VALID signals must be deasserted during reset per AXI spec.
    // Consider inverting then registering active-low reset for higher fmax.
    if (M_AXI_ARESETN == 0)
        axi_awvalid <= 0;
    else if (mst_state == S_STORE_INIT) // Signal a new write request.
        axi_awvalid <= 1;
    else if (M_AXI_AWREADY && axi_awvalid) // Address accepted by the slave
        axi_awvalid <= 0;                  //  (slave issued M_AXI_AWREADY).
    else
        axi_awvalid <= axi_awvalid;
end

always @(posedge M_AXI_ACLK) begin// Write Addresses
    if (M_AXI_ARESETN == 0)
        axi_awaddr <= 0;
    else if (mst_state == S_STORE_INIT)
        axi_awaddr <= M_DEVICE_store_addr;
end

// --------------------
//  Write Data Channel
// --------------------
// The write data channel is for transfering the actual data.

// Signaling valid write data on the bus.
always @(posedge M_AXI_ACLK) begin
    if (M_AXI_ARESETN == 0)
        axi_wvalid <= 0;
    else if (mst_state == S_STORE_INIT)
        axi_wvalid <= 1;                 // Signal that data is on WDATA.
    else if (M_AXI_WREADY && axi_wvalid) // Data accepted by the slave
        axi_wvalid <= 0;                 //  (slave issued M_AXI_WREADY).
    else
        axi_wvalid <= axi_wvalid;
end

// Write data generation.
assign M_AXI_WDATA = M_DEVICE_store_data;

// ----------------------------
//  Write Response (B) Channel
// ----------------------------
// The write response channel provides feedback that the write has committed
// to memory. BREADY will occur after both the data and the write address
// has arrived and been accepted by the slave, and can guarantee that no
// other accesses launched afterwards will be able to be reordered before it.

// The BRESP bit [1] is used indicate any errors from the interconnect or
// slave for the entire write burst. This example will capture the error.

// While not necessary per spec, it is advisable to reset READY signals in
// case of differing reset latencies between master/slave.

always @(posedge M_AXI_ACLK) begin
    if (M_AXI_ARESETN == 0)
        axi_bready <= 0;
    else if (M_AXI_BVALID && ~axi_bready) // Accept/ack. bresp with axi_bready
        axi_bready <= 1;                  // by the master when M_AXI_BVALID
    else if (axi_bready)                  // is asserted by slave.
        axi_bready <= 0;                  // Deassert after one clock cycle.
    else
        axi_bready <= axi_bready;         // Retain the previous value.
end

// ----------------------
//  Read Address Channel
// ----------------------
// A new axi_arvalid is asserted when there is a valid read address
// available by the master. read_trigger triggers a new read transaction.
always @(posedge M_AXI_ACLK) begin
    if (M_AXI_ARESETN == 0)
        axi_arvalid <= 0;
    else if (mst_state == S_LOAD_INIT) // Signal a new read request.
        axi_arvalid <= 1;
    else if (M_AXI_ARREADY && axi_arvalid)    // Address accepted by the slave
        axi_arvalid <= 0;                     //  (slave issued M_AXI_ARREADY).
end

always @(posedge M_AXI_ACLK) begin// Read Addresses
    if (M_AXI_ARESETN == 0)
        axi_araddr <= 0;
    else if (mst_state == S_LOAD_INIT)
        axi_araddr <= M_DEVICE_load_addr;
end

// ----------------------------------
//  Read Data (and Response) Channel
// ----------------------------------
// The Read Data channel returns the results of the read request. The master
// will accept the read data by asserting axi_rready when a valid read data
// is available. While not necessary per spec, it is advisable to reset
// READY signals in case of differing reset latencies between master/slave.

// Signaling valid read data on the bus.
always @(posedge M_AXI_ACLK) begin
    if (M_AXI_ARESETN == 0)
        axi_rready <= 0;
    else if (M_AXI_RVALID && ~axi_rready) // Accept/ack. rdata/rresp with
        axi_rready <= 1;                  // axi_rready by the master when
    else if (axi_rready)                  // M_AXI_RVALID is asserted by slave.
        axi_rready <= 0;                  // Deassert after one clock cycle.
end

// Read data generation.
assign M_DEVICE_load_data = fetched_data_dly;

always @(posedge M_AXI_ACLK) begin // Read Addresses
    if (M_AXI_ARESETN == 0) begin
        fetched_data <= 32'b0;
    end else if (mst_state == S_LOAD && load_is_internal) begin
        if (load_internal_offset == 8'h00)
            fetched_data <= cycle_counter_ctrl;
        else if (load_internal_offset == 8'h04)
            fetched_data <= cycle_counter[31:0];
        else if (load_internal_offset == 8'h08)
            fetched_data <= cycle_counter[63:32];
        else if (load_internal_offset == 8'h0C)
            fetched_data <= instret_counter[31:0];
        else if (load_internal_offset == 8'h10)
            fetched_data <= instret_counter[63:32];
        else
            fetched_data <= 32'hdeadbeef;
    end else if (M_AXI_RVALID) begin
        fetched_data <= M_AXI_RDATA;
    end else begin
        fetched_data <= fetched_data;
    end
end

always @(posedge M_AXI_ACLK)
    if (M_AXI_ARESETN == 0)
        fetched_data_dly <= 0;
    else
        fetched_data_dly <= fetched_data;

// ----------------------------------
//  User Logics
// ----------------------------------

// Check for write_done completion.
always @(posedge M_AXI_ACLK) begin
    if (M_AXI_BVALID && axi_bready) // The write_done should be associated
        write_done <= 1;            //     with a bready response.
    else if (mst_state == S_STORE && store_is_internal && ~write_done)
        write_done <= 1;
    else
        write_done <= 0;
end

// Check for read_done completion.
always @(posedge M_AXI_ACLK) begin
    if (M_AXI_ARESETN == 0)
        read_done <= 0;
    else if ((M_AXI_RVALID && axi_rready) || 
              (mst_state == S_LOAD && load_is_internal && ~read_done)) // The read_done should be associated
        read_done <= 1;                 //     with a rready response.
    else
        read_done <= 0;
end

assign M_DEVICE_store_finished = write_done;
assign M_DEVICE_load_hit = read_done;
assign M_DEVICE_load_miss = ~read_done;

always @(posedge M_AXI_ACLK) begin
    if (M_AXI_ARESETN == 0)
        M_DEVICE_load_data_ready <= 0;
    else if (load_kill || M_DEVICE_load_ck_hit_load_kill)
        M_DEVICE_load_data_ready <= 0;
    else
        M_DEVICE_load_data_ready <= read_done;
end 


// ========================================
// cycle counter
// ========================================

always @(posedge M_AXI_ACLK) begin
    if (M_AXI_ARESETN == 0)
        cycle_counter <= 0;
    else if (cycle_counter_ctrl[1])
        cycle_counter <= 0;
    else if (cycle_counter_ctrl[0])
        cycle_counter <= cycle_counter + 1;
    else
        cycle_counter <= cycle_counter;
end

always @(posedge M_AXI_ACLK) begin
    if (M_AXI_ARESETN == 0)
        cycle_counter_ctrl <= 1;
    else if (mst_state == S_STORE &&
             store_is_internal &&
             M_DEVICE_store_byte_enable[0] == 1 &&
             store_internal_offset == 4'h0)
        cycle_counter_ctrl <= M_DEVICE_store_data[7:0];
    else
        cycle_counter_ctrl <= cycle_counter_ctrl;
end
// ========================================
// instret counter
// ========================================

always @(posedge M_AXI_ACLK) begin
    if (M_AXI_ARESETN == 0)
        instret_counter <= 0;
    else
        instret_counter <= instret_counter + M_CORE_commit_count;
end

endmodule

