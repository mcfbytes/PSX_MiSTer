// Sim stand-in for the HPS LW bridge atom: the testbench drives the master side.
module cyclonev_hps_interface_hps2fpga_light_weight
(
	input         clk,
	output [11:0] awid,  output [20:0] awaddr, output [3:0] awlen, output [2:0] awsize,
	output  [1:0] awburst, output [1:0] awlock, output [3:0] awcache, output [2:0] awprot,
	output        awvalid, input awready,
	output [11:0] wid, output [31:0] wdata, output [3:0] wstrb, output wlast,
	output        wvalid, input wready,
	input  [11:0] bid, input [1:0] bresp, input bvalid, output bready,
	output [11:0] arid,  output [20:0] araddr, output [3:0] arlen, output [2:0] arsize,
	output  [1:0] arburst, output [1:0] arlock, output [3:0] arcache, output [2:0] arprot,
	output        arvalid, input arready,
	input  [11:0] rid, input [31:0] rdata, input [1:0] rresp, input rlast,
	input         rvalid, output rready
);
assign {awid, awaddr, awlen, awsize, awburst, awvalid} = {tb.awid, tb.awaddr, tb.awlen, tb.awsize, 2'b01, tb.awvalid};
assign {awlock, awcache, awprot} = 0;
assign {wid, wdata, wstrb, wlast, wvalid} = {tb.awid, tb.wdata, tb.wstrb, tb.wlast, tb.wvalid};
assign bready = tb.bready;
assign {arid, araddr, arlen, arsize, arburst, arvalid} = {tb.arid, tb.araddr, tb.arlen, tb.arsize, 2'b01, tb.arvalid};
assign {arlock, arcache, arprot} = 0;
assign rready = tb.rready;
endmodule
