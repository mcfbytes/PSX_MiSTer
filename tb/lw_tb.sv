`timescale 1ns/1ps

module emu_ports_probe(
`include "emu_ports.vh"
);
endmodule

module tb;

reg clk = 0;            // clk_100m
always #5 clk = ~clk;

reg sys_run = 1;        // 0 = core held in reset with clk_sys stopped
reg clk_sys = 0;
always #5.8 if (sys_run) clk_sys = ~clk_sys;

// AXI3 master side, read by the stub atom
reg [11:0] awid = 0, arid = 0;
reg [20:0] awaddr = 0, araddr = 0;
reg  [3:0] awlen = 0, arlen = 0;
reg  [2:0] awsize = 2, arsize = 2;
reg        awvalid = 0, wvalid = 0, wlast = 1, bready = 1, arvalid = 0, rready = 1;
reg [31:0] wdata = 0;
reg  [3:0] wstrb = 4'hF;

wire [20:0] lw_raddr, lw_waddr;
wire        lw_ren, lw_rd_ack, lw_rd_word, lw_write;
wire [31:0] lw_rdata, lw_wdata;
wire  [3:0] lw_be;
wire [48:0] lw_pad;

hps_lw_bridge bridge
(
	.clk(clk),
	.lw_raddr(lw_raddr), .lw_ren(lw_ren), .lw_rd_ack(lw_rd_ack), .lw_rd_word(lw_rd_word), .lw_rdata(lw_rdata),
	.lw_waddr(lw_waddr), .lw_write(lw_write), .lw_wdata(lw_wdata), .lw_be(lw_be)
);

lw_sys_window win
(
	.clk(clk),
	.lw_raddr(lw_raddr), .lw_rdata(lw_rdata),
	.lw_waddr(lw_waddr), .lw_write(lw_write), .lw_wdata(lw_wdata), .lw_be(lw_be),
	.clk_sys(clk_sys), .lw_pad(lw_pad)
);

wire awready = bridge.hps2fpga_light_weight.awready;
wire wready  = bridge.hps2fpga_light_weight.wready;
wire bvalid  = bridge.hps2fpga_light_weight.bvalid;
wire arready = bridge.hps2fpga_light_weight.arready;
wire rvalid  = bridge.hps2fpga_light_weight.rvalid;
wire rlast   = bridge.hps2fpga_light_weight.rlast;
wire [31:0] rdata = bridge.hps2fpga_light_weight.rdata;

wire [94:0] hps_bus;
assign hps_bus[94:46] = lw_pad;
assign hps_bus[45:38] = 0;
assign hps_bus[35:33] = 0;
assign hps_bus[31:16] = 0;

localparam CONF_STR = {"PSX;", `SYS_LW_CAPS, ",SS3E000000:400000;", "V,v1;"};

wire [31:0] joy0, joy1, joy2, joy3;
wire [15:0] ana0, ana3, rana0, rana3;
wire  [7:0] pdl1;

hps_io #(.CONF_STR(CONF_STR), .WIDE(1), .VDNUM(4), .BLKSZ(3)) hps_io
(
	.clk_sys(clk_sys),
	.HPS_BUS(hps_bus),
	.joystick_0(joy0), .joystick_1(joy1), .joystick_2(joy2), .joystick_3(joy3),
	.joystick_l_analog_0(ana0), .joystick_l_analog_3(ana3),
	.joystick_r_analog_0(rana0), .joystick_r_analog_3(rana3), .paddle_1(pdl1)
);

integer fails = 0, checks = 0;
integer wlat_ref = -1, rlat_ref = -1;

task check(input string what, input [31:0] got, input [31:0] exp);
	checks = checks + 1;
	if (got !== exp) begin
		fails = fails + 1;
		$display("FAIL %s: got %08h exp %08h", what, got, exp);
	end
	else $display("pass %s = %08h", what, got);
endtask

task lat_check(input string what, input integer lat, inout integer ref_lat);
	checks = checks + 1;
	if (ref_lat < 0) ref_lat = lat;
	if (lat !== ref_lat) begin
		fails = fails + 1;
		$display("FAIL %s latency %0d, expected fixed %0d", what, lat, ref_lat);
	end
endtask

// Drive at negedge, sample at negedge: cycles from VALID asserted to BVALID/RVALID seen.
task axi_write(input [20:0] a, input [31:0] d, input [3:0] s);
	integer n;
	reg aw_hs, w_hs;
	begin
		@(negedge clk);
		awaddr = a; awlen = 0; awsize = 2; awvalid = 1;
		wdata = d; wstrb = s; wlast = 1; wvalid = 1;
		n = 0;
		while (!bvalid && n <= 64) begin
			aw_hs = awvalid & awready; w_hs = wvalid & wready;
			@(negedge clk); n = n + 1;
			if (aw_hs) awvalid = 0;
			if (w_hs) wvalid = 0;
		end
		awvalid = 0; wvalid = 0;
		if (!bvalid) begin $display("FAIL write %06h: no BVALID", a); fails = fails + 1; end
		lat_check("BVALID", n, wlat_ref);
		@(negedge clk);
	end
endtask

task axi_read(input [20:0] a, input [3:0] len, output [31:0] d0, output [31:0] dl, output integer beats);
	integer n;
	reg ar_hs, done;
	begin
		@(negedge clk);
		araddr = a; arlen = len; arsize = 2; arvalid = 1;
		n = 0; beats = 0;
		while (!rvalid && n <= 64) begin
			ar_hs = arvalid & arready;
			@(negedge clk); n = n + 1;
			if (ar_hs) arvalid = 0;
		end
		arvalid = 0;
		if (!rvalid) begin $display("FAIL read %06h: no RVALID", a); fails = fails + 1; end
		lat_check("RVALID", n, rlat_ref);
		d0 = rdata; dl = rdata;
		done = !rvalid;
		while (!done) begin
			if (rvalid) begin
				beats = beats + 1; dl = rdata;
				done = rlast;
			end
			@(negedge clk);
		end
	end
endtask

function [20:0] pad_addr(input [7:0] op, input [7:0] idx);
	pad_addr = 21'h40000 + 21'h40000 + {op, 10'd0} + {idx, 2'b00};
endfunction

reg [31:0] d, dl;
integer beats, i;

task rd(input string what, input [20:0] a, input [31:0] exp);
	begin axi_read(a, 0, d, dl, beats); check(what, d, exp); end
endtask

task settle; repeat (40) @(negedge clk); endtask

initial begin
	repeat (4) @(negedge clk);

	checks = checks + 1;
	if (`SYS_LW_CAPS != $sformatf("AXI:%0h:%0h:LW", win.WIN_OFF, win.WIN_SIZE)) begin
		fails = fails + 1; $display("FAIL SYS_LW_CAPS %s does not match the decoded window", `SYS_LW_CAPS);
	end
	else $display("pass SYS_LW_CAPS = %s", `SYS_LW_CAPS);

	$display("-- directory");
	rd("magic",        21'h40000, 32'h31535953);
	rd("dir version",  21'h40004, 32'h00010000);
	rd("count",        21'h40008, 32'd1);
	rd("scratch reset",21'h4000C, 32'd0);
	rd("e0 kind",      21'h40010, 32'd1);
	rd("e0 version",   21'h40014, 32'h00010000);
	rd("e0 offset",    21'h40018, 32'h00040000);
	rd("e0 size",      21'h4001C, 32'h00040000);
	rd("e0 bitmap w0", 21'h40020, 32'h040F000C);
	rd("e0 bitmap w1", 21'h40024, 32'h20000000);
	for (i = 2; i < 8; i = i + 1) rd("e0 bitmap wN", 21'h40020 + 4*i, 32'd0);
	for (i = 0; i < 4; i = i + 1) rd("e0 reserved", 21'h40040 + 4*i, 32'd0);

	$display("-- scratch");
	axi_write(21'h4000C, 32'hA5A55A5A, 4'hF);
	rd("scratch", 21'h4000C, 32'hA5A55A5A);
	axi_write(21'h4000C, 32'h00C30000, 4'h4);
	rd("scratch byte lane", 21'h4000C, 32'hA5C35A5A);

	$display("-- pads");
	axi_write(pad_addr(8'h02, 0), 32'h8001F00F, 4'hF); settle;
	check("joystick_0", joy0, 32'h8001F00F);
	axi_write(pad_addr(8'h03, 0), 32'hDEAD0011, 4'hF); settle;
	check("joystick_1", joy1, 32'hDEAD0011);
	axi_write(pad_addr(8'h10, 0), 32'h12345678, 4'hF); settle;
	check("joystick_2", joy2, 32'h12345678);
	axi_write(pad_addr(8'h11, 0), 32'hFFFFFFFF, 4'hF); settle;
	check("joystick_3", joy3, 32'hFFFFFFFF);
	axi_write(pad_addr(8'h1A, 8'h00), 32'h0000817F, 4'hF); settle;
	check("analog_0", {16'd0, ana0}, 32'h0000817F);
	axi_write(pad_addr(8'h3D, 8'h00), 32'h00007F81, 4'hF); settle;
	check("right analog_0", {16'd0, rana0}, 32'h00007F81);
	check("left analog_0 untouched by 3D", {16'd0, ana0}, 32'h0000817F);
	axi_write(pad_addr(8'h1A, 8'h03), 32'h00001122, 4'hF);
	axi_write(pad_addr(8'h3D, 8'h03), 32'h00003344, 4'hF); settle;
	check("left analog_3", {16'd0, ana3}, 32'h00001122);
	check("right analog_3", {16'd0, rana3}, 32'h00003344);
	axi_write(pad_addr(8'h1A, 8'h1F), 32'h000000C8, 4'hF); settle;
	check("paddle_1 (idx {pdsp=1,stick=15})", {24'd0, pdl1}, 32'h000000C8);
	axi_write(pad_addr(8'h02, 1), 32'h11111111, 4'hF); settle;
	check("op 02 idx 1 dropped", joy0, 32'h8001F00F);
	axi_write(pad_addr(8'h05, 0), 32'h22222222, 4'hF); settle;
	check("unhonoured op dropped", joy0, 32'h8001F00F);
	axi_write(pad_addr(8'h02, 0), 32'h0000A0A0, 4'hF);
	axi_write(pad_addr(8'h03, 0), 32'h0000B0B0, 4'hF);
	axi_write(pad_addr(8'h10, 0), 32'h0000C0C0, 4'hF);
	axi_write(pad_addr(8'h11, 0), 32'h0000D0D0, 4'hF);
	settle;
	check("back-to-back joystick_0", joy0, 32'h0000A0A0);
	check("back-to-back joystick_1", joy1, 32'h0000B0B0);
	check("back-to-back joystick_2", joy2, 32'h0000C0C0);
	check("back-to-back joystick_3", joy3, 32'h0000D0D0);
	rd("pad block read", pad_addr(8'h02, 0), 32'd0);
	rd("pad block unhonoured read", pad_addr(8'h05, 3), 32'd0);

	$display("-- default responder");
	rd("LW 0x000000", 21'h000000, 32'hDEADBEEF);
	rd("below window", 21'h03FFFC, 32'hDEADBEEF);
	rd("directory hole", 21'h040050, 32'hDEADBEEF);
	rd("above window", 21'h0C0000, 32'hDEADBEEF);
	rd("LW 0x1FFFFC", 21'h1FFFFC, 32'hDEADBEEF);
	axi_write(21'h000100, 32'h0, 4'hF);
	axi_read(21'h40000, 4'd3, d, dl, beats);
	check("burst beats", beats, 4);
	check("burst last (scratch)", dl, 32'hA5C35A5A);

	$display("-- clk_sys stopped (core held in reset)");
	sys_run = 0;
	rd("magic, clk_sys stopped", 21'h40000, 32'h31535953);
	axi_write(21'h4000C, 32'h0BADF00D, 4'hF);
	rd("scratch, clk_sys stopped", 21'h4000C, 32'h0BADF00D);
	for (i = 1; i <= 5; i = i + 1) axi_write(pad_addr(8'h02, 0), i, 4'hF);
	axi_write(pad_addr(8'h03, 0), 32'h00000077, 4'hF);
	axi_write(pad_addr(8'h03, 0), 32'h00000000, 4'hF);
	for (i = 1; i <= 10; i = i + 1) axi_write(pad_addr(8'h1A, 0), i, 4'hF);
	for (i = 11; i <= 20; i = i + 1) axi_write(pad_addr(8'h3D, 0), i, 4'hF);
	rd("undecoded, clk_sys stopped", 21'h1FFFFC, 32'hDEADBEEF);
	check("joystick_0 unchanged while stopped", joy0, 32'h0000A0A0);
	rd("analog drops (16 queued of 20, a pad store in flight)", pad_addr(8'h1A, 0), 32'd4);
	sys_run = 1; repeat (300) @(negedge clk);
	check("joystick_0 last of 5 lands on restart", joy0, 32'h00000005);
	check("joystick_1 release (last) lands on restart", joy1, 32'h00000000);
	check("analog_0 last left queued lands on restart", {16'd0, ana0}, 32'h0000000A);
	check("right analog_0 last queued lands on restart", {16'd0, rana0}, 32'h00000010);

	$display("fixed latency: BVALID %0d cycles after AWVALID, RVALID %0d cycles after ARVALID", wlat_ref, rlat_ref);
	$display("%s: %0d checks, %0d failures", fails ? "FAILED" : "PASSED", checks, fails);
	$finish;
end

initial begin #2000000 $display("FAILED: timeout"); $finish; end

endmodule
