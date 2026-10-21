`timescale 1ns/1ps

module emu_ports_probe(
`include "emu_ports.vh"
);
endmodule

// One osd output, captured: bit (y, x) of the OSD as the pixel path draws it (din = 0, so dout[21] =
// OSD_COLOR[2] marks the OSD box and dout[23] is the pixel). `lines` is the box height of the last frame.
module osd_cap(input clk, input [23:0] d, input de, input vs);
reg        cap [0:32767];
integer    x = 0, y = 0, lines = -1, frames = 0, wbad = 0, wbad_l = 0;
reg        vs_d = 0, de_d = 0, has = 0;
always @(posedge clk) begin
	vs_d <= vs;
	de_d <= de;
	if (vs && !vs_d) begin
		lines  <= y;
		wbad_l <= wbad;
		frames <= frames + 1;
		y      <= 0;
		wbad   <= 0;
	end
	if (de && d[21] === 1'b1) begin
		if (y < 128 && x < 256) cap[y*256 + x] <= d[23];
		x   <= x + 1;
		has <= 1;
	end
	if (!de && de_d) begin
		if (has) begin
			y <= y + 1;
			if (x != 256) wbad <= wbad + 1;
		end
		x   <= 0;
		has <= 0;
	end
end
endmodule

module tb;

`ifdef LW_DIAG
localparam DIAG = `LW_DIAG;
`else
localparam DIAG = 0;
`endif
function [31:0] dg(input [31:0] v); dg = DIAG ? v : 32'd0; endfunction

reg clk = 0;            // clk_100m
always #5 clk = ~clk;

reg sys_run = 1;        // 0 = core held in reset with clk_sys stopped
reg clk_sys = 0;        // PSX clk_1x, 33.8688 MHz
always #14.7625 if (sys_run) clk_sys = ~clk_sys;

// AXI3 master side, read by the stub atom
reg [11:0] awid = 0, arid = 0;
reg [20:0] awaddr = 0, araddr = 0;
reg  [3:0] awlen = 0, arlen = 0;
reg  [2:0] awsize = 2, arsize = 2;
reg  [2:0] rsize = 2;   // ARSIZE used by axi_read
reg        awvalid = 0, wvalid = 0, wlast = 1, bready = 1, arvalid = 0, rready = 1;
reg [31:0] wdata = 0;
reg  [3:0] wstrb = 4'hF;

wire [20:0] lw_raddr, lw_waddr;
wire        lw_ren, lw_rd_ack, lw_rd_word, lw_write;
wire [31:0] lw_rdata, lw_wdata;
wire  [3:0] lw_be;
wire [48:0] lw_pad;
wire        doorbell;
wire [34:0] blk_up;
wire [18:0] blk_dn;
wire        osd_we;
wire  [9:0] osd_addr;
wire [31:0] osd_data;
wire  [3:0] osd_be;
wire  [1:0] osd_ctrl;

hps_lw_bridge bridge
(
	.clk(clk),
	.lw_raddr(lw_raddr), .lw_ren(lw_ren), .lw_rd_ack(lw_rd_ack), .lw_rd_word(lw_rd_word), .lw_rdata(lw_rdata),
	.lw_waddr(lw_waddr), .lw_write(lw_write), .lw_wdata(lw_wdata), .lw_be(lw_be)
);

lw_sys_window win
(
	.clk(clk),
	.lw_raddr(lw_raddr), .lw_ren(lw_ren), .lw_rd_ack(lw_rd_ack), .lw_rd_word(lw_rd_word), .lw_rdata(lw_rdata),
	.lw_waddr(lw_waddr), .lw_write(lw_write), .lw_wdata(lw_wdata), .lw_be(lw_be),
	.doorbell(doorbell),
	.clk_sys(clk_sys), .lw_pad(lw_pad), .blk_up(blk_up), .blk_dn(blk_dn),
	.osd_we(osd_we), .osd_addr(osd_addr), .osd_data(osd_data), .osd_be(osd_be), .osd_ctrl(osd_ctrl)
);

// Both osd instances as sys_top wires them: one SPI select (Main drives ss0 and ss2 together), one LW store bus.
reg clk_vid = 0;
always #3.25 clk_vid = ~clk_vid;
reg   [8:0] vh = 0, vv = 0;           // 272x140 active in 288x148: ce_pix every clock, OSD 256 wide
always @(posedge clk_vid) begin
	vh <= (vh == 287) ? 9'd0 : vh + 9'd1;
	if (vh == 287) vv <= (vv == 147) ? 9'd0 : vv + 9'd1;
end
wire        v_de = vh < 272 && vv < 140;
wire        v_hs = vh >= 276 && vh < 282;
wire        v_vs = vv >= 142 && vv < 144;
reg         io_osd = 0;
wire [23:0] ho_d, vo_d;
wire        ho_de, ho_vs, ho_hs, vo_de, vo_vs, vo_hs, osd_status;

osd hdmi_osd
(
	.clk_sys(clk_sys), .io_osd(io_osd), .io_strobe(io_strobe), .io_din(io_din),
	.clk_video(clk_vid), .din(24'd0), .de_in(v_de), .vs_in(v_vs), .hs_in(v_hs),
	.dout(ho_d), .de_out(ho_de), .vs_out(ho_vs), .hs_out(ho_hs),
	.clk_lw(clk), .lw_we(osd_we), .lw_addr(osd_addr), .lw_data(osd_data), .lw_be(osd_be), .lw_ctrl(osd_ctrl)
);

osd vga_osd
(
	.clk_sys(clk_sys), .io_osd(io_osd), .io_strobe(io_strobe), .io_din(io_din), .osd_status(osd_status),
	.clk_video(clk_vid), .din(24'd0), .de_in(v_de), .vs_in(v_vs), .hs_in(v_hs),
	.dout(vo_d), .de_out(vo_de), .vs_out(vo_vs), .hs_out(vo_hs),
	.clk_lw(clk), .lw_we(osd_we), .lw_addr(osd_addr), .lw_data(osd_data), .lw_be(osd_be), .lw_ctrl(osd_ctrl)
);

osd_cap cap_h(.clk(clk_vid), .d(ho_d), .de(ho_de), .vs(ho_vs));
osd_cap cap_v(.clk(clk_vid), .d(vo_d), .de(vo_de), .vs(vo_vs));

wire awready = bridge.hps2fpga_light_weight.awready;
wire wready  = bridge.hps2fpga_light_weight.wready;
wire bvalid  = bridge.hps2fpga_light_weight.bvalid;
wire arready = bridge.hps2fpga_light_weight.arready;
wire rvalid  = bridge.hps2fpga_light_weight.rvalid;
wire rlast   = bridge.hps2fpga_light_weight.rlast;
wire [31:0] rdata = bridge.hps2fpga_light_weight.rdata;

// HPS_BUS as sys_top wires it: {blk_up, blk_dn, lw_pad, ...}; the SPI side is driven by the tb (io_*)
reg         io_enable = 0, io_strobe = 0;
reg  [15:0] io_din = 0;
wire [148:0] hps_bus;
assign blk_up = hps_bus[148:114];
assign hps_bus[113:95] = blk_dn;
assign hps_bus[94:46] = lw_pad;
assign hps_bus[45:38] = 0;
assign hps_bus[35:33] = {1'b0, io_enable, io_strobe};
assign hps_bus[31:16] = io_din;

localparam CONF_STR = {"PSX;", `SYS_LW_CAPS, ",SS3E000000:400000;", "V,v1;"};

wire [31:0] joy0, joy1, joy2, joy3;
wire [15:0] ana0, ana3, rana0, rana3;
wire  [7:0] pdl1;
reg         cd_reset = 0;
reg         cd_req = 0;
reg  [31:0] cd_lba = 32'hFFFFFFFF;
wire        cd_ack, cd_wr;
wire [15:0] cd_data;
wire  [3:0] sd_rd, sd_ack;
wire [15:0] sd_buff_dout;
wire        sd_buff_wr;
wire [31:0] sd_lba [4];
wire [15:0] sd_buff_din [4];
assign sd_rd[0] = 0;
assign sd_rd[3:2] = 0;
assign sd_lba[0] = 0;
assign sd_lba[1] = cd_lba;
assign sd_lba[2] = 0;
assign sd_lba[3] = 0;
assign sd_buff_din[0] = 0;
assign sd_buff_din[1] = 0;
assign sd_buff_din[2] = 0;
assign sd_buff_din[3] = 0;

hps_io #(.CONF_STR(CONF_STR), .WIDE(1), .VDNUM(4), .BLKSZ(3)) hps_io
(
	.clk_sys(clk_sys),
	.HPS_BUS(hps_bus),
	.joystick_0(joy0), .joystick_1(joy1), .joystick_2(joy2), .joystick_3(joy3),
	.joystick_l_analog_0(ana0), .joystick_l_analog_3(ana3),
	.joystick_r_analog_0(rana0), .joystick_r_analog_3(rana3), .paddle_1(pdl1),
	.sd_lba(sd_lba), .sd_rd(sd_rd), .sd_wr(4'd0), .sd_ack(sd_ack),
	.sd_buff_dout(sd_buff_dout), .sd_buff_din(sd_buff_din), .sd_buff_wr(sd_buff_wr)
);

// The PSX.sv CD mux, as instantiated there.
lw_blk_mux cd_lw
(
	.HPS_BUS(hps_bus), .reset(cd_reset),
	.core_req(cd_req), .core_lba(cd_lba), .core_ack(cd_ack), .core_wr(cd_wr), .core_data(cd_data),
	.hps_req(sd_rd[1]), .hps_ack(sd_ack[1]), .hps_wr(sd_buff_wr), .hps_data(sd_buff_dout)
);

// cd_top's fetch FSM (cd_top.vhd SFETCH_START/HPSACK/HPSWORD/HPSDATA): req with the LBA, drop req on ack,
// then exactly 1176 strobes, low half then high half, into one staging buffer. It consumes ANY strobe.
reg   [1:0] cst = 0;
reg   [9:0] fcnt = 0;
reg  [15:0] clo = 0;
reg  [31:0] cbuf [0:587];
integer     cd_frames = 0, cd_strobes = 0, cd_go = 0;
reg  [31:0] cd_next = 0;

always @(posedge clk_sys) begin
	if (cd_wr) cd_strobes <= cd_strobes + 1;
	if (cd_reset) begin
		cd_req <= 0;
		cst    <= 0;
	end
	else case (cst)
		0: if (cd_go && !cd_ack) begin
			cd_go  <= 0;
			cd_req <= 1;
			cd_lba <= cd_next;
			cst    <= 1;
		end
		1: if (cd_ack) begin
			cd_req <= 0;
			fcnt   <= 0;
			cst    <= 2;
		end
		2: if (cd_wr) begin
			clo <= cd_data;
			cst <= 3;
		end
		3: if (cd_wr) begin
			cbuf[fcnt] <= {cd_data, clo};
			if (fcnt == 587) begin
				cst       <= 0;
				cd_frames <= cd_frames + 1;
			end
			else begin
				fcnt <= fcnt + 10'd1;
				cst  <= 2;
			end
		end
	endcase
end

// A memcard on hps_io slot 2 (memcard.vhd gates the shared strobe with its own ack).
integer mc_words = 0, mc_sum = 0;
always @(posedge clk_sys) if (sd_buff_wr && sd_ack[2]) begin
	mc_words <= mc_words + 1;
	mc_sum   <= mc_sum + sd_buff_dout;
end

integer fails = 0, checks = 0, lchecks = 0;
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
	lchecks = lchecks + 1;
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
		araddr = a; arlen = len; arsize = rsize; arvalid = 1;
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

task settle; repeat (100) @(negedge clk); endtask


localparam [20:0] SEC = 21'h41000;
reg [31:0] gen0, d0;
integer n0, s0, overlap = 0;

// a memcard strobe while the LW feeder holds the CD ack: the overlap the mux exists for
always @(posedge clk_sys) if (sd_buff_wr && sd_ack[2] && blk_dn[17] && !blk_dn[16]) overlap = 1;

function [31:0] pat(input [31:0] lba, input integer i);
	pat = (lba * 32'h9E3779B1) ^ (i * 32'h01000193) ^ {i[15:0], 16'h5A00};
endfunction

function [31:0] frame_sum(input [31:0] lba, input integer n);
	integer m;
	reg [31:0] w;
	begin
		frame_sum = 0;
		for (m = 0; m < n; m = m + 1) begin
			w = pat(lba, m);
			frame_sum = frame_sum + w[31:24] + w[23:16] + w[15:8] + w[7:0];
		end
	end
endfunction

task sec_wr(input [20:0] off, input [31:0] v); axi_write(SEC + off, v, 4'hF); endtask
task sec_data(input integer i, input [31:0] v); axi_write(SEC + 21'h1000 + 4*i, v, 4'hF); endtask

task frame_data(input [31:0] lba, input integer from, input integer to);
	integer m;
	for (m = from; m < to; m = m + 1) sec_data(m, pat(lba, m));
endtask

task cd_request(input [31:0] lba);
	integer t;
	begin
		@(negedge clk_sys);
		cd_next = lba; cd_go = 1;
		t = 0;
		while (!(cd_req && cd_lba == lba) && t < 10000) begin @(negedge clk_sys); t = t + 1; end
	end
endtask

task wait_frame(input integer n);
	integer t;
	begin
		t = 0;
		while (cd_frames == n && t < 20000) begin @(negedge clk_sys); t = t + 1; end
		checks = checks + 1;
		if (cd_frames == n) begin fails = fails + 1; $display("FAIL frame did not complete"); end
	end
endtask

// words [0, n) carry the pattern, the rest of the 588 must be zero (fill)
task check_buf(input string what, input [31:0] lba, input integer n);
	integer m, bad, first;
	reg [31:0] e;
	begin
		bad = 0; first = -1;
		for (m = 0; m < 588; m = m + 1) begin
			e = (m < n) ? pat(lba, m) : 32'd0;
			if (cbuf[m] !== e) begin bad = bad + 1; if (first < 0) first = m; end
		end
		if (bad) $display("     %s: first bad word %0d got %08h", what, first, cbuf[first]);
		check(what, bad, 0);
	end
endtask

// hps_io SPI side: one command word then n data words, one strobe every `gap` clk_sys
task spi_sector(input [15:0] cmd, input integer n, input [15:0] seed, input integer gap);
	integer m;
	begin
		@(negedge clk_sys); io_enable = 1;
		io_din = cmd; io_strobe = 1; @(negedge clk_sys); io_strobe = 0; repeat (gap - 1) @(negedge clk_sys);
		for (m = 0; m < n; m = m + 1) begin
			io_din = seed + m; io_strobe = 1; @(negedge clk_sys); io_strobe = 0; repeat (gap - 1) @(negedge clk_sys);
		end
		io_enable = 0; @(negedge clk_sys);
	end
endtask

// stock path: UIO_SECTOR_RD on slot 1 with the frame as 1176 16-bit words
task spi_frame(input [31:0] lba);
	integer m;
	reg [31:0] w;
	begin
		@(negedge clk_sys); io_enable = 1;
		io_din = 16'h0117; io_strobe = 1; @(negedge clk_sys); io_strobe = 0; @(negedge clk_sys);
		for (m = 0; m < 1176; m = m + 1) begin
			w = pat(lba, m >> 1);
			io_din = m[0] ? w[31:16] : w[15:0]; io_strobe = 1; @(negedge clk_sys); io_strobe = 0; @(negedge clk_sys);
		end
		io_enable = 0; @(negedge clk_sys);
	end
endtask

// OSDBUF: the tb's models of the two images, the stores that fill them and the pixel-path check.
localparam [20:0] OSD = 21'h44000;
reg  [7:0] spi_img [0:4095];
reg  [7:0] lw_img  [0:4095];

function [7:0] lwp(input integer i, input integer k);
	lwp = (i * 37) ^ (i >> 4) ^ (k * 8'h5B) ^ 8'hA5;
endfunction
function [7:0] spp(input integer i, input integer k);
	spp = (i * 113) ^ (i >> 6) ^ (k * 8'hC7) ^ 8'h3C;
endfunction

task osd_word(input integer w, input [31:0] v, input [3:0] be);
	integer b;
	begin
		axi_write(OSD + 4*w, v, be);
		for (b = 0; b < 4; b = b + 1) if (be[b]) lw_img[4*w + b] = v[8*b +: 8];
	end
endtask

task lw_row(input integer r, input integer k);
	integer w;
	for (w = r*64; w < r*64 + 64; w = w + 1) osd_word(w, {lwp(4*w+3, k), lwp(4*w+2, k), lwp(4*w+1, k), lwp(4*w, k)}, 4'hF);
endtask

task spi_osd_begin(input [7:0] cmd);
	begin
		@(negedge clk_sys); io_osd = 1;
		io_din = {8'h00, cmd}; io_strobe = 1; @(negedge clk_sys); io_strobe = 0; @(negedge clk_sys);
	end
endtask
task spi_osd_end; begin io_osd = 0; repeat (2) @(negedge clk_sys); end endtask
task spi_osd_cmd(input [7:0] cmd); begin spi_osd_begin(cmd); spi_osd_end; end endtask

// OSDCMDWRITE row r: 0x20|r, so bit 3 (rows 8-15) re-arms the SPI highres (osd.v:80)
task spi_osd_row(input integer r, input integer k);
	integer m;
	begin
		spi_osd_begin(8'h20 | r[4:0]);
		for (m = 0; m < 256; m = m + 1) begin
			spi_img[r*256 + m] = spp(r*256 + m, k);
			io_din = {8'h00, spi_img[r*256 + m]}; io_strobe = 1; @(negedge clk_sys); io_strobe = 0; @(negedge clk_sys);
		end
		spi_osd_end;
	end
endtask

// After `nf` frames, both outputs drew an OSD box `exp_lines` high and 256 wide, bit (y, x) from the image.
task osd_check(input string what, input from_lw, input integer exp_lines, input integer nf);
	integer f, y, x, bh, bv;
	reg [7:0] b;
	begin
		f = cap_h.frames;
		wait (cap_h.frames >= f + nf);
		@(negedge clk_vid);
		bh = 0; bv = 0;
		for (y = 0; y < exp_lines && y < 128; y = y + 1)
			for (x = 0; x < 256; x = x + 1) begin
				b = from_lw ? lw_img[(y >> 3)*256 + x] : spi_img[(y >> 3)*256 + x];
				if (cap_h.cap[y*256 + x] !== b[y & 7]) bh = bh + 1;
				if (cap_v.cap[y*256 + x] !== b[y & 7]) bv = bv + 1;
			end
		check({what, ": hdmi lines"}, cap_h.lines, exp_lines);
		check({what, ": vga lines"}, cap_v.lines, exp_lines);
		check({what, ": widths 256"}, cap_h.wbad_l + cap_v.wbad_l, 0);
		check({what, ": hdmi pixels"}, bh, 0);
		check({what, ": vga pixels"}, bv, 0);
	end
endtask

function [31:0] lanes_h(input integer w);
	lanes_h = {hdmi_osd.lw_buf3[w], hdmi_osd.lw_buf2[w], hdmi_osd.lw_buf1[w], hdmi_osd.lw_buf0[w]};
endfunction
function [31:0] lanes_v(input integer w);
	lanes_v = {vga_osd.lw_buf3[w], vga_osd.lw_buf2[w], vga_osd.lw_buf1[w], vga_osd.lw_buf0[w]};
endfunction
function [31:0] img_w(input integer w);
	img_w = {lw_img[4*w+3], lw_img[4*w+2], lw_img[4*w+1], lw_img[4*w]};
endfunction

integer r, bad;

initial begin
	repeat (4) @(negedge clk);

	checks = checks + 1;
	if (`SYS_LW_CAPS != $sformatf("AXI:%0h:%0h:LW,IRQ:BLK=1:%0h", win.WIN_OFF, win.WIN_SIZE, win.WIN_OFF + win.SEC_OFF)) begin
		fails = fails + 1; $display("FAIL SYS_LW_CAPS %s does not match the decoded window", `SYS_LW_CAPS);
	end
	else $display("pass SYS_LW_CAPS = %s", `SYS_LW_CAPS);

	$display("-- directory");
	rd("magic",        21'h40000, 32'h31535953);
	rd("dir version",  21'h40004, 32'h00010000);
	rd("count",        21'h40008, 32'd3);
	rd("scratch reset",21'h4000C, 32'd0);
	rd("e0 kind",      21'h40010, 32'd1);
	rd("e0 version",   21'h40014, 32'h00010000);
	rd("e0 offset",    21'h40018, 32'h00040000);
	rd("e0 size",      21'h4001C, 32'h00040000);
	rd("e0 bitmap w0", 21'h40020, 32'h040F000C);
	rd("e0 bitmap w1", 21'h40024, 32'h20000000);
	for (i = 2; i < 8; i = i + 1) rd("e0 bitmap wN", 21'h40020 + 4*i, 32'd0);
	for (i = 0; i < 4; i = i + 1) rd("e0 reserved", 21'h40040 + 4*i, 32'd0);
	rd("e1 kind SECTOR", 21'h40050, 32'd2);
	rd("e1 version",     21'h40054, 32'h00010000);
	rd("e1 offset",      21'h40058, 32'h00001000);
	rd("e1 size",        21'h4005C, 32'h00002000);
	rd("e1 slot/frame",  21'h40060, 32'h0100024C);
	rd("e1 cause (abs)", 21'h40064, 32'h00041000);
	rd("e1 depth/gap",   21'h40068, 32'h02000002);
	for (i = 7; i < 16; i = i + 1) rd("e1 reserved", 21'h40050 + 4*i, 32'd0);
	rd("e2 kind OSDBUF", 21'h40090, 32'd3);
	rd("e2 version",     21'h40094, 32'h00010000);
	rd("e2 offset",      21'h40098, 32'h00004000);
	rd("e2 size",        21'h4009C, 32'h00002000);
	rd("e2 geometry",    21'h400A0, 32'h00800100);
	for (i = 5; i < 16; i = i + 1) rd("e2 reserved", 21'h40090 + 4*i, 32'd0);

	$display("-- scratch");
	axi_write(21'h4000C, 32'hA5A55A5A, 4'hF);
	rd("scratch", 21'h4000C, dg(32'hA5A55A5A));
	axi_write(21'h4000C, 32'h00C30000, 4'h4);
	rd("scratch byte lane", 21'h4000C, dg(32'hA5C35A5A));

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
	rd("directory hole", 21'h0400D0, 32'hDEADBEEF);
	rd("below SECTOR", 21'h040FFC, 32'hDEADBEEF);
	rd("above SECTOR", 21'h043000, 32'hDEADBEEF);
	rd("below OSDBUF", 21'h043FFC, 32'hDEADBEEF);
	rd("above OSDBUF", 21'h046000, 32'hDEADBEEF);
	rd("above window", 21'h0C0000, 32'hDEADBEEF);
	rd("LW 0x1FFFFC", 21'h1FFFFC, 32'hDEADBEEF);
	axi_write(21'h000100, 32'h0, 4'hF);
	axi_read(21'h40000, 4'd3, d, dl, beats);
	check("burst beats", beats, 4);
	check("burst last (scratch)", dl, dg(32'hA5C35A5A));

	$display("-- clk_sys stopped (core held in reset)");
	sys_run = 0;
	rd("magic, clk_sys stopped", 21'h40000, 32'h31535953);
	axi_write(21'h4000C, 32'h0BADF00D, 4'hF);
	rd("scratch, clk_sys stopped", 21'h4000C, dg(32'h0BADF00D));
	for (i = 1; i <= 5; i = i + 1) axi_write(pad_addr(8'h02, 0), i, 4'hF);
	axi_write(pad_addr(8'h03, 0), 32'h00000077, 4'hF);
	axi_write(pad_addr(8'h03, 0), 32'h00000000, 4'hF);
	for (i = 1; i <= 10; i = i + 1) axi_write(pad_addr(8'h1A, 0), i, 4'hF);
	for (i = 11; i <= 20; i = i + 1) axi_write(pad_addr(8'h3D, 0), i, 4'hF);
	rd("undecoded, clk_sys stopped", 21'h1FFFFC, 32'hDEADBEEF);
	check("joystick_0 unchanged while stopped", joy0, 32'h0000A0A0);
	rd("analog drops (16 queued of 20, a pad store in flight)", pad_addr(8'h1A, 0), dg(32'd4));
	sys_run = 1; repeat (300) @(negedge clk);
	check("joystick_0 last of 5 lands on restart", joy0, 32'h00000005);
	check("joystick_1 release (last) lands on restart", joy1, 32'h00000000);
	check("analog_0 last left queued lands on restart", {16'd0, ana0}, 32'h0000000A);
	check("right analog_0 last queued lands on restart", {16'd0, rana0}, 32'h00000010);


	$display("-- SECTOR block: registers");
	rd("ID SEC1", SEC + 21'h38, 32'h31434553);
	rd("CTRL reset", SEC + 21'h10, 32'd0);
	rd("STATUS idle", SEC + 21'h14, 32'd0);
	rd("REQ idle", SEC + 21'h04, 32'd0);
	rd("CAUSE idle", SEC + 21'h00, 32'd0);
	rd("BEGIN reads 0", SEC + 21'h0C, 32'd0);
	rd("register hole", SEC + 21'h3C, 32'd0);
	rd("block hole", SEC + 21'hFFC, 32'd0);
	rd("DATA[0] reads 0", SEC + 21'h1000, 32'd0);
	rd("DATA[588] reads 0", SEC + 21'h1000 + 4*588, 32'd0);
	rd("block end", SEC + 21'h1FFC, 32'd0);
	check("doorbell low", doorbell, 0);

	$display("-- doorbell on cd_hps_req");
	axi_read(SEC + 21'h24, 0, gen0, dl, beats);
	cd_request(32'h00001234);
	settle;
	check("doorbell raised", doorbell, 1);
	check("SPI slot 1 offered while EN=0", sd_rd[1], 1);
	rd("REQ pending + sdstat", SEC + 21'h04, 32'h800080C5);
	rd("LBA captured", SEC + 21'h08, 32'h00001234);
	rd("GEN +1", SEC + 21'h24, dg(gen0 + 1));
	rsize = 0; rd("CAUSE byte read: data", SEC + 21'h00, 32'h000180C5); rsize = 2;
	check("byte read does not clear", doorbell, 1);
	rd("CAUSE word read: BLK tag + 0x80C5", SEC + 21'h00, 32'h000180C5);
	check("word read clears the line", doorbell, 0);
	rd("CAUSE zero iff idle", SEC + 21'h00, 32'd0);
	rd("REQ is a level (still pending)", SEC + 21'h04, 32'h800080C5);
	sec_wr(21'h10, 32'h1); settle;
	rd("STATUS en_sys", SEC + 21'h14, 32'h00000001);
	check("SPI slot 1 hidden while en_sys", sd_rd[1], 0);

	$display("-- full 588-store frame");
	n0 = cd_frames; s0 = cd_strobes;
	sec_wr(21'h0C, 32'h00001234);
	frame_data(32'h00001234, 0, 588);
	rd("ACCEPT ok, 588", SEC + 21'h18, 32'h8000024C);
	wait_frame(n0);
	check_buf("frame 0x1234", 32'h00001234, 588);
	repeat (200) @(negedge clk_sys);
	check("1176 strobes", cd_strobes - s0, 1176);
	check("cd_top saw ack then dropped req", cd_req, 0);
	rd("REQ cleared by ack", SEC + 21'h04, 32'd0);
	axi_read(SEC + 21'h14, 0, d, dl, beats);
	check("STATUS: 1 frame, idle, en_sys", {d[31:16], d[3:0]}, DIAG ? 20'h00011 : 20'h00001);
	if (!DIAG) check("STATUS[31:4] lean", d[31:4], 0);
	$display("     FIFO high-water %0d entries", d[15:8]);
	rd("SUM", SEC + 21'h34, dg(frame_sum(32'h00001234, 588)));
	axi_read(SEC + 21'h28, 0, d, dl, beats);
	axi_read(SEC + 21'h2C, 0, dl, d0, beats);
	if (DIAG) check("TSTAMP_DONE after TSTAMP_REQ", dl > d, 1);
	else check("TSTAMP_REQ/DONE lean", {d, dl}, 0);
	if (!DIAG) begin
		rd("lean CYCLES reads 0", SEC + 21'h30, 32'd0);
		rd("lean GEN reads 0", SEC + 21'h24, 32'd0);
		rd("lean DROPS reads 0", SEC + 21'h1C, 32'd0);
		rd("lean COUNTS reads 0", SEC + 21'h20, 32'd0);
	end
	$display("     req -> done %0d clk_100m cycles", dl - d);

	$display("-- out-of-order and duplicated stores");
	n0 = cd_frames;
	cd_request(32'h00001235); settle;
	sec_wr(21'h0C, 32'h00001235);
	frame_data(32'h00001235, 0, 10);
	sec_data(11, pat(32'h00001235, 11));
	sec_data(10, pat(32'h00001235, 10));
	sec_data(10, pat(32'h00001235, 10));
	sec_data(11, pat(32'h00001235, 11));
	frame_data(32'h00001235, 12, 588);
	rd("ACCEPT ok", SEC + 21'h18, 32'h8000024C);
	wait_frame(n0);
	check_buf("frame 0x1235 not shifted", 32'h00001235, 588);
	rd("COUNTS skipped 2", SEC + 21'h20, dg(32'h00020000));

	$display("-- lost store detected by ACCEPT, resent from first_drop");
	n0 = cd_frames;
	cd_request(32'h00001236); settle;
	sec_wr(21'h0C, 32'h00001236);
	frame_data(32'h00001236, 0, 100);
	frame_data(32'h00001236, 101, 588);
	rd("ACCEPT !ok, first_drop 100", SEC + 21'h18, 32'h00000064);
	repeat (2000) @(negedge clk);
	check("frame held open (cd_top waits)", cd_frames, n0);
	frame_data(32'h00001236, 100, 588);
	rd("ACCEPT ok after resend", SEC + 21'h18, 32'h8000024C);
	wait_frame(n0);
	check_buf("frame 0x1236 after resend", 32'h00001236, 588);

	$display("-- FIFO full with clk_sys stopped: P2 drops, fixed latency, resend");
	n0 = cd_frames;
	cd_request(32'h00001237); settle;
	sec_wr(21'h0C, 32'h00001237);
	settle;
	sys_run = 0;
	frame_data(32'h00001237, 0, 588);
	rd("ACCEPT drops 76, first_drop 512", SEC + 21'h18, 32'h004C0200);
	rd("DROPS 76", SEC + 21'h1C, dg(32'd76));
	axi_read(SEC + 21'h14, 0, d, dl, beats);
	check("STATUS high-water saturates at 255", d[15:8], dg(255));
	rd("CAUSE pending, clk_sys stopped", SEC + 21'h00, 32'h000180C5);
	check("CAUSE read clears with clk_sys stopped", doorbell, 0);
	rd("ID, clk_sys stopped", SEC + 21'h38, 32'h31434553);
	rd("undecoded, clk_sys stopped", 21'h1FFFFC, 32'hDEADBEEF);
	sys_run = 1;
	repeat (8000) @(negedge clk);
	frame_data(32'h00001237, 512, 588);
	rd("ACCEPT ok after resend (drops kept)", SEC + 21'h18, 32'h804C024C);
	wait_frame(n0);
	check_buf("frame 0x1237 after FIFO-full resend", 32'h00001237, 588);

	$display("-- ABORT mid-frame zero-fills");
	n0 = cd_frames;
	cd_request(32'h00001238); settle;
	sec_wr(21'h0C, 32'h00001238);
	frame_data(32'h00001238, 0, 300);
	sec_wr(21'h10, 32'h3);
	wait_frame(n0);
	check_buf("frame 0x1238: 300 words then zeros", 32'h00001238, 300);
	axi_read(SEC + 21'h20, 0, d, dl, beats);
	check("COUNTS: rejected 0, aborted 1, fills 0", {d[31:24], d[15:0]}, dg(24'h000100));
	check("COUNTS: skipped saturated (lost-store + resend skips)", d[23:16], dg(8'hFF));
	rd("CTRL keeps EN", SEC + 21'h10, 32'h1);

	$display("-- memcard SECTOR_RD over SPI overlapping an LW frame");
	n0 = cd_frames; s0 = cd_strobes; overlap = 0;
	cd_request(32'h00001239); settle;
	sec_wr(21'h0C, 32'h00001239);
	fork
		frame_data(32'h00001239, 0, 588);
		begin repeat (100) @(negedge clk); spi_sector(16'h0217, 256, 16'h7700, 3); end
	join
	rd("ACCEPT ok", SEC + 21'h18, 32'h8000024C);
	wait_frame(n0);
	repeat (200) @(negedge clk_sys);
	check("memcard took its 256 words", mc_words, 256);
	check("memcard strobes landed while the LW frame was open", overlap, 1);
	check("cd_top saw only its 1176 strobes", cd_strobes - s0, 1176);
	check_buf("frame 0x1239 untouched by the memcard", 32'h00001239, 588);

	$display("-- stale BEGIN rejected");
	n0 = cd_frames;
	cd_request(32'h0000123A); settle;
	sec_wr(21'h0C, 32'h00009999);
	frame_data(32'h00009999, 0, 8);
	settle;
	axi_read(SEC + 21'h14, 0, d, dl, beats);
	check("STATUS rejecting, ack low", d[3:1], 3'b100);
	axi_read(SEC + 21'h20, 0, d, dl, beats);
	check("COUNTS: rejected 1", {d[31:24], d[15:0]}, dg(24'h010100));
	sec_wr(21'h0C, 32'h0000123A);
	frame_data(32'h0000123A, 0, 588);
	rd("ACCEPT ok", SEC + 21'h18, 32'h8000024C);
	wait_frame(n0);
	check_buf("frame 0x123A after the reject", 32'h0000123A, 588);

	$display("-- core reset mid-frame zero-fills");
	n0 = cd_frames;
	cd_request(32'h0000123B); settle;
	sec_wr(21'h0C, 32'h0000123B);
	frame_data(32'h0000123B, 0, 200);
	@(negedge clk_sys) cd_reset = 1;
	repeat (4) @(negedge clk_sys);
	cd_reset = 0;
	repeat (2400) @(negedge clk_sys);
	axi_read(SEC + 21'h14, 0, d, dl, beats);
	check("STATUS idle after the fill, ack low", d[3:1], 3'b000);
	axi_read(SEC + 21'h20, 0, d, dl, beats);
	check("COUNTS: reset_fills 1", {d[31:24], d[15:0]}, dg(24'h010101));
	check("cd_top (in reset) completed no frame", cd_frames, n0);
	frame_data(32'h0000123B, 200, 588);
	cd_request(32'h0000123C); settle;
	sec_wr(21'h0C, 32'h0000123C);
	frame_data(32'h0000123C, 0, 588);
	wait_frame(n0);
	check_buf("next frame after the reset", 32'h0000123C, 588);

	$display("-- TEST mode: SUM only, consumer untouched (LW_DIAG=0: TEST ignored, the BEGIN is rejected)");
	n0 = cd_frames; s0 = cd_strobes;
	axi_read(SEC + 21'h14, 0, d0, dl, beats);
	sec_wr(21'h10, 32'h9);
	settle;
	rd("CTRL TEST bit", SEC + 21'h10, DIAG ? 32'h9 : 32'h1);
	sec_wr(21'h0C, 32'h00ABCDEF);
	frame_data(32'h00ABCDEF, 0, 588);
	repeat (8000) @(negedge clk);
	rd("SUM", SEC + 21'h34, dg(frame_sum(32'h00ABCDEF, 588)));
	axi_read(SEC + 21'h14, 0, d, dl, beats);
	check("TEST frame counted", d[31:16] - d0[31:16], dg(1));
	if (!DIAG) check("lean: the BEGIN with no request is rejected", d[3:1], 3'b100);
	check("no strobes reached cd_top", cd_strobes - s0, 0);
	check("no ack reached cd_top", cd_frames, n0);
	sec_wr(21'h10, 32'h1);

	$display("-- CLR counters");
	sec_wr(21'h10, 32'h5);
	rd("COUNTS cleared", SEC + 21'h20, 32'd0);
	rd("DROPS cleared", SEC + 21'h1C, 32'd0);
	axi_read(SEC + 21'h14, 0, d, dl, beats);
	check("frames + high-water cleared", d[31:8], 0);

	$display("-- EN off: the slot goes back to hps_io");
	sec_wr(21'h10, 32'h0); settle;
	rd("STATUS en_sys 0", SEC + 21'h14, DIAG ? 32'd0 : 32'h8);   // lean: still rejecting the TEST-less BEGIN
	n0 = cd_frames;
	axi_read(SEC + 21'h00, 0, d, dl, beats);
	cd_request(32'h00002000); settle;
	check("SPI slot 1 offered", sd_rd[1], 1);
	check("doorbell still rings", doorbell, 1);
	spi_frame(32'h00002000);
	wait_frame(n0);
	check_buf("frame 0x2000 over SPI through the mux", 32'h00002000, 588);

	$display("-- OSDBUF: registers");
	rd("OSD ID OSD1", OSD + 21'h1004, 32'h3144534F);
	rd("OSD CTRL reset", OSD + 21'h1000, 32'd0);
	rd("OSD image reads 0", OSD + 21'h0000, 32'd0);
	rd("OSD image end reads 0", OSD + 21'h0FFC, 32'd0);
	rd("OSD register hole", OSD + 21'h1008, 32'd0);
	rd("OSD block end", OSD + 21'h1FFC, 32'd0);
	check("LWSEL/HR low at reset", osd_ctrl, 0);

	$display("-- OSDBUF: fill both images (LW 1024 stores, SPI 16 rows)");
	for (r = 0; r < 16; r = r + 1) lw_row(r, 0);
	for (r = 0; r < 16; r = r + 1) spi_osd_row(r, 0);
	spi_osd_cmd(8'h40);                   // rows 8-15 re-armed highres; DISABLE clears it
	spi_osd_cmd(8'h41);
	check("osd_status after ENABLE", osd_status, 1);
	bad = 0;
	for (i = 0; i < 1024; i = i + 1) if (lanes_h(i) !== img_w(i) || lanes_v(i) !== img_w(i)) bad = bad + 1;
	check("both shadows hold the LW image", bad, 0);
	bad = 0;
	for (i = 0; i < 4096; i = i + 1) if (hdmi_osd.osd_buffer[i] !== spi_img[i] || vga_osd.osd_buffer[i] !== spi_img[i]) bad = bad + 1;
	check("both SPI buffers hold the SPI image", bad, 0);

	$display("-- OSDBUF: one store at (row 12, col 100) and a byte-lane store at (row 3, col 41)");
	osd_word(12*64 + 25, 32'hC3A55A3C, 4'hF);
	repeat (4) @(negedge clk);
	check("hdmi shadow word (12, 100..103)", lanes_h(12*64 + 25), 32'hC3A55A3C);
	check("vga shadow word (12, 100..103)", lanes_v(12*64 + 25), 32'hC3A55A3C);
	osd_word(3*64 + 10, 32'h0000E700, 4'h2);
	repeat (4) @(negedge clk);
	check("hdmi byte lane 1 only", lanes_h(3*64 + 10), img_w(3*64 + 10));
	check("vga byte lane 1 only", lanes_v(3*64 + 10), img_w(3*64 + 10));
	check("the stored byte", lw_img[3*256 + 41], 32'hE7);
	check("SPI buffer untouched by LW stores", hdmi_osd.osd_buffer[12*256 + 100], spi_img[12*256 + 100]);

	$display("-- OSDBUF: pixel path, SPI image (LWSEL=0, highres 0)");
	osd_check("LWSEL=0", 0, 64, 6);
	axi_write(OSD + 21'h1000, 32'h1, 4'hF);
	rd("CTRL LWSEL", OSD + 21'h1000, 32'h1);
	osd_check("LWSEL=1 HR=0", 1, 64, 3);
	axi_write(OSD + 21'h1000, 32'h3, 4'hF);
	osd_check("LWSEL=1 HR=1: 128 lines", 1, 128, 3);
	axi_write(OSD + 21'h1000, 32'h2, 4'hF);
	rd("CTRL HR only", OSD + 21'h1000, 32'h2);
	osd_check("HR without LWSEL is ignored", 0, 64, 3);

	$display("-- OSDBUF: SPI row and LW row in flight together (LWSEL=1)");
	axi_write(OSD + 21'h1000, 32'h3, 4'hF);
	fork
		spi_osd_row(2, 1);
		lw_row(5, 1);
	join
	bad = 0;
	for (i = 0; i < 256; i = i + 1) if (hdmi_osd.osd_buffer[2*256 + i] !== spi_img[2*256 + i] || vga_osd.osd_buffer[2*256 + i] !== spi_img[2*256 + i]) bad = bad + 1;
	check("SPI row 2 landed in both SPI buffers", bad, 0);
	bad = 0;
	for (i = 5*64; i < 6*64; i = i + 1) if (lanes_h(i) !== img_w(i) || lanes_v(i) !== img_w(i)) bad = bad + 1;
	check("LW row 5 landed in both shadows", bad, 0);
	osd_check("LWSEL=1 shows the shadow", 1, 128, 3);
	axi_write(OSD + 21'h1000, 32'h0, 4'hF);
	osd_check("LWSEL=0 shows the SPI buffer (row 2 new)", 0, 64, 3);

	$display("-- OSDBUF: highres = spi_highres | (LWSEL & HR)");
	spi_osd_row(9, 2);
	osd_check("SPI row 9 (bit 3) re-arms SPI highres", 0, 128, 3);
	spi_osd_cmd(8'h40);
	check("osd_status after DISABLE", osd_status, 0);
	osd_check("OSD disabled: no box", 0, 0, 3);
	axi_write(OSD + 21'h1000, 32'h3, 4'hF);
	spi_osd_cmd(8'h41);
	osd_check("LWSEL&HR survives DISABLE/ENABLE", 1, 128, 6);
	axi_write(OSD + 21'h1000, 32'h0, 4'hF);
	osd_check("SPI highres stays cleared by DISABLE", 0, 64, 3);

	if (!DIAG) begin
		$display("-- LW_DIAG=0: the removed registers read 0");
		rd("lean SCRATCH", 21'h4000C, 32'd0);
		rd("lean analog drops", pad_addr(8'h1A, 0), 32'd0);
		axi_read(SEC + 21'h14, 0, d, dl, beats);
		check("lean STATUS[31:4]", d[31:4], 0);
		for (i = 7; i < 14; i = i + 1) rd("lean SECTOR reg", SEC + 4*i, 32'd0);
		rd("lean ID kept", SEC + 21'h38, 32'h31434553);
	end

	$display("fixed latency: BVALID %0d cycles after AWVALID, RVALID %0d cycles after ARVALID", wlat_ref, rlat_ref);
	$display("%s: %0d checks (%0d functional + %0d fixed-latency, one per transaction), %0d failures",
	         fails ? "FAILED" : "PASSED", checks, checks - lchecks, lchecks, fails);
	$finish;
end

initial begin #200000000 $display("FAILED: timeout"); $finish; end

endmodule
