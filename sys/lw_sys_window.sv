// LW window declared by `SYS_LW_CAPS (emu_ports.vh): self-describing directory, PAD, SECTOR and OSDBUF blocks.
// Bridge side runs on clk_100m with no reset. PAD stores park in per-op latest-wins slots (digital)
// or a 16-deep FIFO (both analog ops, drop-and-count when full), both in one M10K; a toggle handshake
// drains them round-robin to clk_sys. The SECTOR block (lw_sector.sv) serves the CD slot and both
// memory-card slots; entry 1 words 7-12 declare each card slot, its DDR image base (wsrc) and extent.
// OSDBUF decodes image stores and CTRL; the shadow RAMs live in each osd instance (osd.v).
// LW_DIAG=0 (shipping) compiles out SCRATCH (reads 0, writes ignored), the analog-drop count (reads 0)
// and lw_sector's diagnostics; LW_DIAG=1 (rig builds) keeps them.

`ifndef LW_DIAG
`define LW_DIAG 0
`endif

module lw_sys_window
(
	input             clk,          // clk_100m

	input      [20:0] lw_raddr,
	input             lw_ren,
	input             lw_rd_ack,
	input             lw_rd_word,
	output reg [31:0] lw_rdata,

	input      [20:0] lw_waddr,
	input             lw_write,
	input      [31:0] lw_wdata,
	input       [3:0] lw_be,

	output            doorbell,     // SECTOR cause level -> f2h_irq[8]

	input             clk_sys,
	output reg [48:0] lw_pad = 0,   // {we, op[7:0], idx[7:0], data[31:0]} on clk_sys
	input      [34:0] blk_up,       // {reset, hps_ack, req, lba[31:0]} on clk_sys, from emu
	output     [18:0] blk_dn,       // {en_sys, ack, wr, data[15:0]} on clk_sys, to emu
	input      [19:0] card_up,      // {card 2, card 1}, each {hps_ack, wr, rd, lba[6:0]} on clk_sys, from emu
	output     [12:0] card_dn,      // {ack2, en_sys2, ack1, en_sys1, addr[8:0]} on clk_sys, to emu

	output            osd_we,       // OSDBUF image store, clk_100m: word osd_addr of the 256x128 image
	output      [9:0] osd_addr,
	output     [31:0] osd_data,
	output      [3:0] osd_be,
	output      [1:0] osd_ctrl      // {LWSEL & HR, LWSEL}, clk_100m registers; each osd synchronizes them
);

localparam        DIAG     = `LW_DIAG;

localparam [20:0] WIN_OFF  = 21'h40000;
localparam [20:0] WIN_SIZE = 21'h80000;
localparam [20:0] SEC_OFF  = 21'h01000;   // relative to the window
localparam [20:0] SEC_SIZE = 21'h02000;
localparam [20:0] PAD_OFF  = 21'h40000;
localparam [20:0] PAD_SIZE = 21'h40000;
localparam  [7:0] SEC_SLOT  = 8'd1;
localparam  [9:0] SEC_FRAME = 10'd588;
localparam        SEC_AW    = 9;          // FIFO depth 512
localparam [15:0] SEC_GAP   = 16'd2;      // clk_sys cycles between write strobes
localparam  [3:0] MC_SLOT0  = 4'd2;       // memory card 1 (hps_io slot 2) and 2 (slot 3)
localparam  [3:0] MC_SLOT1  = 4'd3;
localparam  [9:0] MC_FRAME  = 10'd256;    // 1 KB blocks
localparam  [7:0] MC_FLAGS  = 8'h03;      // RD: BEGIN_S frames; WR: WACK_S, bytes from the DDR image
localparam [31:0] MC_WSRC0  = 32'h30100000;   // psx_top.vhd arbiter x"01"/x"02" & addr, DDRAM at 0x30000000
localparam [31:0] MC_WSRC1  = 32'h30200000;
localparam [31:0] MC_EXTENT = 32'h00020000;   // 128 blocks (7-bit sd_lba2/3) x 1 KB
localparam [20:0] OSD_OFF  = 21'h04000;
localparam [20:0] OSD_SIZE = 21'h02000;
localparam [12:0] OSD_CTRL = 13'h1000;   // block-relative; the image is 0x0000-0x0FFF
localparam [31:0] OSD_GEOM = {16'd128, 16'd256};
localparam  [5:0] DIR_WORDS = 6'd52;      // header + 3 entries

localparam [255:0] PAD_OPS = (256'd1 << 8'h02) | (256'd1 << 8'h03) |
                             (256'd1 << 8'h10) | (256'd1 << 8'h11) |
                             (256'd1 << 8'h12) | (256'd1 << 8'h13) |
                             (256'd1 << 8'h1A) | (256'd1 << 8'h3D);

reg [31:0] scratch = 0;
reg [31:0] adrop   = 0;               // analog stores dropped on a full FIFO

wire [20:0] rwin = lw_raddr - WIN_OFF;
wire [20:0] wwin = lw_waddr - WIN_OFF;
wire        r_in = (lw_raddr >= WIN_OFF) && (lw_raddr < WIN_OFF + WIN_SIZE);
wire        w_in = (lw_waddr >= WIN_OFF) && (lw_waddr < WIN_OFF + WIN_SIZE);
wire        r_pad = r_in && rwin >= PAD_OFF;
wire        w_pad = w_in && wwin >= PAD_OFF;
wire        r_sec = r_in && rwin >= SEC_OFF && rwin < SEC_OFF + SEC_SIZE;
wire        w_sec = w_in && wwin >= SEC_OFF && wwin < SEC_OFF + SEC_SIZE;
wire        r_osd = r_in && rwin >= OSD_OFF && rwin < OSD_OFF + OSD_SIZE;
wire        w_osd = w_in && wwin >= OSD_OFF && wwin < OSD_OFF + OSD_SIZE;
wire [12:0] rosd  = rwin[12:0];           // OSD_OFF is 8 KB aligned
wire [12:0] wosd  = wwin[12:0];
wire [20:0] wblk = wwin - PAD_OFF;
wire [20:0] rsec = rwin - SEC_OFF;
wire [20:0] wsec = wwin - SEC_OFF;
wire [31:0] sec_rdata;

always @(*) begin
	lw_rdata = 32'hDEADBEEF;
	if (r_pad) lw_rdata = (DIAG && rwin == PAD_OFF + (21'h1A << 10)) ? adrop : 32'd0;
	else if (r_sec) lw_rdata = sec_rdata;
	else if (r_osd) lw_rdata = (rosd == OSD_CTRL)       ? {30'd0, octrl} :
	                           (rosd == OSD_CTRL + 13'd4) ? 32'h3144534F :   // "OSD1"
	                           32'd0;
	else if (r_in && rwin < {DIR_WORDS, 2'b00}) begin
		case (rwin[7:2])
			6'd0:  lw_rdata = 32'h31535953;          // "SYS1"
			6'd1:  lw_rdata = 32'h00010000;
			6'd2:  lw_rdata = 32'd3;
			6'd3:  lw_rdata = DIAG ? scratch : 32'd0;
			6'd4:  lw_rdata = 32'd1;                 // entry 0: kind PAD
			6'd5:  lw_rdata = 32'h00010000;
			6'd6:  lw_rdata = {11'd0, PAD_OFF};
			6'd7:  lw_rdata = {11'd0, PAD_SIZE};
			6'd8:  lw_rdata = PAD_OPS[ 31:  0];
			6'd9:  lw_rdata = PAD_OPS[ 63: 32];
			6'd10: lw_rdata = PAD_OPS[ 95: 64];
			6'd11: lw_rdata = PAD_OPS[127: 96];
			6'd12: lw_rdata = PAD_OPS[159:128];
			6'd13: lw_rdata = PAD_OPS[191:160];
			6'd14: lw_rdata = PAD_OPS[223:192];
			6'd15: lw_rdata = PAD_OPS[255:224];
			6'd20: lw_rdata = 32'd2;                 // entry 1: kind SECTOR
			6'd21: lw_rdata = 32'h00010001;                 // minor 1: save slots (words 7-12)
			6'd22: lw_rdata = {11'd0, SEC_OFF};
			6'd23: lw_rdata = {11'd0, SEC_SIZE};
			6'd24: lw_rdata = {SEC_SLOT, 8'd0, 6'd0, SEC_FRAME};
			6'd25: lw_rdata = {11'd0, WIN_OFF + SEC_OFF};   // CAUSE, absolute LW (= the IRQ:BLK offset)
			6'd26: lw_rdata = {16'd1 << SEC_AW, SEC_GAP};
			6'd27: lw_rdata = {4'd0, MC_SLOT0, MC_FLAGS, 6'd0, MC_FRAME};
			6'd28: lw_rdata = MC_WSRC0;
			6'd29: lw_rdata = {4'd0, MC_SLOT1, MC_FLAGS, 6'd0, MC_FRAME};
			6'd30: lw_rdata = MC_WSRC1;
			6'd31: lw_rdata = MC_EXTENT;
			6'd32: lw_rdata = MC_EXTENT;
			6'd36: lw_rdata = 32'd3;                 // entry 2: kind OSDBUF
			6'd37: lw_rdata = 32'h00010000;
			6'd38: lw_rdata = {11'd0, OSD_OFF};
			6'd39: lw_rdata = {11'd0, OSD_SIZE};
			6'd40: lw_rdata = OSD_GEOM;              // {height, width} in pixels, 1 bit each
			default: lw_rdata = 32'd0;
		endcase
	end
end

lw_sector #(.SLOT(SEC_SLOT), .FRAME(SEC_FRAME), .AW(SEC_AW), .DIAG(DIAG),
            .XSLOT0(MC_SLOT0), .XSLOT1(MC_SLOT1), .XFRAME(MC_FRAME)) sector
(
	.clk(clk),
	.raddr(rsec[12:0]), .rsel(r_sec), .ren(lw_ren), .rd_ack(lw_rd_ack), .rd_word(lw_rd_word), .rdata(sec_rdata),
	.waddr(wsec[12:0]), .wsel(w_sec), .write(lw_write), .wdata(lw_wdata), .be(lw_be),
	.doorbell(doorbell),
	.clk_sys(clk_sys),
	.c_reset(blk_up[34]), .hps_ack(blk_up[33]), .c_req(blk_up[32]), .c_lba(blk_up[31:0]),
	.en_sys(blk_dn[18]), .o_ack(blk_dn[17]), .o_wr(blk_dn[16]), .o_dat(blk_dn[15:0]),
	.o_adr(card_dn[8:0]),
	.x_hps_ack({card_up[19], card_up[9]}), .x_wr({card_up[18], card_up[8]}), .x_rd({card_up[17], card_up[7]}),
	.x_lba({card_up[16:10], card_up[6:0]}),
	.x_en_sys({card_dn[11], card_dn[9]}), .x_ack({card_dn[12], card_dn[10]})
);

// OSDBUF: image stores go to both osd shadows (byte lanes honour BE); CTRL is {HR, LWSEL}.
reg  [1:0] octrl = 0;
reg        ohr   = 0;

assign osd_we   = lw_write && w_osd && ~wosd[12];
assign osd_addr = wosd[11:2];
assign osd_data = lw_wdata;
assign osd_be   = lw_be;
assign osd_ctrl = {ohr, octrl[0]};

always @(posedge clk) if (lw_write && w_osd && wosd == OSD_CTRL && lw_be[0]) begin
	octrl <= lw_wdata[1:0];
	ohr   <= &lw_wdata[1:0];
end

// clk_100m side: SCRATCH, the pending slots, and the round-robin drain into the handshake.
function automatic [2:0] slot_of(input [7:0] op);
	case (op)
		8'h02: slot_of = 3'd0;
		8'h03: slot_of = 3'd1;
		8'h10: slot_of = 3'd2;
		8'h11: slot_of = 3'd3;
		8'h12: slot_of = 3'd4;
		8'h13: slot_of = 3'd5;
		default: slot_of = 3'd7;
	endcase
endfunction

function automatic [7:0] op_of(input [2:0] slot);
	case (slot)
		3'd0: op_of = 8'h02;
		3'd1: op_of = 8'h03;
		3'd2: op_of = 8'h10;
		3'd3: op_of = 8'h11;
		3'd4: op_of = 8'h12;
		default: op_of = 8'h13;
	endcase
endfunction

// one M10K: digital masks at 0..5, the analog FIFO {right, idx, data[15:0]} at 16..31
(* ramstyle = "M10K" *) reg [31:0] pram [0:31];
reg [31:0] pq;
reg  [5:0] dirty = 0;

reg  [4:0] awp = 0, arp = 0;
wire       aempty = (awp == arp);
wire       afull  = (awp - arp) == 5'd16;

wire       st   = lw_write && w_pad && &lw_be;
wire [7:0] wop  = wblk[17:10];
wire [7:0] widx = wblk[9:2];
wire [2:0] wsl  = slot_of(wop);
wire       st_d = st && wsl != 3'd7 && widx == 8'd0;
wire       st_a = st && (wop == 8'h1A || wop == 8'h3D) && !st_d;

reg        req_t = 0;
reg        ack_t = 0;
reg  [1:0] ack_s = 0;
reg [47:0] hold  = 0;
wire       busy  = req_t ^ ack_s[1];

wire [6:0] ready = {~aempty, dirty};
reg  [2:0] rr = 0;
reg  [2:0] pick;
reg        pick_v;
reg  [1:0] ds = 0;                     // drain: 0 pick, 1 read, 2 launch
reg  [2:0] psel = 0;
integer    i, k;

always @(*) begin
	pick_v = 0;
	pick   = 0;
	for (i = 0; i < 7; i = i + 1) begin
		k = rr + i;
		if (k >= 7) k = k - 7;
		if (!pick_v && ready[k]) begin
			pick_v = 1;
			pick   = k[2:0];
		end
	end
end

wire [4:0] pra = (psel == 3'd6) ? {1'b1, arp[3:0]} : {2'b00, psel};

always @(posedge clk) begin
	if (st_d) pram[{2'b00, wsl}] <= lw_wdata;
	else if (st_a && !afull) pram[{1'b1, awp[3:0]}] <= {7'd0, wop == 8'h3D, widx, lw_wdata[15:0]};
	pq <= pram[pra];
end

always @(posedge clk) begin
	ack_s <= {ack_s[0], ack_t};

	if (DIAG && lw_write && w_in && wwin == 21'h0C) begin
		if (lw_be[0]) scratch[ 7: 0] <= lw_wdata[ 7: 0];
		if (lw_be[1]) scratch[15: 8] <= lw_wdata[15: 8];
		if (lw_be[2]) scratch[23:16] <= lw_wdata[23:16];
		if (lw_be[3]) scratch[31:24] <= lw_wdata[31:24];
	end

	case (ds)
		2'd0: if (!busy && pick_v) begin
			psel <= pick;
			rr   <= (pick == 3'd6) ? 3'd0 : pick + 3'd1;
			ds   <= 2'd1;
		end
		2'd1: begin
			if (psel == 3'd6) arp <= arp + 5'd1;
			else dirty[psel] <= 0;
			ds <= 2'd2;
		end
		default: begin
			hold  <= (psel == 3'd6) ? {pq[24] ? 8'h3D : 8'h1A, pq[23:16], 16'd0, pq[15:0]} : {op_of(psel), 8'd0, pq};
			req_t <= ~req_t;
			ds    <= 2'd0;
		end
	endcase

	// after the drain, so a store landing in the read cycle stays dirty (its RAM read returned the old word)
	if (st_d) dirty[wsl] <= 1;
	else if (st_a) begin
		if (afull) begin
			if (DIAG) adrop <= adrop + 32'd1;
		end
		else awp <= awp + 5'd1;
	end
end

// clk_sys side: hold is stable for >=2 clk_sys cycles when the toggle edge arrives.
reg  [2:0] req_s = 0;

always @(posedge clk_sys) begin
	req_s  <= {req_s[1:0], req_t};
	ack_t  <= req_s[2];
	lw_pad <= {req_s[2] ^ req_s[1], hold};
end

endmodule
