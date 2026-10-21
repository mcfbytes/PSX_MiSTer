// LW window declared by `SYS_LW_CAPS (emu_ports.vh): self-describing directory + PAD block.
// Bridge side runs on clk_100m with no reset. PAD stores park in per-op latest-wins slots (digital)
// or a 16-deep FIFO (both analog ops, drop-and-count when full); a toggle handshake drains them round-robin to clk_sys.

module lw_sys_window
(
	input             clk,          // clk_100m

	input      [20:0] lw_raddr,
	output reg [31:0] lw_rdata,

	input      [20:0] lw_waddr,
	input             lw_write,
	input      [31:0] lw_wdata,
	input       [3:0] lw_be,

	input             clk_sys,
	output reg [48:0] lw_pad = 0    // {we, op[7:0], idx[7:0], data[31:0]} on clk_sys
);

localparam [20:0] WIN_OFF  = 21'h40000;
localparam [20:0] WIN_SIZE = 21'h80000;
localparam [20:0] PAD_OFF  = 21'h40000;   // relative to the window
localparam [20:0] PAD_SIZE = 21'h40000;

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
wire [20:0] wblk = wwin - PAD_OFF;

always @(*) begin
	lw_rdata = 32'hDEADBEEF;
	if (r_pad) lw_rdata = (rwin == PAD_OFF + (21'h1A << 10)) ? adrop : 32'd0;
	else if (r_in && rwin < 21'h50) begin
		case (rwin[6:2])
			5'd0:  lw_rdata = 32'h31535953;          // "SYS1"
			5'd1:  lw_rdata = 32'h00010000;
			5'd2:  lw_rdata = 32'd1;
			5'd3:  lw_rdata = scratch;
			5'd4:  lw_rdata = 32'd1;                 // kind PAD
			5'd5:  lw_rdata = 32'h00010000;
			5'd6:  lw_rdata = {11'd0, PAD_OFF};
			5'd7:  lw_rdata = {11'd0, PAD_SIZE};
			5'd8:  lw_rdata = PAD_OPS[ 31:  0];
			5'd9:  lw_rdata = PAD_OPS[ 63: 32];
			5'd10: lw_rdata = PAD_OPS[ 95: 64];
			5'd11: lw_rdata = PAD_OPS[127: 96];
			5'd12: lw_rdata = PAD_OPS[159:128];
			5'd13: lw_rdata = PAD_OPS[191:160];
			5'd14: lw_rdata = PAD_OPS[223:192];
			5'd15: lw_rdata = PAD_OPS[255:224];
			default: lw_rdata = 32'd0;
		endcase
	end
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

reg [31:0] mask [0:5];
reg  [5:0] dirty = 0;

reg [24:0] afifo [0:15];                // {right, idx, data[15:0]}
reg  [4:0] awp = 0, arp = 0;
wire       aempty = (awp == arp);
wire       afull  = (awp - arp) == 5'd16;

wire       st   = lw_write && w_pad && &lw_be;
wire [7:0] wop  = wblk[17:10];
wire [7:0] widx = wblk[9:2];
wire [2:0] wsl  = slot_of(wop);

reg        req_t = 0;
reg        ack_t = 0;
reg  [1:0] ack_s = 0;
reg [47:0] hold  = 0;
wire       busy  = req_t ^ ack_s[1];

wire [6:0] ready = {~aempty, dirty};
reg  [2:0] rr = 0;
reg  [2:0] pick;
reg        pick_v;
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

always @(posedge clk) begin
	ack_s <= {ack_s[0], ack_t};

	if (lw_write && w_in && wwin == 21'h0C) begin
		if (lw_be[0]) scratch[ 7: 0] <= lw_wdata[ 7: 0];
		if (lw_be[1]) scratch[15: 8] <= lw_wdata[15: 8];
		if (lw_be[2]) scratch[23:16] <= lw_wdata[23:16];
		if (lw_be[3]) scratch[31:24] <= lw_wdata[31:24];
	end

	if (!busy && pick_v) begin
		req_t <= ~req_t;
		rr    <= (pick == 3'd6) ? 3'd0 : pick + 3'd1;
		if (pick == 3'd6) begin
			hold <= {afifo[arp[3:0]][24] ? 8'h3D : 8'h1A, afifo[arp[3:0]][23:16], 16'd0, afifo[arp[3:0]][15:0]};
			arp  <= arp + 5'd1;
		end
		else begin
			hold        <= {op_of(pick), 8'd0, mask[pick]};
			dirty[pick] <= 0;
		end
	end

	// after the drain, so a store landing in the launch cycle stays dirty
	if (st) begin
		if (wsl != 3'd7 && widx == 8'd0) begin
			mask[wsl]  <= lw_wdata;
			dirty[wsl] <= 1;
		end
		else if (wop == 8'h1A || wop == 8'h3D) begin
			if (afull) adrop <= adrop + 32'd1;
			else begin
				afifo[awp[3:0]] <= {wop == 8'h3D, widx, lw_wdata[15:0]};
				awp <= awp + 5'd1;
			end
		end
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
