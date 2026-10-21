// SECTOR block (LW directory kind 2): one hps_io read slot served over the LW bridge.
// clk_100m side: the register map, the BLK doorbell cause and the store path into a dual-clock FIFO.
// clk_sys side: the feeder that re-creates hps_io's ack + 16-bit write strobes for the slot's consumer.
// Bridge side has no reset and answers in fixed cycles (P2); a full FIFO drops the store and counts it.
// DIAG=0 (LW_DIAG=0, shipping) compiles out STATUS[31:4], DROPS, COUNTS, GEN, TSTAMP_*, CYCLES, SUM and TEST:
// they read 0 and CTRL.TEST/CLR are ignored. CAUSE, REQ, LBA, BEGIN, CTRL.EN/ABORT, STATUS[3:0], ACCEPT, ID stay.

module lw_sector #(parameter [7:0] SLOT = 1, parameter [9:0] FRAME = 588, parameter [2:0] BLKSZ = 3, parameter AW = 9,
                   parameter DIAG = 1)
(
	input             clk,          // clk_100m

	input      [12:0] raddr,        // block-relative
	input             rsel,
	input             ren,
	input             rd_ack,
	input             rd_word,
	output reg [31:0] rdata,
	input      [12:0] waddr,
	input             wsel,
	input             write,
	input      [31:0] wdata,
	input       [3:0] be,

	output            doorbell,     // level, high while CAUSE != 0

	input             clk_sys,
	input             c_reset,      // consumer reset (zero-fills an open frame)
	input             c_req,
	input      [31:0] c_lba,
	input             hps_ack,      // the SPI path's ack for this slot
	output reg        en_sys = 0,   // mux state: 1 = this block owns the consumer's ack/write/data
	output reg        o_ack  = 0,
	output reg        o_wr   = 0,
	output reg [15:0] o_dat  = 0
);

localparam [15:0] SDSTAT = {1'b1, 6'd0, BLKSZ, SLOT[3:0], 1'b0, 1'b1};
localparam [1:0]  T_DATA = 2'd0, T_BEGIN = 2'd1, T_ABORT = 2'd2;

`define SYNC (* altera_attribute = {"-name SYNCHRONIZER_IDENTIFICATION FORCED_IF_ASYNCHRONOUS"} *)

/////////////////////////////// FIFO (M10K) ///////////////////////////////

localparam DEPTH = 1 << AW;

(* ramstyle = "M10K" *) reg [43:0] fram [0:DEPTH-1];

reg  [AW:0] wptr = 0, wgray = 0;
reg  [AW:0] rptr = 0, rgray = 0;
`SYNC reg [AW:0] rg_s0 = 0;
`SYNC reg [AW:0] rg_s1 = 0;
`SYNC reg [AW:0] wg_s0 = 0;
`SYNC reg [AW:0] wg_s1 = 0;

function automatic [AW:0] g2b(input [AW:0] g);
	integer n;
	begin
		g2b[AW] = g[AW];
		for (n = AW - 1; n >= 0; n = n - 1) g2b[n] = g2b[n + 1] ^ g[n];
	end
endfunction

wire [AW:0] used  = wptr - g2b(rg_s1);
wire        ffull = used[AW];

/////////////////////////////// clk_100m //////////////////////////////////

localparam F_IDLE = 3'd0, F_ACK = 3'd1, F_STREAM = 3'd2, F_FILL = 3'd3, F_REJECT = 3'd4;

reg   [2:0] fs = F_IDLE;
reg   [9:0] fexp = 0;
reg         em = 0, qv = 0, tmode = 0;
reg   [1:0] ph = 0, ackc = 0;
reg  [15:0] hi = 0;
reg  [43:0] q;
reg  [19:0] fsum = 0, sum_l = 0;
reg   [4:0] evt = 0;                // {done, reset_fill, abort, skip, reject} toggles

reg         pending = 0;
reg  [31:0] gen = 0, tsreq = 0, tsdone = 0, cycles = 0;
reg   [3:0] snap = 0;
reg  [31:0] lba = 0;
reg         en = 0, test = 0;
reg   [9:0] exp_w = 0, fdrops = 0;
reg         bfail = 0;
reg  [15:0] drops = 0, frames = 0;
reg   [7:0] hw = 0;
reg   [7:0] n_rej = 0, n_skip = 0, n_abort = 0, n_rfill = 0;
reg  [19:0] sum = 0;

// clk_sys -> clk_100m: request level, status bits, event toggles
wire  [9:0] sys_bits;
`SYNC reg [9:0] sb0 = 0;
`SYNC reg [9:0] sb1 = 0;
reg   [9:0] sb2 = 0;
wire        req_s = sb1[0];
wire        req_rise = sb1[0] & ~sb2[0];
wire  [3:0] st_s = sb1[4:1];          // {rejecting, streaming, ack, en_sys}
wire  [4:0] ev   = sb1[9:5] ^ sb2[9:5]; // {done, reset_fill, abort, skip, reject}

wire        rreg  = rsel & ~raddr[12] & ~|raddr[11:6];
wire        wreg  = wsel & write & ~waddr[12] & ~|waddr[11:6] & &be;
wire        wdat  = wsel & write &  waddr[12] & &be & (waddr[11:2] < FRAME);
wire        w_cause = wreg && waddr[5:2] == 4'h0;
wire        w_begin = wreg && waddr[5:2] == 4'h3;
wire        w_ctrl  = wreg && waddr[5:2] == 4'h4;
wire        w_abort = w_ctrl & wdata[1];
wire        clr     = w_ctrl & wdata[2];
wire        rd_cause = rreg && raddr[5:2] == 4'h0 && rd_word;

wire        fire  = req_rise;
wire        clr_c = (rd_cause & rd_ack & (gen[3:0] == snap)) | w_cause;
wire        push  = w_begin | w_abort | wdat;
wire        fwe   = push & ~ffull;
wire [43:0] entry = w_begin ? {T_BEGIN, 10'd0, wdata} : w_abort ? {T_ABORT, 42'd0} : {T_DATA, waddr[11:2], wdata};

always @(posedge clk) if (fwe) fram[wptr[AW-1:0]] <= entry;

function automatic [7:0] sat8(input [7:0] v, input inc);
	sat8 = (inc && ~&v) ? v + 8'd1 : v;
endfunction

always @(posedge clk) begin
	if (DIAG) cycles <= cycles + 32'd1;
	{sb2, sb1, sb0} <= {sb1, sb0, sys_bits};
	{rg_s1, rg_s0}  <= {rg_s0, rgray};

	// doorbell: the request rise raises, a raise beats a concurrent clear
	gen     <= gen + fire;
	pending <= fire | (pending & ~clr_c);
	if (fire) lba <= c_lba;           // stable while the request is open
	if (DIAG && fire) tsreq <= cycles;
	if (rd_cause & ren) snap <= gen[3:0];

	if (DIAG && ev[4]) begin
		tsdone <= cycles;
		sum    <= sum_l;
	end

	if (w_ctrl) begin
		en   <= wdata[0];
		test <= DIAG ? wdata[3] : 1'b0;
	end

	// store path: never back-pressured, a full FIFO drops and counts
	if (push & ffull) begin
		if (DIAG && ~&drops) drops <= drops + 16'd1;
		if (~&fdrops) fdrops <= fdrops + 10'd1;
	end
	if (fwe) wptr <= wptr + 1'd1;
	wgray <= wptr ^ (wptr >> 1);

	if (w_begin) begin
		exp_w  <= 0;
		bfail  <= ffull;
		fdrops <= ffull ? 10'd1 : 10'd0;
	end
	else if (wdat && !ffull && waddr[11:2] == exp_w) exp_w <= exp_w + 10'd1;

	if (DIAG) begin
		if (clr) begin
			drops   <= 0;
			frames  <= 0;
			hw      <= 0;
			{n_rej, n_skip, n_abort, n_rfill} <= 0;
		end
		else begin
			if (used > hw) hw <= (used > 255) ? 8'd255 : used[7:0];
			frames  <= frames + ev[4];
			n_rej   <= sat8(n_rej,   ev[0]);
			n_skip  <= sat8(n_skip,  ev[1]);
			n_abort <= sat8(n_abort, ev[2]);
			n_rfill <= sat8(n_rfill, ev[3]);
		end
	end
end

assign doorbell = pending;

wire ok = (exp_w == FRAME) & ~bfail;

always @(*) begin
	rdata = 32'd0;
	if (rreg) case (raddr[5:2])
		4'h0: rdata = pending ? {12'd0, 4'b0001, SDSTAT} : 32'd0;
		4'h1: rdata = req_s ? {1'b1, 15'd0, SDSTAT} : 32'd0;
		4'h2: rdata = lba;
		4'h4: rdata = {28'd0, test, 2'b00, en};
		4'h5: rdata = DIAG ? {frames, hw, 4'd0, st_s} : {28'd0, st_s};
		4'h6: rdata = {ok, 5'd0, fdrops, 6'd0, exp_w};
		4'h7: rdata = DIAG ? {16'd0, drops} : 32'd0;
		4'h8: rdata = DIAG ? {n_rej, n_skip, n_abort, n_rfill} : 32'd0;
		4'h9: rdata = DIAG ? gen : 32'd0;
		4'hA: rdata = DIAG ? tsreq : 32'd0;
		4'hB: rdata = DIAG ? tsdone : 32'd0;
		4'hC: rdata = DIAG ? cycles : 32'd0;
		4'hD: rdata = DIAG ? {12'd0, sum} : 32'd0;
		4'hE: rdata = 32'h31434553;     // "SEC1"
		default: rdata = 32'd0;
	endcase
end

/////////////////////////////// clk_sys: feeder ///////////////////////////

`SYNC reg [1:0] cs0 = 0;
`SYNC reg [1:0] cs1 = 0;
wire en_s = cs1[0], test_s = cs1[1];

wire       rempty = (rgray == wg_s1);
wire       pop    = ~rempty & ~qv & (fs != F_FILL) & (fs != F_ACK) & (~em | ph == 2'd3);
wire [1:0] qt = q[43:42];
wire [9:0] qi = q[41:32];
wire [31:0] qd = q[31:0];

always @(posedge clk_sys) q <= fram[rptr[AW-1:0]];

assign sys_bits = {DIAG ? evt : 5'd0, fs == F_REJECT, fs == F_STREAM || fs == F_FILL || fs == F_ACK, o_ack, en_sys, c_req};

always @(posedge clk_sys) begin
	{cs1, cs0}     <= {cs0, test, en};
	{wg_s1, wg_s0} <= {wg_s0, wgray};

	qv <= pop;
	if (pop) rptr <= rptr + 1'd1;
	rgray <= rptr ^ (rptr >> 1);

	o_wr <= 0;

	if ((fs == F_IDLE || fs == F_REJECT) && !hps_ack) en_sys <= en_s;

	if (fs == F_ACK) begin
		ackc <= ackc - 1'd1;
		if (ackc == 1) fs <= F_STREAM;
	end

	if (fs == F_FILL && !em) begin
		o_wr  <= ~tmode;
		o_dat <= 0;
		hi    <= 0;
		em    <= 1;
		ph    <= 1;
	end

	if (qv) case (fs)
		F_IDLE, F_REJECT:
			if (qt == T_BEGIN) begin
				fexp <= 0;
				if (DIAG) fsum <= 0;
				if (test_s) begin
					tmode <= 1;
					fs    <= F_STREAM;
				end
				else if (en_sys && c_req && qd == c_lba) begin
					tmode <= 0;
					o_ack <= 1;
					ackc  <= 2;
					fs    <= F_ACK;
				end
				else begin
					fs     <= F_REJECT;
					evt[0] <= ~evt[0];
				end
			end
			else if (qt == T_ABORT) fs <= F_IDLE;

		F_STREAM:
			if (qt != T_DATA) begin
				fs     <= F_FILL;
				evt[2] <= ~evt[2];
			end
			else if (qi == fexp) begin
				o_wr  <= ~tmode;
				o_dat <= qd[15:0];
				hi    <= qd[31:16];
				em    <= 1;
				ph    <= 1;
				if (DIAG) fsum <= fsum + qd[31:24] + qd[23:16] + qd[15:8] + qd[7:0];
			end
			else evt[1] <= ~evt[1];

		default: ;
	endcase

	if (em) case (ph)
		2'd1: ph <= 2;
		2'd2: begin
			o_wr  <= ~tmode;
			o_dat <= hi;
			ph    <= 3;
		end
		2'd3: begin
			em <= 0;
			ph <= 0;
			if (fexp == FRAME - 1'd1) begin
				fexp   <= 0;
				o_ack  <= 0;
				fs     <= F_IDLE;
				if (DIAG) sum_l <= fsum;
				evt[4] <= ~evt[4];
			end
			else fexp <= fexp + 1'd1;
		end
		default: ;
	endcase

	if (c_reset) begin
		if (fs == F_STREAM || fs == F_ACK) begin
			fs     <= F_FILL;
			evt[3] <= ~evt[3];
		end
		else if (fs == F_REJECT) fs <= F_IDLE;
	end
end

`undef SYNC

endmodule
