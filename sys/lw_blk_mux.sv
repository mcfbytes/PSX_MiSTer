// emu side of the LW SECTOR route (lw_sector.sv): one hps_io read slot's consumer is fed either by hps_io
// or, while sys_top's en_sys is set, by the LW feeder. The consumer's ack/write/data are muxed here, never
// the shared sd_buff_wr, so an SPI transfer on another slot cannot land in an LW frame. With en_sys set the
// slot's request is hidden from hps_io, so the slot has one writer. All signals are on hps_io's clk_sys.

module lw_blk_mux
(
	inout  [313:0] HPS_BUS,

	input          reset,       // the consumer's reset: zero-fills an open LW frame
	input          core_req,
	input   [31:0] core_lba,
	output         core_ack,
	output         core_wr,
	output  [15:0] core_data,

	output         hps_req,     // -> hps_io sd_rd[slot]
	input          hps_ack,     // <- hps_io sd_ack[slot]
	input          hps_wr,      // <- hps_io sd_buff_wr
	input   [15:0] hps_data     // <- hps_io sd_buff_dout
);

wire en = HPS_BUS[113];

assign HPS_BUS[148:114] = {reset, hps_ack, core_req, core_lba};

assign hps_req   = core_req & ~en;
assign core_ack  = en ? HPS_BUS[112]     : hps_ack;
assign core_wr   = en ? HPS_BUS[111] & HPS_BUS[112] : hps_wr;   // never a card frame's strobe
assign core_data = en ? HPS_BUS[110:95]  : hps_data;

endmodule

// emu side of a memory-card slot on the LW SECTOR route: CARD 0 = hps_io slot 2 (HPS_BUS[169:158]),
// CARD 1 = slot 3 ([181:170]); the strobe address [157:149] is shared. While the slot's en_sys is set its
// rd/wr are hidden from hps_io and the card's ack/write/addr/data come from the feeder, the write gated by
// the card's own ack. memcard_dataOut keeps feeding hps_io (the SPI save path). All on hps_io's clk_sys.

module lw_card_mux #(parameter CARD = 0)
(
	inout  [313:0] HPS_BUS,

	input          core_rd,
	input          core_wr,
	input    [6:0] core_lba,
	output         core_ack,
	output         core_write,
	output   [8:0] core_addr,
	output  [15:0] core_data,

	output         hps_rd,      // -> hps_io sd_rd[slot]
	output         hps_wr,      // -> hps_io sd_wr[slot]
	input          hps_ack,     // <- hps_io sd_ack[slot]
	input          hps_write,   // <- hps_io sd_buff_wr
	input    [8:0] hps_addr,    // <- hps_io sd_buff_addr
	input   [15:0] hps_data     // <- hps_io sd_buff_dout
);

localparam B = CARD ? 170 : 158;

wire en  = HPS_BUS[B];
wire ack = HPS_BUS[B + 1];

assign HPS_BUS[B + 11:B + 2] = {hps_ack, core_wr, core_rd, core_lba};

assign hps_rd     = core_rd & ~en;
assign hps_wr     = core_wr & ~en;
assign core_ack   = en ? ack                  : hps_ack;
assign core_write = en ? HPS_BUS[111] & ack   : hps_write;
assign core_addr  = en ? HPS_BUS[157:149]     : hps_addr;
assign core_data  = en ? HPS_BUS[110:95]      : hps_data;

endmodule
