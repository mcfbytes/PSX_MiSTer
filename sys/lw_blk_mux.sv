// emu side of the LW SECTOR route (lw_sector.sv): one hps_io read slot's consumer is fed either by hps_io
// or, while sys_top's en_sys is set, by the LW feeder. The consumer's ack/write/data are muxed here, never
// the shared sd_buff_wr, so an SPI transfer on another slot cannot land in an LW frame. With en_sys set the
// slot's request is hidden from hps_io, so the slot has one writer. All signals are on hps_io's clk_sys.

module lw_blk_mux
(
	inout  [148:0] HPS_BUS,

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
assign core_wr   = en ? HPS_BUS[111]     : hps_wr;
assign core_data = en ? HPS_BUS[110:95]  : hps_data;

endmodule
