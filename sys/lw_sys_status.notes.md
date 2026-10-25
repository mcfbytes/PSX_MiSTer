# lw_sys_status.sv — notes

## What it carries

hps_io's status back-channel (`0x29`) without the SPI poll. hps_io keeps its own `stflg` and
`status_req` (now at module scope, exported on `HPS_BUS[313:182]`); the core's RTL is untouched and
the SPI `0x29` arm answers exactly as before. Directory entry 3 (kind 4, `SYS1` window) declares the
block, the line (pool ordinal 2 = `f2h_irq[9]`, GIC SPI 49), the CAUSE word's absolute LW offset, the
class tag (`0x0010_0000`, bit 20 = SYS) and the payload bits it raises (bit 0 = STATUS).

| offset | register | |
|---|---|---|
| 0x00 | CAUSE | `{8'h00, 8'h10, payload}` while pending, else 0; a word-exact read clears it |
| 0x04 | STFLG | `{SEQ[15:0], 8'h00, 4'hA, flag}`: the `0x29` head byte, plus the capture count |
| 0x08-0x14 | STATUS[0..3] | the snapshot, `status_req[31:0]` first |
| 0x1C | ID | `"STS1"` |

The read is a seqlock: STFLG, the four words, STFLG. Equal STFLG words mean the words between them
are one capture; unequal, read again. SEQ makes the comparison immune to the 4-bit flag wrapping.

## The crossing and its clock-ratio bound

- **clk_sys side.** `st_g` is the flag's Gray code, REGISTERED, so it moves one clock after
  `stflg`/`status_req`, and one bit per change.
- **clk_100m side.** `st_g` passes a two-flop synchroniser; a decoded flag that differs from the
  snapshot's captures `status_req` (quasi-static: stable since before the Gray code moved), bumps SEQ
  and raises STATUS. The compare is a level, not an edge, so changes that arrive faster than they are
  seen coalesce into one capture of the latest; none is lost.
- **A torn capture is always superseded.** If a second request lands while a capture samples
  `status_req`, its own Gray step follows one clk_sys later and forces another capture about
  `1 x T_sys + 3 x T_100` after it. A reader's six reads span microseconds, so the torn snapshot
  can never sit between two equal STFLG reads.
- **Bound:** hps_io moves `stflg` at most once per two clk_sys cycles (a rising edge of
  `status_set`), and a Gray code is safe to sample while consecutive steps are at least one
  destination period plus setup apart: `2 x T_sys > T_100 + margin`, i.e. clk_sys below about
  100 MHz with margin to spare at 50 MHz. **PSX:** clk_sys is `clk_1x`, 33.8688 MHz (59 ns between
  steps, six clk_100m periods). A core above the bound must widen the crossing to a handshake.
- **CAUSE clear.** The fetch records the raise count; the word-exact ack clears only if no capture
  came between them, and a raise in the same cycle as the clear wins. Main reads the snapshot after
  CAUSE in any case; the kept bit makes the level honest too: the line stays up for a capture the
  read did not return. The bench sweeps 64 read offsets around a capture and checks that level;
  dropping either guard turns it RED.
