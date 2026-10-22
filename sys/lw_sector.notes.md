# lw_sector.sv — notes

## The reject path's clock-ratio bound

A BEGIN/WACK the feeder rejects re-opens its request in the clk_100m domain. The feeder
(clk_sys) writes the rejected entry's {slot, tag} (`rjs`, `rjt`) on the edge that rejects it,
and toggles `evt[0]` one edge later (`rjp`). All three cross through the same two-flop
synchroniser (`sb0`/`sb1`), so clk_100m reads the toggle together with the {slot, tag} beside it.

The feeder pops at most one FIFO entry every two clk_sys cycles (`pop` needs `~qv`), so the
earliest next reject rewrites {slot, tag} one clk_sys period after the toggle. **A toggle and its
{slot, tag} therefore coexist for exactly ONE clk_sys period.** clk_100m must sample inside it,
and two toggles one period apart must each be seen as a level change.

- **Bound:** one clk_sys period must exceed about two clk_100m periods (the synchroniser's sampling
  uncertainty plus setup), i.e. clk_sys below roughly 50 MHz to be conservative (the analysis puts the
  real edge near 70-100 MHz). Above it, two adjacent rejects can merge into one toggle (one lost
  re-open: that request stays hidden) or carry the next reject's {slot, tag}.
- **PSX:** clk_sys is `clk_1x`, 33.8688 MHz (29.5 ns, about three clk_100m periods): inside the bound.
  The bench case "adjacent rejects of two slots at PSX clk_sys" queues two stale answers back to
  back and checks both requests re-open.
- **A faster core** reusing this template must hold `F_REJECT` (and so `rjs`/`rjt`) for
  N = ceil(2 x f_sys / 100 MHz) clk_sys cycles before the next pop, or widen the crossing to a
  handshake. No such core exists today; the RTL is unchanged for PSX.
