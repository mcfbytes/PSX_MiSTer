#!/bin/sh
# LW window testbench, run three times: LW_DIAG=1 (rig build), LW_DIAG=0 (shipping), and MENU_CORE (the
# osd header-row gate only).
# Needs Icarus >= 12 (11 cannot parse hps_io's default-less CONF_STR); else the r513-vl:local image.
cd "$(dirname "$0")/.." && C='rc=0; for g in 1 0 M; do
  if [ $g = M ]; then D="-DLW_DIAG=0 -DMENU_CORE"; else D=-DLW_DIAG=$g; fi
  echo "== $D"
  iverilog -g2012 $D -I sys -o /tmp/lw_tb$g.vvp tb/lw_tb.sv tb/lw_stub_atom.sv sys/hps_lw_bridge.sv sys/lw_sys_window.sv sys/lw_sector.sv sys/lw_blk_mux.sv sys/hps_io.sv sys/osd.v 2>/tmp/lw_tb$g.err || { cat /tmp/lw_tb$g.err; exit 1; }
  vvp -n /tmp/lw_tb$g.vvp | tee /tmp/lw_tb$g.log | grep -v "^pass "; grep -q "^PASSED" /tmp/lw_tb$g.log || rc=1
done; exit $rc'
if iverilog -V 2>/dev/null | grep -q 'version 1[2-9]'; then sh -c "$C"; else docker run --rm -u "$(id -u):$(id -g)" -v "$PWD:/w" -w /w r513-vl:local sh -c "$C"; fi
