#!/bin/sh
# LW window testbench. Needs Icarus >= 12 (11 cannot parse hps_io's default-less CONF_STR); else the r513-vl:local image.
cd "$(dirname "$0")/.." && C='iverilog -g2012 -I sys -o /tmp/lw_tb.vvp tb/lw_tb.sv tb/lw_stub_atom.sv sys/hps_lw_bridge.sv sys/lw_sys_window.sv sys/hps_io.sv 2>/tmp/lw_tb.err || { cat /tmp/lw_tb.err; exit 1; }; vvp -n /tmp/lw_tb.vvp'
if iverilog -V 2>/dev/null | grep -q 'version 1[2-9]'; then sh -c "$C"; else docker run --rm -u "$(id -u):$(id -g)" -v "$PWD:/w" -w /w r513-vl:local sh -c "$C"; fi
