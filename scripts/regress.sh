#!/bin/sh
# regress.sh -- full test-suite runner (module benches + core + firmware suite).
#   usage: scripts/regress.sh [name ...]     (default: all)
# Names: mmu l1_i l1_d wb_ic 16550 clint plic core c8 soc t01 t02 t03 t04 t05
set -e
cd "$(dirname "$0")/.."
RTL="rtl/soc.v rtl/core.v rtl/mmu.v rtl/l1_i.v rtl/l1_d.v rtl/wb_ic.v rtl/wb_ic_mfifo.v rtl/wb_rom.v rtl/wb_ram.v rtl/16550.v rtl/clint.v rtl/plic.v"
CORE_RTL="rtl/core.v rtl/mmu.v rtl/l1_i.v rtl/l1_d.v"
OUT=${OUT:-/tmp/opencode/regress}
mkdir -p "$OUT"
TARGETS="$*"
[ -z "$TARGETS" ] && TARGETS="mmu l1_i l1_d wb_ic 16550 clint plic core c8 t01 t02 t03 t04 t05 soc"
FAIL=0

run() { # name build-cmd [vvp plusargs...]
    n=$1; shift
    build=$1; shift
    PA="$*"
    echo "== $n"
    # shellcheck disable=SC2068
    sh -c "$build" > "$OUT/$n.build.log" 2>&1 || { echo "  BUILD-FAIL (see $OUT/$n.build.log)"; FAIL=1; return; }
    RC=0
    timeout ${WDT_SEC:-600} vvp "$OUT/$n.vvp" $PA > "$OUT/$n.log" 2>&1 || RC=$?
    if [ $RC -ne 0 ]; then echo "  RUN-FAIL rc=$RC (see $OUT/$n.log)"; tail -5 "$OUT/$n.log"; FAIL=1; return; fi
    if grep -q "ALL TESTS PASSED" "$OUT/$n.log"; then
        grep "ALL TESTS PASSED" "$OUT/$n.log" | sed 's/^/  /'
    else
        echo "  NO-PASS-MARKER"; tail -8 "$OUT/$n.log"; FAIL=1
    fi
}

for n in $TARGETS; do
    case $n in
    mmu)   run mmu "iverilog -g2005 -s tb_mmu tb/tb_mmu.v rtl/mmu.v -o $OUT/mmu.vvp" "" ;;
    l1_i)  run l1_i "iverilog -g2005 -s tb_l1_i tb/tb_l1_i.v rtl/l1_i.v -o $OUT/l1_i.vvp" "" ;;
    l1_d)  run l1_d "iverilog -g2005 -s tb_l1_d tb/tb_l1_d.v rtl/l1_d.v -o $OUT/l1_d.vvp" "" ;;
    wb_ic) run wb_ic "iverilog -g2005 -s tb_wb_ic tb/tb_wb_ic.v rtl/wb_ic.v rtl/wb_ic_mfifo.v -o $OUT/wb_ic.vvp" "" ;;
    16550) run 16550 "iverilog -g2005 -s tb_16550 tb/tb_16550.v rtl/16550.v -o $OUT/16550.vvp" "" ;;
    clint) run clint "iverilog -g2005 -s tb_clint tb/tb_clint.v rtl/clint.v -o $OUT/clint.vvp" "" ;;
    plic)  run plic "iverilog -g2005 -s tb_plic tb/tb_plic.v rtl/plic.v -o $OUT/plic.vvp" "" ;;
    core)  run core "sh scripts/core_asm.sh tb/tb_core.S tb/tb_core.hex tb/tb_core_exp.vh && iverilog -g2005 -s tb_core tb/tb_core.v $CORE_RTL -o $OUT/core.vvp" "" ;;
    c8)    run c8 "sh scripts/core_asm.sh tb/tb_c8.S tb/tb_c8.hex /tmp/opencode/c8_exp.vh && sed 's/C1_NCOMMIT/C8_NCOMMIT/' /tmp/opencode/c8_exp.vh > tb/tb_c8_exp.vh && iverilog -g2005 -s tb_c8 tb/tb_c8.v $CORE_RTL -o $OUT/c8.vvp" "" ;;
    t01)   run t01 "sh scripts/fw_asm.sh tests/t01_isa.S tb/t_fw_t01.hex && iverilog -g2005 -s tb_fw -DROM_HEX='\"tb/t_fw_t01.hex\"' tb/tb_fw.v $RTL -o $OUT/t01.vvp" "" ;;
    t02)   run t02 "sh scripts/fw_asm.sh tests/t02_traps.S tb/t_fw_t02.hex && iverilog -g2005 -s tb_fw -DROM_HEX='\"tb/t_fw_t02.hex\"' tb/tb_fw.v $RTL -o $OUT/t02.vvp" "" ;;
    t03)   run t03 "sh scripts/fw_build.sh tests/t03_intr.c tb/t_fw_t03.hex && iverilog -g2005 -s tb_fw -DROM_HEX='\"tb/t_fw_t03.hex\"' tb/tb_fw.v $RTL -o $OUT/t03.vvp" "+LOOP" ;;
    t04)   run t04 "sh scripts/fw_build.sh tests/t04_mmu.c tb/t_fw_t04.hex && iverilog -g2005 -s tb_fw -DROM_HEX='\"tb/t_fw_t04.hex\"' tb/tb_fw.v $RTL -o $OUT/t04.vvp" "" ;;
    t05)   run t05 "sh scripts/fw_build.sh tests/t05_soc.c tb/t_fw_t05.hex && iverilog -g2005 -s tb_fw -DROM_HEX='\"tb/t_fw_t05.hex\"' -DWDT=500000 tb/tb_fw.v $RTL -o $OUT/t05.vvp" "+LOOP" ;;
    soc)   run soc "sh scripts/soc_asm.sh tb/tb_soc_fw.S tb/tb_soc_fw.hex && iverilog -g2005 -s tb_soc tb/tb_soc.v $RTL -o $OUT/soc.vvp" "" ;;
    *) echo "unknown target: $n"; FAIL=1 ;;
    esac
done

[ $FAIL -eq 0 ] && echo "REGRESSION: ALL GREEN" || echo "REGRESSION: FAILURES (see $OUT/)"
exit $FAIL
