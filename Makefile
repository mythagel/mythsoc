# mythrv32 -- build RTL testbenches, firmware ROM images, run the suite.
# (Replaces scripts/*.sh; scripts/ now holds link.lds only.)
#
# usage:
#   make regress              full suite, continue-on-fail, pass-marker checked
#                             (mmu l1_i l1_d wb_ic 16550 clint plic core c8
#                              t01 t02 t03 t04 t05 t06 soc)
#   make <name>               build + run one bench (same names)
#   make images               rebuild all generated ROM/golden artifacts only
#   make clean                remove build/ + sim output
# knobs:
#   OUT=dir       vvp binaries + sim logs      (default /tmp/opencode/regress)
#   WDT_SEC=n     per-run wall-clock timeout   (default 600)
#   RUNS=n        t06 Dhrystone iterations     (default 100)
#                 (RUNS is baked via -DDRY_RUNS; make won't notice a changed
#                 flag by itself -- `touch tests/dry1.c` to force the rebuild)
#
# Artifacts live where the TBs expect them (hardcoded paths in tb/*.v):
#   tb/tb_core.hex/_exp.vh/_irq.vh  tb/tb_c8.hex/_exp.vh  tb/tb_soc_fw.hex
#   tb/t_fw_t01..06.hex  (all tracked); intermediates (.o/.elf/.bin) in build/.

# ---- toolchain --------------------------------------------------------------
# (:= for the llvm set: make predefines CC=cc, but the riscv32 target needs
#  clang; command-line overrides (make CC=clang-17 ...) still win)
CC       := clang
NM       := llvm-nm
OC       := llvm-objcopy
OD       := llvm-objdump
IV       ?= iverilog
VVP      ?= vvp
PY       ?= python3

OUT      ?= /tmp/opencode/regress
B        := build
WDT_SEC  ?= 600
RUNS     ?= 100

RV        := -target riscv32-unknown-elf -march=rv32im -mabi=ilp32
CFLAGS    := $(RV) -O2 -Itests -Wall -Wextra -Werror -ffreestanding -nostdlib
DRY_FLAGS := $(RV) -O2 -ffreestanding -fno-builtin -Itests -Wall -Wextra -Werror \
             -DDRY_RUNS=$(RUNS)
IVFLAGS   := -g2005

RTL      := rtl/soc.v rtl/core.v rtl/mmu.v rtl/l1_i.v rtl/l1_d.v rtl/wb_ic.v \
            rtl/wb_ic_mfifo.v rtl/wb_rom.v rtl/wb_ram.v rtl/16550.v \
            rtl/clint.v rtl/plic.v
CORE_RTL := rtl/core.v rtl/mmu.v rtl/l1_i.v rtl/l1_d.v

EMU       := tools/emu32.py
BIN2HEX   := tools/bin2hex.py
FW_RANGES := tools/fw_ranges.py
LDS       := scripts/link.lds

TESTS := mmu l1_i l1_d wb_ic 16550 clint plic core c8 t01 t02 t03 t04 t05 t06 soc
IMAGES := tb/tb_core.hex tb/tb_core_exp.vh tb/tb_core_irq.vh \
          tb/tb_c8.hex tb/tb_c8_exp.vh \
          tb/t_fw_t01.hex tb/t_fw_t02.hex tb/t_fw_t03.hex tb/t_fw_t04.hex \
          tb/t_fw_t05.hex tb/t_fw_t06.hex tb/tb_soc_fw.hex

.PHONY: all images regress clean $(TESTS)

all: images

$(B) $(B)/c $(B)/dry $(OUT):
	mkdir -p $@

# ---- canned recipes ---------------------------------------------------------
# run <name> [plusargs]: vvp under WDT_SEC timeout, log to $OUT/<name>.log,
# require the "ALL TESTS PASSED" marker in it (mirrors scripts/regress.sh run()).
run = \
	mkdir -p $(OUT); \
	echo "== $(1)"; \
	timeout $(WDT_SEC) $(VVP) $(OUT)/$(1).vvp $(2) > $(OUT)/$(1).log 2>&1 \
	  || { echo "  RUN-FAIL rc=$$? (see $(OUT)/$(1).log)"; tail -5 $(OUT)/$(1).log; exit 1; }; \
	if grep -q "ALL TESTS PASSED" $(OUT)/$(1).log; then \
	  grep "ALL TESTS PASSED" $(OUT)/$(1).log | sed 's/^/  /'; \
	else echo "  NO-PASS-MARKER"; tail -8 $(OUT)/$(1).log; exit 1; fi

# check_fw <elf>: C-framework link sanity (mirrors scripts/fw_build.sh).
check_fw = \
	$(NM) $(1) | awk '{ v[$$3] = $$1 } \
	  END { if (v["_start"] != "00000000") { printf "_start @ 0x%s != 0\n", v["_start"]; exit 1 } \
	        if (v["dram"]    != "80000000") { printf "dram @ 0x%s != 0x80000000\n", v["dram"]; exit 1 } \
	        split("__data_start __data_end __data_load __bss_start __bss_end", s, " "); \
	        for (i in s) if (!(s[i] in v)) { printf "MISSING linker symbol %s\n", s[i]; exit 1 } }'

# check_rom <bin>: ROM image must fit the 16 KiB boot ROM.
check_rom = \
	SZ=$$(wc -c < $(1)); [ $$SZ -le 16384 ] \
	  || { echo "ROM image $$SZ bytes > 16 KiB"; exit 1; }

# ---- sim binaries -----------------------------------------------------------
$(OUT)/mmu.vvp:   tb/tb_mmu.v   rtl/mmu.v   | $(OUT)
	$(IV) $(IVFLAGS) -s tb_mmu   $^ -o $@
$(OUT)/l1_i.vvp:  tb/tb_l1_i.v  rtl/l1_i.v  | $(OUT)
	$(IV) $(IVFLAGS) -s tb_l1_i  $^ -o $@
$(OUT)/l1_d.vvp:  tb/tb_l1_d.v  rtl/l1_d.v  | $(OUT)
	$(IV) $(IVFLAGS) -s tb_l1_d  $^ -o $@
$(OUT)/wb_ic.vvp: tb/tb_wb_ic.v rtl/wb_ic.v rtl/wb_ic_mfifo.v | $(OUT)
	$(IV) $(IVFLAGS) -s tb_wb_ic $^ -o $@
$(OUT)/16550.vvp: tb/tb_16550.v rtl/16550.v | $(OUT)
	$(IV) $(IVFLAGS) -s tb_16550 $^ -o $@
$(OUT)/clint.vvp: tb/tb_clint.v rtl/clint.v | $(OUT)
	$(IV) $(IVFLAGS) -s tb_clint $^ -o $@
$(OUT)/plic.vvp:  tb/tb_plic.v  rtl/plic.v  | $(OUT)
	$(IV) $(IVFLAGS) -s tb_plic  $^ -o $@

# core/c8 include their emu32 goldens at COMPILE time -> vh files are deps.
$(OUT)/core.vvp: tb/tb_core.v $(CORE_RTL) tb/tb_core_exp.vh tb/tb_core_irq.vh | $(OUT)
	$(IV) $(IVFLAGS) -s tb_core tb/tb_core.v $(CORE_RTL) -o $@
$(OUT)/c8.vvp:   tb/tb_c8.v   $(CORE_RTL) tb/tb_c8_exp.vh | $(OUT)
	$(IV) $(IVFLAGS) -s tb_c8   tb/tb_c8.v $(CORE_RTL)   -o $@

# tb_fw reads its ROM at sim start -> the hex is a RUN dep, not a build dep.
$(OUT)/t01.vvp: tb/tb_fw.v $(RTL) | $(OUT)
	$(IV) $(IVFLAGS) -s tb_fw -DROM_HEX='"tb/t_fw_t01.hex"' $^ -o $@
$(OUT)/t02.vvp: tb/tb_fw.v $(RTL) | $(OUT)
	$(IV) $(IVFLAGS) -s tb_fw -DROM_HEX='"tb/t_fw_t02.hex"' $^ -o $@
$(OUT)/t03.vvp: tb/tb_fw.v $(RTL) | $(OUT)
	$(IV) $(IVFLAGS) -s tb_fw -DROM_HEX='"tb/t_fw_t03.hex"' $^ -o $@
$(OUT)/t04.vvp: tb/tb_fw.v $(RTL) | $(OUT)
	$(IV) $(IVFLAGS) -s tb_fw -DROM_HEX='"tb/t_fw_t04.hex"' -DWDT=800000 $^ -o $@
$(OUT)/t05.vvp: tb/tb_fw.v $(RTL) | $(OUT)
	$(IV) $(IVFLAGS) -s tb_fw -DROM_HEX='"tb/t_fw_t05.hex"' -DWDT=500000 $^ -o $@
$(OUT)/t06.vvp: tb/tb_fw.v $(RTL) | $(OUT)
	$(IV) $(IVFLAGS) -s tb_fw -DROM_HEX='"tb/t_fw_t06.hex"' -DWDT=1000000 $^ -o $@
$(OUT)/soc.vvp: tb/tb_soc.v $(RTL) | $(OUT)
	$(IV) $(IVFLAGS) -s tb_soc $^ -o $@

# ---- run --------------------------------------------------------------------
mmu:   $(OUT)/mmu.vvp
	@$(call run,mmu)
l1_i:  $(OUT)/l1_i.vvp
	@$(call run,l1_i)
l1_d:  $(OUT)/l1_d.vvp
	@$(call run,l1_d)
wb_ic: $(OUT)/wb_ic.vvp
	@$(call run,wb_ic)
16550: $(OUT)/16550.vvp
	@$(call run,16550)
clint: $(OUT)/clint.vvp
	@$(call run,clint)
plic:  $(OUT)/plic.vvp
	@$(call run,plic)
core:  $(OUT)/core.vvp tb/tb_core.hex
	@$(call run,core)
c8:    $(OUT)/c8.vvp tb/tb_c8.hex
	@$(call run,c8)
t01:   $(OUT)/t01.vvp tb/t_fw_t01.hex
	@$(call run,t01)
t02:   $(OUT)/t02.vvp tb/t_fw_t02.hex
	@$(call run,t02)
t03:   $(OUT)/t03.vvp tb/t_fw_t03.hex
	@$(call run,t03,+LOOP)
t04:   $(OUT)/t04.vvp tb/t_fw_t04.hex
	@$(call run,t04)
t05:   $(OUT)/t05.vvp tb/t_fw_t05.hex
	@$(call run,t05,+LOOP)
t06:   $(OUT)/t06.vvp tb/t_fw_t06.hex
	@$(call run,t06,+DUMP)
soc:   $(OUT)/soc.vvp tb/tb_soc_fw.hex
	@$(call run,soc)

regress:
	@fail=0; \
	for t in $(TESTS); do $(MAKE) --no-print-directory $$t || fail=1; done; \
	if [ $$fail -eq 0 ]; then echo "REGRESSION: ALL GREEN"; \
	else echo "REGRESSION: FAILURES (see $(OUT)/)"; exit 1; fi

# ---- ROM images -------------------------------------------------------------
# core_asm flow (was scripts/core_asm.sh): assemble + link @0 + emu32 goldens.
$(B)/tb_core.o: tb/tb_core.S | $(B)
	$(CC) -c $(RV) -o $@ $<
$(B)/tb_core.elf: $(B)/tb_core.o
	$(CC) $(RV) -nostdlib -nostartfiles -Wl,-Ttext=0x0 -Wl,--entry=_start -o $@ $<
$(B)/tb_core.bin: $(B)/tb_core.elf
	$(OC) -O binary $< $@
tb/tb_core.hex tb/tb_core_exp.vh tb/tb_core_irq.vh &: \
        $(B)/tb_core.bin $(B)/tb_core.elf tb/tb_core_pt.hex $(EMU)
	@AM=$$($(NM) $(B)/tb_core.elf | awk '/irq_arm_m/{print "0x"$$1}'); \
	 AS=$$($(NM) $(B)/tb_core.elf | awk '/irq_arm_s/{print "0x"$$1}'); \
	 EM=""; \
	 [ -n "$$AM" ] && EM="$$EM --arm-m $$AM"; \
	 [ -n "$$AS" ] && EM="$$EM --arm-s $$AS"; \
	 $(PY) $(EMU) $(B)/tb_core.bin --hex tb/tb_core.hex \
	   --exp tb/tb_core_exp.vh --pt tb/tb_core_pt.hex $$EM; \
	 $(NM) $(B)/tb_core.elf \
	   | awk '/irq_arm_m/{printf "`define PC_ARM_M 32\x27h%s\n", $$1} \
	            /irq_arm_s/{printf "`define PC_ARM_S 32\x27h%s\n", $$1}' \
	   > tb/tb_core_irq.vh

# c8: same flow; golden reuses the C1 counter name -> sed to C8_NCOMMIT.
$(B)/tb_c8.o: tb/tb_c8.S | $(B)
	$(CC) -c $(RV) -o $@ $<
$(B)/tb_c8.elf: $(B)/tb_c8.o
	$(CC) $(RV) -nostdlib -nostartfiles -Wl,-Ttext=0x0 -Wl,--entry=_start -o $@ $<
$(B)/tb_c8.bin: $(B)/tb_c8.elf
	$(OC) -O binary $< $@
tb/tb_c8.hex tb/tb_c8_exp.vh &: $(B)/tb_c8.bin $(B)/tb_c8.elf tb/tb_core_pt.hex $(EMU)
	@$(PY) $(EMU) $(B)/tb_c8.bin --hex tb/tb_c8.hex \
	   --exp tb/tb_c8_exp.vh --pt tb/tb_core_pt.hex; \
	 sed -i 's/C1_NCOMMIT/C8_NCOMMIT/' tb/tb_c8_exp.vh

# fw_asm flow (was scripts/fw_asm.sh): fw_tb.o links FIRST (_start @0).
$(B)/fw_tb.o: tests/fw_tb.S | $(B)
	$(CC) -c $(RV) -o $@ $<
$(B)/t01.o: tests/t01_isa.S | $(B)
	$(CC) -c $(RV) -o $@ $<
$(B)/t01.elf: $(B)/fw_tb.o $(B)/t01.o
	$(CC) $(RV) -nostdlib -nostartfiles -Wl,-Ttext=0x0 -Wl,--entry=_start -o $@ $^
$(B)/t01.bin: $(B)/t01.elf
	$(OC) -O binary $< $@
tb/t_fw_t01.hex: $(B)/t01.bin
	@$(PY) $(BIN2HEX) $< $@
	@echo "built $@ ($$(wc -l < $@) words)"
$(B)/t02.o: tests/t02_traps.S | $(B)
	$(CC) -c $(RV) -o $@ $<
$(B)/t02.elf: $(B)/fw_tb.o $(B)/t02.o
	$(CC) $(RV) -nostdlib -nostartfiles -Wl,-Ttext=0x0 -Wl,--entry=_start -o $@ $^
$(B)/t02.bin: $(B)/t02.elf
	$(OC) -O binary $< $@
tb/t_fw_t02.hex: $(B)/t02.bin
	@$(PY) $(BIN2HEX) $< $@
	@echo "built $@ ($$(wc -l < $@) words)"

# fw_build C flow (was scripts/fw_build.sh): fw_entry.S + test.c + fw_traps.S,
# link.lds layout, .text+.data ROM image, fw_ranges SPAN check.
$(B)/c/e.o: tests/fw_entry.S | $(B)/c
	$(CC) -c $(CFLAGS) -o $@ $<
$(B)/c/r.o: tests/fw_traps.S | $(B)/c
	$(CC) -c $(CFLAGS) -o $@ $<
$(B)/c/%.o: tests/%.c | $(B)/c
	$(CC) -c $(CFLAGS) -o $@ $<

$(B)/c/t03.elf: $(B)/c/e.o $(B)/c/t03_intr.o $(B)/c/r.o
	$(CC) $(RV) -nostdlib -T $(LDS) -e _start -o $@ $^
	@$(call check_fw,$@)
$(B)/c/t03.bin: $(B)/c/t03.elf
	$(OC) --only-section=.text --only-section=.data -O binary $< $@
tb/t_fw_t03.hex: $(B)/c/t03.bin
	@$(PY) $(BIN2HEX) $< $@
	@$(call check_rom,$(B)/c/t03.bin)
	@$(PY) $(FW_RANGES) tests/t03_intr.c $(B)/c/t03.elf \
	  || { echo "RANGE-FAIL: fix SPANs in tests/t03_intr.c"; exit 1; }
	@echo "built $@ ($$(wc -l < $@) words)"

$(B)/c/t04.elf: $(B)/c/e.o $(B)/c/t04_mmu.o $(B)/c/r.o
	$(CC) $(RV) -nostdlib -T $(LDS) -e _start -o $@ $^
	@$(call check_fw,$@)
$(B)/c/t04.bin: $(B)/c/t04.elf
	$(OC) --only-section=.text --only-section=.data -O binary $< $@
tb/t_fw_t04.hex: $(B)/c/t04.bin
	@$(PY) $(BIN2HEX) $< $@
	@$(call check_rom,$(B)/c/t04.bin)
	@$(PY) $(FW_RANGES) tests/t04_mmu.c $(B)/c/t04.elf \
	  || { echo "RANGE-FAIL: fix SPANs in tests/t04_mmu.c"; exit 1; }
	@echo "built $@ ($$(wc -l < $@) words)"

$(B)/c/t05.elf: $(B)/c/e.o $(B)/c/t05_soc.o $(B)/c/r.o
	$(CC) $(RV) -nostdlib -T $(LDS) -e _start -o $@ $^
	@$(call check_fw,$@)
$(B)/c/t05.bin: $(B)/c/t05.elf
	$(OC) --only-section=.text --only-section=.data -O binary $< $@
tb/t_fw_t05.hex: $(B)/c/t05.bin
	@$(PY) $(BIN2HEX) $< $@
	@$(call check_rom,$(B)/c/t05.bin)
	@$(PY) $(FW_RANGES) tests/t05_soc.c $(B)/c/t05.elf \
	  || { echo "RANGE-FAIL: fix SPANs in tests/t05_soc.c"; exit 1; }
	@echo "built $@ ($$(wc -l < $@) words)"

# Dhrystone flow (was scripts/dry_build.sh). $(OD) call-fidelity checks below
# guard the measurement: strcpy/strcmp/... must survive as real calls.
DRY_EXPORTS := Proc_1 Proc_2 Proc_3 Proc_4 Proc_5 Proc_6 Proc_7 Proc_8 \
               Func_1 Func_2 Func_3 strcpy strcmp
DRY_CALLS   := Proc_5 Proc_4 Func_2 Proc_8 Proc_1 Proc_2 Proc_7 Proc_6 \
               Func_1 strcpy strcmp

$(B)/dry/e.o: tests/fw_entry.S | $(B)/dry
	$(CC) -c $(DRY_FLAGS) -o $@ $<
$(B)/dry/r.o: tests/fw_traps.S | $(B)/dry
	$(CC) -c $(DRY_FLAGS) -o $@ $<
$(B)/dry/dry1.o: tests/dry1.c | $(B)/dry
	$(CC) -c $(DRY_FLAGS) -o $@ $<
$(B)/dry/dry2.o: tests/dry2.c | $(B)/dry
	$(CC) -c $(DRY_FLAGS) -o $@ $<
$(B)/dry/dryl.o: tests/dry_lib.c | $(B)/dry
	$(CC) -c $(DRY_FLAGS) -o $@ $<
$(B)/dry/dry.elf: $(B)/dry/e.o $(B)/dry/dry1.o $(B)/dry/dry2.o \
                  $(B)/dry/dryl.o $(B)/dry/r.o
	$(CC) $(RV) -nostdlib -T $(LDS) -e _start -o $@ $^
	@$(call check_fw,$@)
	@for f in $(DRY_EXPORTS); do \
	  $(OD) -d $@ | grep -q "<$$f>:" \
	    || { echo "MISSING symbol $$f (linker dead-code elim?)"; exit 1; }; done
	@for f in $(DRY_CALLS); do \
	  $(OD) -d $@ | grep -q "jal.*<$$f>" \
	    || { echo "NO CALL SITE for $$f (inlined? measurement invalid)"; exit 1; }; done
$(B)/dry/dry.bin: $(B)/dry/dry.elf
	$(OC) --only-section=.text --only-section=.data -O binary $< $@
tb/t_fw_t06.hex: $(B)/dry/dry.bin
	@$(PY) $(BIN2HEX) $< $@
	@$(call check_rom,$(B)/dry/dry.bin)
	@$(PY) $(FW_RANGES) tests/dry1.c $(B)/dry/dry.elf || { echo "RANGE-FAIL"; exit 1; }
	@echo "built $@: $$(wc -c < $(B)/dry/dry.bin) bytes text, RUNS=$(RUNS)"

# soc_asm flow (was scripts/soc_asm.sh): bare .S -> ROM hex.
$(B)/tb_soc_fw.o: tb/tb_soc_fw.S | $(B)
	$(CC) -c $(RV) -o $@ $<
$(B)/tb_soc_fw.elf: $(B)/tb_soc_fw.o
	$(CC) $(RV) -nostdlib -nostartfiles -Wl,-Ttext=0x0 -Wl,--entry=_start -o $@ $<
$(B)/tb_soc_fw.bin: $(B)/tb_soc_fw.elf
	$(OC) -O binary $< $@
tb/tb_soc_fw.hex: $(B)/tb_soc_fw.bin
	@$(PY) $(BIN2HEX) $< $@
	@echo "built $@ ($$(wc -l < $@) words)"

images: $(IMAGES)

clean:
	rm -rf $(B) $(OUT)
