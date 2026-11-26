// lib.h -- C framework for the firmware test suite (design.md sec.10)
//
// Protocol with tb/tb_fw.v:
//   dram[] lives at DRAM word 0 (0x8000_0000); words >= 16 are test data.
//   Done : dram[0] = 0; dram[1] = 0x5A5A0000 (marker, LAST), then spin.
//   Fail : chk(N, ok=0) -> dram[0] = 0x8000_000N; marker; spin.
//
// Trap entry: tests/fw_traps.S provides m_shim/s_shim (mtvec/stvec). The
// shims save every caller-saved reg + sp + s0, move sp to a private
// handler stack, call m_hnd_c()/s_hnd_c(), restore, and mret/sret. The C
// handlers follow the normal ABI, so interrupting main mid-sequence is
// safe (the retired .S handlers preserved only x1/x2/x3 and were
// clobber-prone -- e.g. t03 P9 x15 clobber).
#ifndef FW_LIB_H
#define FW_LIB_H

#include <stdint.h>

extern volatile uint32_t dram[];   // @0x80000000, linked first (scripts/link.lds)

static inline uint32_t rd32(uint32_t a) { return *(volatile uint32_t *)a; }
static inline void     wr32(uint32_t a, uint32_t v) { *(volatile uint32_t *)a = v; }
static inline void     wr8(uint32_t a, uint32_t v) { *(volatile uint8_t *)a = (uint8_t)v; }

// CSR access (numeric csr; "memory" clobber = barrier -- ordering matters).
static inline uint32_t csr_read(uint32_t c)
{ uint32_t v; asm volatile ("csrr %0, %1" : "=r"(v) : "i"(c) : "memory"); return v; }
static inline void csr_write(uint32_t c, uint32_t v) { asm volatile ("csrw %0, %1" : : "i"(c), "r"(v) : "memory"); }
static inline void csr_set(uint32_t c, uint32_t v)   { asm volatile ("csrs %0, %1" : : "i"(c), "r"(v) : "memory"); }
static inline void csr_clear(uint32_t c, uint32_t v)  { asm volatile ("csrc %0, %1" : : "i"(c), "r"(v) : "memory"); }
static inline void mret(void) { asm volatile ("mret" ::: "memory"); }
static inline void sret(void) { asm volatile ("sret" ::: "memory"); }

// v in [lo, lo+span)
static inline int in_range(uint32_t v, uint32_t lo, uint32_t span)
{ return v >= lo && v - lo < span; }

static inline void busy(uint32_t n) { for (volatile uint32_t i = 0; i < n; i++) ; }

// Check N: on failure write result + marker and spin (TB reports FW-result).
static inline void chk(uint32_t n, uint32_t ok)
{
    if (!ok) { dram[0] = 0x80000000u | n; dram[1] = 0x5A5A0000u; for (;;) ; }
}

__attribute__((noreturn)) static inline void fw_done(void)
{ dram[0] = 0; dram[1] = 0x5A5A0000u; for (;;) ; }

// Trap plumbing (tests/fw_traps.S defines the shims; the test .c defines
// the C handlers). mtvec = m_shim, stvec = s_shim.
void m_hnd_c(void);
void s_hnd_c(void);
void m_shim(void) __attribute__((noreturn));
void s_shim(void) __attribute__((noreturn));

#endif
