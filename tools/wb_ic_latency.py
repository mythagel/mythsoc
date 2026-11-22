#!/usr/bin/env python3
# tools/wb_ic_latency.py -- cycle-accurate model of tb_wb_ic timing
#
# Derives the expected latency constants checked in tb/tb_wb_ic.v, per
# AGENTS.md rule 2 (no hand bit/cycle math).
#
# Conventions modeled (from tb/tb_wb_ic.v + rtl/wb_ic.v + rtl/wb_ic_mfifo.v):
#   - posedge index i = i-th posedge; "t0" sampled after posedge 0 (cyc A).
#   - push task: STB sampled at the first posedge after the negedge
#     following the call -> push at P1 when called right after posedge 0.
#   - slave tb_wb_slv (DELAY): STB sampled at P -> ACK_O high over
#     [P+off, P+off+1), off = max(1, DELAY-1)  (design.md sec.6: slave ACK
#     one or more cycles after STB; DELAY=1: next cycle).
#   - wb_ic: m_k_ACK is COMBINATIONAL (s_ACK & s_busy) -> high over the
#     slave's ACK interval.
#   - capture: always @(posedge clk) #1 if (m_ACK) -> sampled at P+1ps of
#     the interval m_ACK is high (i.e. at posedge P if high over [P,P+1)).
#   - wait_ack: wakes at capture, then consumes exactly one posedge.
#   - grant: combinational at the push's posedge (bypass on empty FIFO,
#     else FIFO head); 1-cyc slave -> next grant for same master at P+2
#     (busy at P+1 while s_busy still set).
#
# Usage: python3 tools/wb_ic_latency.py
#   prints (and asserts) the expected values used by the TB.

# ---------------------------------------------------------------- push->ack
def slave_ack_off(delay):
    """cycles from STB posedge to the slave-ACK interval start
    (m_ACK = comb(s_ACK & s_busy) is high over that same interval)."""
    return max(1, delay - 1)

def push_to_ack_dt(delay):
    """t1-t0 for: t0 after P0, one push (STB sampled P1), slave DELAY.
    m_ACK high over [P1+off, P2+off); capture at P1+off(+1ps);
    wait_ack consumes one posedge."""
    p_push = 1
    p_ack  = p_push + slave_ack_off(delay)   # m_ACK high over [p_ack, p_ack+1)
    return (p_ack + 1) - 0

# ---------------------------------------------------------------- T7 burst
def burst_total(n, delay=1):
    """t1-t0 for n back-to-back m0 pushes (one slave).
    t0 after P0; pushes P1..Pn (FIFO pops keep cnt low -> no STALL).
    Grant at g: busy/s_busy high over [g, g+off+1) -> next grant g+off+2
    (grant at g+1/g+off+1 sees busy still set). t1 after last wait_ack:
    last m_ACK over [g+off, g+off+1) -> capture g+off -> wait g+off+1."""
    off    = slave_ack_off(delay)
    period = off + 2
    p_last = 1 + (n - 1) * period
    return p_last + off + 1

# ---------------------------------------------------------------- T14 FIFO
class Fifo:
    """wb_ic_mfifo: 4-entry shift FIFO (push/pop semantics of the RTL)."""
    def __init__(self, depth=4):
        self.depth, self.e, self.c = depth, [None] * depth, 0

    def push(self, v):
        if self.c < self.depth:          # push@full ignored
            self.e[self.c] = v
            self.c += 1

    def pop(self):
        if self.c:
            for i in range(self.c - 1):
                self.e[i] = self.e[i + 1]
            self.c -= 1

    def push_pop(self, v):
        if self.c == 0:
            pass                          # no-op (entry pushed IS popped)
        else:
            for i in range(self.c - 1):
                self.e[i] = self.e[i + 1]
            self.e[self.c - 1] = v        # c unchanged

    @property
    def head(self):
        return self.e[0] if self.c else None

    @property
    def full(self):
        return self.c == self.depth


def t14_trace():
    """Replay T14's exact sequence; return (head_after_pushpop,
    head_after_drain2)."""
    f = Fifo()
    for i in range(4):                   # d_i = 0x0A*(i+1)
        f.push(0x0A * (i + 1))
    assert f.full
    assert f.head == 0x0A
    f.push(0x42)                          # push@full: ignored
    assert f.c == 4
    f.pop()                               # head -> d1
    assert f.head == 0x14
    f.push_pop(0x99)                     # head -> d2
    head_pp = f.head
    f.pop(); f.pop()                     # drain 2
    head_drain = f.head
    f.pop()
    assert f.c == 0
    return head_pp, head_drain


def main():
    dt1, dt3 = push_to_ack_dt(1), push_to_ack_dt(3)
    t7 = burst_total(8, delay=1)
    head_pp, head_drain = t14_trace()
    # anchors: design.md sec.6 "1-cycle slave => master STB at c, master
    # ACK at c+1" -> slave ACK interval [c+1, c+2), m_ACK comb, capture
    # there, wait_ack +1 -> dt1 = 3 (t0 one posedge before STB posedge);
    # T7 measured total == 24 (period off+2 = 3 for 1-cyc slave).
    assert dt1 == 3, dt1
    assert dt3 == dt1 + 1, dt3           # +1 per +2 slave latency
    assert t7 == 24, t7
    assert head_pp == 0x1E, hex(head_pp)
    assert head_drain == 0x99, hex(head_drain)
    print("T2  1cyc slave push->ack dt =", dt1)
    print("T2  3cyc slave push->ack dt =", dt3)
    print("T7  8-push burst total      =", t7)
    print("T14 head after pop+pushpop  = %08x (d2)" % head_pp)
    print("T14 head after drain 2      = %08x (0x99)" % head_drain)


if __name__ == "__main__":
    main()
