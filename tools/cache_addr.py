#!/usr/bin/env python3
# cache_addr -- L1 cache line math (mythrv32 l1_i/l1_d)
#
# 4 KiB direct-mapped, 1-word lines:
#   idx = PA[11:2]  ((PA >> 2) & 0x3FF)
#   tag = PA[31:12] (PA >> 12)
#   word = PA[17:2] (TB RAM index; pattern ram[w] = w -> data = word)
#
# usage: cache_addr.py PA [PA...]   (hex or decimal)

def bits(pa):
    return (pa >> 2) & 0x3FF, (pa >> 12) & 0xFFFFF, (pa >> 2) & 0xFFFF


if __name__ == "__main__":
    import sys
    for a in sys.argv[1:]:
        pa = int(a, 0)
        idx, tag, word = bits(pa)
        print("PA=%08X idx=%3d tag=%05X word=%04X (data)" % (pa, idx, tag, word))
