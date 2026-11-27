#!/usr/bin/env python3
"""bin2hex.py -- raw binary -> $readmemh word image (ROM format, one little-
endian 32-bit word as 8 hex digits per line; trailing partial word dropped).

usage: tools/bin2hex.py <in.bin> <out.hex>
"""
import sys


def main():
    if len(sys.argv) != 3:
        sys.exit(__doc__)
    d = open(sys.argv[1], 'rb').read()
    d = d[:len(d) // 4 * 4]
    with open(sys.argv[2], 'w') as f:
        f.write(''.join(format(int.from_bytes(d[i:i + 4], 'little'), '08x') + '\n'
                        for i in range(0, len(d), 4)))


if __name__ == '__main__':
    main()
