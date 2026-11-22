#!/usr/bin/env python3
"""tools/extract_tables.py -- pull clean spec tables from specs/*.pdf via pdfplumber.

Modes:
  unpriv-instr : RV32I (Table 13) + M ext (Table 14) instruction tables
  priv-cause   : mcause/scause exception + interrupt cause-code tables
  priv-csr     : CSR address mapping (Table 3) + mstatus field tables

Usage:
  python3 tools/extract_tables.py unpriv-instr
  python3 tools/extract_tables.py priv-cause
  python3 tools/extract_tables.py priv-csr
"""
import sys

import pdfplumber

UNPRIV = "specs/riscv-unprivileged.pdf"
PRIV = "specs/riscv-privileged.pdf"


def _norm(row):
    return [ (c or "").strip() for c in row ]


def _table_caption_page(pdf, caption):
    for i, p in enumerate(pdf.pages):
        if caption in (p.extract_text() or ""):
            return i
    raise SystemExit(f"caption not found: {caption}")


def _dump_pages(pdf, p0, p1, stop_marker=None):
    """Print table rows from pages p0..p1; stop after stop_marker is seen."""
    for i in range(p0, p1):
        for table in pdf.pages[i].extract_tables():
            for row in table:
                row = _norm(row)
                if stop_marker and any(c and stop_marker in c for c in row):
                    return
                if any(row):
                    print(" | ".join(row))


def unpriv_instr():
    with pdfplumber.open(UNPRIV) as pdf:
        for label, start_cap, stop_cap in (
                ("RV32I (Table 13)",
                 "Table 13. RV32I Base Integer Instruction Set",
                 "Table 14"),
                ("RV32M (Table 15)",
                 "Table 15. RV32M Standard Extension",
                 "Table 16")):
            p0 = _table_caption_page(pdf, start_cap)
            p1 = _table_caption_page(pdf, stop_cap)
            marker = stop_cap.split(".")[0]  # "Table 14" / "Table 16"
            print(f"=== {label} (page {p0}+) ===")
            _dump_pages(pdf, p0, min(p1 + 1, len(pdf.pages)), marker)
            print()


def priv_cause():
    with pdfplumber.open(PRIV) as pdf:
        for cap in ("Table 16. Machine cause (mcause) register values after trap",
                    "Table 37. Supervisor cause (scause) register values after trap"):
            i0 = _table_caption_page(pdf, cap)
            print(f"=== {cap} (pages {i0}-{i0+1}) ===")
            for i in range(i0, i0 + 2):
                for table in pdf.pages[i].extract_tables():
                    for row in table:
                        row = _norm(row)
                        if any(row):
                            print(" | ".join(row))
            print()


def priv_csr():
    with pdfplumber.open(PRIV) as pdf:
        for i, page in enumerate(pdf.pages):
            text = page.extract_text() or ""
            if "Table 3. Allocation of RISC-V CSR address ranges" in text:
                print(f"=== Table 3. CSR address ranges (page {i}) ===")
                for table in page.extract_tables():
                    for row in table:
                        row = _norm(row)
                        if any(row):
                            print(" | ".join(row))
                print()
            for cap in ("Table 5. Fields in the mstatus register",
                        "Table 6. Fields in the mstatush register",
                        "Table 7. Fields in the mstatus and mstatush registers"):
                if cap in text:
                    print(f"=== {cap} (page {i}) ===")
                    for table in page.extract_tables():
                        for row in table:
                            row = _norm(row)
                            if any(row):
                                print(" | ".join(row))
                    print()


def main():
    mode = sys.argv[1] if len(sys.argv) > 1 else "unpriv-instr"
    {"unpriv-instr": unpriv_instr,
     "priv-cause": priv_cause,
     "priv-csr": priv_csr}[mode]()


if __name__ == "__main__":
    main()
