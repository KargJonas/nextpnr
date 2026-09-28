#!/usr/bin/env python3
"""Decrypt EFLX "encrypted" architecture files.

The eda-placer uses a trivial per-character substitution cipher based on a
Numerical Recipes LCG (seed=0, multiplier=0x19660d, increment=0x3c6ef35f).
Only printable ASCII (0x20-0x7e) is transformed; all other bytes pass through
unchanged.

Encrypted files start with the header:
    @@@ EFLX encrypted file. Do not modify. @@@

The payload after the header may contain multiple embedded files separated by
the delimiter: @^#$a~%_!D+=/>,-@

Usage:
    python3 tools/decrypt_eflx.py <encrypted_file> [output_file]
    python3 tools/decrypt_eflx.py --all <directory>   # decrypt all encrypted files
"""

import sys
import os
import argparse

HEADER = "@@@ EFLX encrypted file. Do not modify. @@@"
DELIMITER = "@^#$a~%_!D+=/>,-@"

# Numerical Recipes LCG constants
LCG_MULT = 0x19660D
LCG_INC  = 0x3C6EF35F
LCG_MASK = 0xFFFFFFFF


def lcg_step(state: int) -> int:
    """Step the LCG and return new state."""
    return (state * LCG_MULT + LCG_INC) & LCG_MASK


def decrypt_payload(text: str) -> str:
    """Decrypt an EFLX cipher payload (after header removal)."""
    state = 0
    prev_plain = 0
    out = []

    for ch in text:
        c = ord(ch)
        if 0x20 <= c <= 0x7E:
            state = lcg_step(state)
            shift = (state & 0x0FFFFFFF) + prev_plain
            p = ((c - 0x20) - shift) % 95
            if p < 0:
                p += 95
            p += 0x20
            prev_plain = p
            out.append(chr(p))
        else:
            out.append(ch)

    return "".join(out)


def decrypt_file(data: bytes) -> str:
    """Decrypt an EFLX encrypted file, returning the plaintext."""
    text = data.decode("latin-1")

    if not text.startswith(HEADER):
        raise ValueError("Not an EFLX encrypted file (missing header)")

    # Skip header + newline
    payload = text[len(HEADER):]
    if payload.startswith("\n"):
        payload = payload[1:]

    return decrypt_payload(payload)


def encrypt_payload(text: str) -> str:
    """Inverse of decrypt_payload. The cipher feeds back the previous PLAINTEXT
    char, so encryption is exact: decrypt_payload(encrypt_payload(x)) == x."""
    state = 0
    prev_plain = 0
    out = []

    for ch in text:
        p = ord(ch)
        if 0x20 <= p <= 0x7E:
            state = lcg_step(state)
            shift = (state & 0x0FFFFFFF) + prev_plain
            c = (((p - 0x20) + shift) % 95) + 0x20
            prev_plain = p
            out.append(chr(c))
        else:
            out.append(ch)

    return "".join(out)


def encrypt_file(plaintext: str) -> bytes:
    """Re-encrypt plaintext into an EFLX file (header + ciphered payload). Used
    to feed a tweaked configure.txt back to the placer (it rejects plaintext)."""
    return (HEADER + "\n" + encrypt_payload(plaintext)).encode("latin-1")


def main():
    parser = argparse.ArgumentParser(description="Decrypt EFLX encrypted architecture files")
    parser.add_argument("input", help="Encrypted file or directory (with --all)")
    parser.add_argument("output", nargs="?", help="Output file (default: stdout)")
    parser.add_argument("--all", action="store_true",
                        help="Decrypt all encrypted files in directory")
    parser.add_argument("--encrypt", action="store_true",
                        help="Encrypt plaintext input into an EFLX file (inverse of decrypt)")
    parser.add_argument("--split", action="store_true",
                        help="Split multi-file payloads on delimiter")
    args = parser.parse_args()

    if args.all:
        # Decrypt all encrypted files in directory
        dirpath = args.input
        outdir = args.output or os.path.join(dirpath, "decrypted")
        os.makedirs(outdir, exist_ok=True)

        count = 0
        for fname in sorted(os.listdir(dirpath)):
            fpath = os.path.join(dirpath, fname)
            if not os.path.isfile(fpath):
                continue
            with open(fpath, "rb") as f:
                first_line = f.readline()
            if HEADER.encode("latin-1") not in first_line:
                continue

            with open(fpath, "rb") as f:
                data = f.read()

            try:
                plaintext = decrypt_file(data)
            except ValueError as e:
                print(f"  skip {fname}: {e}", file=sys.stderr)
                continue

            outpath = os.path.join(outdir, fname)
            with open(outpath, "w", encoding="latin-1") as f:
                f.write(plaintext)
            count += 1
            # Show first 80 chars as preview
            preview = plaintext[:80].replace("\n", "\\n")
            print(f"  {fname} -> {outpath}  ({len(plaintext)} chars)  [{preview}...]")

        print(f"\nDecrypted {count} files to {outdir}/")
        return

    if args.encrypt:
        with open(args.input, "r", encoding="latin-1") as f:
            plaintext = f.read()
        blob = encrypt_file(plaintext)
        if args.output:
            with open(args.output, "wb") as f:
                f.write(blob)
            print(f"Encrypted {len(plaintext)} chars -> {args.output}", file=sys.stderr)
        else:
            sys.stdout.buffer.write(blob)
        return

    # Single file mode
    with open(args.input, "rb") as f:
        data = f.read()

    plaintext = decrypt_file(data)

    if args.split and DELIMITER in plaintext:
        parts = plaintext.split(DELIMITER)
        for i, part in enumerate(parts):
            print(f"=== Part {i} ({len(part)} chars) ===")
            print(part[:500])
            if len(part) > 500:
                print(f"... ({len(part) - 500} more chars)")
            print()
        return

    if args.output:
        with open(args.output, "w", encoding="latin-1") as f:
            f.write(plaintext)
        print(f"Decrypted {len(data)} bytes -> {args.output} ({len(plaintext)} chars)",
              file=sys.stderr)
    else:
        sys.stdout.write(plaintext)


if __name__ == "__main__":
    main()
