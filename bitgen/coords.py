"""Bitstream coordinate math: internal bit-writer (x,y,BL,WL) -> packed image.

Transcribed in-port (not imported) from the verified mapping the vendor's
serializer implements. The bit-writer FUN_00556ad0 addresses config bits by a
per-tile (BL, WL) inside a tile at grid (x, y); the serializer FUN_0055f840
then lays those into the 1024 x 352 raster that becomes EFLX_bitstream.bin.

Verified relationships (cross-checked against the vendor .log<->.bin packing and
the 282,624-entry trace corpus this project recovered):

  * axis swap     : image X = trace y,  image Y = WL
  * column packing: 32 trace-x columns x 11 BL rows interleave into 11 words
                    via P = (31 - x)*11 + bl' , word = P//32, serial = P%32
  * BL mirroring  : interior odd x (3,5,..,29) keep bl; even x and edge odd x
                    (1, 31) mirror bl -> (10 - bl)
  * byte order    : bits pack MSB-first within each little-endian 32-bit word,
                    so bit = (serial//8)*8 + (7 - serial%8)

The final flat word index is X_bs*32*11 + Y_bs*11 + pos_bs (11 words per tile
column, 32 tile rows). 11264 words total = 45056 bytes.
"""

WORDS = 11264          # 45056 bytes / 4
COLS = 352             # raster columns (11 words * 32 bits)
ROWS = 1024            # raster rows


def trace_to_bs(x, y, bl, wl):
    """(grid x, grid y, BL 0..10, WL 0..31) -> (X_bs, Y_bs, pos_bs, bit_bs)."""
    X_bs = y
    Y_bs = wl
    if x % 2 == 0 or x in (1, 31):
        P = (31 - x) * 11 + (10 - bl)
    else:
        P = (31 - x) * 11 + bl
    pos_bs = P // 32
    serial = P % 32
    bit_bs = (serial // 8) * 8 + (7 - serial % 8)
    return X_bs, Y_bs, pos_bs, bit_bs


def bs_to_word_idx(x_bs, y_bs, pos_bs):
    """Flat 32-bit-word index for a bitstream grid position."""
    return x_bs * 32 * 11 + y_bs * 11 + pos_bs


def trace_to_word_bit(x, y, bl, wl):
    """(x, y, BL, WL) -> (word_idx, bit) directly."""
    X_bs, Y_bs, pos_bs, bit_bs = trace_to_bs(x, y, bl, wl)
    return bs_to_word_idx(X_bs, Y_bs, pos_bs), bit_bs
