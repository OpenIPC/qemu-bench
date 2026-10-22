/*
 * LZMA ("LZMA alone" / .lzma) decoder for the Novatek boot ROM stand-in.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#ifndef HW_ARM_NVT_LZMA_H
#define HW_ARM_NVT_LZMA_H

/*
 * Decode the .lzma stream at in (13-byte header: properties, dictionary
 * size, 64-bit unpacked size or all ones for "ends with a marker") into
 * out.  Returns the unpacked length, or -1 when the stream is corrupt,
 * does not fit in outmax bytes, or runs past inlen.
 */
ssize_t nvt_lzma_decode(const uint8_t *in, size_t inlen,
                        uint8_t *out, size_t outmax);

/* Whether in plausibly starts a .lzma stream (lc/lp/pb in range). */
bool nvt_lzma_probe(const uint8_t *in, size_t inlen);

#endif /* HW_ARM_NVT_LZMA_H */
