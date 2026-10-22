/*
 * Novatek NA51089 media pixels: the sensor test pattern, RAW packing, the
 * IPP's Bayer-to-YUV path and the JPEG encoder (hw/misc/nvt-pixel.c),
 * for the nvt-media engines.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_NVT_PIXEL_H
#define HW_MISC_NVT_PIXEL_H

/*
 * One line of the sensor's test pattern, as 12-bit Bayer (RGGB) samples:
 * colour bars over the top two thirds, a grey ramp below, and a white box
 * that moves with the frame count.
 */
void nvt_pixel_pattern_line(uint16_t *out, uint32_t width, uint32_t height,
                            uint32_t y, uint32_t frame);

/*
 * Pack / unpack n 12-bit samples at 'bits' (8, 10, 12 or 16) a sample,
 * little-endian, scaled to the depth's range.
 */
size_t nvt_pixel_packed_size(uint32_t n, unsigned bits);
void nvt_pixel_pack(uint8_t *dst, const uint16_t *src, uint32_t n,
                    unsigned bits);
void nvt_pixel_unpack(uint16_t *dst, const uint8_t *src, uint32_t n,
                      unsigned bits);

/*
 * RGGB Bayer (12-bit samples, in_w x in_h, in_stride samples a row; an odd
 * last column or row is ignored) to YUV 4:2:0 semi-planar (Y plane, then
 * interleaved U/V), scaled to out_w x out_h.  uv_stride must hold out_w
 * rounded up to even, and uv (out_h + 1) / 2 rows.
 */
void nvt_pixel_bayer_to_nv12(const uint16_t *bayer, uint32_t in_stride,
                             uint32_t in_w, uint32_t in_h,
                             uint8_t *y, uint32_t y_stride,
                             uint8_t *uv, uint32_t uv_stride,
                             uint32_t out_w, uint32_t out_h);

/*
 * Baseline JPEG scan of a YUV 4:2:0 semi-planar picture of mcu_w x mcu_h
 * 16x16 MCUs, with the given quantisation tables (zig-zag order) and the
 * standard Huffman tables, ending in EOI.  Returns the bytes written, 0 if
 * they would not fit in 'room'.
 */
size_t nvt_pixel_jpeg_scan(uint8_t *out, size_t room,
                           const uint8_t *y, uint32_t y_stride,
                           const uint8_t *uv, uint32_t uv_stride,
                           uint32_t mcu_w, uint32_t mcu_h,
                           const uint8_t qy[64], const uint8_t qc[64]);

#endif
