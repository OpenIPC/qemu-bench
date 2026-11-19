/* Export pipe 0's ISP, AWB and AE parameter blocks as a PQ bin, the file
 * the SDK's sample/pq_bin writes and xmedia_isp_import() reads back:
 * "ISP_BIN" in 16 bytes, the pipe, the module records, then each block.
 * Runs beside the process that owns the ISP. */
#include <stdio.h>
#include <xmedia_isp.h>

int main(int argc, char **argv) {
    if (argc != 2) {
        fprintf(stderr, "usage: %s out.bin\n", argv[0]);
        return 2;
    }
    xmedia_isp_module_info m[XMEDIA_ISP_MODULE_MAX_NUM] = {0};
    m[0].module = XMEDIA_ISP_MODULE_ID_ISP;
    m[1].module = XMEDIA_ISP_MODULE_ID_AWB;
    m[2].module = XMEDIA_ISP_MODULE_ID_AE;
    int r = xmedia_isp_export(0, m);
    if (r) {
        fprintf(stderr, "xmedia_isp_export: %#x\n", r);
        return 1;
    }
    FILE *f = fopen(argv[1], "wb");
    if (!f) {
        perror(argv[1]);
        return 1;
    }
    char magic[16] = "ISP_BIN";
    int pipe = 0;
    fwrite(magic, 1, sizeof magic, f);
    fwrite(&pipe, sizeof pipe, 1, f);
    fwrite(m, sizeof m, 1, f);
    for (int i = 0; i < XMEDIA_ISP_MODULE_MAX_NUM; i++) {
        printf("%.16s v%u %u bytes\n", m[i].name, m[i].version, m[i].length);
        if (m[i].length)
            fwrite(m[i].addr, 1, m[i].length, f);
    }
    return fclose(f) ? 1 : 0;
}
