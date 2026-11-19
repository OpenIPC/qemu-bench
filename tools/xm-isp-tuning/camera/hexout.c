/* print files as hex between markers, for a serial console to carry */
#include <stdio.h>
int main(int c, char **v) {
    int sum_only = c > 1 && v[1][0] == '-' && v[1][1] == 's';
    for (int i = 1 + sum_only; i < c; i++) {
        FILE *f = fopen(v[i], "rb");
        if (!f) { printf("@@MISSING %s\n", v[i]); continue; }
        unsigned char b[48]; size_t n, tot = 0; unsigned sum = 0;
        printf("@@BEGIN %s\n", v[i]);
        while ((n = fread(b, 1, sizeof b, f)) > 0) {
            if (!sum_only) printf("@@");
            for (size_t k = 0; k < n; k++) { if (!sum_only) printf("%02x", b[k]); sum = sum * 31 + b[k]; }
            if (!sum_only) printf("\n");
            tot += n;
        }
        printf("@@END %s %zu %08x\n", v[i], tot, sum);
        fclose(f);
    }
    return 0;
}
