/* What the SDK's static (uClibc) objects need from a static musl link,
 * for the twin's stock firmware: its rootfs is uClibc, so the dynamic
 * tools cannot run there. */
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/syscall.h>
#include <unistd.h>

void __assert(const char *a, const char *f, int l) {
    fprintf(stderr, "assert %s %s:%d\n", a, f, l);
    abort();
}
FILE *__stdin, *__stdout;
__attribute__((constructor)) static void std_streams(void) {
    __stdin = stdin;
    __stdout = stdout;
}
int __fputc_unlocked(int c, FILE *f) { return fputc(c, f); }
int __fgetc_unlocked(FILE *f) { return fgetc(f); }
int __isinf(double x) { return isinf(x); }
int __isinff(float x) { return isinf(x); }

/* The vendor objects pass uClibc's 32-bit byte offset; musl's mmap takes
 * a 64-bit one, so its upper half would be garbage. */
void *mmap(void *start, size_t len, int prot, int flags, int fd, uint32_t off) {
    return (void *)syscall(SYS_mmap2, start, len, prot, flags, fd, off >> 12);
}
