/* The uClibc symbols the SDK's libraries import and musl does not have,
 * enough for these small tools (the firmware's uclibc-compat has the rest). */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

void __assert(const char *a, const char *f, int l) {
    fprintf(stderr, "assertion failed: %s (%s:%d)\n", a, f, l);
    abort();
}
FILE *__stdin, *__stdout;
__attribute__((constructor)) static void init(void) {
    __stdin = stdin;
    __stdout = stdout;
}
int __fputc_unlocked(int c, FILE *f) { return fputc(c, f); }
int __fgetc_unlocked(FILE *f) { return fgetc(f); }
int __isinf(double x) { return isinf(x); }
int __isinff(float x) { return isinf(x); }
