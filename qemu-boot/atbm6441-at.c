/*
 * Send one AT command to an ATBM6441 through the atbm6441 driver's
 * /dev/atbm_ioctl (ATBM_FW_AT_CMD, WSM 0x0023) and print what comes back:
 * the ioctl's result, then the status events the driver queues for readers
 * (struct status_async; type 6 carries the chip's customer event).  The
 * read is non-blocking in the driver (an empty list returns -1), so poll.
 *
 *   atbm6441-at 'AT+SET_PWM 1 50 100 100000'
 *
 * Build: mipsel-openipc-linux-musl-gcc -static -O2 -o atbm6441-at atbm6441-at.c
 */
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

#define ATBM_FW_AT_CMD  0x8004791c      /* _IOW(121, 0x1c, unsigned int), MIPS */

struct at_cmd_req {
    unsigned int status;
    char cmd[256];
};

struct status_async {
    unsigned char is_connected, type, driver_mode, list_empty;
    unsigned char event[64];            /* struct HostConnectEvent */
    unsigned char buf[504];
};

int main(int argc, char **argv)
{
    struct at_cmd_req req;
    struct status_async st;
    int fd, rc, i, got = 0;

    if (argc < 2) {
        fprintf(stderr, "usage: %s 'AT...'\n", argv[0]);
        return 2;
    }
    fd = open("/dev/atbm_ioctl", O_RDWR);
    if (fd < 0) {
        perror("open /dev/atbm_ioctl");
        return 1;
    }
    memset(&req, 0, sizeof(req));
    snprintf(req.cmd, sizeof(req.cmd), "%s", argv[1]);
    rc = ioctl(fd, ATBM_FW_AT_CMD, (unsigned long)&req);
    printf("AT ioctl: %d\n", rc);
    for (i = 0; i < 30 && !got; i++) {
        while (read(fd, &st, sizeof(st)) == (ssize_t)sizeof(st)) {
            printf("AT event: type %u connected %u\n", st.type,
                   st.is_connected);
            if (st.type == 6) {
                st.buf[sizeof(st.buf) - 1] = 0;
                printf("AT reply: %s\n", (char *)st.buf);
                got = 1;
            }
        }
        usleep(100000);
    }
    close(fd);
    return rc || !got;
}
