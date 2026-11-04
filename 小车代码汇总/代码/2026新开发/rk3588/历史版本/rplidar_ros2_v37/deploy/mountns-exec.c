#define _GNU_SOURCE
#include <errno.h>
#include <fcntl.h>
#include <limits.h>
#include <sched.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mount.h>
#include <sys/types.h>
#include <unistd.h>

/* Private mounts + optional process group. No motor I/O, network changes,
 * device writes, setuid bit or target userspace installation. */
int main(int argc, char **argv) {
    int first = 1;
    int join_pid = 0;
    if (argc > 1 && strcmp(argv[1], "--session") == 0) {
        if (setsid() < 0) { perror("setsid"); return 1; }
        first++;
    } else if (argc > 2 && strcmp(argv[1], "--join") == 0) {
        char *end = NULL;
        long parsed;
        errno = 0;
        parsed = strtol(argv[2], &end, 10);
        if (errno || !end || *end || parsed <= 1 || parsed > INT_MAX) {
            fputs("Invalid mount namespace PID\n", stderr); return 2;
        }
        join_pid = (int)parsed;
        first += 2;
    }
    if (argc <= first) {
        fputs("Usage: mountns-exec [--session | --join PID] COMMAND [ARG...]\n", stderr);
        return 2;
    }
    if (join_pid) {
        char path[64]; int fd;
        snprintf(path, sizeof(path), "/proc/%d/ns/mnt", join_pid);
        fd = open(path, O_RDONLY | O_CLOEXEC);
        if (fd < 0) { perror("open(mount namespace)"); return 1; }
        if (setns(fd, CLONE_NEWNS) < 0) { perror("setns(CLONE_NEWNS)"); close(fd); return 1; }
        close(fd);
    } else {
        if (unshare(CLONE_NEWNS) < 0) { perror("unshare(CLONE_NEWNS)"); return 1; }
        if (mount(NULL, "/", NULL, MS_REC | MS_PRIVATE, NULL) < 0) {
            perror("mount(/, MS_PRIVATE)"); return 1;
        }
    }
    if (setenv("ACG138_ROS2_MOUNTNS", "1", 1) < 0) { perror("setenv"); return 1; }
    execvp(argv[first], &argv[first]);
    { int saved_errno = errno; perror("execvp"); return saved_errno == ENOENT ? 127 : 126; }
}
