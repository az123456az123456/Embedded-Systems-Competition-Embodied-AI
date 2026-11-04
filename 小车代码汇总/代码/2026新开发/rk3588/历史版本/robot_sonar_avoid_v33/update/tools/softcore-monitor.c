#define _POSIX_C_SOURCE 200809L
#define _DEFAULT_SOURCE
#include <arpa/inet.h>
#include <dirent.h>
#include <errno.h>
#include <inttypes.h>
#include <limits.h>
#include <linux/if_packet.h>
#include <net/if.h>
#include <poll.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

/* Passive status protocol. This program has no send/sendto/write socket path. */
#define NAV_ETHERTYPE 0x88b7
#define NAV_BYTES 28
#define STALE_HEARTBEAT_MS 3500
static const uint8_t fpga_mac[6] = {2, 0xac, 0x13, 0x80, 0, 1};
static volatile sig_atomic_t stopped;

struct nav_status {
    uint8_t flags;
    uint16_t path_length, firmware_error;
    uint32_t heartbeat, astar_cycles, expanded, imu_sequence;
};
enum parse_result { PARSE_OK, PARSE_NOT_NAV, PARSE_TRUNCATED,
                    PARSE_VERSION, PARSE_CRC, PARSE_FLAGS };

static uint16_t read16(const uint8_t *p) {
    return (uint16_t)((uint16_t)p[0] << 8 | p[1]);
}
static uint32_t read32(const uint8_t *p) {
    return (uint32_t)p[0] << 24 | (uint32_t)p[1] << 16 |
           (uint32_t)p[2] << 8 | p[3];
}
static uint16_t crc16(const uint8_t *p, size_t n) {
    uint16_t crc = 0xffff;
    for (size_t i = 0; i < n; ++i) {
        crc = (uint16_t)(crc ^ ((uint16_t)p[i] << 8));
        for (unsigned j = 0; j < 8; ++j)
            crc = (uint16_t)((crc & 0x8000) ? (crc << 1) ^ 0x1021 : crc << 1);
    }
    return crc;
}
static enum parse_result parse_frame(const uint8_t *frame, size_t n,
                                     struct nav_status *out) {
    if (!frame || !out || n < 14) return PARSE_TRUNCATED;
    if (read16(frame + 12) != NAV_ETHERTYPE || memcmp(frame + 6, fpga_mac, 6))
        return PARSE_NOT_NAV;
    if (n < 14 + NAV_BYTES) return PARSE_TRUNCATED;
    const uint8_t *p = frame + 14;
    if (memcmp(p, "NAV1", 4)) return PARSE_NOT_NAV;
    if (p[4] != 1) return PARSE_VERSION;
    if (crc16(p, 26) != read16(p + 26)) return PARSE_CRC;
    if (p[5] & 0xf0) return PARSE_FLAGS;
    struct nav_status decoded = {
        .flags = p[5], .path_length = read16(p + 6),
        .heartbeat = read32(p + 8), .astar_cycles = read32(p + 12),
        .expanded = read32(p + 16), .imu_sequence = read32(p + 20),
        .firmware_error = read16(p + 24)
    };
    *out = decoded;
    return PARSE_OK;
}
static uint64_t monotonic_ms(void) {
    struct timespec ts;
    if (clock_gettime(CLOCK_MONOTONIC, &ts)) {
        perror("clock_gettime"); exit(1);
    }
    return (uint64_t)ts.tv_sec * 1000 + (uint64_t)ts.tv_nsec / 1000000;
}
static void on_signal(int signum) { (void)signum; stopped = 1; }
static bool healthy(const struct nav_status *s) {
    return (s->flags & 3) == 3 && !(s->flags & 12) && !s->firmware_error;
}
struct heartbeat_state {
    bool seen;
    uint32_t last;
    uint64_t advances, resets, last_advance_ms;
};
static void note_heartbeat(struct heartbeat_state *h, uint32_t value, uint64_t now) {
    if (h->seen && value != h->last) {
        uint32_t delta = value - h->last;
        if (!(delta & UINT32_C(0x80000000))) {
            ++h->advances;
            h->last_advance_ms = now;
        } else {
            ++h->resets;
            /* A reset needs a subsequent forward heartbeat before live=yes. */
            h->last_advance_ms = 0;
        }
    }
    h->seen = true;
    h->last = value;
}
static bool heartbeat_live(const struct heartbeat_state *h, uint64_t now) {
    return h->advances && h->last_advance_ms &&
           now - h->last_advance_ms <= STALE_HEARTBEAT_MS;
}
static int observation_exit(uint64_t valid, uint64_t wanted, bool count_complete,
                            bool timed_out, const struct nav_status *s,
                            const struct heartbeat_state *h, uint64_t now) {
    if (!valid) return 2;
    /* Valid status carrying a trap is a POSITIVE fault observation. It must
     * never be mislabeled as no-data because its heartbeat cannot advance. */
    if (!healthy(s) || !heartbeat_live(h, now)) return 3;
    if (wanted && !count_complete && timed_out) return 2;
    return 0;
}
static void print_status(const struct nav_status *s, const struct heartbeat_state *h,
                         uint64_t now) {
    const char *live = heartbeat_live(h, now) ? "yes" :
                       h->advances ? "stalled" : "unverified";
    printf("NAV1 ready=%s selftest=%s live=%s heartbeat=%" PRIu32
           " path_length=%u astar_cycles=%" PRIu32 " astar_ms=%.3f"
           " expanded=%" PRIu32 " imu_seq=%" PRIu32
           " hardware_trap=%s firmware_fault=%s firmware_error=%u flags=0x%02x\n",
           (s->flags & 1) ? "yes" : "no", (s->flags & 2) ? "pass" : "pending/fail",
           live, s->heartbeat, s->path_length, s->astar_cycles,
           (double)s->astar_cycles / 50000.0, s->expanded, s->imu_sequence,
           (s->flags & 4) ? "yes" : "no", (s->flags & 8) ? "yes" : "no",
           s->firmware_error, s->flags);
    fflush(stdout);
}
static void usage(const char *name) {
    printf("Usage: %s [--interface IFACE] [--count N] [--timeout SECONDS]\n"
           "       %s --self-test\n"
           "Passive FPGA NAV1 monitor, EtherType 0x88b7. Sends no frames or commands.\n"
           "Interface: auto-detect the internal GMAC device containing fe1c0000;\n"
           "           if missing/ambiguous, specify --interface explicitly.\n"
           "--count N   minimum valid samples before completing, 0=full observation\n"
           "            window (default 0). A forward heartbeat is also required.\n"
           "--timeout S total observation window, default 10 seconds; 0=unlimited.\n"
           "Default runs 10 seconds. --count 0 --timeout 0 watches indefinitely.\n"
           "Exit: 0=healthy and advancing heartbeat; 1=usage/system error;\n"
           "      2=no valid data/count unmet by deadline; 3=unhealthy or no live heartbeat.\n"
           "Keep the unified RK3588 program running to obtain FPGA status traffic.\n",
           name, name);
}
static bool parse_unsigned(const char *value, uint64_t maximum, uint64_t *out) {
    if (!value || !*value) return false;
    for (const char *p = value; *p; ++p) if (*p < '0' || *p > '9') return false;
    char *end = NULL; errno = 0;
    unsigned long long n = strtoull(value, &end, 10);
    if (errno || !end || *end || n > maximum) return false;
    *out = n; return true;
}
static int discover_interface(char result[IFNAMSIZ]) {
    DIR *dir = opendir("/sys/class/net");
    if (!dir) { perror("/sys/class/net"); return -1; }
    unsigned matches = 0;
    struct dirent *entry;
    while ((entry = readdir(dir))) {
        if (entry->d_name[0] == '.' || strlen(entry->d_name) >= IFNAMSIZ) continue;
        char path[PATH_MAX], target[PATH_MAX];
        int n = snprintf(path, sizeof(path), "/sys/class/net/%s/device", entry->d_name);
        if (n < 0 || (size_t)n >= sizeof(path)) continue;
        if (realpath(path, target) && strstr(target, "fe1c0000")) {
            snprintf(result, IFNAMSIZ, "%s", entry->d_name);
            ++matches;
        }
    }
    if (matches != 1) {
        fprintf(stderr, "Internal GMAC discovery found %u matches. Available interfaces:\n", matches);
        rewinddir(dir);
        while ((entry = readdir(dir)))
            if (entry->d_name[0] != '.') fprintf(stderr, "  %s\n", entry->d_name);
        fprintf(stderr, "Use --interface IFACE; a Wi-Fi/USB/Ethernet name is not guessed.\n");
    }
    closedir(dir);
    return matches == 1 ? 0 : -1;
}

/* Local tests construct frames only in memory. No networking or privileges. */
static void put16(uint8_t *p, uint16_t n) { p[0] = (uint8_t)(n >> 8); p[1] = (uint8_t)n; }
static void put32(uint8_t *p, uint32_t n) {
    p[0] = (uint8_t)(n >> 24); p[1] = (uint8_t)(n >> 16);
    p[2] = (uint8_t)(n >> 8); p[3] = (uint8_t)n;
}
static void fixture(uint8_t frame[60]) {
    memset(frame, 0, 60); memset(frame, 0xff, 6);
    memcpy(frame + 6, fpga_mac, 6); put16(frame + 12, NAV_ETHERTYPE);
    uint8_t *p = frame + 14; memcpy(p, "NAV1", 4); p[4] = 1; p[5] = 3;
    put16(p + 6, 19); put32(p + 8, 42); put32(p + 12, 150000);
    put32(p + 16, 72); put32(p + 20, 0x1234abcd); put16(p + 26, crc16(p, 26));
}
#define CHECK(expr) do { ++checks; if (!(expr)) { \
    fprintf(stderr, "self-test failed line %d: %s\n", __LINE__, #expr); return 1; \
} } while (0)
static int self_test(void) {
    unsigned checks = 0;
    uint8_t frame[64]; struct nav_status status;
    CHECK(crc16((const uint8_t *)"123456789", 9) == 0x29b1);
    fixture(frame);
    CHECK(parse_frame(frame, 60, &status) == PARSE_OK);
    CHECK(status.flags == 3 && status.path_length == 19 && status.heartbeat == 42);
    CHECK(status.astar_cycles == 150000 && status.expanded == 72);
    CHECK(status.imu_sequence == UINT32_C(0x1234abcd) && status.firmware_error == 0);
    CHECK(healthy(&status));
    CHECK(parse_frame(frame, 42, &status) == PARSE_OK);
    /* Padding, and an optional driver-retained FCS, are outside NAV1 CRC. */
    memset(frame + 42, 0x5a, 22);
    CHECK(parse_frame(frame, 64, &status) == PARSE_OK);
    for (size_t n = 0; n < 42; ++n) CHECK(parse_frame(frame, n, &status) == PARSE_TRUNCATED);
    CHECK(parse_frame(NULL, 60, &status) == PARSE_TRUNCATED);
    CHECK(parse_frame(frame, 60, NULL) == PARSE_TRUNCATED);
    fixture(frame); frame[14 + 12] ^= 1;
    CHECK(parse_frame(frame, 60, &status) == PARSE_CRC);
    fixture(frame); frame[14 + 4] = 2;
    put16(frame + 14 + 26, crc16(frame + 14, 26));
    CHECK(parse_frame(frame, 60, &status) == PARSE_VERSION);
    fixture(frame); frame[12] = 0x08;
    CHECK(parse_frame(frame, 60, &status) == PARSE_NOT_NAV);
    fixture(frame); frame[6] ^= 1;
    CHECK(parse_frame(frame, 60, &status) == PARSE_NOT_NAV);
    fixture(frame); memcpy(frame + 14, "ACK1", 4);
    CHECK(parse_frame(frame, 60, &status) == PARSE_NOT_NAV);
    fixture(frame); frame[14 + 5] = 0x13;
    put16(frame + 14 + 26, crc16(frame + 14, 26));
    CHECK(parse_frame(frame, 60, &status) == PARSE_FLAGS);
    fixture(frame); frame[14 + 5] = 7;
    put16(frame + 14 + 26, crc16(frame + 14, 26));
    CHECK(parse_frame(frame, 60, &status) == PARSE_OK && !healthy(&status));
    for (unsigned flags = 0; flags < 16; ++flags) {
        status.flags = (uint8_t)flags; status.firmware_error = 0;
        CHECK(healthy(&status) == (flags == 3));
    }
    fixture(frame); frame[14 + 25] = 1;
    put16(frame + 14 + 26, crc16(frame + 14, 26));
    CHECK(parse_frame(frame, 60, &status) == PARSE_OK && !healthy(&status));
    struct heartbeat_state h = {0};
    note_heartbeat(&h, 42, 100); CHECK(!heartbeat_live(&h, 100));
    note_heartbeat(&h, 42, 1100); CHECK(!heartbeat_live(&h, 1100));
    note_heartbeat(&h, 43, 1200); CHECK(heartbeat_live(&h, 1200));
    CHECK(!heartbeat_live(&h, 5000));
    note_heartbeat(&h, 0, 5100); CHECK(h.resets == 1 && !heartbeat_live(&h, 5100));
    note_heartbeat(&h, 1, 6100); CHECK(heartbeat_live(&h, 6100));
    h = (struct heartbeat_state){0};
    note_heartbeat(&h, UINT32_MAX, 10); note_heartbeat(&h, 0, 1010);
    CHECK(heartbeat_live(&h, 1010) && h.resets == 0);
    status.flags = 4; status.firmware_error = 0;
    CHECK(observation_exit(100, 100, false, true, &status, &h, 1010) == 3);
    status.flags = 3;
    struct heartbeat_state stopped_cpu = {0};
    note_heartbeat(&stopped_cpu, 0, 1000);
    CHECK(observation_exit(100, 100, false, true, &status, &stopped_cpu, 1010) == 3);
    CHECK(observation_exit(0, 100, false, true, &status, &h, 1010) == 2);
    CHECK(observation_exit(99, 100, false, true, &status, &h, 1010) == 2);
    CHECK(observation_exit(100, 100, true, true, &status, &h, 1010) == 0);
    uint64_t value = 0;
    CHECK(parse_unsigned("0", UINT64_MAX, &value) && value == 0);
    CHECK(!parse_unsigned("-1", UINT64_MAX, &value));
    CHECK(!parse_unsigned("1x", UINT64_MAX, &value));
    CHECK(!parse_unsigned("18446744073709551616", UINT64_MAX, &value));
    printf("softcore-monitor self-test PASS (%u checks)\n", checks);
    return 0;
}

int main(int argc, char **argv) {
    char interface[IFNAMSIZ] = {0}; uint64_t wanted = 0, timeout = 10;
    for (int i = 1; i < argc; ++i) {
        if (!strcmp(argv[i], "--self-test")) {
            if (argc != 2) { fprintf(stderr, "--self-test must be used alone\n"); return 1; }
            return self_test();
        }
        if (!strcmp(argv[i], "--help") || !strcmp(argv[i], "-h")) { usage(argv[0]); return 0; }
        if (!strcmp(argv[i], "--interface")) {
            if (++i >= argc || !*argv[i] || strlen(argv[i]) >= sizeof(interface)) {
                fprintf(stderr, "Invalid/missing --interface value\n"); return 1;
            }
            strcpy(interface, argv[i]);
        } else if (!strcmp(argv[i], "--count") || !strcmp(argv[i], "--timeout")) {
            bool count = !strcmp(argv[i], "--count"); uint64_t value;
            if (++i >= argc || !parse_unsigned(argv[i], count ? UINT64_MAX : UINT32_MAX, &value)) {
                fprintf(stderr, "Invalid/missing numeric option value\n"); return 1;
            }
            if (count) wanted = value; else timeout = value;
        } else { fprintf(stderr, "Unknown option: %s\n", argv[i]); usage(argv[0]); return 1; }
    }
    if (!*interface && discover_interface(interface)) return 1;
    unsigned index = if_nametoindex(interface);
    if (!index) { fprintf(stderr, "Interface %s not found: %s\n", interface, strerror(errno)); return 1; }
    int fd = socket(AF_PACKET, SOCK_RAW | SOCK_NONBLOCK | SOCK_CLOEXEC, htons(NAV_ETHERTYPE));
    if (fd < 0) {
        fprintf(stderr, "Cannot open passive raw socket: %s (run as root or with CAP_NET_RAW)\n", strerror(errno));
        return 1;
    }
    struct sockaddr_ll address = { .sll_family = AF_PACKET,
        .sll_protocol = htons(NAV_ETHERTYPE), .sll_ifindex = (int)index };
    if (bind(fd, (struct sockaddr *)&address, sizeof(address))) {
        perror("bind"); close(fd); return 1;
    }
    struct sigaction action = {0}; action.sa_handler = on_signal;
    sigemptyset(&action.sa_mask); sigaction(SIGINT, &action, NULL); sigaction(SIGTERM, &action, NULL);
    printf("Listening interface=%s EtherType=0x%04x read_only=yes count=%" PRIu64
           " timeout_seconds=%" PRIu64 " (heartbeat expected about every 100 ms)\n",
           interface, NAV_ETHERTYPE, wanted, timeout); fflush(stdout);
    uint64_t start = monotonic_ms(), deadline = timeout ? start + timeout * 1000 : 0;
    uint64_t valid = 0, rejected[PARSE_FLAGS + 1] = {0}, last_print = 0;
    struct nav_status status = {0}; struct heartbeat_state heartbeat = {0};
    bool count_complete = false, timed_out = false; int runtime_error = 0;
    while (!stopped) {
        uint64_t now = monotonic_ms();
        if (deadline && now >= deadline) { timed_out = true; break; }
        int wait_ms = 250;
        if (deadline && deadline - now < (uint64_t)wait_ms) wait_ms = (int)(deadline - now);
        struct pollfd input = { .fd = fd, .events = POLLIN };
        int result = poll(&input, 1, wait_ms);
        if (result < 0) { if (errno == EINTR) continue; perror("poll"); runtime_error = 1; break; }
        if (!result) continue;
        if (input.revents & (POLLERR | POLLHUP | POLLNVAL)) {
            fprintf(stderr, "Raw socket closed or reported an error\n"); runtime_error = 1; break;
        }
        /* Bound each drain so heavy unrelated traffic cannot prevent timeout/SIGINT. */
        for (unsigned drained = 0; drained < 128 && !stopped; ++drained) {
            uint8_t frame[2048]; struct sockaddr_ll from = {0}; socklen_t len = sizeof(from);
            ssize_t n = recvfrom(fd, frame, sizeof(frame), MSG_DONTWAIT, (struct sockaddr *)&from, &len);
            if (n < 0) {
                if (errno == EINTR) continue;
                if (errno != EAGAIN && errno != EWOULDBLOCK) { perror("recvfrom"); runtime_error = 1; }
                break;
            }
            if (from.sll_pkttype == PACKET_OUTGOING) continue;
            struct nav_status decoded;
            enum parse_result parsed = parse_frame(frame, (size_t)n, &decoded);
            if (parsed != PARSE_OK) { ++rejected[parsed]; continue; }
            now = monotonic_ms(); ++valid; status = decoded;
            note_heartbeat(&heartbeat, status.heartbeat, now);
            if (!last_print || now - last_print >= 1000 || !healthy(&status)) {
                print_status(&status, &heartbeat, now); last_print = now;
            }
            if (wanted && valid >= wanted && heartbeat_live(&heartbeat, now)) {
                count_complete = true; break;
            }
        }
        if (runtime_error || count_complete) break;
    }
    uint64_t now = monotonic_ms(); close(fd);
    if (valid) print_status(&status, &heartbeat, now);
    printf("SUMMARY valid=%" PRIu64 " rejected_other=%" PRIu64 " rejected_truncated=%" PRIu64
           " rejected_version=%" PRIu64 " rejected_crc=%" PRIu64 " rejected_flags=%" PRIu64
           " heartbeat_advances=%" PRIu64 " heartbeat_resets=%" PRIu64 " elapsed_ms=%" PRIu64 "\n",
           valid, rejected[PARSE_NOT_NAV], rejected[PARSE_TRUNCATED], rejected[PARSE_VERSION],
           rejected[PARSE_CRC], rejected[PARSE_FLAGS], heartbeat.advances, heartbeat.resets, now - start);
    if (runtime_error) return 1;
    const int result = observation_exit(valid, wanted, count_complete, timed_out, &status, &heartbeat, now);
    if (!valid) {
        fprintf(stderr, "No valid NAV1 data received. Check the paired FPGA bitstream, internal GMAC "
                "and running RK3588 unified program (EtherType 0x88b7).\n");
    } else if (result == 3) {
        fprintf(stderr, "Received %" PRIu64 " valid NAV1 frames; hardware_trap=%s firmware_fault=%s "
                "firmware_error=%u heartbeat=%" PRIu32 " heartbeat_advances=%" PRIu64 "; %s. "
                "This is observed CPU/firmware health failure, not missing NAV1 data.\n",
                valid, (status.flags & 4) ? "yes" : "no", (status.flags & 8) ? "yes" : "no",
                status.firmware_error, status.heartbeat, heartbeat.advances,
                (status.flags & 4) ? "hardware CPU trap detected" :
                !heartbeat_live(&heartbeat, now) ? "heartbeat is not advancing" : "startup/selftest not healthy");
    } else if (result == 2) {
        fprintf(stderr, "Received %" PRIu64 " valid NAV1 frames with healthy advancing heartbeat, "
                "but requested count=%" PRIu64 " was not met before deadline.\n", valid, wanted);
    }
    return result;
}
