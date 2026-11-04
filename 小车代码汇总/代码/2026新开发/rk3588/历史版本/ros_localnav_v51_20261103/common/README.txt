wrong.log recorder, schema 1
===========================

Four C ABI functions are declared in wrong_log.h. Link wrong_log.cc once into each
process. Without wrong_log_start(), every logging call is a no-op. Start once at
application initialization; shutdown only at orderly process exit. Snapshot and
event producers never open files, acquire a file lock, perform network I/O, or
wait for the writer thread. Their bounded work is JSON parsing, sanitization and a
short memory lock. Logger failures do not change motor, navigation or sensor state.

Default file: /root/robot-navigation/wrong.log
Override: ROBOT_WRONG_LOG_PATH (must have an existing, writable parent directory).
The navigation and dashboard processes append to the same file; compare boot_id,
mono_ms, pid/run_id and seq to reconstruct the timeline. wall_time is UTC and may
jump when the system clock is corrected. mono_ms is CLOCK_MONOTONIC. Event_kind
"event" means a caller-reported transition, not an inferred severity or hardware
fault. Caller data may contain category/event_level and source details.

Call snapshot with full safe state at about 10 Hz per process/component. A bounded
ring holds the preceding 10 s. An event writes history and itself, then snapshots
for the following 5 s (at most 10 Hz per component). Overlapping events share
history already written; context_start_mono_ms on each event gives the lower bound
for context lookup in the common timeline. Samples include sample_seq; record seq
is distinct. Post-event snapshots include related_incident_ids for up to 32 active
incidents, so their relationship survives overlapping error events. The current
incident_id is always present. The producer must keep calling snapshot while the
process is running; the logger does not invent missing sensor observations.

Memory bounds: 16 KiB per payload, 512 history samples, 1024 pending records,
256 duplicate keys, 128 post-rate keys, 32 overlapping incident references. History
can cover less than 10 s if callers exceed these bounds; history_dropped_total
makes this explicit. Queue overflow and failed writes can also lose context;
queue_dropped_total, io_dropped_total, io_failures_total, producer_errors_total and
deduplicated_total appear on later records. Identical component+reason+sanitized
payload repeats within 1 s are suppressed; different transitions are retained.

Only the writer thread performs disk access. Its interprocess flock is acquired
nonblocking with at most 100 ms bounded retry per record. The main log rotates to
wrong.log.1 and wrong.log.2 at 5 MiB (roughly 15 MiB total). Active/lock/rotated files
must be regular files; symlinks and hard links are rejected. The log/lock are opened
with O_NOFOLLOW/O_NONBLOCK/O_CLOEXEC, 0600. A partial trailing line after interruption
is removed before the next append. A failed partial write is truncated back under
the same lock. Separate processes cooperate through the independent .lock file.
ROBOT_WRONG_LOG_MAX_BYTES accepts 4096..67108864 bytes for fixture/administrative
use; default remains 5 MiB. One line can exceed a deliberately tiny test limit.

Sensitive JSON keys are recursively removed (token, session, authorization,
password/passwd, pair_code/pairing_code, secret, credentials, cookie, API/access/
private keys and bearer variants). Deep or malformed JSON records only an invalid
payload marker and never the original text. Callers must still pass structured,
safe fields: an arbitrary exception string containing a password under an unknown
key cannot be reliably identified. No pairing code, token, session identifier,
raw HTTP request, Wi-Fi password or image data should be passed as event text.

A writable disk is required. Persistent disk failure is reported to stderr at most
once every 5 s. SIGKILL, power loss and an unwritable/full filesystem cannot be
promised to preserve the last event; no fsync is performed on control paths. The
log supports reconstruction from captured facts, not proof of uninstrumented
hardware conditions.

Standalone host regression:
  g++ -std=c++17 -O2 -Wall -Wextra -Werror -pthread \
    common/wrong_log.cc common/tests/test_wrong_log.cc -o /tmp/test-wrong-log
  /tmp/test-wrong-log

This test waits through real 10 s history / 5 s post-event windows and also covers
JSON redaction, invalid/deep payloads, distinct transitions, duplicate metrics,
restart append, truncated-tail recovery, 3-file rotation, two-process append,
symlink/FIFO/unwritable paths, nonblocking producers and visible queue overflow.

Failure boundaries: all four C ABI entry points are noexcept in C++. Every startup
allocation, including the initial event, completes before worker creation. The
worker owns its state separately and catches all top-level exceptions. Optional
shutdown-record allocation failure still sets the stop flag and joins the worker.
Even irrecoverable OS join/detach errors cannot free state still used by a worker.
A separate allocation regression injects 240 main-thread allocation failure points,
a worker allocation failure and a shutdown allocation failure in child processes:
  g++ -std=c++17 -O2 -Wall -Wextra -Werror -pthread \
    common/wrong_log.cc common/tests/test_wrong_log_allocation.cc \
    -o /tmp/test-wrong-log-allocation
  /tmp/test-wrong-log-allocation
