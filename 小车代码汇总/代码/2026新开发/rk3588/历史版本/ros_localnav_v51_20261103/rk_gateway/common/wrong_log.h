#ifndef ACG138_WRONG_LOG_H
#define ACG138_WRONG_LOG_H
#ifdef __cplusplus
extern "C" {
#define WRONG_LOG_NOEXCEPT noexcept
#else
#define WRONG_LOG_NOEXCEPT
#endif
/* Disabled until start. No network, motor or hardware actions. Producers never
 * perform file I/O. JSON payloads are capped at 16 KiB and sensitive keys removed.
 * Call snapshot at <=10 Hz per component. event captures 10 s before / 5 s after.
 * Only call shutdown on orderly process exit (it joins the I/O worker).
 * ROBOT_WRONG_LOG_PATH overrides /root/robot-navigation/wrong.log.
 */
void wrong_log_start(const char *process, const char *version) WRONG_LOG_NOEXCEPT;
void wrong_log_snapshot(const char *component, const char *json) WRONG_LOG_NOEXCEPT;
void wrong_log_event(const char *component, const char *reason, const char *json) WRONG_LOG_NOEXCEPT;
void wrong_log_shutdown(void) WRONG_LOG_NOEXCEPT;
#ifdef __cplusplus
}
#endif
#undef WRONG_LOG_NOEXCEPT
#endif
