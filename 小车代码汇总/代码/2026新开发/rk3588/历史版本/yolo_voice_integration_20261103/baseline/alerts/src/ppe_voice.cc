#include "ppe_core.h"
#include <arpa/inet.h>
#include <cerrno>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <fstream>
#include <iostream>
#include <poll.h>
#include <sstream>
#include <sys/file.h>
#include <sys/socket.h>
#include <sys/wait.h>
#include <unistd.h>

static volatile sig_atomic_t stopping = 0;
static void stop_signal(int) { stopping = 1; }
static uint64_t now_ms() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
static std::string read_file(const std::string& path) {
    std::ifstream f(path);
    if (!f) throw std::runtime_error("cannot read " + path);
    std::ostringstream out; out << f.rdbuf(); return out.str();
}
static int open_stream(int port, const std::string& token_path) {
    auto token = read_file(token_path);
    while (!token.empty() && (token.back() == '\r' || token.back() == '\n')) token.pop_back();
    if (token.empty() || token.size() > 1024 || token.find_first_of("\r\n") != std::string::npos)
        throw std::runtime_error("invalid pairing token file");
    const int fd = socket(AF_INET, SOCK_STREAM | SOCK_CLOEXEC, 0);
    if (fd < 0) throw std::runtime_error("socket failed");
    sockaddr_in peer{}; peer.sin_family = AF_INET; peer.sin_port = htons(port);
    peer.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    timeval timeout{1, 0};
    setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
    if (connect(fd, reinterpret_cast<sockaddr*>(&peer), sizeof(peer))) { close(fd); return -1; }
    const auto request = "GET /api/video.mjpg HTTP/1.1\r\nHost: 127.0.0.1\r\nAuthorization: Bearer " + token + "\r\nConnection: close\r\n\r\n";
    size_t sent = 0;
    while (sent < request.size()) {
        const auto n = send(fd, request.data() + sent, request.size() - sent, MSG_NOSIGNAL);
        if (n <= 0) { close(fd); return -1; }
        sent += static_cast<size_t>(n);
    }
    return fd;
}
int main(int argc, char** argv) {
    try {
        std::string config = "/root/robot-ppe-voice/config.json";
        uint64_t run_seconds = 0;
        for (int i = 1; i < argc; ++i) {
            const std::string arg = argv[i];
            if (arg == "--config" && i + 1 < argc) config = argv[++i];
            else if (arg == "--run-seconds" && i + 1 < argc) run_seconds = std::stoull(argv[++i]);
            else if (arg == "--help") {
                std::cout << "ppe-voice --config FILE [--run-seconds N]\nRead-only detection listener; never issues motor commands.\n"; return 0;
            } else throw std::runtime_error("unknown or incomplete option");
        }
        const auto cfg = ppe::Json::parse(read_file(config));
        if (!cfg.value("enabled", true)) return 0;
        const int port = cfg.value("port", 8082);
        const auto player = cfg.value("player", std::string("/root/robot-ppe-voice/play-hdmi.sh"));
        const auto assets = cfg.value("asset_directory", std::string("/root/robot-ppe-voice/assets"));
        const auto status = cfg.value("status_file", std::string("/run/robot-ppe-voice-status.json"));
        const auto token = cfg.value("token_file", std::string("/root/robot-perception/pair-token.txt"));
        const auto lock_path = cfg.value("lock_file", std::string("/run/robot-ppe-voice.lock"));
        const uint64_t player_timeout = cfg.value("player_timeout_ms", uint64_t(10000));
        ppe::Settings settings;
        settings.confidence = cfg.value("min_confidence", .35);
        settings.confirm = cfg.value("confirm_ms", uint64_t(300));
        settings.cooldown = cfg.value("cooldown_ms", uint64_t(15000));
        settings.max_age = cfg.value("max_age_ms", uint64_t(1000));
        if (port < 1 || port > 65535 || !std::isfinite(settings.confidence) || settings.confidence < 0 || settings.confidence > 1 ||
            settings.confirm > 10000 || settings.cooldown < 1000 || settings.max_age < 100 || settings.max_age > 5000 ||
            player_timeout < 1000 || player_timeout > 30000) throw std::runtime_error("configuration outside supported bounds");
        const int lock = open(lock_path.c_str(), O_CREAT | O_RDWR | O_CLOEXEC, 0600);
        if (lock < 0 || flock(lock, LOCK_EX | LOCK_NB)) throw std::runtime_error("another PPE voice listener owns the lock");
        signal(SIGINT, stop_signal); signal(SIGTERM, stop_signal); signal(SIGPIPE, SIG_IGN);
        ppe::Engine engine(settings); ppe::Multipart parser;
        uint64_t next_connect = 0, last_data = 0, last_status = 0, last_valid = 0, player_started = 0;
        const auto start = now_ms();
        int fd = -1, active = -1; pid_t child = -1;
        std::string state = "waiting_for_detector", last_error;
        uint64_t valid_frames = 0, failures = 0;
        auto disconnect = [&] { if (fd >= 0) close(fd); fd = -1; parser = ppe::Multipart(); next_connect = now_ms() + 1500; };
        std::cout << "PPE_VOICE started port=" << port << " confirm_ms=" << settings.confirm << " cooldown_ms=" << settings.cooldown << std::endl;
        while (!stopping && (!run_seconds || now_ms() - start < run_seconds * 1000)) {
            const auto now = now_ms();
            if (fd < 0 && now >= next_connect) {
                try { fd = open_stream(port, token); last_data = now; }
                catch (const std::exception&) { last_error = "pairing_file_unavailable"; }
                next_connect = now + 1500;
            }
            if (fd >= 0) {
                pollfd pfd{fd, POLLIN, 0};
                const int result = poll(&pfd, 1, 50);
                if (result > 0 && (pfd.revents & POLLIN)) {
                    char data[32768]; const auto n = recv(fd, data, sizeof(data), 0);
                    if (n <= 0) disconnect();
                    else {
                        last_data = now_ms();
                        try {
                            parser.feed(data, static_cast<size_t>(n), [&](const ppe::Json& meta) {
                                if (engine.observe(meta, now_ms())) { last_valid = now_ms(); ++valid_frames; }
                            });
                        } catch (const std::exception& ex) { last_error = ex.what(); disconnect(); }
                    }
                } else if ((result > 0 && (pfd.revents & (POLLHUP | POLLERR | POLLNVAL))) || now - last_data > 5000) disconnect();
            } else usleep(50000);
            if (child > 0) {
                int code = 0;
                const auto done = waitpid(child, &code, WNOHANG);
                if (done == child) {
                    const bool okay = WIFEXITED(code) && WEXITSTATUS(code) == 0;
                    engine.completed(static_cast<unsigned>(active), now_ms(), okay);
                    if (!okay) { ++failures; last_error = "hdmi_playback_failed_or_disconnected"; }
                    else last_error.clear();
                    std::cout << "PPE_VOICE result label=" << ppe::labels[active] << " success=" << okay << std::endl;
                    child = -1; active = -1;
                } else if (now_ms() - player_started > player_timeout) {
                    kill(-child, SIGKILL); waitpid(child, &code, 0);
                    ++failures; last_error = "hdmi_player_timeout"; child = -1; active = -1;
                }
            }
            if (child < 0 && last_valid && now_ms() - last_valid <= settings.max_age) {
                const int ready = engine.ready(now_ms());
                if (ready >= 0) {
                    engine.attempted(static_cast<unsigned>(ready), now_ms());
                    const auto file = assets + '/' + ppe::files[ready];
                    child = fork();
                    if (child == 0) { setsid(); execl(player.c_str(), player.c_str(), file.c_str(), static_cast<char*>(nullptr)); _exit(127); }
                    if (child > 0) {
                        active = ready; player_started = now_ms();
                        std::cout << "PPE_VOICE alert label=" << ppe::labels[ready] << " text=" << ppe::phrases[ready] << std::endl;
                    } else { ++failures; last_error = "player_fork_failed"; }
                }
            }
            state = child > 0 ? "speaking" : last_valid && now_ms() - last_valid <= settings.max_age ? "monitoring" : "waiting_for_detector";
            if (now_ms() - last_status >= 1000) {
                ppe::Json out{{"version", "1.0"}, {"state", state}, {"source_port", port}, {"valid_frames", valid_frames},
                              {"playback_failures", failures}, {"last_error", last_error}, {"updated_mono_ms", now_ms()}, {"alerts", ppe::Json::array()}};
                for (unsigned i = 0; i < 3; ++i) out["alerts"].push_back({{"label", ppe::labels[i]}, {"text", ppe::phrases[i]}, {"spoken", engine.slots[i].count}});
                const auto tmp = status + ".tmp";
                { std::ofstream f(tmp); if (f) f << out.dump(2) << '\n'; }
                rename(tmp.c_str(), status.c_str()); last_status = now_ms();
            }
        }
        if (child > 0) { kill(-child, SIGKILL); waitpid(child, nullptr, 0); }
        if (fd >= 0) close(fd);
        close(lock);
        return 0;
    } catch (const std::exception& ex) { std::cerr << "PPE_VOICE: " << ex.what() << '\n'; return 1; }
}
