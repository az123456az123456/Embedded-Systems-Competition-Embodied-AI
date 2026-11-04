#pragma once
#include <array>
#include <cmath>
#include <cstdint>
#include <string>
#include "nlohmann/json.hpp"

namespace ppe {
using Json = nlohmann::json;
inline constexpr std::array<const char*, 3> labels = {"NO-Hardhat", "NO-Mask", "NO-Safety Vest"};
inline constexpr std::array<const char*, 3> files = {"no-hardhat.wav", "no-mask.wav", "no-vest.wav"};
inline constexpr std::array<const char*, 3> phrases = {"检测到未穿安全帽", "检测到未穿口罩", "检测到未穿反光衣"};
struct Settings {
    double confidence = .35;
    uint64_t confirm = 300, cooldown = 15000, max_age = 1000;
};
struct Slot {
    uint64_t first = 0, seen = 0, attempted = 0, spoken = 0, count = 0;
    unsigned samples = 0;
};
class Engine {
    Settings cfg_;
    uint64_t last_stamp_ = 0;
    unsigned cursor_ = 0;
public:
    std::array<Slot, 3> slots{};
    explicit Engine(Settings cfg = {}) : cfg_(cfg) {}
    bool observe(const Json& m, uint64_t now) {
        try {
            if (m.value("demo", true) || !m.value("control_eligible", false) ||
                m.value("detections_pair_status", "") != "PAIR_COMPLETE" ||
                m.find("detection_timestamp_ms") == m.end() || !m["detection_timestamp_ms"].is_number_unsigned() ||
                m.find("detections") == m.end() || !m["detections"].is_array()) return false;
            const auto stamp = m["detection_timestamp_ms"].get<uint64_t>();
            if (!stamp || stamp > now || now - stamp > cfg_.max_age || stamp <= last_stamp_) return false;
            std::array<bool, 3> found{};
            if (m["detections"].size() > 256) return false;
            for (const auto& d : m["detections"]) {
                if (!d.is_object() || d.find("label") == d.end() || !d["label"].is_string() ||
                    d.find("score") == d.end() || !d["score"].is_number()) continue;
                const double score = d["score"].get<double>();
                if (!std::isfinite(score) || score < cfg_.confidence || score > 1) continue;
                for (unsigned i = 0; i < labels.size(); ++i)
                    if (d["label"] == labels[i]) found[i] = true;
            }
            last_stamp_ = stamp;
            for (unsigned i = 0; i < slots.size(); ++i) {
                auto& s = slots[i];
                if (!found[i]) { s.first = s.seen = 0; s.samples = 0; continue; }
                if (!s.seen || stamp - s.seen > cfg_.max_age) { s.first = now; s.samples = 0; }
                s.seen = stamp;
                ++s.samples;
            }
            return true;
        } catch (const std::exception&) { return false; }
    }
    int ready(uint64_t now) const {
        for (unsigned n = 0; n < slots.size(); ++n) {
            const unsigned i = (cursor_ + n) % slots.size();
            const auto& s = slots[i];
            if (s.first && s.samples >= 3 && now >= s.first && now - s.first >= cfg_.confirm &&
                now >= s.seen && now - s.seen <= cfg_.max_age &&
                (!s.spoken || now - s.spoken >= cfg_.cooldown) &&
                (!s.attempted || now - s.attempted >= 3000)) return static_cast<int>(i);
        }
        return -1;
    }
    void attempted(unsigned i, uint64_t now) { slots.at(i).attempted = now; cursor_ = (i + 1) % 3; }
    void completed(unsigned i, uint64_t now, bool success) {
        if (success) { slots.at(i).spoken = now; ++slots.at(i).count; }
    }
};
inline std::string unbase64(const std::string& text) {
    const std::string alphabet = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    std::string out;
    uint32_t acc = 0; unsigned bits = 0;
    if (text.size() > 262144) throw std::runtime_error("metadata header too large");
    for (char c : text) {
        if (c == '=') break;
        const auto pos = alphabet.find(c);
        if (pos == std::string::npos) throw std::runtime_error("invalid metadata base64");
        acc = (acc << 6) | static_cast<uint32_t>(pos); bits += 6;
        if (bits >= 8) { bits -= 8; out.push_back(static_cast<char>((acc >> bits) & 255)); }
    }
    return out;
}
// Consume complete multipart headers and skip JPEG bodies, never scan image data
// for metadata. The receive buffer is bounded independently of JPEG frame size.
class Multipart {
    std::string buffer_;
    size_t skip_ = 0;
public:
    template<class F> void feed(const char* data, size_t size, F callback) {
        buffer_.append(data, size);
        while (true) {
            if (skip_) {
                const auto n = std::min(skip_, buffer_.size());
                skip_ -= n; buffer_.erase(0, n);
                if (skip_) return;
            }
            const auto end = buffer_.find("\r\n\r\n");
            if (end == std::string::npos) {
                if (buffer_.size() > 262144) throw std::runtime_error("multipart header too large");
                return;
            }
            const auto header = buffer_.substr(0, end);
            buffer_.erase(0, end + 4);
            if (header.find("HTTP/1.") != std::string::npos) {
                if (header.find(" 200 ") == std::string::npos) throw std::runtime_error("detection HTTP request rejected");
                continue;
            }
            const std::string length_key = "Content-Length: ", meta_key = "X-Frame-Meta: ";
            const auto lp = header.find(length_key), mp = header.find(meta_key);
            if (lp == std::string::npos || mp == std::string::npos) throw std::runtime_error("missing multipart metadata");
            skip_ = std::stoull(header.substr(lp + length_key.size()));
            if (skip_ > 8 * 1024 * 1024) throw std::runtime_error("JPEG exceeds size limit");
            const auto begin = mp + meta_key.size(), finish = header.find("\r\n", begin);
            callback(Json::parse(unbase64(header.substr(begin, finish == std::string::npos ? finish : finish - begin))));
        }
    }
};
}
