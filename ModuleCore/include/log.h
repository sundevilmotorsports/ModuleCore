#pragma once

#include <atomic>
#include <chrono>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <string>

class ModuleCoreLogger {
public:
    enum LogLevel {
        Debug = 0,
        Info = 1,
        Warn = 2,
        Error = 3,
        None = 4,
    };

    static ModuleCoreLogger & instance() {
        static ModuleCoreLogger inst;
        return inst;
    }

    template<typename... Args>
    static void debug(const char *fmt, Args... args) { instance().formatAndLog(Debug, fmt, args...); }
    template<typename... Args>
    static void info(const char *fmt, Args... args) { instance().formatAndLog(Info, fmt, args...); }
    template<typename... Args>
    static void warn(const char *fmt, Args... args) { instance().formatAndLog(Warn, fmt, args...); }
    template<typename... Args>
    static void error(const char *fmt, Args... args) { instance().formatAndLog(Error, fmt, args...); }

    void setLevel(LogLevel level) {
        level_.store(level);
    }

    [[nodiscard]] LogLevel level() const { return level_.load(); }

    ModuleCoreLogger(const ModuleCoreLogger &) = delete;
    ModuleCoreLogger & operator=(const ModuleCoreLogger &) = delete;

    void set_serial_logging(const bool on) { serial_ = on; }
    void set_can_logging(const bool on) { can_ = on; }

private:
    // TODO: Implement sending logs over CAN conditionally
    bool serial_ = true;
    bool can_    = false;

    ModuleCoreLogger() = default;
    ~ModuleCoreLogger() = default;

    void writeImpl(LogLevel lvl, const std::string &msg) {
        std::lock_guard<std::mutex> lock(mu_);

        const char *lvlstr = levelToString(lvl);
        std::printf("%s %s\n", lvlstr, msg.c_str());
        std::fflush(stdout);
    }

    [[nodiscard]] static const char * levelToString(LogLevel l) {
        switch (l) {
            case Debug: return "\x1B[32m[DEBUG]";
            case Info: return "\x1B[34m[INFO]";
            case Warn: return "\x1B[33m[WARN]";
            case Error: return "\x1B[31m[ERROR]";
            default: return "UNKNOWN";
        }
    }

    template<typename... Args>
    void formatAndLog(LogLevel lvl, const char *fmt, Args... args) {
        char buf[512];
        int n = std::snprintf(buf, sizeof(buf), fmt, args...);
        if (n < 0) return;
        if (static_cast<size_t>(n) < sizeof(buf)) {
            writeImpl(lvl, std::string(buf, static_cast<size_t>(n)));
            return;
        }

        size_t need = static_cast<size_t>(n) + 1;
        std::string out;
        out.resize(need);
        std::snprintf(&out[0], need, fmt, args...);
        if (!out.empty() && out.back() == '\0') out.pop_back();
        writeImpl(lvl, out);
    }

    std::atomic<LogLevel> level_{Info};
    std::mutex mu_;
};
