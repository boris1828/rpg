#pragma once

#include <chrono>
#include <unordered_map>
#include <string>
#include <fstream>
#include <iomanip>

// #define USE_REMOTARY

#ifdef USE_REMOTARY

extern "C" 
{
    #include "Remotery.h"
}

struct RmtScopeGuard 
{
    rmtU32 hash = 0; 

    RmtScopeGuard(const char* name) { _rmt_BeginCPUSample(name, 0, &hash); }
    ~RmtScopeGuard() { _rmt_EndCPUSample(); }
};

#define PROFILE_FUNCTION() RmtScopeGuard rmt_guard_func(__func__)
// #define PROFILE_SCOPE(name) RmtScopeGuard rmt_guard_##__LINE__(name)

#else
    #define PROFILE_FUNCTION()
    #define PROFILE_SCOPE(name)
#endif

#if true

template <typename T, typename = void>
struct is_complete : std::false_type {};

template <typename T>
struct is_complete<T, std::void_t<decltype(sizeof(T))>> : std::true_type {};

#define ASSERT_STRUCT_IS_DEFINED(struct_name) static_assert(is_complete<struct_name>::value, #struct_name " must be defined!");

#define ASSERT(expr, msg)                                                \
    do {                                                                 \
        if (!(expr)) [[unlikely]] {                                      \
            std::cerr << "[FATAL] " << msg                               \
                        << " (" << __FILE__ << ":" << __LINE__ << ")\n"; \
            std::abort();                                                \
        }                                                                \
    } while(0)

#define WARNING(expr, msg)                                               \
    do {                                                                 \
        if ((expr)) [[unlikely]] {                                       \
            std::cerr << "[WARNING] " << msg                             \
                        << " (" << __FILE__ << ":" << __LINE__ << ")\n"; \
        }                                                                \
    } while(0)

template<typename T, typename... Ts>
constexpr bool isOneOf(T value, Ts... allowed)
{
    return ((value == allowed) || ...); 
}

#define ASSERT_ONE_OF(value, message, ...) ASSERT(isOneOf(value, __VA_ARGS__), message)

#else

#define ASSERT(expr, msg)                   do { } while(0)
#define WARNING(expr, msg)                  do { } while(0)
#define ASSERT_ONE_OF(value, message, ...)  do { } while(0)
#define ASSERT_STRUCT_IS_DEFINED(struct_name) 

#endif

// PROFILING

inline std::ofstream& getSystemProfileStream() {
    static bool first = true;
    static std::ofstream profile_file("system_profile.log", std::ios::out);
    if (first) {
        profile_file.close();
        profile_file.open("system_profile.log", std::ios::out | std::ios::trunc);
        first = false;
    }
    return profile_file;
}

inline auto& getSystemProfilerMap() {
    static std::unordered_map<const char*, double> time_spent_ms;
    return time_spent_ms;
}

inline auto& getSystemStartTimes() {
    static std::unordered_map<const char*, std::chrono::high_resolution_clock::time_point> start_times;
    return start_times;
}

inline void profileSystemPrint(int steps) 
{
    std::ofstream out("system_profile.log", std::ios::out | std::ios::trunc); 
    float tot_ms_per_sec = 0.0f;
    for (const auto& p : getSystemProfilerMap())                              
    {                                                                         
        float ms_per_sec = p.second / ((float)steps) * 60.f;
        tot_ms_per_sec += ms_per_sec;                  
        out << std::setw(30) << std::left << p.first << " : " << ms_per_sec << " ms\n";                                
    }   
    out << std::setw(30) << std::left << "tot time" << " : " << tot_ms_per_sec << " ms\n";                                                                  
    out.close(); 
}

#define PROFILE_SYSTEM_PRINT(steps) do { profileSystemPrint(steps); } while (0)

#define PROFILE_SYSTEM_BEGIN() \
    do { \
        auto _now = std::chrono::high_resolution_clock::now(); \
        getSystemStartTimes()[__func__] = _now; \
    } while(0)

#define PROFILE_SYSTEM_END() \
    do { \
        auto _end = std::chrono::high_resolution_clock::now(); \
        auto _start = getSystemStartTimes()[__func__]; \
        double _elapsed = std::chrono::duration<double, std::milli>(_end - _start).count(); \
        getSystemProfilerMap()[__func__] += _elapsed; \
    } while(0)


// TRACING

inline std::ofstream& getSystemTraceStream() {
    static bool first = true;
    static std::ofstream trace_file("system_trace.log", std::ios::out);
    if (first) {
        trace_file.close();
        trace_file.open("system_trace.log", std::ios::out | std::ios::trunc);
        first = false;
    }
    return trace_file;
}

#define ENABLE_SYSTEM_TRACE 0

#if ENABLE_SYSTEM_TRACE

#define TRACE_SYSTEM() \
    do {                                                                                                                     \
        auto now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());                                   \
        getSystemTraceStream() << "[" << std::put_time(std::localtime(&now), "%H:%M:%S") << "] " << __func__ << " called\n"; \
        getSystemTraceStream().flush();                                                                                      \
    } while(0)

#define CLEAR_SYSTEM_TRACE() \
    do { \
        std::ofstream("system_trace.log", std::ios::trunc).close(); \
    } while(0)

#else

#define TRACE_SYSTEM()       do { } while(0)
#define CLEAR_SYSTEM_TRACE() do { } while(0)

#endif