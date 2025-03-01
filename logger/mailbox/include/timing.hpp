#pragma once
#include <time.h>
#include <cmath>
#include <cstdint>

namespace timing {
    static const uint_least32_t NSEC_PER_SEC = 1000*1000*1000;
    class TimeDuration;
    class TimePoint
    {
    public:
        TimePoint(uint64_t sec = 0, uint32_t nsec = 0) : sec_(sec), nsec_(nsec) {}
        TimePoint(struct timespec t) : sec_(t.tv_sec), nsec_(t.tv_nsec) {}
        TimePoint(const TimePoint&) = default;
        TimePoint(TimePoint&&) = default;
        TimePoint& operator=(const TimePoint&) = default;
        TimePoint& operator=(TimePoint&&) = default;
        ~TimePoint() = default;
        inline explicit operator timespec(void) const
        {
            struct timespec res;
            res.tv_sec = sec_;
            res.tv_nsec = nsec_;
            return res;
        }
        inline explicit operator double(void) const
        {
            return sec_ + static_cast<double>(nsec_)/NSEC_PER_SEC;
        }
        inline uint64_t sec(void) const
        {
            return sec_;
        }
        inline uint32_t nsec(void) const
        {
            return nsec_;
        }
    private:
        uint64_t sec_;
        uint32_t nsec_;
    };

    class TimeDuration
    {
    public:
        TimeDuration(int64_t sec, int32_t nsec): sec_(sec), nsec_(nsec) {}
        TimeDuration(struct timespec t) : sec_(t.tv_sec), nsec_(t.tv_nsec) {}
        TimeDuration(double t): sec_(std::floor(t)), nsec_((t-std::floor(t))*NSEC_PER_SEC) {}
        inline int64_t sec(void) const
        {
            return sec_;
        }
        inline int32_t nsec(void) const
        {
            return nsec_;
        }
        inline explicit operator double(void) const
        {
            return sec_ + static_cast<double>(nsec_)/NSEC_PER_SEC;
        }
        inline explicit operator timespec(void) const
        {
            struct timespec res;
            res.tv_sec = sec_;
            res.tv_nsec = nsec_;
            return res;
        }
    private:
        int64_t sec_;
        int32_t nsec_;
    };

    static inline TimePoint now(void)
    {
        struct timespec t;
        clock_gettime(CLOCK_MONOTONIC, &t);
        return TimePoint(t);
    }

    static inline void sleep(const TimePoint &tp)
    {
        auto t = static_cast<struct timespec>(tp);
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &t, NULL);
    }

    static inline void sleep(const TimeDuration &td)
    {
        auto t = static_cast<struct timespec>(td);
        clock_nanosleep(CLOCK_MONOTONIC, 0, &t, NULL);
    }
}

static inline timing::TimePoint operator+(const timing::TimePoint &tp, const timing::TimeDuration &td)
{
    using namespace timing;
    uint_least32_t nsec = tp.nsec() + td.nsec();
    uint64_t sec = tp.sec() + td.sec();
    while (nsec >= NSEC_PER_SEC) {
        nsec -= NSEC_PER_SEC;
        sec += 1;
    }
    return timing::TimePoint(sec, nsec);
}

static inline timing::TimePoint operator+(const timing::TimePoint &tp, double t)
{
    using namespace timing;
    auto n = std::trunc(t);
    const TimeDuration td{(int64_t)n, (int32_t)((t-n)*NSEC_PER_SEC)};
    return tp + td;
}

static inline timing::TimePoint operator+(const timing::TimeDuration &td, const timing::TimePoint &tp)
{
    return tp + td;
}

static inline timing::TimeDuration operator-(const timing::TimePoint &tp1, const timing::TimePoint &tp2)
{
    using namespace timing;
    int_least32_t nsec = tp1.nsec() - tp2.nsec();
    int64_t sec = tp1.sec() - tp2.sec();
    while (nsec < 0 && sec > 0) {
        nsec += NSEC_PER_SEC;
        sec -= 1;
    }
    while (nsec > 0 && sec < 0) {
        nsec -= NSEC_PER_SEC;
        sec += 1;
    }
    return timing::TimeDuration(sec, nsec);
}

static inline auto operator <=>(const timing::TimePoint &tp1, const timing::TimePoint &tp2)
{
    if (tp1.sec() > tp2.sec()) {
        return 1;
    } else if (tp1.sec() < tp2.sec()) {
        return -1;
    } else {
        if (tp1.nsec() > tp2.nsec()) {
            return 1;
        } else if (tp1.nsec() < tp2.nsec()) {
            return -1;
        } else {
            return 0;
        }
    }
}

static inline auto operator*(const timing::TimeDuration &td, double factor)
{
    using namespace timing;
    const int64_t nsec = static_cast<int64_t>(NSEC_PER_SEC*factor)*td.sec() + static_cast<int64_t>(td.nsec()*factor);
    return timing::TimeDuration{nsec/NSEC_PER_SEC, static_cast<int32_t>(nsec%NSEC_PER_SEC)};
}

static inline auto operator*(double factor, const timing::TimeDuration &td)
{
    return td*factor;
}
