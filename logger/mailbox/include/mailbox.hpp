#pragma once
#include <cstddef>
#include <memory>
#include <type_traits>
#include "mailbox.h"
#include "timing.hpp"

template <typename T, std::size_t N>
    requires std::is_trivially_copyable_v<T>
class Mailbox final
{
public:
    struct mail_t
    {
        mailhead head;
        T payload;
    };
    typedef T payload_t;
    static constexpr std::size_t capacity = N;
    Mailbox(const Mailbox&) = delete;
    Mailbox(Mailbox&&) = delete;
    Mailbox& operator=(const Mailbox&) = delete;
    Mailbox& operator=(Mailbox&&) = delete;
    Mailbox(void)
    {
        mb_init(&mb_);
        const std::size_t size = capacity*sizeof(mail_t);
        buf_ = std::unique_ptr<uint8_t[]>{new uint8_t[size]};
        for (std::size_t i = 0; i < capacity; ++i)
        {
            mail_t *mail = (mail_t*)(buf_.get()+sizeof(mail_t)*i);
            mb_manage(&mb_, &mail->head);
        }
    }
    ~Mailbox()
    {
        mb_fini(&mb_);
    }
    inline auto try_req(void)
    {
        auto mail = (mail_t*)mb_try_req(&mb_);
        return mail? &mail->payload : nullptr;
    }
    inline auto req(void)
    {
        auto mail = (mail_t*)mb_req(&mb_);
        return mail? &mail->payload : nullptr;
    }
    inline auto req(const timing::TimeDuration &d)
    {
        struct timespec ts = static_cast<timespec>(d);
        auto mail = (mail_t*)mb_req_for(&mb_, &ts);
        return mail? &mail->payload : nullptr;

    }
    inline auto req(const timing::TimePoint &t)
    {
        struct timespec ts = static_cast<timespec>(t);
        auto mail = (mail_t*)mb_req_until(&mb_, &ts);
        return mail? &mail->payload : nullptr;
    }
    inline auto take(void)
    {
        auto mail = (mail_t*)mb_take(&mb_);
        return mail? &mail->payload : nullptr;
    }
    inline void send(payload_t *payload)
    {
        auto mail = (mail_t*)(((uint8_t*)payload) - offsetof(mail_t, payload));
        mb_send(&mb_, &mail->head);
    }
    inline auto try_recv(void)
    {
        auto mail = (mail_t*)mb_try_recv(&mb_);
        return mail? &mail->payload : nullptr;
    }
    inline auto recv(void)
    {
        auto mail = (mail_t*)mb_recv(&mb_);
        return mail? &mail->payload : nullptr;
    }
    inline auto recv(const timing::TimeDuration &d)
    {
        struct timespec ts = static_cast<timespec>(d);
        auto mail = (mail_t*)mb_recv_for(&mb_, &ts);
        return mail? &mail->payload : nullptr;
    }
    inline auto recv(const timing::TimePoint &t)
    {
        struct timespec ts = static_cast<timespec>(t);
        auto mail = (mail_t*)mb_recv_until(&mb_, &ts);
        return mail? &mail->payload : nullptr;
    }
    inline void drop(payload_t *payload)
    {
        auto mail = (mail_t*)(((uint8_t*)payload) - offsetof(mail_t, payload));
        mb_drop(&mail->head);
    }
private:
    mailbox mb_; 
    std::unique_ptr<uint8_t[]> buf_;
};
