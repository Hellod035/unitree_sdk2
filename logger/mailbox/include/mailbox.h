#pragma once
#include <stdint.h>
#include <time.h>
#include <threads.h>

#include "bsd-list.h"

struct mailhead;
struct mailbox
{
    SIMPLEQ_HEAD(,mailhead) used;
    SIMPLEQ_HEAD(,mailhead) free;
    cnd_t free_cnd;
    mtx_t free_mtx;
    cnd_t used_cnd;
    mtx_t used_mtx;
};

struct mailhead
{
    SIMPLEQ_ENTRY(mailhead) entry;
    struct mailbox *owner;
};

void mb_init(struct mailbox *mb);

void mb_fini(struct mailbox *mb);

void mb_manage(struct mailbox *mb, struct mailhead *m);

struct mailhead *mb_try_req(struct mailbox *mb);

struct mailhead *mb_req(struct mailbox *mb);

struct mailhead *mb_req_until(struct mailbox *mb, const struct timespec *t);

struct mailhead *mb_req_for(struct mailbox *mb, const struct timespec *t);

struct mailhead *mb_take(struct mailbox *mb);

void mb_send(struct mailbox *mb, struct mailhead *m);

struct mailhead *mb_try_recv(struct mailbox *mb);

struct mailhead *mb_recv(struct mailbox *mb);

struct mailhead *mb_recv_until(struct mailbox *mb, const struct timespec *t);

struct mailhead *mb_recv_for(struct mailbox *mb, const struct timespec *t);

void mb_drop(struct mailhead *m);
