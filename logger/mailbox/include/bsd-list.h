// adapted from OpenBSD/NetBSD queue(www.openbsd.org, www.netbsd.org)
#pragma once

#ifndef NULL
#define	NULL	((void *)0)
#endif

#define _Q_INVALID ((void *)-1)
#define _Q_INVALIDATE(a) (a) = _Q_INVALID
// #define _Q_INVALIDATE(a)

/*! \page list 链表数据结构
 * 本文件包含了4种数据结构：
 *    1. 单向链表: singly-linked list
 *    2. 简单队列: simple queue
 *    3. 双向链表: double-linked list
 *    4. 尾队列:   tail queue
 *  \section slist 单链表
 *  单链表数据结构中每一个节点包含一个指针，指向链表中的下一个节点。
 *  单链表的空间占用是最小的，删除某一元素的时间复杂度是O(n)。从链表头部
 *  移除节点需要使用显式宏以保证效率。
 *  新元素可以插入到任意元素后，或者插入到链表的头部。单链表数据结构只支持
 *  前向遍历。单链表数据结构适用于大数据，不需要或者很少需要删除的应用；或者
 *  实现LIFO队列。
 *  声明链表的方法是
 *  \code{.c}
 *      struct SListNode {
 *          SLIST_ENTRY(SListNode) entry;
 *          int data;
 *      };
 *      SLIST_HEAD(SListHead, SListNode);
 *      struct SListHead head;
 *      struct SListNode node1 = {.data = 1};
 *      struct SListNode node2 = {.data = 2};
 *      SLIST_INSERT_HEAD(&head, &node1, entry);
 *      SLIST_INSERT_HEAD(&head, &node2, entry);
 *  \endcode
 *
 *  \section simpleq 简单队列
 *
 *  \image html simpleq.png
 *
 *  与单向链表，简单队列(simple queue)的头节点包含两个指针，一个指向队列的第一个元素
 *  一个指向队列最后一个节点的next指针。向simple queue尾部插入元素的时间复杂度为O(1)，
 *  合并两个simple queue的时间复杂度也是O(1)。
 *
 *  \code{.c}
 *      struct SimpleQNode {
 *          SIMPLEQ_ENTRY(SimpleQNode) entry;
 *          int data;
 *      };
 *      SIMPLEQ_HEAD(SimpleQHead, SimpleQNode);
 *      struct SimpleQHead head;
 *      struct SimpleQNode node1 = {.data = 1};
 *      struct SimpleQNode node2 = {.data = 2};
 *      SIMPLEQ_INSERT_HEAD(&head, &node1, entry);
 *      SIMPLEQ_INSERT_TAIL(&head, &node2, entry);
 *  \endcode
 *
 *  \section list 双向链表
 *
 *  \image html list.png
 *
 *  与单向链表不同的是，双向链表中每个节点包含两个指针，一个指向链表
 *  中的前一个节点，另一个指针指向前一个节点中的指针。不需要遍历即可
 *  从双向链表中删除任意节点；新元素可以被插入到指定元素的前面或者后面，
 *  链表仍旧只支持前向遍历。
 *  \code{.c}
 *      struct ListNode {
 *          LIST_ENTRY(ListNode) entry;
 *          int data;
 *      };
 *      LIST_HEAD(ListHead, ListNode);
 *      struct ListHead head;
 *      struct ListNode node1 = {.data = 1};
 *      struct ListNode node2 = {.data = 2};
 *      LIST_INSERT_HEAD(&head, &node1, entry);
 *      LIST_INSERT_HEAD(&head, &node2, entry);
 *  \endcode
 *  \section tailq 尾队列
 *
 *  \image html tailq.png
 *
 *  尾队列相比双向链表更复杂，尾队列的头节点包含两个指针。除了支持双向链表的所有操作外，
 *  尾队列还支持前向遍历和后向遍历。
 */

//////////////////////// 单链表 //////////////////////////////////
///

// 单链表头
#define SLIST_HEAD(name, type)						\
struct name {								        \
	struct type *slh_first;                  	    \
}

#define	SLIST_HEAD_INITIALIZER(head)   { NULL }

// 单链表节点
#define SLIST_ENTRY(type)						   \
struct {								           \
	struct type *sle_next;                         \
}

#define	SLIST_FIRST(head)	    ((head)->slh_first)
#define	SLIST_END(head)		    NULL
#define	SLIST_EMPTY(head)	    (SLIST_FIRST(head) == SLIST_END(head))
#define	SLIST_NEXT(elm, field)	((elm)->field.sle_next)

#define	SLIST_FOREACH(var, head, field)             \
	for((var) = SLIST_FIRST(head);                  \
	    (var) != SLIST_END(head);                   \
	    (var) = SLIST_NEXT(var, field))

#define	SLIST_FOREACH_SAFE(var, head, field, tvar)     \
	for ((var) = SLIST_FIRST(head);                    \
	    (var) && ((tvar) = SLIST_NEXT(var, field), 1); \
	    (var) = (tvar))

#define	SLIST_INIT(head) {                             \
	SLIST_FIRST(head) = SLIST_END(head);               \
}

// 将elm插入到slistelm后面，field是节点变量名
#define	SLIST_INSERT_AFTER(slistelm, elm, field) do {   \
	(elm)->field.sle_next = (slistelm)->field.sle_next; \
	(slistelm)->field.sle_next = (elm);                 \
} while (0)

// 将elm作为到链表head的第一个元素
#define	SLIST_INSERT_HEAD(head, elm, field) do {       \
	(elm)->field.sle_next = (head)->slh_first;         \
	(head)->slh_first = (elm);                         \
} while (0)

// 将elm后面的一个元素从链表中删除
#define	SLIST_REMOVE_AFTER(elm, field) do {                        \
	(elm)->field.sle_next = (elm)->field.sle_next->field.sle_next; \
} while (0)

// 将链表中的第一个元素删除
#define	SLIST_REMOVE_HEAD(head, field) do {                  \
	(head)->slh_first = (head)->slh_first->field.sle_next;   \
} while (0)

// 从链表head中删除元素elm，type是节点的类型
#define SLIST_REMOVE(head, elm, type, field) do {            \
	if ((head)->slh_first == (elm)) {                        \
		SLIST_REMOVE_HEAD((head), field);                    \
	} else {                                                 \
		struct type *curelm = (head)->slh_first;             \
									                         \
		while (curelm->field.sle_next != (elm))              \
			curelm = curelm->field.sle_next;                 \
		curelm->field.sle_next =                             \
		    curelm->field.sle_next->field.sle_next;          \
	}                                                        \
	_Q_INVALIDATE((elm)->field.sle_next);                    \
} while (0)

//////////////////////// 双向链表 //////////////////////////////////
///

#define LIST_HEAD(name, type)                       \
struct name {                                       \
	struct type *lh_first;                          \
}

#define LIST_HEAD_INITIALIZER(head)  { NULL }

#define LIST_ENTRY(type)                            \
struct {                                            \
	struct type *le_next;                           \
	struct type **le_prev;                          \
}

#define	LIST_FIRST(head)		((head)->lh_first)
#define	LIST_END(head)			NULL
#define	LIST_EMPTY(head)		(LIST_FIRST(head) == LIST_END(head))
#define	LIST_NEXT(elm, field)	((elm)->field.le_next)

#define LIST_FOREACH(var, head, field)              \
	for((var) = LIST_FIRST(head);                   \
	    (var)!= LIST_END(head);                     \
	    (var) = LIST_NEXT(var, field))

#define	LIST_FOREACH_SAFE(var, head, field, tvar)     \
	for ((var) = LIST_FIRST(head);                    \
	    (var) && ((tvar) = LIST_NEXT(var, field), 1); \
	    (var) = (tvar))

#define	LIST_INIT(head) do {           \
	LIST_FIRST(head) = LIST_END(head); \
} while (0)

#define LIST_INSERT_AFTER(listelm, elm, field) do {                \
	if (((elm)->field.le_next = (listelm)->field.le_next) != NULL) \
		(listelm)->field.le_next->field.le_prev =                  \
		    &(elm)->field.le_next;                                 \
	(listelm)->field.le_next = (elm);                              \
	(elm)->field.le_prev = &(listelm)->field.le_next;              \
} while (0)

#define	LIST_INSERT_BEFORE(listelm, elm, field) do {  \
	(elm)->field.le_prev = (listelm)->field.le_prev;  \
	(elm)->field.le_next = (listelm);                 \
	*(listelm)->field.le_prev = (elm);                \
	(listelm)->field.le_prev = &(elm)->field.le_next; \
} while (0)

#define LIST_INSERT_HEAD(head, elm, field) do {                  \
	if (((elm)->field.le_next = (head)->lh_first) != NULL)       \
		(head)->lh_first->field.le_prev = &(elm)->field.le_next; \
	(head)->lh_first = (elm);                                    \
	(elm)->field.le_prev = &(head)->lh_first;                    \
} while (0)

#define LIST_REMOVE(elm, field) do {                                \
	if ((elm)->field.le_next != NULL)                               \
		(elm)->field.le_next->field.le_prev = (elm)->field.le_prev; \
	*(elm)->field.le_prev = (elm)->field.le_next;			        \
	_Q_INVALIDATE((elm)->field.le_prev);                            \
	_Q_INVALIDATE((elm)->field.le_next);                            \
} while (0)

#define LIST_REPLACE(elm, elm2, field) do {                            \
	if (((elm2)->field.le_next = (elm)->field.le_next) != NULL)        \
		(elm2)->field.le_next->field.le_prev = &(elm2)->field.le_next; \
	(elm2)->field.le_prev = (elm)->field.le_prev;                      \
	*(elm2)->field.le_prev = (elm2);                                   \
	_Q_INVALIDATE((elm)->field.le_prev);                               \
	_Q_INVALIDATE((elm)->field.le_next);                               \
} while (0)

/////////////////////////////////// 简单队列 ///////////////////////////////
///

#define SIMPLEQ_HEAD(name, type)  \
struct name {					  \
	struct type *sqh_first;       \
	struct type **sqh_last;       \
}

#define SIMPLEQ_HEAD_INITIALIZER(head) { NULL, &(head).sqh_first }

#define SIMPLEQ_ENTRY(type)\
struct {                   \
	struct type *sqe_next; \
}

#define	SIMPLEQ_FIRST(head)	        ((head)->sqh_first)
#define	SIMPLEQ_END(head)	        NULL
#define	SIMPLEQ_EMPTY(head)	        (SIMPLEQ_FIRST(head) == SIMPLEQ_END(head))
#define	SIMPLEQ_NEXT(elm, field)    ((elm)->field.sqe_next)

#define SIMPLEQ_FOREACH(var, head, field) \
	for((var) = SIMPLEQ_FIRST(head);      \
	    (var) != SIMPLEQ_END(head);       \
	    (var) = SIMPLEQ_NEXT(var, field))

#define	SIMPLEQ_FOREACH_SAFE(var, head, field, tvar)     \
	for ((var) = SIMPLEQ_FIRST(head);                    \
	    (var) && ((tvar) = SIMPLEQ_NEXT(var, field), 1); \
	    (var) = (tvar))

#define	SIMPLEQ_INIT(head) do {           \
	(head)->sqh_first = NULL;             \
	(head)->sqh_last = &(head)->sqh_first;\
} while (0)

#define SIMPLEQ_INSERT_HEAD(head, elm, field) do {           \
	if (((elm)->field.sqe_next = (head)->sqh_first) == NULL) \
		(head)->sqh_last = &(elm)->field.sqe_next;           \
	(head)->sqh_first = (elm);                               \
} while (0)

#define SIMPLEQ_INSERT_TAIL(head, elm, field) do {           \
	(elm)->field.sqe_next = NULL;                            \
	*(head)->sqh_last = (elm);                               \
	(head)->sqh_last = &(elm)->field.sqe_next;               \
} while (0)

#define SIMPLEQ_INSERT_AFTER(head, listelm, elm, field) do {         \
	if (((elm)->field.sqe_next = (listelm)->field.sqe_next) == NULL) \
		(head)->sqh_last = &(elm)->field.sqe_next;                   \
	(listelm)->field.sqe_next = (elm);                               \
} while (0)

#define SIMPLEQ_REMOVE_HEAD(head, field) do {                            \
	if (((head)->sqh_first = (head)->sqh_first->field.sqe_next) == NULL) \
		(head)->sqh_last = &(head)->sqh_first;                           \
} while (0)

#define SIMPLEQ_REMOVE_AFTER(head, elm, field) do {                              \
	if (((elm)->field.sqe_next = (elm)->field.sqe_next->field.sqe_next) == NULL) \
		(head)->sqh_last = &(elm)->field.sqe_next;                               \
} while (0)

#define SIMPLEQ_CONCAT(head1, head2) do {       \
	if (!SIMPLEQ_EMPTY((head2))) {              \
		*(head1)->sqh_last = (head2)->sqh_first;\
		(head1)->sqh_last = (head2)->sqh_last;  \
		SIMPLEQ_INIT((head2));                  \
	}                                           \
} while (0)

////////////////////////////////////// 尾队列 ///////////////////////////////////////
#define TAILQ_HEAD(name, type) \
struct name {                  \
	struct type *tqh_first;    \
	struct type **tqh_last;    \
}

#define TAILQ_HEAD_INITIALIZER(head) { NULL, &(head).tqh_first }

#define TAILQ_ENTRY(type)   \
struct {                    \
	struct type *tqe_next;  \
	struct type **tqe_prev; \
}

#define	TAILQ_FIRST(head)		         ((head)->tqh_first)
#define	TAILQ_END(head)			         NULL
#define	TAILQ_NEXT(elm, field)		     ((elm)->field.tqe_next)
#define TAILQ_LAST(head, headname)       (*(((struct headname *)((head)->tqh_last))->tqh_last))
#define TAILQ_PREV(elm, headname, field) (*(((struct headname *)((elm)->field.tqe_prev))->tqh_last))
#define	TAILQ_EMPTY(head)                (TAILQ_FIRST(head) == TAILQ_END(head))

#define TAILQ_FOREACH(var, head, field) \
	for((var) = TAILQ_FIRST(head);      \
	    (var) != TAILQ_END(head);       \
	    (var) = TAILQ_NEXT(var, field))

#define	TAILQ_FOREACH_SAFE(var, head, field, tvar)                        \
	for ((var) = TAILQ_FIRST(head);                                       \
	    (var) != TAILQ_END(head) && ((tvar) = TAILQ_NEXT(var, field), 1); \
	    (var) = (tvar))


#define TAILQ_FOREACH_REVERSE(var, head, headname, field) \
	for((var) = TAILQ_LAST(head, headname);               \
	    (var) != TAILQ_END(head);                         \
	    (var) = TAILQ_PREV(var, headname, field))

#define	TAILQ_FOREACH_REVERSE_SAFE(var, head, headname, field, tvar)                \
	for ((var) = TAILQ_LAST(head, headname);                                        \
	    (var) != TAILQ_END(head) && ((tvar) = TAILQ_PREV(var, headname, field), 1); \
	    (var) = (tvar))

#define	TAILQ_INIT(head) do {             \
	(head)->tqh_first = NULL;             \
	(head)->tqh_last = &(head)->tqh_first;\
} while (0)

#define TAILQ_INSERT_HEAD(head, elm, field) do {                    \
	if (((elm)->field.tqe_next = (head)->tqh_first) != NULL)        \
		(head)->tqh_first->field.tqe_prev = &(elm)->field.tqe_next; \
	else                                                            \
		(head)->tqh_last = &(elm)->field.tqe_next;                  \
	(head)->tqh_first = (elm);                                      \
	(elm)->field.tqe_prev = &(head)->tqh_first;                     \
} while (0)

#define TAILQ_INSERT_TAIL(head, elm, field) do {   \
	(elm)->field.tqe_next = NULL;                  \
	(elm)->field.tqe_prev = (head)->tqh_last;      \
	*(head)->tqh_last = (elm);                     \
	(head)->tqh_last = &(elm)->field.tqe_next;     \
} while (0)

#define TAILQ_INSERT_AFTER(head, listelm, elm, field) do {              \
	if (((elm)->field.tqe_next = (listelm)->field.tqe_next) != NULL)    \
		(elm)->field.tqe_next->field.tqe_prev = &(elm)->field.tqe_next; \
	else                                                                \
		(head)->tqh_last = &(elm)->field.tqe_next;                      \
	(listelm)->field.tqe_next = (elm);                                  \
	(elm)->field.tqe_prev = &(listelm)->field.tqe_next;                 \
} while (0)

#define	TAILQ_INSERT_BEFORE(listelm, elm, field) do {       \
	(elm)->field.tqe_prev = (listelm)->field.tqe_prev;		\
	(elm)->field.tqe_next = (listelm);                      \
	*(listelm)->field.tqe_prev = (elm);                     \
	(listelm)->field.tqe_prev = &(elm)->field.tqe_next;		\
} while (0)

#define TAILQ_REMOVE(head, elm, field) do {                            \
	if (((elm)->field.tqe_next) != NULL)                               \
		(elm)->field.tqe_next->field.tqe_prev = (elm)->field.tqe_prev; \
	else                                                               \
		(head)->tqh_last = (elm)->field.tqe_prev;                      \
	*(elm)->field.tqe_prev = (elm)->field.tqe_next;                    \
	_Q_INVALIDATE((elm)->field.tqe_prev);                              \
	_Q_INVALIDATE((elm)->field.tqe_next);                              \
} while (0)

#define TAILQ_REPLACE(head, elm, elm2, field) do {                        \
	if (((elm2)->field.tqe_next = (elm)->field.tqe_next) != NULL)         \
		(elm2)->field.tqe_next->field.tqe_prev = &(elm2)->field.tqe_next; \
	else                                                                  \
		(head)->tqh_last = &(elm2)->field.tqe_next;                       \
	(elm2)->field.tqe_prev = (elm)->field.tqe_prev;                       \
	*(elm2)->field.tqe_prev = (elm2);                                     \
	_Q_INVALIDATE((elm)->field.tqe_prev);                                 \
	_Q_INVALIDATE((elm)->field.tqe_next);                                 \
} while (0)

#define TAILQ_CONCAT(head1, head2, field) do {                  \
	if (!TAILQ_EMPTY(head2)) {                                  \
		*(head1)->tqh_last = (head2)->tqh_first;                \
		(head2)->tqh_first->field.tqe_prev = (head1)->tqh_last; \
		(head1)->tqh_last = (head2)->tqh_last;                  \
		TAILQ_INIT((head2));                                    \
	}                                                           \
} while (0)
