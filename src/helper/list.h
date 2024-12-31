/* SPDX-License-Identifier: BSD-2-Clause */

/*
 * Copyright (c) 2010 Isilon Systems, Inc.
 * Copyright (c) 2010 iX Systems, Inc.
 * Copyright (c) 2010 Panasas, Inc.
 * Copyright (c) 2013-2016 Mellanox Technologies, Ltd.
 * Copyright (c) 2021-2024 by Antonio Borneo <borneo.antonio@gmail.com>
 * All rights reserved.
 */

/*
 * Circular doubly linked list implementation.
 *
 * The content of this file is mainly copied/inspired from FreeBSD code in:
 * https://cgit.freebsd.org/src/tree/
 * files:
 * sys/compat/linuxkpi/common/include/linux/list.h
 * sys/compat/linuxkpi/common/include/linux/types.h
 *
 * Last aligned with release/13.3.0:
 *
 * - Skip 'hlist_*' double linked lists with a single pointer list head.
 * - Expand WRITE_ONCE().
 * - Use TAB for indentation.
 * - Put macro arguments within parenthesis.
 * - Remove blank lines after an open brace '{'.
 * - Remove multiple assignment.
 *
 * There is an example of using this file in contrib/list_example.c.
 */

#ifndef OPENOCD_HELPER_LIST_H
#define OPENOCD_HELPER_LIST_H

/* begin OpenOCD changes */

#include <stddef.h>

struct list_head {
	struct list_head *next;
	struct list_head *prev;
};

/* end OpenOCD changes */

#define LIST_HEAD_INIT(name) { &(name), &(name) }

#define OOCD_LIST_HEAD(name) \
	struct list_head name = LIST_HEAD_INIT(name)

static inline void
INIT_LIST_HEAD(struct list_head *list)
{
	list->next = list;
	list->prev = list;
}

static inline int
list_empty(const struct list_head *head)
{
	return (head->next == head);
}

static inline int
list_empty_careful(const struct list_head *head)
{
	struct list_head *next = head->next;

	return ((next == head) && (next == head->prev));
}

static inline void
__list_del(struct list_head *prev, struct list_head *next)
{
	next->prev = prev;
	prev->next = next;
}

static inline void
__list_del_entry(struct list_head *entry)
{
	__list_del(entry->prev, entry->next);
}

static inline void
list_del(struct list_head *entry)
{
	__list_del(entry->prev, entry->next);
}

static inline void
list_replace(struct list_head *old, struct list_head *new)
{
	new->next = old->next;
	new->next->prev = new;
	new->prev = old->prev;
	new->prev->next = new;
}

static inline void
list_replace_init(struct list_head *old, struct list_head *new)
{
	list_replace(old, new);
	INIT_LIST_HEAD(old);
}

static inline void
linux_list_add(struct list_head *new, struct list_head *prev,
	struct list_head *next)
{
	next->prev = new;
	new->next = next;
	new->prev = prev;
	prev->next = new;
}

static inline void
list_del_init(struct list_head *entry)
{
	list_del(entry);
	INIT_LIST_HEAD(entry);
}

#define	list_entry(ptr, type, field)	container_of(ptr, type, field)

#define	list_first_entry(ptr, type, member) \
	list_entry((ptr)->next, type, member)

#define	list_last_entry(ptr, type, member)	\
	list_entry((ptr)->prev, type, member)

#define	list_first_entry_or_null(ptr, type, member) \
	(!list_empty(ptr) ? list_first_entry(ptr, type, member) : NULL)

#define	list_next_entry(ptr, member)					\
	list_entry(((ptr)->member.next), typeof(*(ptr)), member)

#define	list_safe_reset_next(ptr, n, member) \
	(n) = list_next_entry(ptr, member)

#define	list_prev_entry(ptr, member)					\
	list_entry(((ptr)->member.prev), typeof(*(ptr)), member)

#define	list_for_each(p, head)						\
	for (p = (head)->next; p != (head); p = (p)->next)

#define	list_for_each_safe(p, n, head)					\
	for (p = (head)->next, n = (p)->next; p != (head); p = n, n = (p)->next)

#define list_for_each_entry(p, h, field)				\
	for (p = list_entry((h)->next, typeof(*p), field); &(p)->field != (h); \
	    p = list_entry((p)->field.next, typeof(*p), field))

#define list_for_each_entry_safe(p, n, h, field)			\
	for (p = list_entry((h)->next, typeof(*p), field),		\
	    n = list_entry((p)->field.next, typeof(*p), field); &(p)->field != (h);\
	    p = n, n = list_entry(n->field.next, typeof(*n), field))

#define	list_for_each_entry_from(p, h, field) \
	for ( ; &(p)->field != (h); \
	    p = list_entry((p)->field.next, typeof(*p), field))

#define	list_for_each_entry_continue(p, h, field)			\
	for (p = list_next_entry((p), field); &(p)->field != (h);	\
	    p = list_next_entry((p), field))

#define	list_for_each_entry_safe_from(pos, n, head, member)			\
	for (n = list_entry((pos)->member.next, typeof(*pos), member);		\
	     &(pos)->member != (head);						\
	     pos = n, n = list_entry(n->member.next, typeof(*n), member))

#define	list_for_each_entry_reverse(p, h, field)			\
	for (p = list_entry((h)->prev, typeof(*p), field); &(p)->field != (h); \
	    p = list_entry((p)->field.prev, typeof(*p), field))

#define	list_for_each_entry_safe_reverse(p, n, h, field)		\
	for (p = list_entry((h)->prev, typeof(*p), field),		\
	    n = list_entry((p)->field.prev, typeof(*p), field); &(p)->field != (h); \
	    p = n, n = list_entry(n->field.prev, typeof(*n), field))

#define	list_for_each_entry_continue_reverse(p, h, field) \
	for (p = list_entry((p)->field.prev, typeof(*p), field); &(p)->field != (h); \
	    p = list_entry((p)->field.prev, typeof(*p), field))

#define	list_for_each_prev(p, h) for (p = (h)->prev; p != (h); p = (p)->prev)

#define	list_for_each_entry_from_reverse(p, h, field)	\
	for (; &p->field != (h);			\
	     p = list_prev_entry(p, field))

static inline void
list_add(struct list_head *new, struct list_head *head)
{
	linux_list_add(new, head, head->next);
}

static inline void
list_add_tail(struct list_head *new, struct list_head *head)
{
	linux_list_add(new, head->prev, head);
}

static inline void
list_move(struct list_head *list, struct list_head *head)
{
	list_del(list);
	list_add(list, head);
}

static inline void
list_move_tail(struct list_head *entry, struct list_head *head)
{
	list_del(entry);
	list_add_tail(entry, head);
}

static inline void
list_rotate_to_front(struct list_head *entry, struct list_head *head)
{
	list_move_tail(entry, head);
}

static inline void
list_bulk_move_tail(struct list_head *head, struct list_head *first,
	struct list_head *last)
{
	first->prev->next = last->next;
	last->next->prev = first->prev;
	head->prev->next = first;
	first->prev = head->prev;
	last->next = head;
	head->prev = last;
}

static inline void
linux_list_splice(const struct list_head *list, struct list_head *prev,
	struct list_head *next)
{
	struct list_head *first;
	struct list_head *last;

	if (list_empty(list))
		return;
	first = list->next;
	last = list->prev;
	first->prev = prev;
	prev->next = first;
	last->next = next;
	next->prev = last;
}

static inline void
list_splice(const struct list_head *list, struct list_head *head)
{
	linux_list_splice(list, head, head->next);
}

static inline void
list_splice_tail(struct list_head *list, struct list_head *head)
{
	linux_list_splice(list, head->prev, head);
}

static inline void
list_splice_init(struct list_head *list, struct list_head *head)
{
	linux_list_splice(list, head, head->next);
	INIT_LIST_HEAD(list);
}

static inline void
list_splice_tail_init(struct list_head *list, struct list_head *head)
{
	linux_list_splice(list, head->prev, head);
	INIT_LIST_HEAD(list);
}

/*
 * Double linked lists with a single pointer list head.
 * IGNORED
 */

static inline int list_is_singular(const struct list_head *head)
{
	return !list_empty(head) && (head->next == head->prev);
}

static inline void __list_cut_position(struct list_head *list,
		struct list_head *head, struct list_head *entry)
{
	struct list_head *new_first = entry->next;
	list->next = head->next;
	list->next->prev = list;
	list->prev = entry;
	entry->next = list;
	head->next = new_first;
	new_first->prev = head;
}

static inline void list_cut_position(struct list_head *list,
		struct list_head *head, struct list_head *entry)
{
	if (list_empty(head))
		return;
	if (list_is_singular(head) &&
		(head->next != entry && head != entry))
		return;
	if (entry == head)
		INIT_LIST_HEAD(list);
	else
		__list_cut_position(list, head, entry);
}

static inline int list_is_first(const struct list_head *list,
				const struct list_head *head)
{
	return (list->prev == head);
}

static inline int list_is_last(const struct list_head *list,
				const struct list_head *head)
{
	return list->next == head;
}

static inline size_t
list_count_nodes(const struct list_head *list)
{
	const struct list_head *lh;
	size_t count;

	count = 0;
	list_for_each(lh, list) {
		count++;
	}

	return (count);
}

/*
 * Double linked lists with a single pointer list head.
 * IGNORED
 */

/* begin OpenOCD extensions */

/**
 * list_for_each_entry_direction - iterate forward/backward over list of given type
 * @param is_fwd the iterate direction, true for forward, false for backward.
 * @param p      the type * to use as a loop cursor.
 * @param h      the head of the list.
 * @param field  the name of the list_head within the struct.
 */
#define list_for_each_entry_direction(is_fwd, p, h, field)					\
	for (p = list_entry(is_fwd ? (h)->next : (h)->prev, typeof(*p), field);	\
		&(p)->field != (h);													\
		p = list_entry(is_fwd ? (p)->field.next : (p)->field.prev, typeof(*p), field))

/**
 * list_rotate_left - rotate the list to the left
 * @param h the head of the list
 */
static inline void list_rotate_left(struct list_head *h)
{
	struct list_head *first;

	if (!list_empty(h)) {
		first = h->next;
		list_move_tail(first, h);
	}
}

/* end OpenOCD extensions */

#endif /* OPENOCD_HELPER_LIST_H */
