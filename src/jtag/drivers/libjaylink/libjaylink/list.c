/*
 * This file is part of the libjaylink project.
 *
 * Copyright (C) 2014-2016 Marc Schink <jaylink-dev@marcschink.de>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdlib.h>

#include "libjaylink-internal.h"

/**
 * @file
 *
 * Singly-linked list functions.
 */

/** @private */
JAYLINK_PRIV struct list *list_prepend(struct list *list, void *data)
{
	struct list *item;

	item = malloc(sizeof(struct list));

	if (!item)
		return NULL;

	item->data = data;
	item->next = list;

	return item;
}

/** @private */
JAYLINK_PRIV struct list *list_remove(struct list *list, const void *data)
{
	struct list *item;
	struct list *tmp;

	if (!list)
		return NULL;

	item = list;

	if (item->data == data) {
		tmp = item->next;
		free(item);
		return tmp;
	}

	while (item->next) {
		if (item->next->data == data) {
			tmp = item->next;
			item->next = item->next->next;
			free(tmp);
			break;
		}

		item = item->next;
	}

	return list;
}

/** @private */
JAYLINK_PRIV struct list *list_find_custom(struct list *list,
		list_compare_callback callback, const void *user_data)
{
	if (!callback)
		return NULL;

	while (list) {
		if (callback(list->data, user_data))
			return list;

		list = list->next;
	}

	return NULL;
}

/** @private */
JAYLINK_PRIV size_t list_length(struct list *list)
{
	size_t length;

	for (length = 0; list; length++)
		list = list->next;

	return length;
}

/** @private */
JAYLINK_PRIV void list_free(struct list *list)
{
	struct list *tmp;

	while (list) {
		tmp = list;
		list = list->next;
		free(tmp);
	}
}
