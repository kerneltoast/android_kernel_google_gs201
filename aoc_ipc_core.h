// SPDX-License-Identifier: GPL-2.0 OR Apache-2.0
/*
 * Copyright 2020 Google LLC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   https://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef AOC_IPC_CORE_H
#define AOC_IPC_CORE_H

#ifdef __cplusplus
extern "C" {
#endif

#ifdef __KERNEL__
#include <linux/kernel.h>
#else
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#endif

typedef void aoc_service;

/**
 * Direction indicates Tx/Rx for each side of the link
 * AOC UP	FW -> AP communication
 * AOC_DOWN 	AP -> FW communication
 */
typedef enum {
	AOC_UP = 0,
	AOC_DOWN = 1,
} aoc_direction;

/**
 * Check if a passed in service is a queue
 *
 * \param[in] service pointer to a service
 *
 * \return true if service is a message queue, otherwise false
 */
bool aoc_service_is_queue(aoc_service *service);

/**
 * Check if a passed in service is a ring
 *
 * \param[in] service pointer to a service
 *
 * \return true if service is a ring service, otherwise false
 */
bool aoc_service_is_ring(aoc_service *service);

/**
 * Check if a passed in service is a buffer
 *
 * \param[in] service pointer to a service
 *
 * \return true if service is a named buffer, otherwise false
 */
bool aoc_service_is_buffer(aoc_service *service);

/**
 * Return the interrupt index for a service
 *
 * \param[in] service pointer to a service
 *
 * \return interrupt index for the service
 */
int aoc_service_irq_index(aoc_service *service);

/**
 * Return the name of a service
 *
 * \param[in] service pointer to a service
 *
 * \return the name of the service, or NULL on error
 */
const char *aoc_service_name(aoc_service *service);

/**
 * Return the size of a message slot
 *
 * \param[in] service pointer to a service
 * \param[in] dir direction for the message
 *
 * \return the size of a message slot, or 0 on error
 */
size_t aoc_service_message_size(aoc_service *service, aoc_direction dir);

/**
 * Return the total number of message slots
 *
 * \param[in] service pointer to a service
 * \param[in] dir direction for the message
 *
 * \return total number of message slots, or 0 on error
 */
size_t aoc_service_message_slots(aoc_service *service, aoc_direction dir);

/**
 * Return the total size of the IPC region
 *
 * \param[in] service pointer to a service
 * \param[in] dir direction for the message
 *
 * \return total size for all messages on a service
 */
size_t aoc_service_total_size(aoc_service *service, aoc_direction dir);

/**
 * Return the number of pending messages to be read
 *
 * \param[in] service pointer to a service
 * \param[in] dir direction for the message
 *
 * \return number of pending messages
 */
size_t aoc_service_slots_available_to_read(aoc_service *service,
					   aoc_direction dir);

/**
 * Return the number of free slots available to write
 *
 * \param[in] service pointer to a service
 * \param[in] dir direction for the message
 *
 * \return number of slots available to write
 */
size_t aoc_service_slots_available_to_write(aoc_service *service,
					    aoc_direction dir);

/**
 * Check if any messages are available to read
 *
 * \param[in] service pointer to a service
 * \param[in] dir direction for the message
 *
 * \return true if there are pending messages, otherwise false
 */
bool aoc_service_can_read_message(aoc_service *service, aoc_direction dir);

/**
 * Check if there is a free message slot
 *
 * \param[in] service pointer to a service
 * \param[in] dir direction for the message
 *
 * \return true if a message can be written, otherwise false
 */
bool aoc_service_can_write_message(aoc_service *service, aoc_direction dir);

/**
 * Read one message out of the queue
 *
 * \param[in] service pointer to a service
 * \param[in] base pointer to the base of IPC memory
 * \param[in] dir direction for the message
 * \param[out] dst memory to copy the message to
 * \param[inout] size size of the buffer, then size of the outgoing message
 *
 * \return true if a message was read, otherwise false
 */
bool aoc_service_read_message(aoc_service *service, void *base,
			      aoc_direction dir, void *dst, size_t *size);

/**
 * Write a message to the queue
 *
 * \param[in] service pointer to a service
 * \param[in] base pointer to the base of IPC memory
 * \param[in] dir direction for the message
 * \param[in] dst memory to copy the message from
 * \param[in] size size of the incoming message
 *
 * \return true if a message was read, otherwise false
 */
bool aoc_service_write_message(aoc_service *service, void *base,
			       aoc_direction dir, const void *dst, size_t size);

/**
 * Total number of bytes read from a ring service
 *
 * \param[in] service pointer to a service
 * \param[in] dir direction for the message
 *
 * \return total number of bytes read from the service, up to UINT32_MAX
 */
size_t aoc_ring_bytes_read(aoc_service *service, aoc_direction dir);

/**
 * Total number of bytes written to a ring service
 *
 * \param[in] service pointer to a service
 * \param[in] dir direction for the message
 *
 * \return total number of bytes written to the service, up to UINT32_MAX
 */
size_t aoc_ring_bytes_written(aoc_service *service, aoc_direction dir);

/**
 * Tells if more data has been written to the ring than it's capacity
 *
 * \param[in] service pointer to a service
 * \param[in] dir direction for the message
 *
 * \return true if data has been overflowed, otherwise false
 */
bool aoc_ring_did_overflow(aoc_service *service, aoc_direction dir);

/**
 * Tells if notifications should be sent after writing data into the ring
 *
 * \param[in] service pointer to a service
 *
 * \return true if the receiver expects a notification after writes
 */
bool aoc_ring_is_push(aoc_service *service);

#ifdef __cplusplus
}
#endif

#endif /* AOC_IPC_CORE_H */
