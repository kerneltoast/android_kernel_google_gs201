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

#include "aoc_ipc_core.h"

#ifdef __KERNEL__
#include <asm/barrier.h>
#include <linux/string.h>

#define WRITE_BARRIER() wmb()
#elif defined(EFW)
#include "memory_barrier.h"

#define WRITE_BARRIER() MB()
#else
#include <stdint.h>

typedef uint64_t u64;
typedef uint32_t u32;
typedef uint16_t u16;
typedef uint8_t u8;

#define WRITE_BARRIER()
#endif

#ifdef __cplusplus
extern "C" {
#endif

#define AOC_SERVICE_NAME_LENGTH 32
#define AOC_VERSION_LENGTH 48

#define AOC_SERVICE_FLAG_SYSTEM_MASK 0xffff0000
#define AOC_SERVICE_FLAG_RING        0x00010000
#define AOC_SERVICE_FLAG_RING_PUSH   0x01000000

#define AOC_SERVICE_TYPE_MASK   0x00070000
#define AOC_SERVICE_IRQ_MASK    0x00f80000
#define AOC_SERVICE_IRQ_SHIFT   19

enum AOC_SERVICE_TYPE {
	AOC_SERVICE_TYPE_QUEUE = 0,
	AOC_SERVICE_TYPE_RING,
	AOC_SERVICE_TYPE_BUFFER,
};

#define AOC_MAGIC 0xA0C00A0C
struct aoc_control_block {
	u32 magic;
	u32 fw_version;
	u32 hw_version;
	u32 status;
	u64 supported_features;
	u32 services;
	u32 service_size;
	u32 services_offset;
	u8 fw_version_name[AOC_VERSION_LENGTH];
	u8 hw_version_name[AOC_VERSION_LENGTH];
	u64 system_clock_offset;
} __attribute__((packed, aligned(4)));

struct aoc_ipc_memory_region {
	u32 offset;
	u32 size;
	u32 slots;
	u32 tx;
	u32 rx;
	u32 wp;
	u32 rp;
} __attribute__((packed, aligned(4)));

enum REGION_COMM {
	REGION_COMM_TX = 0,
	REGION_COMM_RX,

	REGION_COMM_TOT = 2
};

struct aoc_ipc_service_header {
	char name[AOC_SERVICE_NAME_LENGTH];
	u32 flags;

	struct aoc_ipc_memory_region regions[2];
} __attribute__((packed, aligned(4)));

struct aoc_ipc_message_header {
	u16 length;
	u16 r1;
	u16 r2;
	u16 r3;
} __attribute__((packed));

size_t aoc_service_current_message_size(aoc_service *service, void *base,
					aoc_direction dir);

size_t aoc_ring_bytes_available_to_read(aoc_service *service,
					aoc_direction dir);

/* Returns the amount of data that can be written *without* overwriting any
 * existing data.  Writes will always succeed, even if this returns 0 */
size_t aoc_ring_bytes_available_to_write(aoc_service *service,
					 aoc_direction dir);

bool aoc_ring_flush_read_data(aoc_service *service, aoc_direction dir, size_t bytes_to_leave);
bool aoc_ring_reset_write_pointer(aoc_service *service, aoc_direction dir);

void *aoc_service_current_message_pointer(aoc_service *service, void *base, aoc_direction dir);
void *aoc_service_current_read_pointer(struct aoc_ipc_service_header *service,
				       void *base, aoc_direction dir);
void *aoc_service_current_write_pointer(struct aoc_ipc_service_header *service,
					void *base, aoc_direction dir);

/* Convenience function to advance the indexes by 1 */
bool aoc_service_increment_write_index(aoc_service *service, aoc_direction dir);
bool aoc_service_increment_read_index(aoc_service *service, aoc_direction dir);

bool aoc_service_advance_write_index(aoc_service *service, aoc_direction dir,
				     u32 amount);
bool aoc_service_advance_read_index(aoc_service *service, aoc_direction dir,
				    u32 amount);

void *aoc_service_ring_base(aoc_service *service, void *base,
			    aoc_direction dir);
size_t aoc_service_ring_size(aoc_service *service, aoc_direction dir);
size_t aoc_service_ring_read_offset(aoc_service *service, aoc_direction dir);
size_t aoc_service_ring_write_offset(aoc_service *service, aoc_direction dir);

#ifdef __cplusplus
}
#endif
