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
#include "aoc_ipc_core_internal.h"

/*
 * Due to this file being shared between the linux kernel and EFW with
 * very different coding styles, disable automatic formatting
 */

#define AOC_ASSERT(x)
#define VALID_DIRECTION(d) ((d == AOC_UP) || (d == AOC_DOWN))

#if __KERNEL__
#include <linux/io.h>
#define copy_to_buffer(dst, src, len) memcpy_toio(dst, src, len)
#define copy_from_buffer(dst, src, len) memcpy_fromio(dst, src, len)

#else
#define copy_to_buffer(dst, src, len) memcpy(dst, src, len)
#define copy_from_buffer(dst, src, len) memcpy(dst, src, len)

static inline void iowrite32(u32 value, void *addr)
{
	volatile u32 *r = addr;
	*r = value;
}

static inline u32 ioread32(void *addr)
{
	volatile u32 *r = addr;
	return *r;
}

#define EXPORT_SYMBOL(x)
#endif

static inline void _region_advance_tx(struct aoc_ipc_memory_region *r,
				      u32 amount)
{
	u32 sz = ioread32(&r->size);
	u32 tx = ioread32(&r->tx);
	u32 wp = ioread32(&r->wp);

	u32 updated_tx = (tx + amount);
	u32 updated_wp = (wp + amount) % sz;

	iowrite32(updated_tx, &r->tx);
	iowrite32(updated_wp, &r->wp);
}

static inline void _region_advance_rx(struct aoc_ipc_memory_region *r,
				      u32 amount)
{
	u32 sz = ioread32(&r->size);
	u32 rx = ioread32(&r->rx);
	u32 rp = ioread32(&r->rp);

	u32 updated_rx = (rx + amount);
	u32 updated_rp = (rp + amount) % sz;

	iowrite32(updated_rx, &r->rx);
	iowrite32(updated_rp, &r->rp);
}

static u32 _aoc_queue_write_buffer(u8 *dest, const u8 *src, u32 length);
static u32 _aoc_ring_read_buffer(const u8 *ring,
				 struct aoc_ipc_memory_region *r,
				 u8 *dest,
				 size_t *dest_size);

static u32 _read_write_delta(struct aoc_ipc_memory_region *r)
{
	u32 tx = ioread32(&r->tx);
	u32 rx = ioread32(&r->rx);

	if (tx >= rx) {
		return tx - rx;
	} else {
      // Here we are assuming that this is a wraparound and not a read
      // pointer that has been advanced past the write pointer or
      // a unsynced read pointer after a reset write pointer
		return (SIZE_MAX - rx) + tx + 1;
	}
}

static u32 _aoc_queue_write_buffer(u8 *dest, const u8 *src, u32 length)
{
	struct aoc_ipc_message_header hdr = {
		.length = length, .r1 = 0, .r2 = 0, .r3 = 0
	};

	copy_to_buffer(dest, &hdr, sizeof(hdr));
	dest += sizeof(hdr);

	copy_to_buffer(dest, src, length);
	return length;
}

static bool _aoc_ring_reader_sync_offset(struct aoc_ipc_memory_region *r,
					 size_t bytes_to_leave)
{
	u32 sz = r->size;
	u32 delta = _read_write_delta(r);

	if (bytes_to_leave > sz)
		bytes_to_leave = sz;

	if (delta < bytes_to_leave)
		bytes_to_leave = delta;

	_region_advance_rx(r, delta - bytes_to_leave);
	return true;
}

/* Returns the amount of bytes read from buffer */
static u32 _aoc_ring_read_buffer(const u8 *ring,
				 struct aoc_ipc_memory_region *r,
				 u8 *dest,
				 size_t *dest_size)
{
	u32 sz;
	u32 tx;
	u32 rx;
	u32 wp;
	u32 rp;
	u32 to_read;
	u32 bytes_available;

	bytes_available = _read_write_delta(r);
	sz = ioread32(&r->size);

	if (bytes_available > sz)
		_aoc_ring_reader_sync_offset(r, sz);

	tx = ioread32(&r->tx);
	rx = ioread32(&r->rx);
	wp = ioread32(&r->wp);
	rp = ioread32(&r->rp);

	if (wp > rp) {
		bytes_available = wp - rp;
	} else if (wp < rp) {
		bytes_available = wp + sz - rp;
	} else {
		if (tx == rx)
			bytes_available = 0;
		else
			bytes_available = sz;
	}

	if (bytes_available > *dest_size)
		to_read = *dest_size;
	else
		to_read = bytes_available;

	if (rp + to_read <= sz) {
		copy_from_buffer(dest, ring + rp, to_read);
	} else {
		u32 partial = sz - rp;

		copy_from_buffer(dest, ring + rp, partial);
		copy_from_buffer(dest + partial, ring, to_read - partial);
	}

	*dest_size = to_read;
	return to_read;
}

static u32 _aoc_ring_write_buffer(u8 *ring,
				  struct aoc_ipc_memory_region *r,
				  const u8 *src,
				  size_t size)
{
	u32 wp = ioread32(&r->wp);
	u32 sz = r->size;

	u32 to_write = size;

	if (to_write > sz) {
		/* Allow the write, but only commit the beginning */
		to_write = sz;
	}

	if (wp + to_write <= sz) {
		copy_to_buffer(ring + wp, src, to_write);
	} else {
		size_t partial = sz - wp;

		copy_to_buffer(ring + wp, src, partial);
		copy_to_buffer(ring, src + partial, to_write - partial);
	}

	return to_write;
}

const char *aoc_service_name(aoc_service *service)
{
	struct aoc_ipc_service_header *header = service;
	if (header->name[AOC_SERVICE_NAME_LENGTH - 1] != '\0')
		return NULL;

	return header->name;
}

static inline int aoc_service_type(aoc_service *service)
{
	struct aoc_ipc_service_header *header = service;
	return (header->flags & AOC_SERVICE_TYPE_MASK) >> 16;
}

bool aoc_service_is_queue(aoc_service *service)
{
	return aoc_service_type(service) == AOC_SERVICE_TYPE_QUEUE;
}

bool aoc_service_is_ring(aoc_service *service)
{
	return aoc_service_type(service) == AOC_SERVICE_TYPE_RING;
}

bool aoc_service_is_buffer(aoc_service *service)
{
	return aoc_service_type(service) == AOC_SERVICE_TYPE_BUFFER;
}

int aoc_service_irq_index(aoc_service *service)
{
        struct aoc_ipc_service_header *header = service;
        return (header->flags & AOC_SERVICE_IRQ_MASK) >> AOC_SERVICE_IRQ_SHIFT;
}

size_t aoc_ring_bytes_read(aoc_service *service, aoc_direction dir)
{
	struct aoc_ipc_service_header *header = service;
	struct aoc_ipc_memory_region *region;

	if (!service || !aoc_service_is_ring(service) || !VALID_DIRECTION(dir))
		return 0;

	region = &header->regions[dir];
	return ioread32(&region->rx);
}
EXPORT_SYMBOL(aoc_ring_bytes_read);

size_t aoc_ring_bytes_written(aoc_service *service, aoc_direction dir)
{
	struct aoc_ipc_service_header *header = service;
	struct aoc_ipc_memory_region *region;

	if (!service || !aoc_service_is_ring(service) || !VALID_DIRECTION(dir))
		return 0;

	region = &header->regions[dir];
	return ioread32(&region->tx);
}
EXPORT_SYMBOL(aoc_ring_bytes_written);

bool aoc_ring_did_overflow(aoc_service *service, aoc_direction dir)
{
	struct aoc_ipc_service_header *header = service;
	struct aoc_ipc_memory_region *region;

	if (!service || !aoc_service_is_ring(service) || !VALID_DIRECTION(dir))
		return false;

	region = &header->regions[dir];

	return (_read_write_delta(region) > region->size);
}

size_t aoc_service_message_size(aoc_service *service, aoc_direction dir)
{
	struct aoc_ipc_service_header *header = service;
	struct aoc_ipc_memory_region *region;

	if (!header || !VALID_DIRECTION(dir))
		return 0;

	region = &header->regions[dir];
	if (aoc_service_is_ring(service) || aoc_service_is_buffer(service)) {
		return region->size;
	} else {
		const size_t header_size =
			sizeof(struct aoc_ipc_message_header);
		if (region->size <= header_size)
			return 0;

		return region->size - header_size;
	}
}

size_t aoc_service_message_slots(aoc_service *service, aoc_direction dir)
{
	struct aoc_ipc_service_header *header = service;
	struct aoc_ipc_memory_region *region;

	if (!header || !VALID_DIRECTION(dir))
		return 0;

	region = &header->regions[dir];
	return ioread32(&region->slots);
}

size_t aoc_service_total_size(aoc_service *service, aoc_direction dir)
{
	struct aoc_ipc_service_header *header = service;
	struct aoc_ipc_memory_region *region;

	if (!header || !VALID_DIRECTION(dir))
		return 0;

	region = &header->regions[dir];
	return ioread32(&region->slots) * region->size;
}

size_t aoc_ring_bytes_available_to_read(aoc_service *service, aoc_direction dir)
{
	struct aoc_ipc_service_header *s;
	struct aoc_ipc_memory_region *region;
	size_t difference;

	if (!service || !VALID_DIRECTION(dir) ||
	    !(aoc_service_is_ring(service)))
		return 0;

	s = service;
	region = &s->regions[dir];
	difference = _read_write_delta(region);

	return difference > region->size ? region->size : difference;
}
EXPORT_SYMBOL(aoc_ring_bytes_available_to_read);

size_t aoc_ring_bytes_available_to_write(aoc_service *service,
					 aoc_direction dir)
{
	struct aoc_ipc_service_header *s;
	struct aoc_ipc_memory_region *region;

	if (!service || !VALID_DIRECTION(dir) ||
	    !(aoc_service_is_ring(service)))
		return 0;

	s = service;
	region = &s->regions[dir];

	return region->size - aoc_ring_bytes_available_to_read(service, dir);
}
EXPORT_SYMBOL(aoc_ring_bytes_available_to_write);

size_t aoc_service_slots_available_to_read(aoc_service *service,
					   aoc_direction dir)
{
	struct aoc_ipc_service_header *s = service;
	struct aoc_ipc_memory_region *region;
	if (!s || !VALID_DIRECTION(dir))
		return 0;

	region = &s->regions[dir];
	if (ioread32(&region->slots) == 0)
		return 0;

	if (aoc_service_is_buffer(service))
		return 0;

	if (aoc_service_is_ring(service)) {
		/*
     * Rings have one slot and the tx/rx counters specify bytes.  If the
     * byte counters match, there is nothing to read
     */
		return ioread32(&region->tx) != ioread32(&region->rx) ? 1 : 0;
	} else {
		size_t diff = _read_write_delta(region);

		/*
     * TODO : The API contract says that the difference will never be larger
     * than the # of slots on the channel.  Figure out what to do if this
     * assumption is ever false.
     */
		return diff > region->slots ? region->slots : diff;
	}
}

size_t aoc_service_slots_available_to_write(aoc_service *service,
					    aoc_direction dir)
{
	struct aoc_ipc_service_header *s = service;
	struct aoc_ipc_memory_region *region;
	if (!s || !VALID_DIRECTION(dir))
		return 0;

	region = &s->regions[dir];
	if (ioread32(&region->slots) == 0)
		return 0;

	if (aoc_service_is_ring(service)) {
		return 1;
	} else if (aoc_service_is_buffer(service)) {
		return 0;
	} else {
		size_t diff = _read_write_delta(region);

		AOC_ASSERT(diff <= ioread32(&region->slots));
		return ioread32(&region->slots) - diff;
	}
}

void *aoc_service_current_read_pointer(struct aoc_ipc_service_header *service,
				       void *base, aoc_direction dir)
{
	struct aoc_ipc_memory_region *region;
	u8 *ptr;
	u32 offset;

	if (!service || !base || !VALID_DIRECTION(dir))
		return NULL;

	region = &service->regions[dir];
	ptr = base;

	if (aoc_service_is_ring(service)) {
		offset = ioread32(&region->rp);
	} else {
		offset = (region->size *
			  (ioread32(&region->rx) % region->slots));
	}

	return ptr + region->offset + offset;
}

void *aoc_service_current_write_pointer(struct aoc_ipc_service_header *service,
					void *base, aoc_direction dir)
{
	struct aoc_ipc_memory_region *region;
	u8 *ptr = base;
	u32 offset;

	if (!service || !base || !VALID_DIRECTION(dir))
		return NULL;

	region = &service->regions[dir];
	ptr = base;

	if (aoc_service_is_ring(service)) {
		offset = ioread32(&region->wp);
	} else {
		offset = (region->size *
			  (ioread32(&region->tx) % region->slots));
	}

	return ptr + region->offset + offset;
}

size_t aoc_service_current_message_size(aoc_service *service, void *base,
					aoc_direction dir)
{
	struct aoc_ipc_message_header *hdr;

	if (!aoc_service_can_read_message(service, dir))
		return 0;

	hdr = aoc_service_current_read_pointer(service, base, dir);
	return hdr->length;
}

void *aoc_service_current_message_pointer(aoc_service *service, void *base,
					  aoc_direction dir)
{
	uint8_t *ptr;

	if (!aoc_service_can_read_message(service, dir)) {
		return NULL;
	}

	ptr = aoc_service_current_read_pointer(service, base, dir);
	ptr += sizeof(struct aoc_ipc_message_header);

	return ptr;
}

bool aoc_service_increment_write_index(aoc_service *service, aoc_direction dir)
{
	return aoc_service_advance_write_index(service, dir, 1);
}

bool aoc_service_increment_read_index(aoc_service *service, aoc_direction dir)
{
	return aoc_service_advance_read_index(service, dir, 1);
}

bool aoc_service_advance_write_index(aoc_service *service, aoc_direction dir,
				     u32 amount)
{
	struct aoc_ipc_memory_region *region;
	struct aoc_ipc_service_header *s;
	size_t available;

	if (!service)
		return false;

	available = aoc_service_slots_available_to_write(service, dir);
	if (aoc_service_is_ring(service))
		available = aoc_service_message_size(service, dir);

	if (available < amount)
		return false;

	s = service;

	region = &s->regions[dir];
	_region_advance_tx(region, amount);
	return true;
}
EXPORT_SYMBOL_GPL(aoc_service_advance_write_index);

bool aoc_service_advance_read_index(aoc_service *service, aoc_direction dir,
				    u32 amount)
{
	struct aoc_ipc_memory_region *region;
	struct aoc_ipc_service_header *s;
	u32 bytes_to_advance = amount;

	s = service;
	if (!s)
		return false;

	if (aoc_service_is_ring(service)) {
		u32 bytes_available = aoc_ring_bytes_available_to_read(service, dir);

		if (bytes_available < amount)
			bytes_to_advance = bytes_available;
	} else {
		if (aoc_service_slots_available_to_read(service, dir) < amount)
			return false;
	}

	region = &s->regions[dir];
	_region_advance_rx(region, bytes_to_advance);
	return (bytes_to_advance == amount);
}
EXPORT_SYMBOL_GPL(aoc_service_advance_read_index);

bool aoc_ring_flush_read_data(aoc_service *service, aoc_direction dir,
			      size_t bytes_to_leave)
{
	struct aoc_ipc_memory_region *r;
	struct aoc_ipc_service_header *s;

	s = service;
	if (!s || !aoc_service_is_ring(service))
		return false;

	r = &s->regions[dir];
	return _aoc_ring_reader_sync_offset(r, bytes_to_leave);
}
EXPORT_SYMBOL(aoc_ring_flush_read_data);


/*
 * To reset the aoc ring write ptr to the beginning of the aoc ring buffer.
 */
bool aoc_ring_reset_write_pointer(aoc_service *service, aoc_direction dir)
{
	struct aoc_ipc_memory_region *r;
	struct aoc_ipc_service_header *s;
	int bytes_remaining;
	u32 sz;
	u32 wp;

	if (!service || !aoc_service_is_ring(service))
		return false;

	s = service;
	r = &s->regions[dir];

	sz = ioread32(&r->size);
	wp = ioread32(&r->wp);
	bytes_remaining = sz - wp;

	if (bytes_remaining > 0)
		_region_advance_tx(r, bytes_remaining);

	return true;
}
EXPORT_SYMBOL_GPL(aoc_ring_reset_write_pointer);

bool aoc_service_can_read_message(aoc_service *service, aoc_direction dir)
{
	return aoc_service_slots_available_to_read(service, dir) > 0;
}

bool aoc_service_can_write_message(aoc_service *service, aoc_direction dir)
{
	return aoc_service_slots_available_to_write(service, dir) > 0;
}

bool aoc_service_read_message(aoc_service *service, void *base,
			      aoc_direction dir, void *buffer, size_t *size)
{
	struct aoc_ipc_service_header *s =
		(struct aoc_ipc_service_header *)service;
	struct aoc_ipc_memory_region *r;
	struct aoc_ipc_message_header *hdr;
	u8 *ptr;

	if (!service || !base || !buffer || !size || !VALID_DIRECTION(dir))
		return false;
	if (!aoc_service_can_read_message(service, dir))
		return false;

	r = &s->regions[dir];
	if (aoc_service_is_ring(service)) {
		u32 bytes_read;

		ptr = aoc_service_ring_base(service, base, dir);
		bytes_read = _aoc_ring_read_buffer(ptr, r, buffer, size);

		_region_advance_rx(r, bytes_read);
	} else {
		ptr = aoc_service_current_read_pointer(service, base, dir);
		hdr = (struct aoc_ipc_message_header *)ptr;
		ptr += sizeof(struct aoc_ipc_message_header);

		/* Validate length */
		if (*size < hdr->length)
			return false;

		/* TODO: Length should always be little endian */
		copy_from_buffer(buffer, ptr, hdr->length);
		*size = hdr->length;

		aoc_service_increment_read_index(service, dir);
	}

	return true;
}

bool aoc_service_write_message(aoc_service *service, void *base,
			       aoc_direction dir, const void *buffer,
			       size_t size)
{
	struct aoc_ipc_service_header *s =
		(struct aoc_ipc_service_header *)service;
	struct aoc_ipc_memory_region *r;
	u8 *ptr;

	if (!service || !base || !buffer || !VALID_DIRECTION(dir))
		return false;
	r = &s->regions[dir];

	if (!aoc_service_can_write_message(service, dir))
		return false;
	if (size > aoc_service_message_size(service, dir))
		return false;

	if (aoc_service_is_ring(service)) {
		u32 bytes_written;

		ptr = aoc_service_ring_base(service, base, dir);
		bytes_written = _aoc_ring_write_buffer(ptr, r, buffer, size);

		WRITE_BARRIER();
		_region_advance_tx(r, bytes_written);
	} else {
		ptr = aoc_service_current_write_pointer(service, base, dir);
		_aoc_queue_write_buffer(ptr, buffer, size);

		WRITE_BARRIER();
		aoc_service_increment_write_index(service, dir);
	}

	return true;
}

void *aoc_service_ring_base(aoc_service *service, void *base, aoc_direction dir)
{
	struct aoc_ipc_service_header *s =
		(struct aoc_ipc_service_header *)service;
	u32 size, offset;

	if (!service || !aoc_service_is_ring(service) || !VALID_DIRECTION(dir))
		return NULL;

	size = s->regions[dir].size;
	offset = s->regions[dir].offset;

	return (size > 0) ? (void *)((uintptr_t)base + offset) : NULL;
}

size_t aoc_service_ring_size(aoc_service *service, aoc_direction dir)
{
	struct aoc_ipc_service_header *s =
		(struct aoc_ipc_service_header *)service;

	if (!service || !aoc_service_is_ring(service) || !VALID_DIRECTION(dir))
		return 0;

	return s->regions[dir].size;
}

size_t aoc_service_ring_read_offset(aoc_service *service, aoc_direction dir)
{
	struct aoc_ipc_service_header *s =
		(struct aoc_ipc_service_header *)service;

	if (!service || !aoc_service_is_ring(service) || !VALID_DIRECTION(dir))
		return 0;

	return ioread32(&s->regions[dir].rp);
}

size_t aoc_service_ring_write_offset(aoc_service *service, aoc_direction dir)
{
	struct aoc_ipc_service_header *s =
		(struct aoc_ipc_service_header *)service;

	if (!service || !aoc_service_is_ring(service) || !VALID_DIRECTION(dir))
		return 0;

	return ioread32(&(s->regions[dir].wp));
}

bool aoc_ring_is_push(aoc_service *service)
{
	struct aoc_ipc_service_header *s =
		(struct aoc_ipc_service_header *)service;

	if (!service || !aoc_service_is_ring(service))
		return false;

	return (s->flags & AOC_SERVICE_FLAG_RING_PUSH) != 0;
}
