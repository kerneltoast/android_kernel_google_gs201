/* SPDX-License-Identifier: GPL-2.0
 *
 * Synaptics TouchCom touchscreen driver
 *
 * Copyright (C) 2017-2020 Synaptics Incorporated. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * INFORMATION CONTAINED IN THIS DOCUMENT IS PROVIDED "AS-IS," AND SYNAPTICS
 * EXPRESSLY DISCLAIMS ALL EXPRESS AND IMPLIED WARRANTIES, INCLUDING ANY
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE,
 * AND ANY WARRANTIES OF NON-INFRINGEMENT OF ANY INTELLECTUAL PROPERTY RIGHTS.
 * IN NO EVENT SHALL SYNAPTICS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, PUNITIVE, OR CONSEQUENTIAL DAMAGES ARISING OUT OF OR IN CONNECTION
 * WITH THE USE OF THE INFORMATION CONTAINED IN THIS DOCUMENT, HOWEVER CAUSED
 * AND BASED ON ANY THEORY OF LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * NEGLIGENCE OR OTHER TORTIOUS ACTION, AND EVEN IF SYNAPTICS WAS ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE. IF A TRIBUNAL OF COMPETENT JURISDICTION DOES
 * NOT PERMIT THE DISCLAIMER OF DIRECT DAMAGES OR ANY OTHER DAMAGES, SYNAPTICS'
 * TOTAL CUMULATIVE LIABILITY TO ANY PARTY SHALL NOT EXCEED ONE HUNDRED U.S.
 * DOLLARS.
 */

/**
 * @file syna_tcm2_runtime.h
 *
 * This file abstracts platform-specific headers and C runtime APIs being used
 * on the target platform.
 */

#ifndef _SYNAPTICS_TCM2_C_RUNTIME_H_
#define _SYNAPTICS_TCM2_C_RUNTIME_H_

#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/input.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/major.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/regulator/consumer.h>
#include <linux/platform_device.h>
#include <linux/input/mt.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/uaccess.h>
#include <linux/crc32.h>
#include <linux/firmware.h>
#if defined(USE_DRM_BRIDGE)
#include <drm/drm_bridge.h>
#elif defined(CONFIG_DRM_PANEL)
#include <drm/drm_panel.h>
#elif CONFIG_FB
#include <linux/fb.h>
#include <linux/notifier.h>
#endif
#include <linux/fs.h>
#include <linux/moduleparam.h>
#include <linux/kfifo.h>

/**
 * @brief: DEV_MANAGED_API
 *
 * For linux kernel, managed interface was created for resources commonly
 * used by device drivers using devres.
 *
 * Open if willing to use managed-APIs rather than legacy APIs.
 */
#define DEV_MANAGED_API

#if defined(DEV_MANAGED_API) || defined(USE_DRM_PANEL_NOTIFIER)
extern struct device *syna_request_managed_device(void);
#endif

/**
 * @section: Log helpers
 *
 * @brief: LOGD
 *         Output the debug message
 *
 * @brief: LOGI
 *         Output the info message
 *
 * @brief: LOGN
 *         Output the notice message
 *
 * @brief: LOGW
 *         Output the warning message
 *
 * @brief: LOGE
 *         Output the error message
 */
#define LOGD(log, ...) \
	pr_debug("[  debug] %s: " log, __func__, ##__VA_ARGS__)
#define LOGI(log, ...) \
	pr_info("[   info] %s: " log, __func__, ##__VA_ARGS__)
#define LOGN(log, ...) \
	pr_notice("[   info] %s: " log, __func__, ##__VA_ARGS__)
#define LOGW(log, ...) \
	pr_warn("[warning] %s: " log, __func__, ##__VA_ARGS__)
#define LOGE(log, ...) \
	pr_err("[  error] %s: " log, __func__, ##__VA_ARGS__)


/**
 * @section: Error Codes returned
 *           Functions usually return 0 or positive value on success.
 *           Thus, please defines negative value here.
 */
#define _EIO        (-EIO)       /* I/O errors */
#define _ENOMEM     (-ENOMEM)    /* Out of memory */
#define _EINVAL     (-EINVAL)    /* Invalid parameters */
#define _ENODEV     (-ENODEV)    /* No such device */
#define _ETIMEDOUT  (-ETIMEDOUT) /* execution timeout */


/**
 * @section: Data Comparison helpers
 *
 * @brief: MAX
 *         Find the maximum value between
 *
 * @brief: MIN:
 *         Find the minimum value between
 *
 * @brief: GET_BIT
 *         Return the value of target bit
 */
#define MAX(a, b) \
	({__typeof__(a) _a = (a); \
	__typeof__(b) _b = (b); \
	_a > _b ? _a : _b; })

#define MIN(a, b) \
	({__typeof__(a) _a = (a); \
	__typeof__(b) _b = (b); \
	_a < _b ? _a : _b; })

#define GET_BIT(var, pos) \
	(((var) & (1 << (pos))) >> (pos))


/**
 * @section: C Atomic operations
 *
 * @brief: ATOMIC_SET
 *         Set an atomic data
 *
 * @brief: ATOMIC_GET:
 *         Get an atomic data
 */
typedef atomic_t syna_pal_atomic_t;

#define ATOMIC_SET(atomic, value) \
	atomic_set(&atomic, value)

#define ATOMIC_GET(atomic) \
	atomic_read(&atomic)


/**
 * @section: C Integer Calculation helpers
 *
 * @brief: syna_pal_le2_to_uint
 *         Convert 2-byte data to an unsigned integer
 *
 * @brief: syna_pal_le4_to_uint
 *         Convert 4-byte data to an unsigned integer
 *
 * @brief: syna_pal_ceil_div
 *         Calculate the ceiling of the integer division
 */

/**
 * syna_pal_le2_to_uint()
 *
 * Convert 2-byte data in little-endianness to an unsigned integer
 *
 * @param
 *    [ in] src: 2-byte data in little-endianness
 *
 * @return
 *    an unsigned integer converted
 */
static inline unsigned int syna_pal_le2_to_uint(const unsigned char *src)
{
	return (unsigned int)src[0] +
		(unsigned int)src[1] * 0x100;
}
/**
 * syna_pal_le4_to_uint()
 *
 * Convert 4-byte data in little-endianness to an unsigned integer
 *
 * @param
 *    [ in] src: 4-byte data in little-endianness
 *
 * @return
 *    an unsigned integer converted
 */
static inline unsigned int syna_pal_le4_to_uint(const unsigned char *src)
{
	return (unsigned int)src[0] +
		(unsigned int)src[1] * 0x100 +
		(unsigned int)src[2] * 0x10000 +
		(unsigned int)src[3] * 0x1000000;
}
/**
 * syna_pal_ceil_div()
 *
 * Calculate the ceiling of the integer division
 *
 * @param
 *    [ in] dividend: the dividend value
 *    [ in] divisor:  the divisor value
 *
 * @return
 *    the ceiling of the integer division
 */
static inline unsigned int syna_pal_ceil_div(unsigned int dividend,
		unsigned int divisor)
{
	return (dividend + divisor - 1) / divisor;
}


/**
 * @section: C Runtime for Memory Management helpers
 *
 * @brief: syna_pal_mem_calloc
 *         Allocate a block of memory space
 *
 * @brief: syna_pal_mem_free
 *         Deallocate a block of memory previously allocated
 *
 * @brief: syna_pal_mem_set
 *         Fill memory with a constant byte
 *
 * @brief: syna_pal_mem_cpy
 *         Ensure the safe size before doing memory copy
 */

/**
 * syna_pal_mem_calloc()
 *
 * Allocates a block of memory for an array of 'num' elements,
 * each of them has 'size' bytes long, and initializes all its bits to zero.
 *
 * @param
 *    [ in] num:  number of elements for an array
 *    [ in] size: number of bytes for each elements
 *
 * @return
 *    On success, a pointer to the memory block allocated by the function.
 */
static inline void *syna_pal_mem_alloc(unsigned int num, unsigned int size)
{
#ifdef DEV_MANAGED_API
	struct device *dev = syna_request_managed_device();

	if (!dev) {
		LOGE("Invalid managed device\n");
		return NULL;
	}
#endif

	if ((int)(num * size) <= 0) {
		LOGE("Invalid parameter\n");
		return NULL;
	}

#ifdef DEV_MANAGED_API
	return devm_kcalloc(dev, num, size, GFP_KERNEL);
#else /* Legacy API */
	return kcalloc(num, size, GFP_KERNEL);
#endif
}
/**
 * syna_pal_mem_free()
 *
 * Deallocate a block of memory previously allocated.
 *
 * @param
 *    [ in] ptr: a memory block  previously allocated
 *
 * @return
 *    none.
 */
static inline void syna_pal_mem_free(void *ptr)
{
#ifdef DEV_MANAGED_API
	struct device *dev = syna_request_managed_device();

	if (!dev) {
		LOGE("Invalid managed device\n");
		return;
	}

	if (ptr)
		devm_kfree(dev, ptr);
#else /* Legacy API */
	kfree(ptr);
#endif
}
/**
 * syna_pal_mem_set()
 *
 * Fill memory with a constant byte
 *
 * @param
 *    [ in] ptr: pointer to a memory block
 *    [ in] c:   the constant value
 *    [ in] n:   number of byte being set
 *
 * @return
 *    none.
 */
static inline void syna_pal_mem_set(void *ptr, int c, unsigned int n)
{
	memset(ptr, c, n);
}
/**
 * syna_pal_mem_cpy()
 *
 * Ensure the safe size before copying the values of num bytes from the
 * location to the memory block pointed to by destination.
 *
 * @param
 *    [out] dest:      pointer to the destination space
 *    [ in] dest_size: size of destination array
 *    [ in] src:       pointer to the source of data to be copied
 *    [ in] src_size:  size of source array
 *    [ in] num:       number of bytes to copy
 *
 * @return
 *    0 on success; otherwise, on error.
 */
static inline int syna_pal_mem_cpy(void *dest, unsigned int dest_size,
		const void *src, unsigned int src_size, unsigned int num)
{
	if (dest == NULL || src == NULL)
		return -1;

	if (num > dest_size || num > src_size) {
		LOGE("Invalid size. src:%d, dest:%d, num:%d\n",
			src_size, dest_size, num);
		return -1;
	}

	memcpy((void *)dest, (const void *)src, num);

	return 0;
}


/**
 * @section: C Runtime for Muxtex Control helpers
 *
 * @brief: syna_pal_mutex_alloc
 *         Create a mutex object
 *
 * @brief: syna_pal_mutex_free
 *         Release the mutex object previously allocated
 *
 * @brief: syna_pal_mutex_lock
 *         Lock the mutex
 *
 * @brief: syna_pal_mutex_unlock
 *         Unlock the mutex previously locked
 */

typedef struct mutex syna_pal_mutex_t;

/**
 * syna_pal_mutex_alloc()
 *
 * Create a mutex object.
 *
 * @param
 *    [out] ptr: pointer to the mutex handle being allocated
 *
 * @return
 *    0 on success; otherwise, on error.
 */
static inline int syna_pal_mutex_alloc(syna_pal_mutex_t *ptr)
{
	mutex_init((struct mutex *)ptr);
	return 0;
}
/**
 * syna_pal_mutex_free()
 *
 * Release the mutex object previously allocated.
 *
 * @param
 *    [ in] ptr: mutex handle previously allocated
 *
 * @return
 *    none.
 */
static inline void syna_pal_mutex_free(syna_pal_mutex_t *ptr)
{
	/* do nothing */
}
/**
 * syna_pal_mutex_lock()
 *
 * Acquire/lock the mutex.
 *
 * @param
 *    [ in] ptr: a mutex handle
 *
 * @return
 *    none.
 */
static inline void syna_pal_mutex_lock(syna_pal_mutex_t *ptr)
{
	mutex_lock((struct mutex *)ptr);
}
/**
 * syna_pal_mutex_unlock()
 *
 * Unlock the locked mutex.
 *
 * @param
 *    [ in] ptr: a mutex handle
 *
 * @return
 *    none.
 */
static inline void syna_pal_mutex_unlock(syna_pal_mutex_t *ptr)
{
	mutex_unlock((struct mutex *)ptr);
}


/**
 * @section: C Runtime for Completion Event
 *
 * @brief: syna_pal_completion_alloc
 *         Allocate a completion event
 *
 * @brief: syna_pal_completion_free
 *         Release the completion event previously allocated
 *
 * @brief: syna_pal_completion_complete
 *         Complete the completion event being waiting for
 *
 * @brief: syna_pal_completion_reset
 *         Reset or reinitialize the completion event
 *
 * @brief: syna_pal_completion_wait_for
 *         Wait for the completion event
 */

typedef struct completion syna_pal_completion_t;

/**
 * syna_pal_completion_alloc()
 *
 * Allocate a completion event, and the default state is not set.
 * Caller must reset the event before each use.
 *
 * @param
 *    [out] ptr: pointer to the completion handle being allocated
 *
 * @return
 *    0 on success; otherwise, on error.
 */
static inline int syna_pal_completion_alloc(syna_pal_completion_t *ptr)
{
	init_completion((struct completion *)ptr);
	return 0;
}
/**
 * syna_pal_completion_free()
 *
 * Release the completion event previously allocated
 *
 * @param
 *    [ in] ptr: the completion event previously allocated
 event
 * @return
 *    none.
 */
static inline void syna_pal_completion_free(syna_pal_completion_t *ptr)
{
	/* do nothing */
}
/**
 * syna_pal_completion_complete()
 *
 * Complete the completion event being waiting for
 *
 * @param
 *    [ in] ptr: the completion event
 *
 * @return
 *    none.
 */
static inline void syna_pal_completion_complete(syna_pal_completion_t *ptr)
{
	complete_all((struct completion *)ptr);
}
/**
 * syna_pal_completion_reset()
 *
 * Reset or reinitialize the completion event
 *
 * @param
 *    [ in] ptr: the completion event
 *
 * @return
 *    none.
 */
static inline void syna_pal_completion_reset(syna_pal_completion_t *ptr)
{
#if (KERNEL_VERSION(3, 13, 0) > LINUX_VERSION_CODE)
		init_completion((struct completion *)ptr);
#else
		reinit_completion((struct completion *)ptr);
#endif
}
/**
 * syna_pal_completion_wait_for()
 *
 * Wait for the completion event during the given time slot
 *
 * @param
 *    [ in] ptr:        the completion event
 *    [ in] timeout_ms: time frame in milliseconds
 *
 * @return
 *    0 if a signal is received; otherwise, on timeout or error occurs.
 */
static inline int syna_pal_completion_wait_for(syna_pal_completion_t *ptr,
		unsigned int timeout_ms)
{
	int retval;

	retval = wait_for_completion_timeout((struct completion *)ptr,
			msecs_to_jiffies(timeout_ms));
	if (retval == 0) /* timeout occurs */
		return -1;

	return 0;
}


/**
 * @section: C Runtime to Pause the Execution
 *
 * @brief: syna_pal_sleep_ms
 *         Sleep for a fixed amount of time in milliseconds
 *
 * @brief: syna_pal_sleep_us
 *         Sleep for a range of time in microseconds
 *
 * @brief: syna_pal_busy_delay_ms
 *         Busy wait for a fixed amount of time in milliseconds
 */

/**
 * syna_pal_sleep_ms()
 *
 * Sleep for a fixed amount of time in milliseconds
 *
 * @param
 *    [ in] time_ms: time frame in milliseconds
 *
 * @return
 *    none.
 */
static inline void syna_pal_sleep_ms(int time_ms)
{
	msleep(time_ms);
}
/**
 * syna_pal_sleep_us()
 *
 * Sleep for a range of time in microseconds
 *
 * @param
 *    [ in] time_us_min: the min. time frame in microseconds
 *    [ in] time_us_max: the max. time frame in microseconds
 *
 * @return
 *    none.
 */
static inline void syna_pal_sleep_us(int time_us_min, int time_us_max)
{
	usleep_range(time_us_min, time_us_max);
}
/**
 * syna_pal_busy_delay_ms()
 *
 * Busy wait for a fixed amount of time in milliseconds
 *
 * @param
 *    [ in] time_ms: time frame in milliseconds
 *
 * @return
 *    none.
 */
static inline void syna_pal_busy_delay_ms(int time_ms)
{
	mdelay(time_ms);
}


/**
 * @section: C Runtime for String operations
 *
 * @brief: syna_pal_str_len
 *         Return the length of C string
 *
 * @brief: syna_pal_str_cpy:
 *         Ensure the safe size before doing C string copy
 *
 * @brief: syna_pal_str_cmp:
 *         Compare whether the given C strings are equal or not
 */

/**
 * syna_pal_str_len()
 *
 * Return the length of C string
 *
 * @param
 *    [ in] str:  an array of characters
 *
 * @return
 *    the length of given string
 */
static inline unsigned int syna_pal_str_len(const char *str)
{
	return (unsigned int)strlen(str);
}
/**
 * syna_pal_str_cpy()
 *
 * Copy the C string pointed by source into the array pointed by destination.
 *
 * @param
 *    [ in] dest:      pointer to the destination C string
 *    [ in] dest_size: size of destination C string
 *    [out] src:       pointer to the source of C string to be copied
 *    [ in] src_size:  size of source C string
 *    [ in] num:       number of bytes to copy
 *
 * @return
 *    0 on success; otherwise, on error.
 */
static inline int syna_pal_str_cpy(char *dest, unsigned int dest_size,
		const char *src, unsigned int src_size, unsigned int num)
{
	if (dest == NULL || src == NULL)
		return -1;

	if (num > dest_size || num > src_size) {
		LOGE("Invalid size. src:%d, dest:%d, num:%d\n",
			src_size, dest_size, num);
		return -1;
	}

	strncpy(dest, src, num);

	return 0;
}
/**
 * syna_pal_str_cmp()
 *
 * Compares up to num characters of the C string str1 to those of the
 * C string str2.
 *
 * @param
 *    [ in] str1: C string to be compared
 *    [ in] str2: C string to be compared
 *    [ in] num:  number of characters to compare
 *
 * @return
 *    0 if both strings are equal; otherwise, not equal.
 */
static inline int syna_pal_str_cmp(const char *str1, const char *str2,
		unsigned int num)
{
	return strncmp(str1, str2, num);
}
/**
 * syna_pal_hex_to_uint()
 *
 * Convert the given string in hex to an integer returned
 *
 * @param
 *    [ in] str:    C string to be converted
 *    [ in] length: target length
 *
 * @return
 *    An integer converted
 */
static inline unsigned int syna_pal_hex_to_uint(char *str, int length)
{
	unsigned int result = 0;
	char *ptr = NULL;

	for (ptr = str; ptr != str + length; ++ptr) {
		result <<= 4;
		if (*ptr >= 'A')
			result += *ptr - 'A' + 10;
		else
			result += *ptr - '0';
	}

	return result;
}

/**
 * @section: C Runtime for Checksum Calculation
 *
 * @brief: syna_pal_crc32
 *         Calculates the CRC32 value
 */

/**
 * syna_pal_crc32()
 *
 * Calculates the CRC32 value of the data
 *
 * @param
 *    [ in] seed: the previous crc32 value
 *    [ in] data: byte data for the calculation
 *    [ in] len:  the byte length of the data.
 *
 * @return
 *    0 if both strings are equal; otherwise, not equal.
 */
static inline unsigned int syna_pal_crc32(unsigned int seed,
		const char *data, unsigned int len)
{
	return crc32(seed, data, len);
}

#endif /* end of _SYNAPTICS_TCM2_C_RUNTIME_H_ */
