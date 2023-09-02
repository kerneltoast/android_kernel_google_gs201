/*
 * Copyright (C) 2022 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/* Ensure assert() catches logical errors during fuzzing */
#ifdef NDEBUG
#undef NDEBUG
#endif

#include <inttypes.h>
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>

#include <sanitizer/asan_interface.h>
#include <sanitizer/msan_interface.h>

#include "libfdt.h"
#include "libfdt_env.h"

/* check memory region is valid, for the purpose of tooling such as asan */
static void check_mem(const void *mem, size_t len) {

  assert(mem);

#if __has_feature(memory_sanitizer)
  /* dumps if check fails */
  __msan_check_mem_is_initialized((void *)mem, len);
#endif

#if __has_feature(address_sanitizer) || defined(__SANITIZE_ADDRESS__)
  assert(!__asan_region_is_poisoned((void *)mem, len));
#else
  const volatile uint8_t *mem8 = mem;

  /* Read each byte of memory for instrumentation */
  for(size_t i = 0; i < len; i++) {
    (void)mem8[i];
  }
#endif
}

static bool phandle_is_valid(uint32_t phandle) {
  return phandle != 0 && phandle != UINT32_MAX;
}

static void walk_device_tree(const void *device_tree, int parent_node) {
  int len = 0;
  const char *node_name = fdt_get_name(device_tree, parent_node, &len);
  if (node_name != NULL) {
    check_mem(node_name, len);
  }

  uint32_t phandle = fdt_get_phandle(device_tree, parent_node);
  if (phandle_is_valid(phandle)) {
    int node = fdt_node_offset_by_phandle(device_tree, phandle);
    assert(node >= 0); // it should at least find parent_node
  }

  // recursively walk the node's children
  for (int node = fdt_first_subnode(device_tree, parent_node); node >= 0;
       node = fdt_next_subnode(device_tree, node)) {
    walk_device_tree(device_tree, node);
  }
}


// Information on device tree is available in external/dtc/Documentation/
// folder.
int LLVMFuzzerTestOneInput(const uint8_t *data, size_t size) {
  // Non-zero return values are reserved for future use.
  if (size < FDT_V17_SIZE) return 0;

  if (fdt_check_full(data, size) != 0) return 0;

  walk_device_tree(data, /* parent_node */ 0);

  return 0;
}
