/*
 * Copyright (C) 2018 The Android Open Source Project
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

#ifndef UFDT_TEST_OVERLAY_H
#define UFDT_TEST_OVERLAY_H

extern "C" {
#include <libfdt.h>
}

/*
 * Verifies that the FDT described by 'main_fdt_header' has been correctly
 * overlaid by the overlays contained in 'overlay_array'
 *
 * @param main_fdt_header Buffer describing the final FDT.
 * @param main_fdt_size Size of main_fdt_header.
 * @param overlay_array Array of buffers containg overlay FDTs.
 * @param overlay_count Number of overlays.
 *
 * @return Will return 0 if the verification is successful.
 */
int ufdt_verify_dtbo(struct fdt_header *main_fdt_header, size_t main_fdt_size,
                     void **overlay_array, size_t overlay_count);
#endif /* UFDT_TEST_OVERLAY_H */
