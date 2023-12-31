/*
 * Copyright (C) 2016 The Android Open Source Project
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

#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include "ufdt_overlay.h"
#include "libufdt_sysdeps.h"

#include "util.h"

int apply_overlay_files(const char *out_filename, const char *base_filename,
                        const char *overlay_filename) {
  int ret = 1;
  char *base_buf = NULL;
  char *overlay_buf = NULL;
  struct fdt_header *new_blob = NULL;

  size_t blob_len;
  base_buf = load_file(base_filename, &blob_len);
  if (!base_buf || fdt_check_full(base_buf, blob_len)) {
    fprintf(stderr, "Can not load base file: %s\n", base_filename);
    goto end;
  }

  size_t overlay_len;
  overlay_buf = load_file(overlay_filename, &overlay_len);
  if (!overlay_buf || fdt_check_full(overlay_buf, overlay_len)) {
    fprintf(stderr, "Can not load overlay file: %s\n", overlay_filename);
    goto end;
  }

  struct fdt_header *blob = ufdt_install_blob(base_buf, blob_len);
  if (!blob) {
    fprintf(stderr, "ufdt_install_blob() returns null\n");
    goto end;
  }

  clock_t start = clock();
  new_blob = ufdt_apply_overlay(blob, blob_len, overlay_buf, overlay_len);
  clock_t end = clock();

  if (!new_blob) {
    fprintf(stderr, "ufdt_apply_overlay() returned null: bad input?\n");
    goto end;
  }

  if (write_fdt_to_file(out_filename, new_blob) != 0) {
    fprintf(stderr, "Write file error: %s\n", out_filename);
    goto end;
  }

  // Outputs the used time.
  double cpu_time_used = ((double)(end - start)) / CLOCKS_PER_SEC;
  printf("ufdt_apply_overlay: took %.9f secs\n", cpu_time_used);
  ret = 0;

end:
  // Do not dto_free(blob) - it's the same as base_buf.

  if (new_blob) dto_free(new_blob);
  if (overlay_buf) dto_free(overlay_buf);
  if (base_buf) dto_free(base_buf);

  return ret;
}

int main(int argc, char **argv) {
  if (argc < 4) {
    fprintf(stderr, "Usage: %s <base_file> <overlay_file> <out_file>\n", argv[0]);
    return 1;
  }

  const char *base_file = argv[1];
  const char *overlay_file = argv[2];
  const char *out_file = argv[3];
  int ret = apply_overlay_files(out_file, base_file, overlay_file);

  return ret;
}
