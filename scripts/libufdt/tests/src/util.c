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

#include "util.h"

#include <stdio.h>
#include <stdlib.h>

#include "libfdt.h"
#include "libufdt_sysdeps.h"

static char *load_file_contents(FILE *fp, size_t *len_ptr) {
  // Gets the file size.
  fseek(fp, 0, SEEK_END);
  size_t len = ftell(fp);
  fseek(fp, 0, SEEK_SET);

  char *buf = malloc(len);
  if (buf == NULL) {
    return NULL;
  }

  if (fread(buf, len, 1, fp) != 1) {
    free(buf);
    return NULL;
  }

  if (len_ptr) {
    *len_ptr = len;
  }

  return buf;
}

char *load_file(const char *filename, size_t *len_ptr) {
  FILE *fp = fopen(filename, "r");
  if (!fp) {
    return NULL;
  }

  char *buf = load_file_contents(fp, len_ptr);

  fclose(fp);

  return buf;
}

int write_buf_to_file(const char *filename,
                      const void *buf, size_t buf_size) {
  int ret = 0;
  FILE *fout = NULL;

  fout = fopen(filename, "wb");
  if (!fout) {
    ret = 1;
    goto end;
  }
  if (fwrite(buf, 1, buf_size, fout) < 1) {
    ret = 2;
    goto end;
  }

end:
  if (fout) fclose(fout);

  return ret;
}

int write_fdt_to_file(const char *filename, const void *fdt) {
  return write_buf_to_file(filename, fdt, fdt_totalsize(fdt));
}
