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

#include <stdio.h>
#include <stdlib.h>
#include <fstream>

#include "ufdt_test_overlay.h"

extern "C" {

#include "ufdt_overlay.h"
#include "libufdt_sysdeps.h"

}

size_t read_file_to_buf(const char *filename, char** buf) {
    size_t size = 0;
    std::ifstream file(filename, std::ios::binary | std::ios::in);

    if (!file) {
        return size;
    }

    file.seekg(0, file.end);
    size = file.tellg();
    file.seekg(0, std::ios::beg);

    *buf = new char[size];
    file.read(*buf, size);
    return size;
}

int verify_overlay_files(const char *final_filename,
                         char** overlay_filenames, size_t overlay_count) {
    char* final_buf = nullptr;
    void** overlay_buf_array = new void*[overlay_count];
    struct fdt_header *blob = nullptr;
    int result = 1;
    size_t final_size = 0, overlay_size = 0;

    final_size = read_file_to_buf(final_filename, &final_buf);
    if (final_size == 0) {
        fprintf(stderr, "Cannot load final DTB: %s \n", final_filename);
        goto end;
    }

    for (size_t i = 0; i < overlay_count; i++) {
        overlay_size = read_file_to_buf(overlay_filenames[i],
                                        reinterpret_cast<char**>(&overlay_buf_array[i]));
        if (overlay_size == 0) {
            fprintf(stderr, "Cannot load DTB Overlay: %s\n", overlay_filenames[i]);
            goto end;
        }
    }

    blob = ufdt_install_blob(final_buf, final_size);
    if (!blob) {
        fprintf(stderr, "ufdt_install_blob() returns null\n");
        goto end;
    }

    result = ufdt_verify_dtbo(blob, final_size, overlay_buf_array, overlay_count);

    if (result != 0) {
        fprintf(stderr, "bad overlay error");
    }

end:
    // Do not dto_free(blob) - it's the same as final_buf.
    for (size_t i = 0; i < overlay_count; i++) {
        if (overlay_buf_array[i]) dto_free(overlay_buf_array[i]);
    }
    delete[] overlay_buf_array;

    if (final_buf) dto_free(final_buf);

    return result;
}

int main(int argc, char **argv) {
    if (argc < 3) {
        fprintf(stderr, "Usage: %s <final_file> <overlay_file1> <overlay_file2> ..\n", argv[0]);
        return 1;
    }

    const char *final_file = argv[1];
    char **overlay_file_names = &argv[2];
    int ret = verify_overlay_files(final_file, overlay_file_names, argc - 2);

    return ret == 0 ? ret : 1;
}
