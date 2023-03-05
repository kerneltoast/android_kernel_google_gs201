/*
 * Copyright (C) 2023 The Android Open Source Project
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

#ifndef MKDTIMG_H
#define MKDTIMG_H

#include <stdio.h>

void handle_usage_help(FILE *out_fp, const char *prog_name);
int handle_command_help(int argc, char *argv[], int arg_start);
void handle_usage_dump(FILE *out_fp, const char *prog_name);
int handle_command_dump(int argc, char *argv[], int arg_start);
void handle_usage_create(FILE *out_fp, const char *prog_name);
int handle_command_create(int argc, char *argv[], int arg_start);
void handle_usage_cfg_create(FILE *out_fp, const char *prog_name);
int handle_command_cfg_create(int argc, char *argv[], int arg_start);

#endif
