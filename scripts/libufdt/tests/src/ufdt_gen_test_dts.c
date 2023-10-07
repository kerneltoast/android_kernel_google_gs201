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

#include <getopt.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

/*
 * The parameters to generate testing DTS
 * /dts-v1/ /plugin/;           <- header and plugin
 * /{
 *   level0 {                   <- depth
 *     level1 {
 *       ...
 *       node0: node0 {         <- node
 *         unused0 {}           <- unused
 *         unused1 {}
 *         ...
 *         status="disabled";
 *       }
 *       ...
 *     };
 *   };
 * };
 *
 * &node0 {                     <- append
 *    new_prop="foo";
 * }
 * ...
 *
 * &node0 {                     <- override
 *    status="okay";
 * }
 * ...
 */

static const char short_options[] = "Hpd:u:n:a:w:o:";
static struct option long_options[] = {
  { "no-header",    no_argument,       NULL, 'H' },
  { "plugin",       no_argument,       NULL, 'p' },
  { "depth",        required_argument, NULL, 'd' },
  { "unused",       required_argument, NULL, 'u' },
  { "node",         required_argument, NULL, 'n' },
  { "append",       required_argument, NULL, 'a' },
  { "override",     required_argument, NULL, 'w' },
  { "output",       required_argument, NULL, 'o' },
  { 0,              0,                 NULL, 0 }
};

struct gen_params {
  int no_header;    /* Doesn't add header */
  int plugin;       /* Add /plugin/ in header */
  int depth;        /* the depth of a node, 0 means generate on root node */
  int unused_num;   /* unused child nodes per node */
  int node_num;     /* the number to generate nodes */
  int append_num;   /* the number to generate appending references */
  int override_num; /* the number to generate overriding references */
};


static void output_header(FILE *fp, int is_plugin) {
  fprintf(fp, "/dts-v1/;\n");
  if (is_plugin) {
    fprintf(fp, "/plugin/;\n");
  }
  fprintf(fp, "\n");
}

static void output_root_begin(FILE *fp, int depth) {
  fprintf(fp, "/ {\n");

  int i;
  for (i = 0; i < depth; i++) {
    fprintf(fp, "level%d {\n", i);
  }
}

static void output_root_end(FILE *fp, int depth) {
  int i;
  for (i = 0; i < depth; i++) {
    fprintf(fp, "};\n");
  }

  fprintf(fp, "};\n\n");
}

static void output_unused_nodes(FILE *fp, int count) {
  int i;
  for (i = 0; i < count; i++) {
    fprintf(fp, "unused%d {};\n", i);
  }
}

static void output_prop_str(FILE *fp, const char *prop, const char *value) {
  /* TODO: should escape value */
  fprintf(fp, "%s=\"%s\";\n", prop, value);
}

static void output_nodes(FILE *fp, int count, const char *prop, const char *value) {
  int i;
  for (i = 0; i < count; i++) {
    fprintf(fp, "node%d: node%d {\n", i, i);
    output_prop_str(fp, prop, value);
    fprintf(fp, "};\n\n");
  }
}

static void output_ref_nodes(FILE *fp, int start_id, int count,
                      const char *prop, const char *value) {
  int i;
  for (i = start_id; i < start_id + count; i++) {
    fprintf(fp, "&node%d {\n", i);
    output_prop_str(fp, prop, value);
    fprintf(fp, "};\n\n");
  }
}

static int gen_dts(FILE *fp, const struct gen_params *params) {
  if (!params->no_header) {
    output_header(fp, params->plugin);
  }

  if (params->node_num > 0) {
    output_root_begin(fp, params->depth);
    output_unused_nodes(fp, params->unused_num);
    output_nodes(fp, params->node_num, "status", "disabled");
    output_root_end(fp, params->depth);
  }

  int start_id = 0;
  output_ref_nodes(fp, start_id, params->append_num, "new_prop", "bar");
  start_id += params->append_num;
  output_ref_nodes(fp, start_id, params->override_num, "status", "okay");

  return 0;
}

int main(int argc, char *argv[]) {
  const char *filename = NULL;
  struct gen_params params;
  memset(&params, 0, sizeof(struct gen_params));

  while (1) {
    int option_index = 0;
    int c = getopt_long(argc, argv, short_options, long_options, &option_index);
    if (c == -1) {
      break;
    }
    switch (c) {
    case 'H':
      params.no_header = 1;
      break;
    case 'p':
      params.plugin = 1;
      break;
    case 'd':
      params.depth = atoi(optarg);
      break;
    case 'u':
      params.unused_num = atoi(optarg);
      break;
    case 'n':
      params.node_num = atoi(optarg);
      break;
    case 'a':
      params.append_num = atoi(optarg);
      break;
    case 'w':
      params.override_num = atoi(optarg);
      break;
    case 'o':
      filename = optarg;
      break;
    case '?':
      break;
    }
  }

  FILE *fp = NULL;
  if (filename) {
    fp = fopen(filename, "wt");
    if (fp == NULL) {
      fprintf(stderr, "Can not create file: %s\n", filename);
      return -1;
    }
  }

  gen_dts(fp ? fp : stdout, &params);

  if (fp) {
    fclose(fp);
  }

  return 0;
}
