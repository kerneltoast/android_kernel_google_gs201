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

#include <string>
#include <unordered_map>

extern "C" {

#include "libufdt.h"
#include "ufdt_node_pool.h"
#include "ufdt_overlay.h"
#include "ufdt_overlay_internal.h"

}

#include "ufdt_test_overlay.h"

static bool ufdt_node_compare(struct ufdt_node *node_a, struct ufdt_node *node_b,
                              struct ufdt* tree_a, struct ufdt* tree_b);

/*
 * Helper method to check if the tree rooted at node_b is a subset of the tree rooted
 * at node_a.
 */
static bool compare_child_nodes(struct ufdt_node *node_a, struct ufdt_node *node_b,
                                struct ufdt * tree_a, struct ufdt * tree_b) {
    bool result = true;
    struct ufdt_node *it;

    for (it = ((struct ufdt_node_fdt_node *)node_b)->child; it; it = it->sibling) {
        struct ufdt_node *cur_node = it;
        struct ufdt_node *target_node = NULL;

        if (ufdt_node_tag(cur_node) == FDT_BEGIN_NODE) {
            target_node =
                    ufdt_node_get_subnode_by_name(node_a, ufdt_node_name(cur_node));
        } else {
            target_node =
                    ufdt_node_get_property_by_name(node_a, ufdt_node_name(cur_node));
        }

        if (target_node == NULL) {
            result = false;
        } else {
            result = ufdt_node_compare(target_node, cur_node, tree_a, tree_b);
        }

        if (!result) {
            break;
        }
    }

    return result;
}

/*
 * Method to compare two nodes with tag FDT_PROP. Also accounts for the cases where
 * the property type is phandle.
 */
static bool ufdt_compare_property(struct ufdt_node* node_final, struct ufdt_node* node_overlay,
                                  struct ufdt* tree_final, struct ufdt* tree_overlay) {
    if (ufdt_node_tag(node_final) == FDT_PROP) {
        /* Return -1 if property names are differemt */
        if (strcmp(ufdt_node_name(node_final), ufdt_node_name(node_overlay)) != 0)
            return false;

        int length_data_final = 0, length_data_overlay = 0;
        char *prop_data_final = ufdt_node_get_fdt_prop_data(node_final, &length_data_final);
        char *prop_data_overlay = ufdt_node_get_fdt_prop_data(node_overlay,
                                                              &length_data_overlay);

        /* Confirm length for the property values are the same */
        if (length_data_final != length_data_overlay) {
            return false;
        }

        if (((length_data_final == 0) && (length_data_overlay ==0)) ||
            (memcmp(prop_data_final, prop_data_overlay, length_data_final) == 0)) {
            // Return if the properties have same value.
            return true;
        } else {
            /* check for the presence of phandles */
            for (int i = 0; i < length_data_final; i += sizeof(fdt32_t)) {
                int offset_data_a = fdt32_to_cpu(
                        *reinterpret_cast<fdt32_t *>(prop_data_final + i));
                int offset_data_b = fdt32_to_cpu(
                        *reinterpret_cast<fdt32_t *>(prop_data_overlay + i));
                if (offset_data_a == offset_data_b) continue;
                /* If the offsets have phandles, they would have valid target nodes */
                struct ufdt_node * target_node_a = ufdt_get_node_by_phandle(tree_final,
                                                                            offset_data_a);
                struct ufdt_node * target_node_b = ufdt_get_node_by_phandle(tree_overlay,
                                                                            offset_data_b);

                /*
                 * verify that the target nodes are valid and point to the same node.
                 */
                if ((target_node_a == NULL) || (target_node_b == NULL) ||
                    strcmp(ufdt_node_name(target_node_a),
                           ufdt_node_name(target_node_b)) != 0) {
                    return false;
                }
            }
        }
    }

    return true;
}

/*
 * Checks if the ufdt tree rooted at node_b is a subtree of the tree rooted at
 * node_a.
 */
static bool ufdt_node_compare(struct ufdt_node *node_final, struct ufdt_node *node_overlay,
                              struct ufdt * tree_final, struct ufdt * tree_overlay) {
    if (ufdt_node_tag(node_final) == FDT_PROP) {
        return ufdt_compare_property(node_final, node_overlay, tree_final, tree_overlay);
    }

    return compare_child_nodes(node_final, node_overlay, tree_final, tree_overlay);
}


/*
 * Multiple fragments may fixup to the same node on the base device tree.
 * Combine these fragments for easier verification.
 */
void ufdt_combine_fixup(struct ufdt *tree, const char *fixup,
                        struct ufdt_node **prev_node, struct ufdt_node_pool *node_pool) {
    char *path, *prop_ptr, *offset_ptr;
    char path_buf[1024];
    char *path_mem = NULL;
    int result = 0;

    size_t fixup_len = strlen(fixup) + 1;
    if (fixup_len > sizeof(path_buf)) {
        path_mem = static_cast<char *>(dto_malloc(fixup_len));
        path = path_mem;
    } else {
        path = path_buf;
    }
    dto_memcpy(path, fixup, fixup_len);

    prop_ptr = dto_strchr(path, ':');
    if (prop_ptr == NULL) {
        dto_error("Missing property part in '%s'\n", path);
        goto fail;
    }

    *prop_ptr = '\0';
    prop_ptr++;

    offset_ptr = dto_strchr(prop_ptr, ':');
    if (offset_ptr == NULL) {
        dto_error("Missing offset part in '%s'\n", path);
        goto fail;
    }

    *offset_ptr = '\0';
    offset_ptr++;

    result = dto_strcmp(prop_ptr, "target");
    /* If the property being fixed up is not target, ignore and return */
    if (result == 0) {
        struct ufdt_node *target_node;
        target_node = ufdt_get_node_by_path(tree, path);
        if (target_node == NULL) {
            dto_error("Path '%s' not found\n", path);
        } else {
            /* The goal is to combine fragments that have a common target */
            if (*prev_node != NULL) {
                ufdt_node_merge_into(*prev_node, target_node, node_pool);
            } else {
                *prev_node = target_node;
            }
        }
    }

fail:
    if (path_mem) {
        dto_free(path_mem);
    }

    return;
}

/*
 * Creates a table of node paths to their corresponding phandles by walking
 * through the 'symbols' node of the main device tree. The table would be
 * used in combining overlay nodes that map to the same nodes in the
 * main device tree.
 */
void create_path_phandle_map(std::unordered_map<uint32_t, std::string>* phandle_path_map,
                             struct ufdt* main_tree) {
    int len = 0;
    struct ufdt_node *main_symbols_node =
            ufdt_get_node_by_path(main_tree, "/__symbols__");
    if (!main_symbols_node) {
        dto_error("No node __symbols__ in main dtb.\n");
        return;
    }

    struct ufdt_node **it = nullptr;
    for_each_prop(it, main_symbols_node) {
        const char* symbol_path = ufdt_node_get_fdt_prop_data(*it, &len);
        struct ufdt_node* symbol_node = ufdt_get_node_by_path(main_tree, symbol_path);
        uint32_t phandle = ufdt_node_get_phandle(symbol_node);
        (*phandle_path_map)[phandle] = std::string(symbol_path);
    }
}

/*
 * Recursively checks whether a node from another overlay fragment had overlaid the
 * target node and if so merges the node into the previous node.
 */
static void combine_overlay_node(std::unordered_map<std::string,
                                 struct ufdt_node*>* path_node_map,
                                 std::string path,
                                 struct ufdt_node* node,
                                 struct ufdt_node_pool* pool) {
    struct ufdt_node **it = nullptr;
    for_each_node(it, node) {
        //skips properties
        if (ufdt_node_tag(*it) == FDT_BEGIN_NODE) {
            combine_overlay_node(path_node_map, path + "/" + ufdt_node_name(*it), *it, pool);
        }
    }

    if (path_node_map->find(path) != path_node_map->end()) {
        ufdt_node_merge_into((*path_node_map)[path], node, pool);
    } else {
        //This is the first node overlaying the target node, add the same to the
        //table.
        (*path_node_map)[path] = node;
    }
}

/* END of doing fixup in the overlay ufdt. */

static bool ufdt_verify_overlay_node(struct ufdt_node *target_node,
                                     struct ufdt_node *overlay_node,
                                     struct ufdt * target_tree,
                                     struct ufdt * overlay_tree) {
    return ufdt_node_compare(target_node, overlay_node, target_tree, overlay_tree);
}

/*
 * verify one overlay fragment (subtree).
 */
static int ufdt_verify_fragment(struct ufdt *tree,
                                struct ufdt *overlay_tree,
                                struct ufdt_node *frag_node) {
    struct ufdt_node *target_node = NULL;
    struct ufdt_node *overlay_node = NULL;
    enum overlay_result target_search_result = ufdt_overlay_get_target(tree, frag_node,
                                                                       &target_node);
    if (target_node == NULL) {
        return target_search_result;
    }

    overlay_node = ufdt_node_get_node_by_path(frag_node, "__overlay__");
    if (overlay_node == NULL) {
        dto_error("missing __overlay__ sub-node\n");
        return OVERLAY_RESULT_MISSING_OVERLAY;
    }

    bool result = ufdt_verify_overlay_node(target_node, overlay_node, tree, overlay_tree);

    if (!result) {
        dto_error("failed to verify fragment node %s to target %s\n",
                  ufdt_node_name(frag_node), ufdt_node_name(target_node));
        return OVERLAY_RESULT_VERIFY_FAIL;
    }

    return OVERLAY_RESULT_OK;
}

/*
 * verify each fragment in overlay.
 */
static int ufdt_overlay_verify_fragments(struct ufdt *final_tree,
                                         struct ufdt *overlay_tree) {
    enum overlay_result err;
    struct ufdt_node **it;
    for_each_node(it, overlay_tree->root) {
        err = static_cast<enum overlay_result>(ufdt_verify_fragment(final_tree, overlay_tree,
                                                                    *it));
        if (err == OVERLAY_RESULT_VERIFY_FAIL) {
            return -1;
        }
    }
    return 0;
}

/*
 * Examine target nodes for fragments in all overlays and combine ones with the
 * same target.
 */
static void ufdt_overlay_combine_common_nodes(struct ufdt** overlay_trees,
                                              size_t overlay_count,
                                              struct ufdt* final_tree,
                                              struct ufdt_node_pool* pool
                                             ) {
    std::unordered_map<std::string, struct ufdt_node*> path_node_map;
    std::unordered_map<uint32_t, std::string> phandle_path_map;

    create_path_phandle_map(&phandle_path_map, final_tree);

    struct ufdt_node **it = nullptr;
    for (size_t i = 0; i < overlay_count; i++) {
        for_each_node(it, overlay_trees[i]->root) {
            uint32_t target = 0;
            const void* val = ufdt_node_get_fdt_prop_data_by_name(*it, "target", NULL);
            if (val) {
                dto_memcpy(&target, val, sizeof(target));
                target = fdt32_to_cpu(target);
                std::string path = phandle_path_map[target];
                struct ufdt_node* overlay_node = ufdt_node_get_node_by_path(*it, "__overlay__");
                if (overlay_node != nullptr) {
                    combine_overlay_node(&path_node_map, path, overlay_node, pool);
                }
            }
        }
    }
}

/*
 * Makes sure that all phandles in the overlays are unique since they will be
 * combined before verification.
 */
int ufdt_resolve_duplicate_phandles(ufdt** overlay_tree, size_t overlay_count) {
  size_t phandle_offset = 0;
  for (size_t i = 0; i < overlay_count; i++) {
        ufdt_try_increase_phandle(overlay_tree[i], phandle_offset);
        if (ufdt_overlay_do_local_fixups(overlay_tree[i], phandle_offset) < 0) {
            return -1;
        }
        phandle_offset += ufdt_get_max_phandle(overlay_tree[i]);
  }

  return 0;
}

/*
 * Combines all overlays into a single tree at overlay_trees[0]
 */
int ufdt_combine_all_overlays(struct ufdt** overlay_trees, size_t overlay_count,
                              struct ufdt* final_tree, struct ufdt_node_pool* pool) {
    struct ufdt* combined_overlay_tree = nullptr;

    if (!overlay_trees || !overlay_count || !final_tree || !pool) {
        return -1;
    }

    /*
     * If there are duplicate phandles amongst the overlays, replace them with
     * unique ones.
     */
    if (ufdt_resolve_duplicate_phandles(overlay_trees, overlay_count) < 0) {
        return -1;
    }

    /*
     * For each overlay, perform fixup for each fragment.
     */
    for (size_t i = 0; i < overlay_count; i++) {
        if (ufdt_overlay_do_fixups(final_tree, overlay_trees[i]) < 0) {
            dto_error("failed to perform fixups in overlay\n");
            return -1;
        }
    }

    /*
     * Iterate through each overlay and combine all nodes with the same target
     * node.
     */
    ufdt_overlay_combine_common_nodes(overlay_trees, overlay_count, final_tree, pool);

    /*
     * Combine all overlays into the tree at overlay_trees[0] for easy
     * verification.
     */
    combined_overlay_tree = overlay_trees[0];
    struct ufdt_node* combined_root_node = combined_overlay_tree->root;

    for (size_t i = 1; i < overlay_count; i++) {
        struct ufdt_node** it = nullptr;
        struct ufdt_node* root_node = overlay_trees[i]->root;
        for_each_node(it, root_node) {
            ufdt_node_add_child(combined_root_node, *it);
        }
        ((struct ufdt_node_fdt_node *)root_node)->child = nullptr;
    }

    /*
     * Rebuild the phandle_table for the combined tree.
     */
    combined_overlay_tree->phandle_table = build_phandle_table(combined_overlay_tree);

    return 0;
}

int ufdt_verify_dtbo(struct fdt_header* final_fdt_header,
                     size_t final_fdt_size, void** overlay_arr,
                     size_t overlay_count) {
    const size_t min_fdt_size = 8;
    struct ufdt_node_pool pool;
    struct ufdt* final_tree = nullptr;
    struct ufdt** overlay_trees = nullptr;
    int result = 1;

    if (final_fdt_header == NULL) {
        goto fail;
    }

    if (final_fdt_size < min_fdt_size || final_fdt_size != fdt_totalsize(final_fdt_header)) {
        dto_error("Bad fdt size!\n");
        goto fail;
    }

    for (size_t i = 0; i < overlay_count; i++) {
        if ((fdt_magic(overlay_arr[i]) != FDT_MAGIC) || !fdt_totalsize(overlay_arr[i])) {
            dto_error("Corrupted or empty overlay\n");
            goto fail;
        }
    }
    ufdt_node_pool_construct(&pool);
    final_tree = ufdt_from_fdt(final_fdt_header, final_fdt_size, &pool);

    overlay_trees = new ufdt*[overlay_count];
    for (size_t i = 0; i < overlay_count; i++) {
        size_t fdt_size = fdt_totalsize(overlay_arr[i]);
        overlay_trees[i] = ufdt_from_fdt(overlay_arr[i], fdt_size, &pool);
    }

    if (ufdt_combine_all_overlays(overlay_trees, overlay_count, final_tree, &pool) < 0) {
        dto_error("Unable to combine overlays\n");
        goto fail;
    }
    if (ufdt_overlay_verify_fragments(final_tree, overlay_trees[0]) < 0) {
        dto_error("Failed to verify overlay application\n");
        goto fail;
    } else {
        result = 0;
    }

fail:
    if (overlay_trees) {
        for (size_t i = 0; i < overlay_count; i++) {
            ufdt_destruct(overlay_trees[i], &pool);
        }
        delete[] overlay_trees;
    }

    ufdt_destruct(final_tree, &pool);
    ufdt_node_pool_destruct(&pool);
    return result;
}
