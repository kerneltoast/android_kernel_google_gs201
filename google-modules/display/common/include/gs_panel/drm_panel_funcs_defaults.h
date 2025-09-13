/* SPDX-License-Identifier: MIT */
/*
 * Copyright 2023 Google LLC
 *
 * Use of this source code is governed by an MIT-style
 * license that can be found in the LICENSE file or at
 * https://opensource.org/licenses/MIT.
 */
#ifndef _DRM_PANEL_FUNCS_DEFAULTS_H_
#define _DRM_PANEL_FUNCS_DEFAULTS_H_

#include <drm/drm_panel.h>

int gs_panel_disable(struct drm_panel *panel);
int gs_panel_unprepare(struct drm_panel *panel);
int gs_panel_prepare(struct drm_panel *panel);
int gs_panel_get_modes(struct drm_panel *panel, struct drm_connector *connector);

#endif // _DRM_PANEL_FUNCS_DEFAULTS_H_
