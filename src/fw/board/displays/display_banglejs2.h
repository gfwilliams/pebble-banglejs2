/*
 * Copyright 2024 Google LLC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once

#define DISPLAY_ORIENTATION_COLUMN_MAJOR_INVERTED 0
#define DISPLAY_ORIENTATION_ROTATED_180 1
#define DISPLAY_ORIENTATION_ROW_MAJOR 0
#define DISPLAY_ORIENTATION_ROW_MAJOR_INVERTED 0

#define PBL_BW 1 // FIXME - but lets try sending to the display as black and white first
#define PBL_COLOR 0

#define PBL_RECT 1
#define PBL_ROUND 0

#define PBL_DISPLAY_WIDTH 176
#define PBL_DISPLAY_HEIGHT 176

#define LEGACY_2X_DISP_COLS PBL_DISPLAY_WIDTH
#define LEGACY_2X_DISP_ROWS PBL_DISPLAY_HEIGHT
#define LEGACY_3X_DISP_COLS PBL_DISPLAY_WIDTH
#define LEGACY_3X_DISP_ROWS PBL_DISPLAY_HEIGHT

#define DISPLAY_FRAMEBUFFER_BYTES \
    (ROUND_TO_MOD_CEIL(PBL_DISPLAY_WIDTH, 32) / 8 * PBL_DISPLAY_HEIGHT)
