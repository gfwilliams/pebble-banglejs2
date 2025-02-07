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

#include "../display.h"

#define DISP_LINE_BYTES (DISP_COLS / 8)
#define DISP_LINE_WORDS (((DISP_COLS - 1) / 32) + 1)

// Bytes_per_line + 2 byte for cmd+ line address + 2 optional byte for a write command
#define DISP_DMA_BUFFER_SIZE_BYTES (DISP_LINE_BYTES + 4)
#define DISP_DMA_BUFFER_SIZE_WORDS (DISP_LINE_WORDS + 1)

typedef enum {
  DISPLAY_STATE_IDLE,
  DISPLAY_STATE_WRITING
} DisplayState;

typedef struct {
  DisplayState state;
  NextRowCallback get_next_row;
  UpdateCompleteCallback complete;
} DisplayContext;

