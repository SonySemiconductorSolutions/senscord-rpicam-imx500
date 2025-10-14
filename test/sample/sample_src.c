/*
 * SPDX-FileCopyrightText: 2023-2025 Sony Semiconductor Solutions Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "sample_src.h"

#include "sample_sub.h"
#include <stdlib.h>

int SampleFunc(void) { return 2 * SampleSub(); }

int* SampleMallocFunc(size_t count) {
  int* ptr = (int*)malloc(count * sizeof(int));
  if (ptr != NULL) {
    for (size_t i = 0; i < count; i++) {
      ptr[i] = (int)i;
    }
  }
  return ptr;
}

void SampleFreeFunc(int* ptr) {
  if (ptr != NULL) {
    free(ptr);
  }
}
