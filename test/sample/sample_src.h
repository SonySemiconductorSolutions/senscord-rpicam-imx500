/*
 * SPDX-FileCopyrightText: 2023-2025 Sony Semiconductor Solutions Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef SENSCORD_RPICAM_IMX500_TEST_SAMPLE_SRC_H_
#define SENSCORD_RPICAM_IMX500_TEST_SAMPLE_SRC_H_

#include <stddef.h>

int SampleFunc(void);
int* SampleMallocFunc(size_t count);
void SampleFreeFunc(int* ptr);

#endif /* SENSCORD_RPICAM_IMX500_TEST_SAMPLE_SRC_H_ */