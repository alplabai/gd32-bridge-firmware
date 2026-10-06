/*
 * Copyright 2026 Alp Lab AB
 * SPDX-License-Identifier: Apache-2.0
 *
 * hal/bridge_hw_stub.c without its ADC entry points, for the adc_e2e suite that
 * links the real hal/gd32/adc*.c instead.
 */
#define BRIDGE_HW_STUB_NO_ADC 1
#include "../../../hal/bridge_hw_stub.c"
