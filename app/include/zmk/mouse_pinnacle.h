/*
 * Copyright (c) 2023 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <zephyr/kernel.h>

struct zmk_trackpad_mouse_data_t {
    uint8_t buttons;
    int8_t xDelta;
    int8_t yDelta;
    int8_t scrollDelta;
};

struct k_work_q *zmk_trackpad_work_q();

#ifndef ZEPHYR_INCLUDE_DRIVERS_SENSOR_GEN4_H_
#define ZEPHYR_INCLUDE_DRIVERS_SENSOR_GEN4_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <zephyr/drivers/sensor.h>

enum sensor_channel_gen4 {
    SENSOR_CHAN_BUTTONS = SENSOR_CHAN_PRIV_START,
    SENSOR_CHAN_WHEEL,
    SENSOR_CHAN_XDELTA,
    SENSOR_CHAN_YDELTA,
};

#ifdef __cplusplus
}
#endif

#endif
