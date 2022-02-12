/*
 * Copyright (c) 2022 Michał Bursztynowski
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_SDS011_H_
#define ZEPHYR_DRIVERS_SENSOR_SDS011_H_

#define SDS011_READY_TIME 30
#define SDS011_RX_SIZE 10

#define CHECK_RET(x) \
    if(x != 0){\
        return x; \
    }

struct sds011_data{
    const struct device *uart;
    int32_t pm2_5_v1;
    int32_t pm2_5_v2;
    int32_t pm_10_v1;
    int32_t pm_10_v2;
}

#endif