#include <init.h>
#include <kernel.h>
#include <drivers/sensor.h>
#include <drivers/uart.h>
#include <string.h>
#include <errno.h>
#include <logging/log.h>

#include "sds011.h"

#define DT_DRV_COMPAT nova_sds011

LOG_MODULE_REGISTER(SDS011, CONFIG_SENSOR_LOG_LEVEL);

static struct sds011_data drv_data;

unsigned char cmd_sleep[19] = {0xAA, 0xB4, 0x06, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x05, 0xAB};
unsigned char cmd_measure[19] = {0xAA, 0xB4, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x02, 0xAB};
unsigned char cmd_work[19] = {0xAA, 0xB4, 0x06, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xFF, 0x06, 0xAB};

unsigned char response_buff[10] = {0xAA,0xAA,0xAA,0xAA,0xAA,0xAA,0xAA,0xAA,0xAA,0xAA};

static int uart_rx_cmd(const struct device *dev, uint32_t timeout){
    uint32_t rx_idx = 0;
    uint64_t to_timestamp = k_uptime_get() + K_MSEC(timeout);
    unsigned char c;
    while(1){
        if(k_uptime_get() > to_timestamp){
            retuen -ETIME;
        }
        if(uart_poll_in(dev, &c) == 0){
            response_buff[rx_idx++] = c;
            if(rx_idx == SDS011_RX_SIZE){
                return 0;
            }
        }
    }
}

static void uart_tx_cmd(const struct device *dev, unsigned char *cmd, uint32_t timeout){
    uint64_t to_timestamp = k_uptime_get() + K_MSEC(timeout);
    while(*cmd){
        if(k_uptime_get() > to_timestamp){
            retuen -ETIME;
        }
        uart_poll_out(dev, *cmd);
        ++cmd;
    }
    return 0;
}

static int uart_routine(const struct device *dev, unsigned char *cmd, uint32_t tx_timeout, uint32_t rx_timeout){
    int ret = uart_tx_cmd(dev, cmd, tx_timeout);
    if(ret == 0){
        ret = uart_rx_cmd(dev, rx_timeout);
    }
    return ret;
}

static int sds011_sample_fetch(const struct device *dev, enum sensor_channel chan){
    int ret = uart_routine(dev, cmd_work, 50, 50);
    CHECK_RET(ret);
    k_sleep(K_SECONDS(SDS011_READY_TIME));
    ret = uart_routine(dev, cmd_measure, 50, 50);
    CHECK_RET(ret);
    uart_routine(dev, cmd_sleep, 50, 50);

    float pm2_5_val = (response_buff[3] << 8) | (response_buff[2])/10;
    float pm10_val = (response_buff[5] << 8) | (response_buff[4])/10;

    struct sds011_data *data = dev->data;
    data->pm2_5_v1 = (int32_t)pm2_5_val;
    data->pm2_5_v2 = (pm2_5_val* 1000000) % 1000000;
    data->pm10_v1 = (int32_t)pm2_5_val;
    data->pm10_v2 = (pm2_5_val* 1000000) % 1000000;
    
    return 0;
}

static int sds011_channel_get(const struct device *dev, enum sensor_channel chan, struct sensor_value *senor_val){
    struct sds011_data *data = dev->data;

    if(chan == SENSOR_CHAN_PM_2_5){
        senor_val->va1 = data->pm2_5_v1;
        senor_val->va2 = data->pm2_5_v2;
    }else if(chan == SENSOR_CHAN_PM_10){
        senor_val->va1 = data->pm10_v1;
        senor_val->va2 = data->pm10_v2;
    }else{
        return -EINVAL;
    }
    return 0;
}

static int sds011_init(const struct device *dev){
    struct sds011_data *data = dev->data;
    data->uart = device_get_binding(DT_INST_BUS_LABEL(0));

    if(!data->uart){
        LOG_DBG("oops... not found given uart device %s", DT_INST_BUS_LABEL(0));
        return EINVAL;
    }

    sds011_sleep();
    
    return 0;
}

static const struct sensor_driver_api sds011_api = {
    .sample_fetch = sds011_sample_fetch,
    .channel_get = sds011_channel_get
}

DEVICE_DT_INST_DEFINE(0, &sds011_init, NULL, &drv_data, 
                    NULL, POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY,
		            &sds011_api);)
