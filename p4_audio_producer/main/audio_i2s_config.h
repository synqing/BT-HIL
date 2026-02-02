/*
 * I2S pin and port config for ESP32-P4. Override per board.
 * Spec: 16 kHz, 16-bit mono RX.
 */
#ifndef P4_AUDIO_PRODUCER_AUDIO_I2S_CONFIG_H
#define P4_AUDIO_PRODUCER_AUDIO_I2S_CONFIG_H

#define AUDIO_I2S_PORT_NUM        0
#define AUDIO_I2S_BCK_GPIO        12
#define AUDIO_I2S_WS_GPIO         10
#define AUDIO_I2S_DI_GPIO         11
#define AUDIO_I2S_DO_GPIO         9
#define AUDIO_I2S_MCK_GPIO        13
#define AUDIO_I2S_MCLK_MULTIPLE   384

#define AUDIO_I2C_PORT_NUM        0
#define AUDIO_I2C_SCL_GPIO        8
#define AUDIO_I2C_SDA_GPIO        7

#define AUDIO_PA_GPIO             53

#endif /* P4_AUDIO_PRODUCER_AUDIO_I2S_CONFIG_H */
