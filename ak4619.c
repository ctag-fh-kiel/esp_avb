#include "ak4619.h"

#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define AK4619_ADDR 0x10
#define AK4619_I2C_HZ 400000

#define REG_POWER 0x00
#define REG_AUDIO_IF0 0x01
#define REG_AUDIO_IF1 0x02
#define REG_SYSCLK 0x03
#define REG_MIC_GAIN1 0x04
#define REG_MIC_GAIN2 0x05
#define REG_ADC_IN_SEL 0x0B
#define REG_DAC_VOL0 0x0E
#define REG_DAC_VOL1 0x0F
#define REG_DAC_VOL2 0x10
#define REG_DAC_VOL3 0x11
#define REG_DAC_INPUT_SEL 0x12
#define REG_DAC_MUTE_FILTER 0x14

#define TAG "AK4619"

static i2c_master_dev_handle_t s_dev;

static esp_err_t ak4619_write(uint8_t reg, uint8_t value) {
  uint8_t data[2] = {reg, value};
  esp_err_t err = i2c_master_transmit(s_dev, data, sizeof(data), 100);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Write reg 0x%02x failed: %s", reg, esp_err_to_name(err));
  }
  return err;
}

static esp_err_t ak4619_read(uint8_t reg, uint8_t *value) {
  return i2c_master_transmit_receive(s_dev, &reg, 1, value, 1, 100);
}

esp_err_t ak4619_set_vol(float db) {
  if (db > 12.0f)
    db = 12.0f;
  if (db < -115.0f)
    db = -115.0f;
  uint8_t value = (uint8_t)(((12.0f - db) * 2.0f) + 0.5f);
  ESP_RETURN_ON_ERROR(ak4619_write(REG_DAC_VOL0, value), TAG, "DAC1L volume");
  ESP_RETURN_ON_ERROR(ak4619_write(REG_DAC_VOL1, value), TAG, "DAC1R volume");
  ESP_RETURN_ON_ERROR(ak4619_write(REG_DAC_VOL2, value), TAG, "DAC2L volume");
  ESP_RETURN_ON_ERROR(ak4619_write(REG_DAC_VOL3, value), TAG, "DAC2R volume");
  ESP_LOGI(TAG, "DAC volume %.1f dB (VOLDA=0x%02x)", db, value);
  return ESP_OK;
}

esp_err_t ak4619_set_mic_gain(float db) {
  if (db < -6.0f)
    db = -6.0f;
  if (db > 27.0f)
    db = 27.0f;
  uint8_t code = (uint8_t)(((db + 6.0f) / 3.0f) + 0.5f);
  uint8_t value = (uint8_t)((code << 4) | code);
  ESP_RETURN_ON_ERROR(ak4619_write(REG_MIC_GAIN1, value), TAG, "ADC1 gain");
  return ak4619_write(REG_MIC_GAIN2, value);
}

esp_err_t ak4619_configure(avb_state_s *state, i2c_master_bus_handle_t bus) {
  if (state->config.default_sample_rate != 48000 ||
      state->config.default_bits_per_sample != 32) {
    ESP_LOGE(TAG, "Only 48 kHz AAF INT32 is supported in TDM128 mode");
    return ESP_ERR_NOT_SUPPORTED;
  }

  if (state->config.codec_pins.reset >= 0) {
    gpio_reset_pin(state->config.codec_pins.reset);
    gpio_set_direction(state->config.codec_pins.reset, GPIO_MODE_OUTPUT);
    gpio_set_level(state->config.codec_pins.reset, 0);
    vTaskDelay(pdMS_TO_TICKS(100));
    gpio_set_level(state->config.codec_pins.reset, 1);
    vTaskDelay(pdMS_TO_TICKS(100));
  }

  i2c_device_config_t device_cfg = {
      .dev_addr_length = I2C_ADDR_BIT_LEN_7,
      .device_address = AK4619_ADDR,
      .scl_speed_hz = AK4619_I2C_HZ,
  };
  ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(bus, &device_cfg, &s_dev), TAG,
                      "Add I2C device");
  ESP_RETURN_ON_ERROR(i2c_master_probe(bus, AK4619_ADDR, 100), TAG,
                      "Codec not detected at 0x10");

  /* TDM128, I2S-compatible, four 32-bit slots. ADC samples are 24-bit
   * left-aligned in the 32-bit slot; the DAC consumes all 32 bits. */
  ESP_RETURN_ON_ERROR(ak4619_write(REG_POWER, 0x00), TAG, "Power-down");
  ESP_RETURN_ON_ERROR(ak4619_write(REG_AUDIO_IF0, 0xAC), TAG, "Audio IF0");
  ESP_RETURN_ON_ERROR(ak4619_write(REG_AUDIO_IF1, 0x1C), TAG, "Audio IF1");
  ESP_RETURN_ON_ERROR(ak4619_write(REG_SYSCLK, 0x02), TAG, "MCLK 384fs");
  ESP_RETURN_ON_ERROR(ak4619_write(REG_ADC_IN_SEL, 0x55), TAG, "ADC inputs");
  /* In TDM mode the multiplexed input is SDIN1. DAC2 defaults to SDIN2,
   * which is ignored in TDM mode; route both DAC banks from SDIN1. */
  ESP_RETURN_ON_ERROR(ak4619_write(REG_DAC_INPUT_SEL, 0x00), TAG,
                      "DAC input select");
  ESP_RETURN_ON_ERROR(ak4619_write(REG_POWER, 0x37), TAG, "Power-up");

  uint8_t if0 = 0, if1 = 0, sysclk = 0, dac_input = 0, dac_ctl = 0, power = 0;
  if (ak4619_read(REG_AUDIO_IF0, &if0) == ESP_OK &&
      ak4619_read(REG_AUDIO_IF1, &if1) == ESP_OK &&
      ak4619_read(REG_SYSCLK, &sysclk) == ESP_OK &&
      ak4619_read(REG_DAC_INPUT_SEL, &dac_input) == ESP_OK &&
      ak4619_read(REG_DAC_MUTE_FILTER, &dac_ctl) == ESP_OK &&
      ak4619_read(REG_POWER, &power) == ESP_OK) {
    ESP_LOGI(TAG,
             "Configured TDM128 IF0=%02x IF1=%02x SYSCLK=%02x DACSEL=%02x "
             "DACCTL=%02x PWR=%02x",
             if0, if1, sysclk, dac_input, dac_ctl, power);
  }
  return ESP_OK;
}
