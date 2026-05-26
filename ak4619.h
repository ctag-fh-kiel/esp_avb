#ifndef _AK4619_H_
#define _AK4619_H_

#include "avb.h"
#include "driver/i2c_master.h"

esp_err_t ak4619_configure(avb_state_s *state, i2c_master_bus_handle_t bus);
esp_err_t ak4619_set_vol(float db);
esp_err_t ak4619_set_mic_gain(float db);

#endif
