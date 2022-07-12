/**
 ******************************************************************************
 * @file    LSM6DSV16XSensor.cpp
 * @author  SRA
 * @version V1.0.0
 * @date    July 2022
 * @brief   Implementation of a LSM6DSV16X inertial measurement sensor.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2022 STMicroelectronics</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */


/* Includes ------------------------------------------------------------------*/

#include "LSM6DSV16XSensor.h"


/* Class Implementation ------------------------------------------------------*/

/** Constructor
 * @param i2c object of an helper class which handles the I2C peripheral
 * @param address the address of the component's instance
 */
LSM6DSV16XSensor::LSM6DSV16XSensor(TwoWire *i2c, uint8_t address) : dev_i2c(i2c), address(address)
{
  reg_ctx.write_reg = LSM6DSV16X_io_write;
  reg_ctx.read_reg = LSM6DSV16X_io_read;
  reg_ctx.handle = (void *)this;
  dev_spi = NULL;
  acc_is_enabled = 0L;
  gyro_is_enabled = 0L;
}

/** Constructor
 * @param spi object of an helper class which handles the SPI peripheral
 * @param cs_pin the chip select pin
 * @param spi_speed the SPI speed
 */
LSM6DSV16XSensor::LSM6DSV16XSensor(SPIClass *spi, int cs_pin, uint32_t spi_speed) : dev_spi(spi), cs_pin(cs_pin), spi_speed(spi_speed)
{
  reg_ctx.write_reg = LSM6DSV16X_io_write;
  reg_ctx.read_reg = LSM6DSV16X_io_read;
  reg_ctx.handle = (void *)this;
  dev_i2c = NULL;
  acc_is_enabled = 0L;
  gyro_is_enabled = 0L;
}

/**
 * @brief  Initialize the LSM6DSV16X sensor
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSV16XStatusTypeDef LSM6DSV16XSensor::begin()
{
  /* Enable register address automatically incremented during a multiple byte
  access with a serial interface. */
  if (lsm6dsv16x_auto_increment_set(&reg_ctx, PROPERTY_ENABLE) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  /* Enable BDU */
  if (lsm6dsv16x_block_data_update_set(&reg_ctx, PROPERTY_ENABLE) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  /* FIFO mode selection */
  if (lsm6dsv16x_fifo_mode_set(&reg_ctx, LSM6DSV16X_BYPASS_MODE) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  /* Select default output data rate. */
  acc_odr = LSM6DSV16X_XL_ODR_AT_120Hz;

  /* Output data rate selection - power down. */
  if (lsm6dsv16x_xl_data_rate_set(&reg_ctx, LSM6DSV16X_XL_ODR_OFF) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  /* Full scale selection. */
  if (lsm6dsv16x_xl_full_scale_set(&reg_ctx, LSM6DSV16X_2g) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  /* Select default output data rate. */
  gyro_odr = LSM6DSV16X_GY_ODR_AT_120Hz;

  /* Output data rate selection - power down. */
  if (lsm6dsv16x_gy_data_rate_set(&reg_ctx, LSM6DSV16X_GY_ODR_OFF) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  /* Full scale selection. */
  if (lsm6dsv16x_gy_full_scale_set(&reg_ctx, LSM6DSV16X_2000dps) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  initialized = 1U;

  return LSM6DSV16X_OK;
}

/**
 * @brief  Deinitialize the LSM6DSV16X sensor
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSV16XStatusTypeDef LSM6DSV16XSensor::end()
{
  /* Disable the component */
  if (Disable_X() != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  if (Disable_G() != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  /* Reset output data rate. */
  acc_odr = LSM6DSV16X_XL_ODR_OFF;
  gyro_odr = LSM6DSV16X_GY_ODR_OFF;

  initialized = 0U;

  return LSM6DSV16X_OK;
}

/**
 * @brief  Read component ID
 * @param  Id the WHO_AM_I value
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSV16XStatusTypeDef LSM6DSV16XSensor::ReadID(uint8_t *Id)
{
  if (lsm6dsv16x_device_id_get(&reg_ctx, Id) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  return LSM6DSV16X_OK;
}

/**
 * @brief  Enable the LSM6DSV16X accelerometer sensor
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSV16XStatusTypeDef LSM6DSV16XSensor::Enable_X()
{
  /* Check if the component is already enabled */
  if (acc_is_enabled == 1U) {
    return LSM6DSV16X_OK;
  }

  /* Output data rate selection. */
  if (lsm6dsv16x_xl_data_rate_set(&reg_ctx, acc_odr) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  acc_is_enabled = 1U;

  return LSM6DSV16X_OK;
}

/**
 * @brief  Disable the LSM6DSV16X accelerometer sensor
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSV16XStatusTypeDef LSM6DSV16XSensor::Disable_X()
{
  /* Check if the component is already disabled */
  if (acc_is_enabled == 0U) {
    return LSM6DSV16X_OK;
  }

  /* Get current output data rate. */
  if (lsm6dsv16x_xl_data_rate_get(&reg_ctx, &acc_odr) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  /* Output data rate selection - power down. */
  if (lsm6dsv16x_xl_data_rate_set(&reg_ctx, LSM6DSV16X_XL_ODR_OFF) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  acc_is_enabled = 0U;

  return LSM6DSV16X_OK;
}

/**
 * @brief  Get the LSM6DSV16X accelerometer sensor sensitivity
 * @param  Sensitivity pointer
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSV16XStatusTypeDef LSM6DSV16XSensor::Get_X_Sensitivity(float *Sensitivity)
{
  LSM6DSV16XStatusTypeDef ret = LSM6DSV16X_OK;
  lsm6dsv16x_xl_full_scale_t full_scale;

  /* Read actual full scale selection from sensor. */
  if (lsm6dsv16x_xl_full_scale_get(&reg_ctx, &full_scale) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  /* Store the Sensitivity based on actual full scale. */
  switch (full_scale) {
    case LSM6DSV16X_2g:
      *Sensitivity = LSM6DSV16X_ACC_SENSITIVITY_FS_2G;
      break;

    case LSM6DSV16X_4g:
      *Sensitivity = LSM6DSV16X_ACC_SENSITIVITY_FS_4G;
      break;

    case LSM6DSV16X_8g:
      *Sensitivity = LSM6DSV16X_ACC_SENSITIVITY_FS_8G;
      break;

    case LSM6DSV16X_16g:
      *Sensitivity = LSM6DSV16X_ACC_SENSITIVITY_FS_16G;
      break;

    default:
      ret = LSM6DSV16X_ERROR;
      break;
  }

  return ret;
}

/**
 * @brief  Get the LSM6DSV16X accelerometer sensor output data rate
 * @param  pObj the device pObj
 * @param  Odr pointer where the output data rate is written
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSV16XStatusTypeDef LSM6DSV16XSensor::Get_X_ODR(float *Odr)
{
  LSM6DSV16XStatusTypeDef ret = LSM6DSV16X_OK;
  lsm6dsv16x_xl_data_rate_t odr_low_level;

  /* Get current output data rate. */
  if (lsm6dsv16x_xl_data_rate_get(&reg_ctx, &odr_low_level) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  switch (odr_low_level) {
    case LSM6DSV16X_XL_ODR_OFF:
      *Odr = 0.0f;
      break;

    case LSM6DSV16X_XL_ODR_AT_1Hz875:
      *Odr = 1.875f;
      break;

    case LSM6DSV16X_XL_ODR_AT_7Hz5:
      *Odr = 7.5f;
      break;

    case LSM6DSV16X_XL_ODR_AT_15Hz:
      *Odr = 15.0f;
      break;

    case LSM6DSV16X_XL_ODR_AT_30Hz:
      *Odr = 30.0f;
      break;

    case LSM6DSV16X_XL_ODR_AT_60Hz:
      *Odr = 60.0f;
      break;

    case LSM6DSV16X_XL_ODR_AT_120Hz:
      *Odr = 120.0f;
      break;

    case LSM6DSV16X_XL_ODR_AT_240Hz:
      *Odr = 240.0f;
      break;

    case LSM6DSV16X_XL_ODR_AT_480Hz:
      *Odr = 480.0f;
      break;

    case LSM6DSV16X_XL_ODR_AT_960Hz:
      *Odr = 960.0f;
      break;

    case LSM6DSV16X_XL_ODR_AT_1920Hz:
      *Odr = 1920.0f;
      break;

    case LSM6DSV16X_XL_ODR_AT_3840Hz:
      *Odr = 3840.0f;
      break;

    case LSM6DSV16X_XL_ODR_AT_7680Hz:
      *Odr = 7680.0f;
      break;

    default:
      ret = LSM6DSV16X_ERROR;
      break;
  }

  return ret;
}

/**
 * @brief  Get the LSM6DSV16X accelerometer sensor axes
 * @param  Acceleration pointer where the values of the axes are written
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSV16XStatusTypeDef LSM6DSV16XSensor::Get_X_Axes(int32_t *Acceleration)
{
  lsm6dsv16x_axis3bit16_t data_raw;
  float sensitivity = 0.0f;

  /* Read raw data values. */
  if (lsm6dsv16x_acceleration_raw_get(&reg_ctx, data_raw.i16bit) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  /* Get LSM6DSV16X actual sensitivity. */
  if (Get_X_Sensitivity(&sensitivity) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  /* Calculate the data. */
  Acceleration[0] = (int32_t)((float)((float)data_raw.i16bit[0] * sensitivity));
  Acceleration[1] = (int32_t)((float)((float)data_raw.i16bit[1] * sensitivity));
  Acceleration[2] = (int32_t)((float)((float)data_raw.i16bit[2] * sensitivity));

  return LSM6DSV16X_OK;
}

/**
 * @brief  Set the LSM6DSV16X accelerometer sensor output data rate
 * @param  Odr the output data rate value to be set
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSV16XStatusTypeDef LSM6DSV16XSensor::Set_X_ODR(float Odr, LSM6DSV16X_ACC_Operating_Mode_t Mode)
{
  switch (Mode) {
    case LSM6DSV16X_ACC_HIGH_PERFORMANCE_MODE: {
        if (lsm6dsv16x_xl_mode_set(&reg_ctx, LSM6DSV16X_XL_HIGH_PERFORMANCE_MD) != LSM6DSV16X_OK) {
          return LSM6DSV16X_ERROR;
        }

        /* Valid ODR: 7.5Hz <= Odr <= 7.68kHz */
        Odr = (Odr <    7.5f) ?    7.5f
              : (Odr > 7680.0f) ? 7680.0f
              :                       Odr;
        break;
      }

    case LSM6DSV16X_ACC_HIGH_ACCURACY_MODE:
      // TODO: Not implemented.
      // NOTE: According to datasheet, section `6.5 High-accuracy ODR mode`:
      // "... the other sensor also has to be configured in high-accuracy ODR (HAODR) mode."
      return LSM6DSV16X_ERROR;

    case LSM6DSV16X_ACC_NORMAL_MODE: {
        if (lsm6dsv16x_xl_mode_set(&reg_ctx, LSM6DSV16X_XL_NORMAL_MD) != LSM6DSV16X_OK) {
          return LSM6DSV16X_ERROR;
        }

        /* Valid ODR: 7.5Hz <= Odr <= 1.92kHz */
        Odr = (Odr <    7.5f) ?    7.5f
              : (Odr > 1920.0f) ? 1920.0f
              :                       Odr;
        break;
      }

    case LSM6DSV16X_ACC_LOW_POWER_MODE1: {
        if (lsm6dsv16x_xl_mode_set(&reg_ctx, LSM6DSV16X_XL_LOW_POWER_2_AVG_MD) != LSM6DSV16X_OK) {
          return LSM6DSV16X_ERROR;
        }

        /* Valid ODR: 1.875Hz;  15Hz <= Odr <= 240kHz */
        Odr = (Odr ==   1.875f) ?    Odr
              : (Odr <   15.000f) ?  15.0f
              : (Odr >  240.000f) ? 240.0f
              :                        Odr;
        break;
      }

    case LSM6DSV16X_ACC_LOW_POWER_MODE2: {
        if (lsm6dsv16x_xl_mode_set(&reg_ctx, LSM6DSV16X_XL_LOW_POWER_4_AVG_MD) != LSM6DSV16X_OK) {
          return LSM6DSV16X_ERROR;
        }

        /* Valid ODR: 1.875Hz;  15Hz <= Odr <= 240kHz */
        Odr = (Odr ==   1.875f) ?    Odr
              : (Odr <   15.000f) ?  15.0f
              : (Odr >  240.000f) ? 240.0f
              :                        Odr;
        break;
      }

    case LSM6DSV16X_ACC_LOW_POWER_MODE3: {
        if (lsm6dsv16x_xl_mode_set(&reg_ctx, LSM6DSV16X_XL_LOW_POWER_8_AVG_MD) != LSM6DSV16X_OK) {
          return LSM6DSV16X_ERROR;
        }

        /* Valid ODR: 1.875Hz;  15Hz <= Odr <= 240kHz */
        Odr = (Odr ==   1.875f) ?    Odr
              : (Odr <   15.000f) ?  15.0f
              : (Odr >  240.000f) ? 240.0f
              :                        Odr;
        break;
      }

    default:
      return LSM6DSV16X_ERROR;
  }

  if (acc_is_enabled == 1U) {
    return Set_X_ODR_When_Enabled(Odr);
  } else {
    return Set_X_ODR_When_Disabled(Odr);
  }
}

/**
 * @brief  Set the LSM6DSV16X accelerometer sensor output data rate when enabled
 * @param  Odr the functional output data rate to be set
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSV16XStatusTypeDef LSM6DSV16XSensor::Set_X_ODR_When_Enabled(float Odr)
{
  lsm6dsv16x_xl_data_rate_t new_odr;

  new_odr = (Odr <=    1.875f) ? LSM6DSV16X_XL_ODR_AT_1Hz875
            : (Odr <=    7.5f) ? LSM6DSV16X_XL_ODR_AT_7Hz5
            : (Odr <=   15.0f) ? LSM6DSV16X_XL_ODR_AT_15Hz
            : (Odr <=   30.0f) ? LSM6DSV16X_XL_ODR_AT_30Hz
            : (Odr <=   60.0f) ? LSM6DSV16X_XL_ODR_AT_60Hz
            : (Odr <=  120.0f) ? LSM6DSV16X_XL_ODR_AT_120Hz
            : (Odr <=  240.0f) ? LSM6DSV16X_XL_ODR_AT_240Hz
            : (Odr <=  480.0f) ? LSM6DSV16X_XL_ODR_AT_480Hz
            : (Odr <=  960.0f) ? LSM6DSV16X_XL_ODR_AT_960Hz
            : (Odr <= 1920.0f) ? LSM6DSV16X_XL_ODR_AT_1920Hz
            : (Odr <= 3840.0f) ? LSM6DSV16X_XL_ODR_AT_3840Hz
            :                    LSM6DSV16X_XL_ODR_AT_7680Hz;

  /* Output data rate selection. */
  if (lsm6dsv16x_xl_data_rate_set(&reg_ctx, new_odr) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  return LSM6DSV16X_OK;
}

/**
 * @brief  Set the LSM6DSV16X accelerometer sensor output data rate when disabled
 * @param  Odr the functional output data rate to be set
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSV16XStatusTypeDef LSM6DSV16XSensor::Set_X_ODR_When_Disabled(float Odr)
{
  acc_odr = (Odr <=    1.875f) ? LSM6DSV16X_XL_ODR_AT_1Hz875
            : (Odr <=    7.5f) ? LSM6DSV16X_XL_ODR_AT_7Hz5
            : (Odr <=   15.0f) ? LSM6DSV16X_XL_ODR_AT_15Hz
            : (Odr <=   30.0f) ? LSM6DSV16X_XL_ODR_AT_30Hz
            : (Odr <=   60.0f) ? LSM6DSV16X_XL_ODR_AT_60Hz
            : (Odr <=  120.0f) ? LSM6DSV16X_XL_ODR_AT_120Hz
            : (Odr <=  240.0f) ? LSM6DSV16X_XL_ODR_AT_240Hz
            : (Odr <=  480.0f) ? LSM6DSV16X_XL_ODR_AT_480Hz
            : (Odr <=  960.0f) ? LSM6DSV16X_XL_ODR_AT_960Hz
            : (Odr <= 1920.0f) ? LSM6DSV16X_XL_ODR_AT_1920Hz
            : (Odr <= 3840.0f) ? LSM6DSV16X_XL_ODR_AT_3840Hz
            :                    LSM6DSV16X_XL_ODR_AT_7680Hz;

  return LSM6DSV16X_OK;
}

/**
 * @brief  Get the LSM6DSV16X accelerometer sensor full scale
 * @param  FullScale pointer where the full scale is written
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSV16XStatusTypeDef LSM6DSV16XSensor::Get_X_FS(int32_t *FullScale)
{
  LSM6DSV16XStatusTypeDef ret = LSM6DSV16X_OK;
  lsm6dsv16x_xl_full_scale_t fs_low_level;

  /* Read actual full scale selection from sensor. */
  if (lsm6dsv16x_xl_full_scale_get(&reg_ctx, &fs_low_level) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  switch (fs_low_level) {
    case LSM6DSV16X_2g:
      *FullScale =  2;
      break;

    case LSM6DSV16X_4g:
      *FullScale =  4;
      break;

    case LSM6DSV16X_8g:
      *FullScale =  8;
      break;

    case LSM6DSV16X_16g:
      *FullScale = 16;
      break;

    default:
      ret = LSM6DSV16X_ERROR;
      break;
  }

  return ret;
}

/**
 * @brief  Set the LSM6DSV16X accelerometer sensor full scale
 * @param  FullScale the functional full scale to be set
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSV16XStatusTypeDef LSM6DSV16XSensor::Set_X_FS(int32_t FullScale)
{
  lsm6dsv16x_xl_full_scale_t new_fs;

  /* Seems like MISRA C-2012 rule 14.3a violation but only from single file statical analysis point of view because
     the parameter passed to the function is not known at the moment of analysis */
  new_fs = (FullScale <= 2) ? LSM6DSV16X_2g
           : (FullScale <= 4) ? LSM6DSV16X_4g
           : (FullScale <= 8) ? LSM6DSV16X_8g
           :                    LSM6DSV16X_16g;

  if (lsm6dsv16x_xl_full_scale_set(&reg_ctx, new_fs) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  return LSM6DSV16X_OK;
}

/**
 * @brief  Get the LSM6DSV16X accelerometer sensor raw axes
 * @param  Value pointer where the raw values of the axes are written
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSV16XStatusTypeDef LSM6DSV16XSensor::Get_X_AxesRaw(int16_t *Value)
{
  lsm6dsv16x_axis3bit16_t data_raw;

  /* Read raw data values. */
  if (lsm6dsv16x_acceleration_raw_get(&reg_ctx, data_raw.i16bit) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  /* Format the data. */
  Value[0] = data_raw.i16bit[0];
  Value[1] = data_raw.i16bit[1];
  Value[2] = data_raw.i16bit[2];

  return LSM6DSV16X_OK;
}

/**
 * @brief  Get the LSM6DSV16X ACC data ready bit value
 * @param  Status the status of data ready bit
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSV16XStatusTypeDef LSM6DSV16XSensor::Get_X_DRDY_Status(uint8_t *Status)
{
  lsm6dsv16x_all_sources_t val;

  if (lsm6dsv16x_all_sources_get(&reg_ctx, &val) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  *Status = val.drdy_xl;
  return LSM6DSV16X_OK;
}

/**
 * @brief  Set the LSM6DSV16X accelerometer power mode
 * @param  PowerMode Value of the powerMode
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSV16XStatusTypeDef LSM6DSV16XSensor::Set_X_Power_Mode(uint8_t PowerMode)
{
  if (lsm6dsv16x_xl_mode_set(&reg_ctx, (lsm6dsv16x_xl_mode_t)PowerMode) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  return LSM6DSV16X_OK;
}

/**
 * @brief  Set the LSM6DSV16X accelerometer filter mode
 * @param  LowHighPassFlag 0/1 for setting low-pass/high-pass filter mode
 * @param  FilterMode Value of the filter Mode
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSV16XStatusTypeDef LSM6DSV16XSensor::Set_X_Filter_Mode(uint8_t LowHighPassFlag, uint8_t FilterMode)
{
  if (LowHighPassFlag == 0) {
    /*Set accelerometer low_pass filter-mode*/

    /*Set to 1 LPF2 bit (CTRL8_XL)*/
    if (lsm6dsv16x_filt_xl_lp2_set(&reg_ctx, 1) != LSM6DSV16X_OK) {
      return LSM6DSV16X_ERROR;
    }
    if (lsm6dsv16x_filt_xl_lp2_bandwidth_set(&reg_ctx, (lsm6dsv16x_filt_xl_lp2_bandwidth_t)FilterMode) != LSM6DSV16X_OK) {
      return LSM6DSV16X_ERROR;
    }
  } else {
    if (lsm6dsv16x_filt_xl_lp2_set(&reg_ctx, 0) != LSM6DSV16X_OK) {
      return LSM6DSV16X_ERROR;
    }
    /*Set accelerometer high_pass filter-mode*/
    if (lsm6dsv16x_filt_xl_lp2_bandwidth_set(&reg_ctx, (lsm6dsv16x_filt_xl_lp2_bandwidth_t)FilterMode) != LSM6DSV16X_OK) {
      return LSM6DSV16X_ERROR;
    }
  }
  return LSM6DSV16X_OK;
}

/**
 * @brief  Enable the LSM6DSV16X gyroscope sensor
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSV16XStatusTypeDef LSM6DSV16XSensor::Enable_G()
{
  /* Check if the component is already enabled */
  if (gyro_is_enabled == 1U) {
    return LSM6DSV16X_OK;
  }

  /* Output data rate selection. */
  if (lsm6dsv16x_gy_data_rate_set(&reg_ctx, gyro_odr) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  gyro_is_enabled = 1U;

  return LSM6DSV16X_OK;
}

/**
 * @brief  Disable the LSM6DSV16X gyroscope sensor
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSV16XStatusTypeDef LSM6DSV16XSensor::Disable_G()
{
  /* Check if the component is already disabled */
  if (gyro_is_enabled == 0U) {
    return LSM6DSV16X_OK;
  }

  /* Get current output data rate. */
  if (lsm6dsv16x_gy_data_rate_get(&reg_ctx, &gyro_odr) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  /* Output data rate selection - power down. */
  if (lsm6dsv16x_gy_data_rate_set(&reg_ctx, LSM6DSV16X_GY_ODR_OFF) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  gyro_is_enabled = 0U;

  return LSM6DSV16X_OK;
}

/**
 * @brief  Get the LSM6DSV16X gyroscope sensor sensitivity
 * @param  Sensitivity pointer
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSV16XStatusTypeDef LSM6DSV16XSensor::Get_G_Sensitivity(float *Sensitivity)
{
  LSM6DSV16XStatusTypeDef ret = LSM6DSV16X_OK;
  lsm6dsv16x_gy_full_scale_t full_scale;

  /* Read actual full scale selection from sensor. */
  if (lsm6dsv16x_gy_full_scale_get(&reg_ctx, &full_scale) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  /* Store the sensitivity based on actual full scale. */
  switch (full_scale) {
    case LSM6DSV16X_125dps:
      *Sensitivity = LSM6DSV16X_GYRO_SENSITIVITY_FS_125DPS;
      break;

    case LSM6DSV16X_250dps:
      *Sensitivity = LSM6DSV16X_GYRO_SENSITIVITY_FS_250DPS;
      break;

    case LSM6DSV16X_500dps:
      *Sensitivity = LSM6DSV16X_GYRO_SENSITIVITY_FS_500DPS;
      break;

    case LSM6DSV16X_1000dps:
      *Sensitivity = LSM6DSV16X_GYRO_SENSITIVITY_FS_1000DPS;
      break;

    case LSM6DSV16X_2000dps:
      *Sensitivity = LSM6DSV16X_GYRO_SENSITIVITY_FS_2000DPS;
      break;

    case LSM6DSV16X_4000dps:
      *Sensitivity = LSM6DSV16X_GYRO_SENSITIVITY_FS_4000DPS;
      break;

    default:
      ret = LSM6DSV16X_ERROR;
      break;
  }

  return ret;
}

/**
 * @brief  Get the LSM6DSV16X gyroscope sensor output data rate
 * @param  Odr pointer where the output data rate is written
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSV16XStatusTypeDef LSM6DSV16XSensor::Get_G_ODR(float *Odr)
{
  LSM6DSV16XStatusTypeDef ret = LSM6DSV16X_OK;
  lsm6dsv16x_gy_data_rate_t odr_low_level;

  /* Get current output data rate. */
  if (lsm6dsv16x_gy_data_rate_get(&reg_ctx, &odr_low_level) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  switch (odr_low_level) {
    case LSM6DSV16X_GY_ODR_OFF:
      *Odr = 0.0f;
      break;

    case LSM6DSV16X_GY_ODR_AT_7Hz5:
      *Odr = 7.5f;
      break;

    case LSM6DSV16X_GY_ODR_AT_15Hz:
      *Odr = 15.0f;
      break;

    case LSM6DSV16X_GY_ODR_AT_30Hz:
      *Odr = 30.0f;
      break;

    case LSM6DSV16X_GY_ODR_AT_60Hz:
      *Odr = 60.0f;
      break;

    case LSM6DSV16X_GY_ODR_AT_120Hz:
      *Odr = 120.0f;
      break;

    case LSM6DSV16X_GY_ODR_AT_240Hz:
      *Odr = 240.0f;
      break;

    case LSM6DSV16X_GY_ODR_AT_480Hz:
      *Odr = 480.0f;
      break;

    case LSM6DSV16X_GY_ODR_AT_960Hz:
      *Odr = 960.0f;
      break;

    case LSM6DSV16X_GY_ODR_AT_1920Hz:
      *Odr = 1920.0f;
      break;

    case LSM6DSV16X_GY_ODR_AT_3840Hz:
      *Odr = 3840.0f;
      break;

    case LSM6DSV16X_GY_ODR_AT_7680Hz:
      *Odr = 7680.0f;
      break;

    default:
      ret = LSM6DSV16X_ERROR;
      break;
  }

  return ret;
}

/**
 * @brief  Set the LSM6DSV16X gyroscope sensor output data rate with operating mode
 * @param  Odr the output data rate value to be set
 * @param  Mode the gyroscope operating mode
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSV16XStatusTypeDef LSM6DSV16XSensor::Set_G_ODR(float Odr, LSM6DSV16X_GYRO_Operating_Mode_t Mode)
{
  switch (Mode) {
    case LSM6DSV16X_GYRO_HIGH_PERFORMANCE_MODE: {
        if (lsm6dsv16x_gy_mode_set(&reg_ctx, LSM6DSV16X_GY_HIGH_PERFORMANCE_MD) != LSM6DSV16X_OK) {
          return LSM6DSV16X_ERROR;
        }

        /* Valid ODR: 7.5Hz <= Odr <= 7.68kHz */
        Odr = (Odr <    7.5f) ?    7.5f
              : (Odr > 7680.0f) ? 7680.0f
              :                       Odr;
        break;
      }

    case LSM6DSV16X_GYRO_HIGH_ACCURACY_MODE:
      // TODO: Not implemented.
      // NOTE: According to datasheet, section `6.5 High-accuracy ODR mode`:
      // "... the other sensor also has to be configured in high-accuracy ODR (HAODR) mode."
      return LSM6DSV16X_ERROR;

    case LSM6DSV16X_GYRO_SLEEP_MODE:
      // TODO: Not implemented.
      // NOTE: Unknown ODR validity for this mode
      return LSM6DSV16X_ERROR;

    case LSM6DSV16X_GYRO_LOW_POWER_MODE: {
        if (lsm6dsv16x_gy_mode_set(&reg_ctx, LSM6DSV16X_GY_LOW_POWER_MD) != LSM6DSV16X_OK) {
          return LSM6DSV16X_ERROR;
        }

        /* Valid ODR: 7.5Hz <= Odr <= 240kHz */
        Odr = (Odr <   7.5f) ?   7.5f
              : (Odr > 240.0f) ? 240.0f
              :                     Odr;
        break;
      }

    default:
      return LSM6DSV16X_ERROR;
  }

  if (gyro_is_enabled == 1U) {
    return Set_G_ODR_When_Enabled(Odr);
  } else {
    return Set_G_ODR_When_Disabled(Odr);
  }
}

/**
 * @brief  Set the LSM6DSV16X gyroscope sensor output data rate when enabled
 * @param  Odr the functional output data rate to be set
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSV16XStatusTypeDef LSM6DSV16XSensor::Set_G_ODR_When_Enabled(float Odr)
{
  lsm6dsv16x_gy_data_rate_t new_odr;

  new_odr = (Odr <=    7.5f) ? LSM6DSV16X_GY_ODR_AT_7Hz5
            : (Odr <=   15.0f) ? LSM6DSV16X_GY_ODR_AT_15Hz
            : (Odr <=   30.0f) ? LSM6DSV16X_GY_ODR_AT_30Hz
            : (Odr <=   60.0f) ? LSM6DSV16X_GY_ODR_AT_60Hz
            : (Odr <=  120.0f) ? LSM6DSV16X_GY_ODR_AT_120Hz
            : (Odr <=  240.0f) ? LSM6DSV16X_GY_ODR_AT_240Hz
            : (Odr <=  480.0f) ? LSM6DSV16X_GY_ODR_AT_480Hz
            : (Odr <=  960.0f) ? LSM6DSV16X_GY_ODR_AT_960Hz
            : (Odr <= 1920.0f) ? LSM6DSV16X_GY_ODR_AT_1920Hz
            : (Odr <= 3840.0f) ? LSM6DSV16X_GY_ODR_AT_3840Hz
            :                    LSM6DSV16X_GY_ODR_AT_7680Hz;

  /* Output data rate selection. */
  if (lsm6dsv16x_gy_data_rate_set(&reg_ctx, new_odr) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  return LSM6DSV16X_OK;
}

/**
 * @brief  Set the LSM6DSV16X gyroscope sensor output data rate when disabled
 * @param  Odr the functional output data rate to be set
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSV16XStatusTypeDef LSM6DSV16XSensor::Set_G_ODR_When_Disabled(float Odr)
{
  gyro_odr = (Odr <=    7.5f) ? LSM6DSV16X_GY_ODR_AT_7Hz5
             : (Odr <=   15.0f) ? LSM6DSV16X_GY_ODR_AT_15Hz
             : (Odr <=   30.0f) ? LSM6DSV16X_GY_ODR_AT_30Hz
             : (Odr <=   60.0f) ? LSM6DSV16X_GY_ODR_AT_60Hz
             : (Odr <=  120.0f) ? LSM6DSV16X_GY_ODR_AT_120Hz
             : (Odr <=  240.0f) ? LSM6DSV16X_GY_ODR_AT_240Hz
             : (Odr <=  480.0f) ? LSM6DSV16X_GY_ODR_AT_480Hz
             : (Odr <=  960.0f) ? LSM6DSV16X_GY_ODR_AT_960Hz
             : (Odr <= 1920.0f) ? LSM6DSV16X_GY_ODR_AT_1920Hz
             : (Odr <= 3840.0f) ? LSM6DSV16X_GY_ODR_AT_3840Hz
             :                    LSM6DSV16X_GY_ODR_AT_7680Hz;

  return LSM6DSV16X_OK;
}

/**
 * @brief  Get the LSM6DSV16X gyroscope sensor full scale
 * @param  FullScale pointer where the full scale is written
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSV16XStatusTypeDef LSM6DSV16XSensor::Get_G_FS(int32_t  *FullScale)
{
  LSM6DSV16XStatusTypeDef ret = LSM6DSV16X_OK;
  lsm6dsv16x_gy_full_scale_t fs_low_level;

  /* Read actual full scale selection from sensor. */
  if (lsm6dsv16x_gy_full_scale_get(&reg_ctx, &fs_low_level) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  switch (fs_low_level) {
    case LSM6DSV16X_125dps:
      *FullScale =  125;
      break;

    case LSM6DSV16X_250dps:
      *FullScale =  250;
      break;

    case LSM6DSV16X_500dps:
      *FullScale =  500;
      break;

    case LSM6DSV16X_1000dps:
      *FullScale = 1000;
      break;

    case LSM6DSV16X_2000dps:
      *FullScale = 2000;
      break;

    case LSM6DSV16X_4000dps:
      *FullScale = 4000;
      break;

    default:
      ret = LSM6DSV16X_ERROR;
      break;
  }

  return ret;
}

/**
 * @brief  Set the LSM6DSV16X gyroscope sensor full scale
 * @param  FullScale the functional full scale to be set
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSV16XStatusTypeDef LSM6DSV16XSensor::Set_G_FS(int32_t FullScale)
{
  lsm6dsv16x_gy_full_scale_t new_fs;

  new_fs = (FullScale <= 125)  ? LSM6DSV16X_125dps
           : (FullScale <= 250)  ? LSM6DSV16X_250dps
           : (FullScale <= 500)  ? LSM6DSV16X_500dps
           : (FullScale <= 1000) ? LSM6DSV16X_1000dps
           : (FullScale <= 2000) ? LSM6DSV16X_2000dps
           :                       LSM6DSV16X_4000dps;

  if (lsm6dsv16x_gy_full_scale_set(&reg_ctx, new_fs) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  return LSM6DSV16X_OK;
}

/**
 * @brief  Get the LSM6DSV16X gyroscope sensor raw axes
 * @param  pObj the device pObj
 * @param  Value pointer where the raw values of the axes are written
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSV16XStatusTypeDef LSM6DSV16XSensor::Get_G_AxesRaw(int16_t *Value)
{
  lsm6dsv16x_axis3bit16_t data_raw;

  /* Read raw data values. */
  if (lsm6dsv16x_angular_rate_raw_get(&reg_ctx, data_raw.i16bit) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  /* Format the data. */
  Value[0] = data_raw.i16bit[0];
  Value[1] = data_raw.i16bit[1];
  Value[2] = data_raw.i16bit[2];

  return LSM6DSV16X_OK;
}

/**
 * @brief  Get the LSM6DSV16X gyroscope sensor axes
 * @param  AngularRate pointer where the values of the axes are written
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSV16XStatusTypeDef LSM6DSV16XSensor::Get_G_Axes(int32_t *AngularRate)
{
  lsm6dsv16x_axis3bit16_t data_raw;
  float sensitivity;

  /* Read raw data values. */
  if (lsm6dsv16x_angular_rate_raw_get(&reg_ctx, data_raw.i16bit) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  /* Get LSM6DSV16X actual sensitivity. */
  if (Get_G_Sensitivity(&sensitivity) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  /* Calculate the data. */
  AngularRate[0] = (int32_t)((float)((float)data_raw.i16bit[0] * sensitivity));
  AngularRate[1] = (int32_t)((float)((float)data_raw.i16bit[1] * sensitivity));
  AngularRate[2] = (int32_t)((float)((float)data_raw.i16bit[2] * sensitivity));

  return LSM6DSV16X_OK;
}

/**
 * @brief  Get the LSM6DSV16X GYRO data ready bit value
 * @param  Status the status of data ready bit
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSV16XStatusTypeDef LSM6DSV16XSensor::Get_G_DRDY_Status(uint8_t *Status)
{
  lsm6dsv16x_all_sources_t val;

  if (lsm6dsv16x_all_sources_get(&reg_ctx, &val) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  *Status = val.drdy_gy;
  return LSM6DSV16X_OK;
}

/**
 * @brief  Set the LSM6DSV16X gyroscope power mode
 * @param  PowerMode Value of the powerMode
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSV16XStatusTypeDef LSM6DSV16XSensor::Set_G_Power_Mode(uint8_t PowerMode)
{
  if (lsm6dsv16x_gy_mode_set(&reg_ctx, (lsm6dsv16x_gy_mode_t)PowerMode) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  return LSM6DSV16X_OK;
}

/**
 * @brief  Set the LSM6DSV16X gyroscope filter mode
 * @param  LowHighPassFlag 0/1 for setting low-pass/high-pass filter mode
 * @param  FilterMode Value of the filter Mode
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSV16XStatusTypeDef LSM6DSV16XSensor::Set_G_Filter_Mode(uint8_t LowHighPassFlag, uint8_t FilterMode)
{
  if (LowHighPassFlag == 0) {
    /*Set gyroscope low_pass 1 filter-mode*/
    /* Enable low-pass filter */
    if (lsm6dsv16x_filt_gy_lp1_set(&reg_ctx, 1) != LSM6DSV16X_OK) {
      return LSM6DSV16X_ERROR;
    }
    if (lsm6dsv16x_filt_gy_lp1_bandwidth_set(&reg_ctx, (lsm6dsv16x_filt_gy_lp1_bandwidth_t)FilterMode) != LSM6DSV16X_OK) {
      return LSM6DSV16X_ERROR;
    }
  } else {
    /*Set gyroscope high_pass filter-mode*/
    /* Enable high-pass filter */
    if (lsm6dsv16x_filt_gy_lp1_set(&reg_ctx, 0) != LSM6DSV16X_OK) {
      return LSM6DSV16X_ERROR;
    }
    if (lsm6dsv16x_filt_gy_lp1_bandwidth_set(&reg_ctx, (lsm6dsv16x_filt_gy_lp1_bandwidth_t)FilterMode) != LSM6DSV16X_OK) {
      return LSM6DSV16X_ERROR;
    }
  }
  return LSM6DSV16X_OK;
}

/**
 * @brief  Get the LSM6DSV16X register value
 * @param  Reg address to be read
 * @param  Data pointer where the value is written
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSV16XStatusTypeDef LSM6DSV16XSensor::Read_Reg(uint8_t Reg, uint8_t *Data)
{
  if (lsm6dsv16x_read_reg(&reg_ctx, Reg, Data, 1) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  return LSM6DSV16X_OK;
}

/**
 * @brief  Set the LSM6DSV16X register value
 * @param  Reg address to be written
 * @param  Data value to be written
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSV16XStatusTypeDef LSM6DSV16XSensor::Write_Reg(uint8_t Reg, uint8_t Data)
{
  if (lsm6dsv16x_write_reg(&reg_ctx, Reg, &Data, 1) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  return LSM6DSV16X_OK;
}

int32_t LSM6DSV16X_io_write(void *handle, uint8_t WriteAddr, uint8_t *pBuffer, uint16_t nBytesToWrite)
{
  return ((LSM6DSV16XSensor *)handle)->IO_Write(pBuffer, WriteAddr, nBytesToWrite);
}

int32_t LSM6DSV16X_io_read(void *handle, uint8_t ReadAddr, uint8_t *pBuffer, uint16_t nBytesToRead)
{
  return ((LSM6DSV16XSensor *)handle)->IO_Read(pBuffer, ReadAddr, nBytesToRead);
}
