#ifndef BMP280_H__
#define BMP280_H__

#include "spi.h"
#include "system.h"


#define BME280_DIG_T1_LSB_REG			0x88
#define BME280_DIG_T1_MSB_REG			0x89
#define BME280_DIG_T2_LSB_REG			0x8A
#define BME280_DIG_T2_MSB_REG			0x8B
#define BME280_DIG_T3_LSB_REG			0x8C
#define BME280_DIG_T3_MSB_REG			0x8D
#define BME280_DIG_P1_LSB_REG			0x8E
#define BME280_DIG_P1_MSB_REG			0x8F
#define BME280_DIG_P2_LSB_REG			0x90
#define BME280_DIG_P2_MSB_REG			0x91
#define BME280_DIG_P3_LSB_REG			0x92
#define BME280_DIG_P3_MSB_REG			0x93
#define BME280_DIG_P4_LSB_REG			0x94
#define BME280_DIG_P4_MSB_REG			0x95
#define BME280_DIG_P5_LSB_REG			0x96
#define BME280_DIG_P5_MSB_REG			0x97
#define BME280_DIG_P6_LSB_REG			0x98
#define BME280_DIG_P6_MSB_REG			0x99
#define BME280_DIG_P7_LSB_REG			0x9A
#define BME280_DIG_P7_MSB_REG			0x9B
#define BME280_DIG_P8_LSB_REG			0x9C
#define BME280_DIG_P8_MSB_REG			0x9D
#define BME280_DIG_P9_LSB_REG			0x9E
#define BME280_DIG_P9_MSB_REG			0x9F
#define BME280_DIG_H1_REG				0xA1
#define BME280_CHIP_ID_REG				0xD0 //Chip ID
#define BME280_RST_REG					0xE0 //Softreset Reg
#define BME280_DIG_H2_LSB_REG			0xE1
#define BME280_DIG_H2_MSB_REG			0xE2
#define BME280_DIG_H3_REG				0xE3
#define BME280_DIG_H4_MSB_REG			0xE4
#define BME280_DIG_H4_LSB_REG			0xE5
#define BME280_DIG_H5_MSB_REG			0xE6
#define BME280_DIG_H6_REG				0xE7
#define BME280_CTRL_HUMIDITY_REG		0xF2 //Ctrl Humidity Reg
#define BME280_STAT_REG					0xF3 //Status Reg
#define BME280_CTRL_MEAS_REG			0xF4 //Ctrl Measure Reg
#define BME280_CONFIG_REG				0xF5 //Configuration Reg
#define BME280_PRESSURE_MSB_REG			0xF7 //Pressure MSB
#define BME280_PRESSURE_LSB_REG			0xF8 //Pressure LSB
#define BME280_PRESSURE_XLSB_REG		0xF9 //Pressure XLSB
#define BME280_TEMPERATURE_MSB_REG		0xFA //Temperature MSB
#define BME280_TEMPERATURE_LSB_REG		0xFB //Temperature LSB
#define BME280_TEMPERATURE_XLSB_REG		0xFC //Temperature XLSB
#define BME280_HUMIDITY_MSB_REG			0xFD //Humidity MSB
#define BME280_HUMIDITY_LSB_REG			0xFE //Humidity LSB


struct SensorSettings
{
  uint8_t runMode;
  uint8_t tStandby;
  uint8_t filter;
  uint8_t tempOverSample;
  uint8_t pressOverSample;
  uint8_t humidOverSample;
};

class TBmp280
{
  static constexpr uint8_t CMD_READ  = 0x80;
  static constexpr uint8_t CMD_WRITE = 0x7f;

public:
  TBmp280( TSpi &Spi, GPIO_TypeDef *const PortNSS, uint32_t const PinNSS );

  bool Setup();

  int32_t GetTemperature( void ) const
  {
    return Temperature;
  }
  uint32_t GetPressure( void ) const
  {
    return Pressure;
  }
  uint32_t GetHumidity( void ) const
  {
    return Humidity;
  }

  void SetRunMode();
  int32_t readTemperature();
  uint32_t readPressure();
  uint32_t readHumidity();

  uint8_t Read( uint8_t const Command ) const;
  void Write( uint8_t const Command, uint8_t const Data ) const;
  void Read( uint8_t const Command, uint8_t *const Buffer, uint32_t const Length ) const;

private:
  TSpi &Spi;
  uint32_t const PinNSS;
  GPIO_TypeDef *const PortNSS;

  int32_t       Temperature;
  uint32_t      Pressure;
  uint32_t      Humidity;
  int32_t       t_fine;
  SensorSettings settings;

  uint16_t T1;
  int16_t  T2;
  int16_t  T3;
  uint16_t P1;
  int16_t  P2;
  int16_t  P3;
  int16_t  P4;
  int16_t  P5;
  int16_t  P6;
  int16_t  P7;
  int16_t  P8;
  int16_t  P9;
  uint8_t  H1;
  int16_t  H2;
  uint8_t  H3;
  int16_t  H4;
  int16_t  H5;
  uint8_t  H6;
};

#endif // BMP280_H__
