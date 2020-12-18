#ifndef MPU6000_H__
#define MPU6000_H__

#include "spi.h"


class TMpu6000
{
  static constexpr uint8_t CMD_READ     = 0x80;
  static constexpr uint8_t CMD_WRITE    = 0x7f;

  static constexpr uint8_t SMPRT_DIV    = 0x19;
  static constexpr uint8_t CONFIG       = 0x1a;
  static constexpr uint8_t GYRO_CONFIG  = 0x1b;
  static constexpr uint8_t ACCEL_CONFIG = 0x1c;

  static constexpr uint8_t ACCEL_XOUT   = 0x3b;
  static constexpr uint8_t ACCEL_XOUT_H = 0x3b;
  static constexpr uint8_t ACCEL_XOUT_L = 0x3c;
  static constexpr uint8_t ACCEL_YOUT   = 0x3d;
  static constexpr uint8_t ACCEL_YOUT_H = 0x3d;
  static constexpr uint8_t ACCEL_YOUT_L = 0x3e;
  static constexpr uint8_t ACCEL_ZOUT   = 0x3f;
  static constexpr uint8_t ACCEL_ZOUT_H = 0x3f;
  static constexpr uint8_t ACCEL_ZOUT_L = 0x40;

  static constexpr uint8_t TEMP_OUT     = 0x41;
  static constexpr uint8_t TEMP_OUT_H   = 0x41;
  static constexpr uint8_t TEMP_OUT_L   = 0x42;

  static constexpr uint8_t GYRO_XOUT    = 0x43;
  static constexpr uint8_t GYRO_XOUT_H  = 0x43;
  static constexpr uint8_t GYRO_XOUT_L  = 0x44;
  static constexpr uint8_t GYRO_YOUT    = 0x45;
  static constexpr uint8_t GYRO_YOUT_H  = 0x45;
  static constexpr uint8_t GYRO_YOUT_L  = 0x46;
  static constexpr uint8_t GYRO_ZOUT    = 0x47;
  static constexpr uint8_t GYRO_ZOUT_H  = 0x47;
  static constexpr uint8_t GYRO_ZOUT_L  = 0x48;

  static constexpr uint8_t SIGNAL_PATH_RESET = 0x68;
  static constexpr uint8_t USER_CTRL    = 0x6a;
  static constexpr uint8_t PWR_MGMT_1   = 0x6b;
  static constexpr uint8_t PWR_MGMT_2   = 0x6c;

  static constexpr uint8_t WHO_AM_I     = 0x75;

  struct TScopedLow
  {
    TScopedLow( GPIO_TypeDef *const PortNSS, uint32_t const PinNSS ) :
      PinNSS( PinNSS ),
      PortNSS( PortNSS )
    {
      LL_GPIO_ResetOutputPin( PortNSS, PinNSS );
    }
    ~TScopedLow()
    {
      LL_GPIO_SetOutputPin( PortNSS, PinNSS );
    }
  private:
    uint32_t const PinNSS;
    GPIO_TypeDef *const PortNSS;
  };

  struct TMpuData
  {
    int16_t AccelX;
    int16_t AccelY;
    int16_t AccelZ;
    int16_t Temp;
    int16_t GyroX;
    int16_t GyroY;
    int16_t GyroZ;
  };

public:
  TMpu6000( TSpi &Spi, GPIO_TypeDef *const PortNSS, uint32_t const PinNSS );

  bool Setup();
  bool Selftest();
  TMpuData GetData();

private:
  uint8_t Read( uint8_t const Command ) const;
  void Write( uint8_t const Command, uint8_t const Data ) const;

private:
  TSpi &Spi;
  uint32_t const PinNSS;
  GPIO_TypeDef *const PortNSS;
};

#endif // MPU6000_H__
