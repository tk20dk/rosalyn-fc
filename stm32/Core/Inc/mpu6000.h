#ifndef MPU6000_H__
#define MPU6000_H__

#include "spi.h"


class TMpu6000
{
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

  static constexpr uint8_t PWR_MGMT_1   = 0x6b;
  static constexpr uint8_t PWR_MGMT_2   = 0x6c;

  static constexpr uint8_t WHO_AM_I     = 0x75;

public:
  TMpu6000( TSpi &Spi, GPIO_TypeDef *const PortNSS, uint32_t const PinNSS );

  bool Setup();
  uint8_t ReadU8( uint8_t const Command );
  uint16_t ReadU16( uint8_t const Command );
  void WriteU8( uint8_t const Command, uint8_t const Data );
  void WriteU16( uint8_t const Command, uint16_t const Data );

  int8_t ReadS8( uint8_t const Command )
  {
    return static_cast< int8_t >( ReadU8( Command ));
  }

  int16_t ReadS16( uint8_t const Command )
  {
    return static_cast< int16_t >( ReadU16( Command ));
  }

private:
  TSpi &Spi;
  uint32_t const PinNSS;
  GPIO_TypeDef *const PortNSS;
};


#endif // MPU6000_H__
