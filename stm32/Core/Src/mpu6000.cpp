#include "mpu6000.h"


TMpu6000::TMpu6000( TSpi &Spi, GPIO_TypeDef *const PortNSS, uint32_t const PinNSS ) :
  Spi( Spi ),
  PinNSS( PinNSS ),
  PortNSS( PortNSS )
{
}

bool TMpu6000::Setup()
{
  if( ReadU8( WHO_AM_I ) != 0x68 )
  {
    return false;
  }

  WriteU8( SMPRT_DIV, 10 );
  WriteU8( CONFIG, 6 );
  WriteU8( GYRO_CONFIG, 0b11100000 );
  WriteU8( ACCEL_CONFIG, 0b00000000 );

  auto const AccelX = ReadS16( ACCEL_XOUT );
  auto const AccelY = ReadS16( ACCEL_YOUT );
  auto const AccelZ = ReadS16( ACCEL_ZOUT );
  auto const Temp   = ReadS16( TEMP_OUT );
  auto const GyroX  = ReadS16( GYRO_XOUT );
  auto const GyroY  = ReadS16( GYRO_YOUT );
  auto const GyroZ  = ReadS16( GYRO_ZOUT );

  if( AccelX == 0 )
  {
    return false;
  }

  if( AccelY == 0 )
  {
    return false;
  }

  if( AccelZ == 0 )
  {
    return false;
  }

  if( Temp == 0 )
  {
    return false;
  }

  if( GyroX == 0 )
  {
    return false;
  }

  if( GyroY == 0 )
  {
    return false;
  }

  if( GyroZ == 0 )
  {
    return false;
  }

  return true;
}

uint8_t TMpu6000::ReadU8( uint8_t const Command )
{
  LL_GPIO_ResetOutputPin( PortNSS, PinNSS );

  Spi.Write( Command | 0x80 );
  auto const Data = Spi.Read();

  LL_GPIO_SetOutputPin( PortNSS, PinNSS );
  return Data;
}

uint16_t TMpu6000::ReadU16( uint8_t const Command )
{
  LL_GPIO_ResetOutputPin( PortNSS, PinNSS );

  Spi.Write( Command | 0x80 );
  auto const DataH = Spi.Read();
  auto const DataL = Spi.Read();

  LL_GPIO_SetOutputPin( PortNSS, PinNSS );

  return DataH * 256 + DataL;
}

void TMpu6000::WriteU8( uint8_t const Command, uint8_t const Data )
{
  LL_GPIO_ResetOutputPin( PortNSS, PinNSS );

  Spi.Write( Command & ~0x80 );
  Spi.Write( Data );

  LL_GPIO_SetOutputPin( PortNSS, PinNSS );
}

void TMpu6000::WriteU16( uint8_t const Command, uint16_t const Data )
{
  LL_GPIO_ResetOutputPin( PortNSS, PinNSS );

  Spi.Write( Command & ~0x80 );
  Spi.Write( Data >> 8 );
  Spi.Write( Data );

  LL_GPIO_SetOutputPin( PortNSS, PinNSS );
}
