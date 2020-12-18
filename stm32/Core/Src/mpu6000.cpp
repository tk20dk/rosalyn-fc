 #include "mpu6000.h"


TMpu6000::TMpu6000( TSpi &Spi, GPIO_TypeDef *const PortNSS, uint32_t const PinNSS ) :
  Spi( Spi ),
  PinNSS( PinNSS ),
  PortNSS( PortNSS )
{
}

bool TMpu6000::Setup()
{
  // Reset device
  Write( PWR_MGMT_1, 0b10000000 );
  LL_mDelay( 100 );
  Write( SIGNAL_PATH_RESET, 0b00000111 );
  LL_mDelay( 100 );

  // Disable I2C interface
  Write( USER_CTRL, 0b00010000 );

  if( Read( WHO_AM_I ) != 0x68 )
  {
    return false;
  }

  Write( SMPRT_DIV, 0 );
  Write( CONFIG, 0 );
  Write( GYRO_CONFIG, 0b00011000 );  // ± 2000 °/s
  Write( ACCEL_CONFIG, 0b00011000 ); // ± 16g

  // Exit sleep mode
  Write( PWR_MGMT_1, 0b00000000 );

  return true;
}

TMpu6000::TMpuData TMpu6000::GetData()
{
  TScopedLow ScopedLow( PortNSS, PinNSS );

  TMpuData MpuData;
  Spi.Write( ACCEL_XOUT | CMD_READ );
  Spi.ReadSwap16( &MpuData, sizeof( MpuData ));

  MpuData.Temp = ( MpuData.Temp / 340 ) + 36;

  return MpuData;
}

bool TMpu6000::Selftest()
{
  return true;
}

uint8_t TMpu6000::Read( uint8_t const Command ) const
{
  TScopedLow ScopedLow( PortNSS, PinNSS );

  Spi.Write( Command | CMD_READ );
  return Spi.Read();
}

void TMpu6000::Write( uint8_t const Command, uint8_t const Data ) const
{
  TScopedLow ScopedLow( PortNSS, PinNSS );

  Spi.Write( Command & CMD_WRITE );
  Spi.Write( Data );
}
