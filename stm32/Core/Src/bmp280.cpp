#include "bmp280.h"


TBmp280::TBmp280( TSpi &Spi, GPIO_TypeDef *const PortNSS, uint32_t const PinNSS ) :
  Spi( Spi ),
  PinNSS( PinNSS ),
  PortNSS( PortNSS ),
  Temperature( 0U ),
  Pressure( 0U ),
  Humidity( 0U ),
  t_fine( 0 ),
  settings(),
  T1( 0U ),
  T2( 0 ),
  T3( 0 ),
  P1( 0U ),
  P2( 0 ),
  P3( 0 ),
  P4( 0 ),
  P5( 0 ),
  P6( 0 ),
  P7( 0 ),
  P8( 0 ),
  P9( 0 ),
  H1( 0U ),
  H2( 0 ),
  H3( 0U ),
  H4( 0 ),
  H5( 0 ),
  H6( 0U )
{
}

bool TBmp280::Setup()
{
  auto const Id = Read( BME280_CHIP_ID_REG );
  if(( Id != 0x58 ) && ( Id != 0x60 ))
  {
    return false;
  }

  settings.runMode = 1;
  settings.tempOverSample = 5; // x16
  settings.pressOverSample = 5; // x16
  settings.humidOverSample = 5; // x16

  uint8_t Buffer[ 26U ];
  Read( BME280_DIG_T1_LSB_REG, Buffer, sizeof( Buffer ));

  T1 = ((Buffer[ 1 ] * 256 ) + Buffer[ 0 ] );
  T2 = ((Buffer[ 3 ] * 256 ) + Buffer[ 2 ] );
  T3 = ((Buffer[ 5 ] * 256 ) + Buffer[ 4 ] );

  P1 = ((Buffer[ 7 ] * 256 ) + Buffer[ 6 ] );
  P2 = ((Buffer[ 9 ] * 256 ) + Buffer[ 8 ] );
  P3 = ((Buffer[ 11 ] * 256 ) + Buffer[ 10 ] );
  P4 = ((Buffer[ 13 ] * 256 ) + Buffer[ 12 ] );
  P5 = ((Buffer[ 15 ] * 256 ) + Buffer[ 14 ] );
  P6 = ((Buffer[ 17 ] * 256 ) + Buffer[ 16 ] );
  P7 = ((Buffer[ 19 ] * 256 ) + Buffer[ 18 ] );
  P8 = ((Buffer[ 21 ] * 256 ) + Buffer[ 20 ] );
  P9 = ((Buffer[ 23 ] * 256 ) + Buffer[ 22 ] );

  H1 = Buffer[ 25 ];

  Read( BME280_DIG_H2_LSB_REG, Buffer, 7 );

  H2 = ((Buffer[ 1 ] * 256 ) + Buffer[ 0 ] );
  H3 = Buffer[ 2 ];
  H4 = (( Buffer[ 3 ] << 4 ) + (Buffer[ 4 ] & 0x0F ));
  H5 = (( Buffer[ 5 ] << 4 ) + ( Buffer[ 4 ] >> 4 ));
  H6 = Buffer[ 6 ];

  return true;
}

void TBmp280::SetRunMode()
{
  //Set the oversampling control words.
  //config will only be writeable in sleep mode, so first insure that.
  Write( BME280_CTRL_MEAS_REG, 0x00 );

  //Set the config word
  uint8_t dataToWrite = (settings.tStandby << 0x5) & 0xE0;
  dataToWrite |= (settings.filter << 0x02) & 0x1C;
  Write( BME280_CONFIG_REG, dataToWrite );

  //Set ctrl_hum first, then ctrl_meas to activate ctrl_hum
  dataToWrite = settings.humidOverSample & 0x07; //all other bits can be ignored
  Write( BME280_CTRL_HUMIDITY_REG, dataToWrite );

  //set ctrl_meas
  //First, set temp oversampling
  dataToWrite = (settings.tempOverSample << 0x5) & 0xE0;
  //Next, pressure oversampling
  dataToWrite |= (settings.pressOverSample << 0x02) & 0x1C;
  //Last, set mode
  dataToWrite |= (settings.runMode) & 0x03;
  Write( BME280_CTRL_MEAS_REG, dataToWrite );
}

// Returns pressure in Pa as unsigned 32 bit integer in Q24.8 format (24 integer bits and 8 fractional bits).
// Output value of “24674867” represents 24674867/256 = 96386.2 Pa = 963.862 hPa
uint32_t TBmp280::readPressure()
{
  uint8_t Buffer[ 3 ];
  Read( BME280_PRESSURE_MSB_REG, Buffer, sizeof( Buffer ));

  int32_t const adc_P = ( Buffer[ 0 ] << 12 ) | ( Buffer[ 1 ] << 4 ) | ( Buffer[ 2 ] >> 4 );

  int64_t var1 = ((int64_t)t_fine) - 128000;
  int64_t var2 = var1 * var1 * (int64_t)P6;
  var2 = var2 + ((var1 * (int64_t)P5)<<17);
  var2 = var2 + (((int64_t)P4)<<35);
  var1 = ((var1 * var1 * (int64_t)P3)>>8) + ((var1 * (int64_t)P2)<<12);
  var1 = (((((int64_t)1)<<47)+var1))*((int64_t)P1)>>33;
  if (var1 == 0)
  {
    return 0U; // avoid exception caused by division by zero
  }
  int64_t p_acc = 1048576 - adc_P;
  p_acc = (((p_acc<<31) - var2)*3125)/var1;
  var1 = (((int64_t)P9) * (p_acc>>13) * (p_acc>>13)) >> 25;
  var2 = (((int64_t)P8) * p_acc) >> 19;
  p_acc = ((p_acc + var1 + var2) >> 8) + (((int64_t)P7)<<4);

  p_acc = p_acc >> 8;
  return p_acc / 10U;
}

// Returns humidity in %RH as unsigned 32 bit integer in Q22. 10 format (22 integer and 10 fractional bits).
// Output value of “47445” represents 47445/1024 = 46. 333 %RH
uint32_t TBmp280::readHumidity()
{
  uint8_t Buffer[ 2 ];
  Read( BME280_HUMIDITY_MSB_REG, Buffer, sizeof( Buffer ));

  int32_t const adc_H = ( Buffer[ 0 ] << 8 ) | Buffer[ 1 ];

  int32_t var1 = (t_fine - ((int32_t)76800));
  var1 = (((((adc_H << 14) - (((int32_t)H4) << 20) - (((int32_t)H5) * var1)) +
  ((int32_t)16384)) >> 15) * (((((((var1 * ((int32_t)H6)) >> 10) * (((var1 * ((int32_t)H3)) >> 11) + ((int32_t)32768))) >> 10) + ((int32_t)2097152)) * ((int32_t)H2) + 8192) >> 14));
  var1 = (var1 - (((((var1 >> 15) * (var1 >> 15)) >> 7) * ((int32_t)H1)) >> 4));
  var1 = (var1 < 0 ? 0 : var1);
  var1 = (var1 > 419430400 ? 419430400 : var1);

  return ((( var1 >> 12 ) * 10 ) >> 10);
}

// Returns temperature in DegC, resolution is 0.01 DegC. Output value of “5123” equals 51.23 DegC.
// t_fine carries fine temperature as global value
int32_t TBmp280::readTemperature()
{
  uint8_t Buffer[ 3 ];
  Read( BME280_TEMPERATURE_MSB_REG, Buffer, sizeof( Buffer ));

  int32_t const adc_T = ( Buffer[ 0 ] << 12) | ( Buffer[ 1 ] << 4 ) | ( Buffer[ 2 ] >> 4 );

  //By datasheet, calibrate
  int64_t var1 = ((((adc_T>>3) - ((int32_t)T1<<1))) * ((int32_t)T2)) >> 11;
  int64_t var2 = (((((adc_T>>4) - ((int32_t)T1)) * ((adc_T>>4) - ((int32_t)T1))) >> 12) *
	((int32_t)T3)) >> 14;
  t_fine = var1 + var2;

  return (( t_fine * 5 + 128 ) >> 8 ) / 10;
}

uint8_t TBmp280::Read( uint8_t const Command ) const
{
  TScopedLow ScopedLow( PortNSS, PinNSS );

  Spi.Write( Command | CMD_READ );
  return Spi.Read();
}

void TBmp280::Write( uint8_t const Command, uint8_t const Data ) const
{
  TScopedLow ScopedLow( PortNSS, PinNSS );

  Spi.Write( Command & CMD_WRITE );
  Spi.Write( Data );
}

void TBmp280::Read( uint8_t const Command, uint8_t *const Buffer, uint32_t const Length ) const
{
  TScopedLow ScopedLow( PortNSS, PinNSS );

  Spi.Write( Command | CMD_READ );
  Spi.Read( Buffer, Length );
}
