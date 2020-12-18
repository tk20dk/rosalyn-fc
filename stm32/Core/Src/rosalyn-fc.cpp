#include "rosalyn-fc.h"


static TRosalynFc RosalynFc;

void TRosalynFc::Loop()
{
  Bmp280.SetRunMode();

  LL_GPIO_ResetOutputPin( LED0_GPIO_Port, LED0_Pin );
  HAL_Delay( 100 );
  LL_GPIO_SetOutputPin( LED0_GPIO_Port, LED0_Pin );
  HAL_Delay( 900 );

  auto const Data = Mpu6000.GetData();

  int32_t t = Bmp280.readTemperature();
  uint32_t p = Bmp280.readPressure();
  uint32_t h = Bmp280.readHumidity();

  UsbPrintf( "%6d %6d %6d %6d %6d %6d %6d T:%d P:%u H:%u\n",
    Data.Temp,
    Data.GyroX,
    Data.GyroY,
    Data.GyroZ,
    Data.AccelX,
    Data.AccelY,
    Data.AccelZ, t, p, h );
}

void TRosalynFc::Setup()
{
  HAL_Delay( 1000 );

  Spi1.Setup();
  Spi3.Setup();

  if( Bmp280.Setup() == false )
  {
    UsbPrintf( "BMP280 not found\n" );
  }

  if( Mpu6000.Setup() == false )
  {
    UsbPrintf( "MPU6000 not found\n" );
  }

  if( W25q128.Setup() == false )
  {
    UsbPrintf( "W25Q128 not found\n" );
  }
}

TRosalynFc::TRosalynFc() :
  Spi1( SPI1 ),
  Spi3( SPI3 ),
  Bmp280( Spi3, BARO_NSS_GPIO_Port, BARO_NSS_Pin ),
  Mpu6000( Spi1, GYRO1_NSS_GPIO_Port, GYRO1_NSS_Pin ),
  W25q128( Spi3, FLASH_CS_GPIO_Port, FLASH_CS_Pin )
{
}

extern "C" void RosalynFcLoop()
{
  RosalynFc.Loop();
}

extern "C" void RosalynFcSetup()
{
  RosalynFc.Setup();
}
