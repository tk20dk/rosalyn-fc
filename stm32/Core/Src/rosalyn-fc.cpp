#include "rosalyn-fc.h"


TRosalynFc RosalynFc;

void TRosalynFc::Loop()
{
  LL_GPIO_ResetOutputPin( LED0_GPIO_Port, LED0_Pin );
  HAL_Delay( 100 );
  LL_GPIO_SetOutputPin( LED0_GPIO_Port, LED0_Pin );
  HAL_Delay( 900 );
}

void TRosalynFc::Setup()
{
  Spi1.Setup();

  if( Mpu6000.Setup() )
  {
    LL_GPIO_ResetOutputPin( LED0_GPIO_Port, LED0_Pin );
  }
}

TRosalynFc::TRosalynFc() :
  Spi1( SPI1 ),
  Mpu6000( Spi1, GYRO1_NSS_GPIO_Port, GYRO1_NSS_Pin )
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
