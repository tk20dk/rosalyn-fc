#ifndef ROSALYN_FC_H__
#define ROSALYN_FC_H__

#include "main.h"
#include "spi.h"
#include "system.h"
#include "bmp280.h"
#include "mpu6000.h"


class TRosalynFc
{
public:
  TRosalynFc();

  void Setup();
  void Loop();

private:
  TSpi Spi1;
  TSpi Spi3;
  TBmp280 Bmp280;
  TMpu6000 Mpu6000;
};


#endif // ROSALYN_FC_H__
