#ifndef SYSTEM_H__
#define SYSTEM_H__

#include "main.h"


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

void UsbPrintf( char const *const Format, ... );

#endif // SYSTEM_H__
