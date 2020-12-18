#ifndef W25Q128_H__
#define W25Q128_H__

#include "spi.h"
#include "system.h"


class TW25q128
{
public:
  TW25q128( TSpi &Spi, GPIO_TypeDef *const PortNSS, uint32_t const PinNSS );

  bool Setup();

private:
  void Read( uint8_t const Code, uint32_t Addr, uint8_t *const Buffer, uint32_t const Length ) const;
  void Write( uint8_t const Code ) const;
  void Write( uint8_t const Code, uint32_t Addr, uint8_t const *const Buffer, uint32_t const Length ) const;

private:
  TSpi &Spi;
  uint32_t const PinNSS;
  GPIO_TypeDef *const PortNSS;
};


#endif // W25Q128_H__
