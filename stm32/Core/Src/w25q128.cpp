#include "w25q128.h"


TW25q128::TW25q128( TSpi &Spi, GPIO_TypeDef *const PortNSS, uint32_t const PinNSS ) :
  Spi( Spi ),
  PinNSS( PinNSS ),
  PortNSS( PortNSS )
{
}

bool TW25q128::Setup()
{
  Write( 0xab );
  LL_mDelay( 2 );

  uint8_t Buffer[ 2 ];
  Read( 0x90, 0x000000, Buffer, sizeof( Buffer ));
  if(( Buffer[ 0 ] != 0xef ) || ( Buffer[ 1 ] != 0x17 ))
  {
    return false;
  }

  return true;
}

void TW25q128::Read( uint8_t const Code, uint32_t Addr, uint8_t *const Buffer, uint32_t const Length ) const
{
  TScopedLow ScopedLow( PortNSS, PinNSS );

  Spi.Write( Code );
  Spi.Write( Addr >> 16 );
  Spi.Write( Addr >> 8 );
  Spi.Write( Addr );
  Spi.Read( Buffer, Length );
}

void TW25q128::Write( uint8_t const Code ) const
{
  TScopedLow ScopedLow( PortNSS, PinNSS );

  Spi.Write( Code );
}

void TW25q128::Write( uint8_t const Code, uint32_t Addr, uint8_t const *const Buffer, uint32_t const Length ) const
{
  TScopedLow ScopedLow( PortNSS, PinNSS );

  Spi.Write( Code );
  Spi.Write( Addr >> 16 );
  Spi.Write( Addr >> 8 );
  Spi.Write( Addr );
  Spi.Write( Buffer, Length );
}
