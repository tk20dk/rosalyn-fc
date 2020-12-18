#include <cstdio>
#include <cstdlib>
#include <cstdarg>
#include "system.h"
#include "usbd_cdc_if.h"


void UsbPrintf( char const *const Format, ... )
{
  static int Length;
  static char Buffer[ 128 ];
  va_list Args;

  va_start( Args, Format );
  Length += vsnprintf( &Buffer[ Length ], sizeof( Buffer ) - Length, Format, Args );
  va_end( Args );

  if( Length > 0 )
  {
    auto Status = CDC_Transmit_FS( reinterpret_cast<uint8_t*>( Buffer ), static_cast<uint16_t>( Length ));
    if( Status != USBD_BUSY )
    {
      Length = 0;
    }
  }
}
