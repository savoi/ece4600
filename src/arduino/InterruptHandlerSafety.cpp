#include "InterruptHandlerSafety.h"

/*inline void* enterISR()
{
  void* linkReg;
  asm volatile
  (
     "   push      {r4-r11}      \n"
     "	 mov       %[out], lr	 \n"
     : [out] "=r" (linkReg)
  ); 
  return linkReg;
}

inline void exitISR(void* linkReg)
{
  asm volatile
  (
    "	mov      lr, %[in]       \n"
    "   pop      {r4-r11}        \n"
    :
    : [in] "r" (linkReg)
  );
}*/


