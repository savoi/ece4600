
#ifndef __INTERRUPT_HANDLER_SAFETY__
#define __INTERRUPT_HANDLER_SAFETY__

//inline void* enterInterrupt();
//inline void exitInterrupt(void* linkReg);

//inline to save time in the ISR in which it will be called

//these must be fully implemented in the .h file to avoid linker errors
//inline functions must be fully defined in every cpp which calls them

inline void* enterISR()
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
}


#endif

