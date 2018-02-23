#include "va108xx.h"
#include "stdio.h"
// #include "reb_log.h"

// Safe memory access routine, that support possible fault recovery
// Needs to be compiled UnOptimized
#pragma push
#pragma O0
uint32_t ReadMem(uint32_t* addr)
{
  register uint32_t value32 = 0xcccc3333;
  /*
	value32 = *addr;
	__nop();  // NOP fills - For HardFault Address Increment, fix
	__nop();
	__nop();
	__nop();
	__nop();
	__nop();
 */
  return value32;
}
#pragma pop

#if 0 
static int32_t stackArray[32], i = 0; 
#endif 

void C_HardFault_Handler(uint32_t _sp)
{
	return; 
#if 0 
  uint32_t *sp = (uint32_t *)_sp; // StackPointer as a valid pointer value
	VOR_Log(LOG_ERROR,"PC hit hardfault. \n");
	VOR_Log(LOG_ERROR,"Snapshot of registers at failure point. \n"); 
	for(i=0; i<32; i++) {
		stackArray[i] = *sp++; 
		VOR_Log(LOG_ERROR,"%x \t", stackArray[i]); 
	}
#endif 
}
// Push extra values on stack before going to handler
// Keil Highlights this as incorrrect, but it is correct when compiled.
/*__asm void HardFault_Handler(void)
{
  REQUIRE8              ; For API compatability used 8 byte stack frame
  PRESERVE8  
  MOVS    r3, #127	
  PUSH    {r3-r7,lr}    ; Save contents of other registers for display (R3 included to meet REQUIRE8)e
	MOV     R0,SP         ; Pass current SP as parameter
  BL      __cpp(C_HardFault_Handler)
  POP     {r3-r7,pc}    ; Restore all registers
} */ 
