
/*
	STM32F4XX arm-none-eabi-g++ Project Template (ROM load)
	Startup script and vector table
	(C) 2023 Kumohakase
	You are free to destribute, modify without modifying this section.
	Please consider support me on kofi.com https://ko-fi.com/kumohakase
*/

#define HANDLER_ATTRS interrupt, weak //attribute for interrupt functions
#include <stm32f4xx.h>

typedef void (*inthwnd_t)(); //type of interrupt handler pointer
extern uint32_t __stack; //stack top from flash.ld
void Reset_Handler();

__attribute((HANDLER_ATTRS)) void HardFault_Handler();
/******  Cortex-M4 Processor Exceptions Numbers ****************************************************************/
__attribute((HANDLER_ATTRS)) void NonMaskableInt_Handler();    /*!< 2 Non Maskable Interrupt                                          */
__attribute((HANDLER_ATTRS)) void MemoryManagement_Handler();    /*!< 4 Cortex-M4 Memory Management Interrupt                           */
__attribute((HANDLER_ATTRS)) void BusFault_Handler();    /*!< 5 Cortex-M4 Bus Fault Interrupt                                   */
__attribute((HANDLER_ATTRS)) void UsageFault_Handler();    /*!< 6 Cortex-M4 Usage Fault Interrupt                                 */
__attribute((HANDLER_ATTRS)) void SVCall_Handler();     /*!< 11 Cortex-M4 SV Call Interrupt                                    */
__attribute((HANDLER_ATTRS)) void DebugMonitor_Handler();     /*!< 12 Cortex-M4 Debug Monitor Interrupt                              */
__attribute((HANDLER_ATTRS)) void PendSV_Handler();     /*!< 14 Cortex-M4 Pend SV Interrupt                                    */
__attribute((HANDLER_ATTRS)) void SysTick_Handler();     /*!< 15 Cortex-M4 System Tick Interrupt                                */
/******  STM32 specific Interrupt Numbers **********************************************************************/
__attribute((HANDLER_ATTRS)) void WWDG_Handler();      /*!< Window WatchDog Interrupt                                         */
__attribute((HANDLER_ATTRS)) void PVD_Handler();      /*!< PVD through EXTI Line detection Interrupt                         */
__attribute((HANDLER_ATTRS)) void TAMP_STAMP_Handler();      /*!< Tamper and TimeStamp interrupts through the EXTI line             */
__attribute((HANDLER_ATTRS)) void RTC_WKUP_Handler();      /*!< RTC Wakeup interrupt through the EXTI line                        */
__attribute((HANDLER_ATTRS)) void FLASH_Handler();      /*!< FLASH global Interrupt                                            */
__attribute((HANDLER_ATTRS)) void RCC_Handler();      /*!< RCC global Interrupt                                              */
__attribute((HANDLER_ATTRS)) void EXTI0_Handler();      /*!< EXTI Line0 Interrupt                                              */
__attribute((HANDLER_ATTRS)) void EXTI1_Handler();      /*!< EXTI Line1 Interrupt                                              */
__attribute((HANDLER_ATTRS)) void EXTI2_Handler();      /*!< EXTI Line2 Interrupt                                              */
__attribute((HANDLER_ATTRS)) void EXTI3_Handler();      /*!< EXTI Line3 Interrupt                                              */
__attribute((HANDLER_ATTRS)) void EXTI4_Handler();     /*!< EXTI Line4 Interrupt                                              */
__attribute((HANDLER_ATTRS)) void DMA1_Stream0_Handler();     /*!< DMA1 Stream 0 global Interrupt                                    */
__attribute((HANDLER_ATTRS)) void DMA1_Stream1_Handler();     /*!< DMA1 Stream 1 global Interrupt                                    */
__attribute((HANDLER_ATTRS)) void DMA1_Stream2_Handler();     /*!< DMA1 Stream 2 global Interrupt                                    */
__attribute((HANDLER_ATTRS)) void DMA1_Stream3_Handler();     /*!< DMA1 Stream 3 global Interrupt                                    */
__attribute((HANDLER_ATTRS)) void DMA1_Stream4_Handler();     /*!< DMA1 Stream 4 global Interrupt                                    */
__attribute((HANDLER_ATTRS)) void DMA1_Stream5_Handler();     /*!< DMA1 Stream 5 global Interrupt                                    */
__attribute((HANDLER_ATTRS)) void DMA1_Stream6_Handler();     /*!< DMA1 Stream 6 global Interrupt                                    */
__attribute((HANDLER_ATTRS)) void ADC_Handler();     /*!< ADC1, ADC2 and ADC3 global Interrupts                             */
__attribute((HANDLER_ATTRS)) void CAN1_TX_Handler();     /*!< CAN1 TX Interrupt                                                 */
__attribute((HANDLER_ATTRS)) void CAN1_RX0_Handler();     /*!< CAN1 RX0 Interrupt                                                */
__attribute((HANDLER_ATTRS)) void CAN1_RX1_Handler();     /*!< CAN1 RX1 Interrupt                                                */
__attribute((HANDLER_ATTRS)) void CAN1_SCE_Handler();     /*!< CAN1 SCE Interrupt                                                */
__attribute((HANDLER_ATTRS)) void EXTI9_5_Handler();     /*!< External Line[9:5] Interrupts                                     */
__attribute((HANDLER_ATTRS)) void TIM1_BRK_TIM9_Handler();     /*!< TIM1 Break interrupt and TIM9 global interrupt                    */
__attribute((HANDLER_ATTRS)) void TIM1_UP_TIM10_Handler();     /*!< TIM1 Update Interrupt and TIM10 global interrupt                  */
__attribute((HANDLER_ATTRS)) void TIM1_TRG_COM_TIM11_Handler();     /*!< TIM1 Trigger and Commutation Interrupt and TIM11 global interrupt */
__attribute((HANDLER_ATTRS)) void TIM1_CC_Handler();     /*!< TIM1 Capture Compare Interrupt                                    */
__attribute((HANDLER_ATTRS)) void TIM2_Handler();     /*!< TIM2 global Interrupt                                             */
__attribute((HANDLER_ATTRS)) void TIM3_Handler();     /*!< TIM3 global Interrupt                                             */
__attribute((HANDLER_ATTRS)) void TIM4_Handler();     /*!< TIM4 global Interrupt                                             */
__attribute((HANDLER_ATTRS)) void I2C1_EV_Handler();     /*!< I2C1 Event Interrupt                                              */
__attribute((HANDLER_ATTRS)) void I2C1_ER_Handler();     /*!< I2C1 Error Interrupt                                              */
__attribute((HANDLER_ATTRS)) void I2C2_EV_Handler();     /*!< I2C2 Event Interrupt                                              */
__attribute((HANDLER_ATTRS)) void I2C2_ER_Handler();     /*!< I2C2 Error Interrupt                                              */  
__attribute((HANDLER_ATTRS)) void SPI1_Handler();     /*!< SPI1 global Interrupt                                             */
__attribute((HANDLER_ATTRS)) void SPI2_Handler();     /*!< SPI2 global Interrupt                                             */
__attribute((HANDLER_ATTRS)) void USART1_Handler();     /*!< USART1 global Interrupt                                           */
__attribute((HANDLER_ATTRS)) void USART2_Handler();     /*!< USART2 global Interrupt                                           */
__attribute((HANDLER_ATTRS)) void USART3_Handler();     /*!< USART3 global Interrupt                                           */
__attribute((HANDLER_ATTRS)) void EXTI15_10_Handler();     /*!< External Line[15:10] Interrupts                                   */
__attribute((HANDLER_ATTRS)) void RTC_Alarm_Handler();     /*!< RTC Alarm (A and B) through EXTI Line Interrupt                   */
__attribute((HANDLER_ATTRS)) void OTG_FS_WKUP_Handler();     /*!< USB OTG FS Wakeup through EXTI line interrupt                     */    
__attribute((HANDLER_ATTRS)) void TIM8_BRK_TIM12_Handler();     /*!< TIM8 Break Interrupt and TIM12 global interrupt                   */
__attribute((HANDLER_ATTRS)) void TIM8_UP_TIM13_Handler();     /*!< TIM8 Update Interrupt and TIM13 global interrupt                  */
__attribute((HANDLER_ATTRS)) void TIM8_TRG_COM_TIM14_Handler();     /*!< TIM8 Trigger and Commutation Interrupt and TIM14 global interrupt */
__attribute((HANDLER_ATTRS)) void TIM8_CC_Handler();     /*!< TIM8 Capture Compare Interrupt                                    */
__attribute((HANDLER_ATTRS)) void DMA1_Stream7_Handler();     /*!< DMA1 Stream7 Interrupt                                            */

#if defined (STM32F40XX) || defined (STM32F427X)
__attribute((HANDLER_ATTRS)) void FSMC_Handler();     /*!< FSMC global Interrupt                                             */
#endif /* STM32F40XX || STM32F427X */

#if defined(STM32F429X)
__attribute((HANDLER_ATTRS)) void FMC_Handler();     /*!< FMC global Interrupt                                              */
#endif /* STM32F429X */ 

__attribute((HANDLER_ATTRS)) void SDIO_Handler();     /*!< SDIO global Interrupt                                             */
__attribute((HANDLER_ATTRS)) void TIM5_Handler();     /*!< TIM5 global Interrupt                                             */
__attribute((HANDLER_ATTRS)) void SPI3_Handler();     /*!< SPI3 global Interrupt                                             */
__attribute((HANDLER_ATTRS)) void UART4_Handler();     /*!< UART4 global Interrupt                                            */
__attribute((HANDLER_ATTRS)) void UART5_Handler();     /*!< UART5 global Interrupt                                            */
__attribute((HANDLER_ATTRS)) void TIM6_DAC_Handler();     /*!< TIM6 global and DAC1&2 underrun error  interrupts                 */
__attribute((HANDLER_ATTRS)) void TIM7_Handler();     /*!< TIM7 global interrupt                                             */
__attribute((HANDLER_ATTRS)) void DMA2_Stream0_Handler();     /*!< DMA2 Stream 0 global Interrupt                                    */
__attribute((HANDLER_ATTRS)) void DMA2_Stream1_Handler();     /*!< DMA2 Stream 1 global Interrupt                                    */
__attribute((HANDLER_ATTRS)) void DMA2_Stream2_Handler();     /*!< DMA2 Stream 2 global Interrupt                                    */
__attribute((HANDLER_ATTRS)) void DMA2_Stream3_Handler();     /*!< DMA2 Stream 3 global Interrupt                                    */
__attribute((HANDLER_ATTRS)) void DMA2_Stream4_Handler();     /*!< DMA2 Stream 4 global Interrupt                                    */
__attribute((HANDLER_ATTRS)) void ETH_Handler();     /*!< Ethernet global Interrupt                                         */
__attribute((HANDLER_ATTRS)) void ETH_WKUP_Handler();     /*!< Ethernet Wakeup through EXTI line Interrupt                       */
__attribute((HANDLER_ATTRS)) void CAN2_TX_Handler();     /*!< CAN2 TX Interrupt                                                 */
__attribute((HANDLER_ATTRS)) void CAN2_RX0_Handler();     /*!< CAN2 RX0 Interrupt                                                */
__attribute((HANDLER_ATTRS)) void CAN2_RX1_Handler();     /*!< CAN2 RX1 Interrupt                                                */
__attribute((HANDLER_ATTRS)) void CAN2_SCE_Handler();     /*!< CAN2 SCE Interrupt                                                */
__attribute((HANDLER_ATTRS)) void OTG_FS_Handler();     /*!< USB OTG FS global Interrupt                                       */
__attribute((HANDLER_ATTRS)) void DMA2_Stream5_Handler();     /*!< DMA2 Stream 5 global interrupt                                    */
__attribute((HANDLER_ATTRS)) void DMA2_Stream6_Handler();     /*!< DMA2 Stream 6 global interrupt                                    */
__attribute((HANDLER_ATTRS)) void DMA2_Stream7_Handler();     /*!< DMA2 Stream 7 global interrupt                                    */
__attribute((HANDLER_ATTRS)) void USART6_Handler();     /*!< USART6 global interrupt                                           */
__attribute((HANDLER_ATTRS)) void I2C3_EV_Handler();     /*!< I2C3 event interrupt                                              */
__attribute((HANDLER_ATTRS)) void I2C3_ER_Handler();     /*!< I2C3 error interrupt                                              */
__attribute((HANDLER_ATTRS)) void OTG_HS_EP1_OUT_Handler();     /*!< USB OTG HS End Point 1 Out global interrupt                       */
__attribute((HANDLER_ATTRS)) void OTG_HS_EP1_IN_Handler();     /*!< USB OTG HS End Point 1 In global interrupt                        */
__attribute((HANDLER_ATTRS)) void OTG_HS_WKUP_Handler();     /*!< USB OTG HS Wakeup through EXTI interrupt                          */
__attribute((HANDLER_ATTRS)) void OTG_HS_Handler();     /*!< USB OTG HS global interrupt                                       */
__attribute((HANDLER_ATTRS)) void DCMI_Handler();     /*!< DCMI global interrupt                                             */
__attribute((HANDLER_ATTRS)) void CRYP_Handler();     /*!< CRYP crypto global interrupt                                      */
__attribute((HANDLER_ATTRS)) void HASH_RNG_Handler();     /*!< Hash and Rng global interrupt                                     */

#if defined(STM32F40XX)
__attribute((HANDLER_ATTRS)) void FPU_Handler();     /*!< FPU global interrupt                                              */
#endif /* STM32F40XX */

#if defined (STM32F427X) 
__attribute((HANDLER_ATTRS)) void FPU_Handler();     /*!< FPU global interrupt                                              */
__attribute((HANDLER_ATTRS)) void UART7_Handler();     /*!< UART7 global interrupt                                            */
__attribute((HANDLER_ATTRS)) void UART8_Handler();     /*!< UART8 global interrupt                                            */
__attribute((HANDLER_ATTRS)) void SPI4_Handler();     /*!< SPI4 global Interrupt                                             */
__attribute((HANDLER_ATTRS)) void SPI5_Handler();     /*!< SPI5 global Interrupt                                             */
__attribute((HANDLER_ATTRS)) void SPI6_Handler();     /*!< SPI6 global Interrupt                                             */
#endif /* STM32F427X */

#if defined (STM32F429X)
__attribute((HANDLER_ATTRS)) void FPU_Handler();     /*!< FPU global interrupt                                              */
__attribute((HANDLER_ATTRS)) void UART7_Handler();     /*!< UART7 global interrupt                                            */
__attribute((HANDLER_ATTRS)) void UART8_Handler();     /*!< UART8 global interrupt                                            */
__attribute((HANDLER_ATTRS)) void SPI4_Handler();     /*!< SPI4 global Interrupt                                             */
__attribute((HANDLER_ATTRS)) void SPI5_Handler();     /*!< SPI5 global Interrupt                                             */
__attribute((HANDLER_ATTRS)) void SPI6_Handler();     /*!< SPI6 global Interrupt                                             */
__attribute((HANDLER_ATTRS)) void SAI1_Handler();     /*!< SAI1 global Interrupt                                             */
__attribute((HANDLER_ATTRS)) void LTDC_Handler();     /*!< LTDC global Interrupt                                              */
__attribute((HANDLER_ATTRS)) void LTDC_ER_Handler();     /*!< LTDC Error global Interrupt                                        */
__attribute((HANDLER_ATTRS)) void DMA2D_Handler();     /*!< DMA2D global Interrupt                                            */
#endif /* STM32F429X */

//Vector table
__attribute((section(".vects"))) const inthwnd_t VectorTable[] = {
	/******  Cortex-M4 Processor Exceptions Numbers ****************************************************************/
	(inthwnd_t)&__stack, //stack top addr on Vector addr = 0
	Reset_Handler, //Entry point
	NonMaskableInt_Handler,    /*!< 2 Non Maskable Interrupt                                          */
	HardFault_Handler,
	MemoryManagement_Handler,    /*!< 4 Cortex-M4 Memory Management Interrupt                           */
	BusFault_Handler,    /*!< 5 Cortex-M4 Bus Fault Interrupt                                   */
	UsageFault_Handler,    /*!< 6 Cortex-M4 Usage Fault Interrupt                                 */
	0,0,0,0,
	SVCall_Handler,     /*!< 11 Cortex-M4 SV Call Interrupt                                    */
	DebugMonitor_Handler,     /*!< 12 Cortex-M4 Debug Monitor Interrupt                              */
	0,
	PendSV_Handler,     /*!< 14 Cortex-M4 Pend SV Interrupt                                    */
	SysTick_Handler,     /*!< 15 Cortex-M4 System Tick Interrupt                                */
/******  STM32 specific Interrupt Numbers **********************************************************************/
	WWDG_Handler,      /*!< Window WatchDog Interrupt                                         */
	PVD_Handler,      /*!< PVD through EXTI Line detection Interrupt                         */
	TAMP_STAMP_Handler,      /*!< Tamper and TimeStamp interrupts through the EXTI line             */
	RTC_WKUP_Handler,      /*!< RTC Wakeup interrupt through the EXTI line                        */
	FLASH_Handler,      /*!< FLASH global Interrupt                                            */
	RCC_Handler,      /*!< RCC global Interrupt                                              */
	EXTI0_Handler,      /*!< EXTI Line0 Interrupt                                              */
	EXTI1_Handler,      /*!< EXTI Line1 Interrupt                                              */
	EXTI2_Handler,      /*!< EXTI Line2 Interrupt                                              */
	EXTI3_Handler,      /*!< EXTI Line3 Interrupt                                              */
	EXTI4_Handler,     /*!< EXTI Line4 Interrupt                                              */
	DMA1_Stream0_Handler,     /*!< DMA1 Stream 0 global Interrupt                                    */
	DMA1_Stream1_Handler,     /*!< DMA1 Stream 1 global Interrupt                                    */
	DMA1_Stream2_Handler,     /*!< DMA1 Stream 2 global Interrupt                                    */
	DMA1_Stream3_Handler,     /*!< DMA1 Stream 3 global Interrupt                                    */
	DMA1_Stream4_Handler,     /*!< DMA1 Stream 4 global Interrupt                                    */
	DMA1_Stream5_Handler,     /*!< DMA1 Stream 5 global Interrupt                                    */
	DMA1_Stream6_Handler,     /*!< DMA1 Stream 6 global Interrupt                                    */
	ADC_Handler,     /*!< ADC1, ADC2 and ADC3 global Interrupts                             */
	CAN1_TX_Handler,     /*!< CAN1 TX Interrupt                                                 */
	CAN1_RX0_Handler,     /*!< CAN1 RX0 Interrupt                                                */
	CAN1_RX1_Handler,     /*!< CAN1 RX1 Interrupt                                                */
	CAN1_SCE_Handler,     /*!< CAN1 SCE Interrupt                                                */
	EXTI9_5_Handler,     /*!< External Line[9:5] Interrupts                                     */
	TIM1_BRK_TIM9_Handler,     /*!< TIM1 Break interrupt and TIM9 global interrupt                    */
	TIM1_UP_TIM10_Handler,     /*!< TIM1 Update Interrupt and TIM10 global interrupt                  */
	TIM1_TRG_COM_TIM11_Handler,     /*!< TIM1 Trigger and Commutation Interrupt and TIM11 global interrupt */
	TIM1_CC_Handler,     /*!< TIM1 Capture Compare Interrupt                                    */
	TIM2_Handler,     /*!< TIM2 global Interrupt                                             */
	TIM3_Handler,     /*!< TIM3 global Interrupt                                             */
	TIM4_Handler,     /*!< TIM4 global Interrupt                                             */
	I2C1_EV_Handler,     /*!< I2C1 Event Interrupt                                              */
	I2C1_ER_Handler,     /*!< I2C1 Error Interrupt                                              */
	I2C2_EV_Handler,     /*!< I2C2 Event Interrupt                                              */
	I2C2_ER_Handler,     /*!< I2C2 Error Interrupt                                              */  
	SPI1_Handler,     /*!< SPI1 global Interrupt                                             */
	SPI2_Handler,     /*!< SPI2 global Interrupt                                             */
	USART1_Handler,     /*!< USART1 global Interrupt                                           */
	USART2_Handler,     /*!< USART2 global Interrupt                                           */
	USART3_Handler,     /*!< USART3 global Interrupt                                           */
	EXTI15_10_Handler,     /*!< External Line[15:10] Interrupts                                   */
	RTC_Alarm_Handler,     /*!< RTC Alarm (A and B) through EXTI Line Interrupt                   */
	OTG_FS_WKUP_Handler,     /*!< USB OTG FS Wakeup through EXTI line interrupt                     */    
	TIM8_BRK_TIM12_Handler,     /*!< TIM8 Break Interrupt and TIM12 global interrupt                   */
	TIM8_UP_TIM13_Handler,     /*!< TIM8 Update Interrupt and TIM13 global interrupt                  */
	TIM8_TRG_COM_TIM14_Handler,     /*!< TIM8 Trigger and Commutation Interrupt and TIM14 global interrupt */
	TIM8_CC_Handler,     /*!< TIM8 Capture Compare Interrupt                                    */
	DMA1_Stream7_Handler,     /*!< DMA1 Stream7 Interrupt                                            */

#if defined (STM32F40XX) || defined (STM32F427X)
	FSMC_Handler,     /*!< FSMC global Interrupt                                             */
#endif /* STM32F40XX || STM32F427X */

#if defined(STM32F429X)
	FMC_Handler,     /*!< FMC global Interrupt                                              */
#endif /* STM32F429X */ 

	SDIO_Handler,     /*!< SDIO global Interrupt                                             */
	TIM5_Handler,     /*!< TIM5 global Interrupt                                             */
	SPI3_Handler,     /*!< SPI3 global Interrupt                                             */
	UART4_Handler,     /*!< UART4 global Interrupt                                            */
	UART5_Handler,     /*!< UART5 global Interrupt                                            */
	TIM6_DAC_Handler,     /*!< TIM6 global and DAC1&2 underrun error  interrupts                 */
	TIM7_Handler,     /*!< TIM7 global interrupt                                             */
	DMA2_Stream0_Handler,     /*!< DMA2 Stream 0 global Interrupt                                    */
	DMA2_Stream1_Handler,     /*!< DMA2 Stream 1 global Interrupt                                    */
	DMA2_Stream2_Handler,     /*!< DMA2 Stream 2 global Interrupt                                    */
	DMA2_Stream3_Handler,     /*!< DMA2 Stream 3 global Interrupt                                    */
	DMA2_Stream4_Handler,     /*!< DMA2 Stream 4 global Interrupt                                    */
	ETH_Handler,     /*!< Ethernet global Interrupt                                         */
	ETH_WKUP_Handler,     /*!< Ethernet Wakeup through EXTI line Interrupt                       */
	CAN2_TX_Handler,     /*!< CAN2 TX Interrupt                                                 */
	CAN2_RX0_Handler,     /*!< CAN2 RX0 Interrupt                                                */
	CAN2_RX1_Handler,     /*!< CAN2 RX1 Interrupt                                                */
	CAN2_SCE_Handler,     /*!< CAN2 SCE Interrupt                                                */
	OTG_FS_Handler,     /*!< USB OTG FS global Interrupt                                       */
	DMA2_Stream5_Handler,     /*!< DMA2 Stream 5 global interrupt                                    */
	DMA2_Stream6_Handler,     /*!< DMA2 Stream 6 global interrupt                                    */
	DMA2_Stream7_Handler,     /*!< DMA2 Stream 7 global interrupt                                    */
	USART6_Handler,     /*!< USART6 global interrupt                                           */
	I2C3_EV_Handler,     /*!< I2C3 event interrupt                                              */
	I2C3_ER_Handler,     /*!< I2C3 error interrupt                                              */
	OTG_HS_EP1_OUT_Handler,     /*!< USB OTG HS End Point 1 Out global interrupt                       */
	OTG_HS_EP1_IN_Handler,     /*!< USB OTG HS End Point 1 In global interrupt                        */
	OTG_HS_WKUP_Handler,     /*!< USB OTG HS Wakeup through EXTI interrupt                          */
	OTG_HS_Handler,     /*!< USB OTG HS global interrupt                                       */
	DCMI_Handler,     /*!< DCMI global interrupt                                             */
	CRYP_Handler,     /*!< CRYP crypto global interrupt                                      */
	HASH_RNG_Handler,     /*!< Hash and Rng global interrupt                                     */

#if defined(STM32F40XX)
	FPU_Handler     /*!< FPU global interrupt                                              */
#endif /* STM32F40XX */

#if defined (STM32F427X) 
	FPU_Handler,     /*!< FPU global interrupt                                              */
	UART7_Handler,     /*!< UART7 global interrupt                                            */
	UART8_Handler,     /*!< UART8 global interrupt                                            */
	SPI4_Handler,     /*!< SPI4 global Interrupt                                             */
	SPI5_Handler,     /*!< SPI5 global Interrupt                                             */
	SPI6_Handler     /*!< SPI6 global Interrupt                                             */
#endif /* STM32F427X */

#if defined (STM32F429X)
	FPU_Handler,     /*!< FPU global interrupt                                              */
	UART7_Handler,     /*!< UART7 global interrupt                                            */
	UART8_Handler,     /*!< UART8 global interrupt                                            */
	SPI4_Handler,     /*!< SPI4 global Interrupt                                             */
	SPI5_Handler,     /*!< SPI5 global Interrupt                                             */
	SPI6_Handler,     /*!< SPI6 global Interrupt                                             */
	SAI1_Handler,     /*!< SAI1 global Interrupt                                             */
	LTDC_Handler,     /*!< LTDC global Interrupt                                              */
	LTDC_ER_Handler,     /*!< LTDC Error global Interrupt                                        */
	DMA2D_Handler     /*!< DMA2D global Interrupt                                            */
#endif /* STM32F429X */
};

//Called on reset, entry point
void Reset_Handler() {
	//Section address boundary info from flash.ld
	extern uint32_t __sbss, __ebss, __sidata, __sdata, __edata;
	extern int main();
	//Copy initial data section in ROM to RAM
	uint8_t *p, *p2, *e;
	p = (uint8_t*)&__sidata; //Start address of initial data
	p2 = (uint8_t*)&__sdata; //Start address of RAM data section
	e = (uint8_t*)&__edata; //End address of RAM data section
	for(uint32_t i = 0; i < (e - p2); i++) {
		p2[i] = p[i];
	}
	//Fill out BSS section with 0
	p = (uint8_t*)&__sbss; //Start address of bss
	e = (uint8_t*)&__ebss; //End address
	for(uint32_t i = 0; i < (e - p); i++) {
		p[i] = 0;
	}
	main();
}
