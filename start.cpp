
/*
	STM32F4XX arm-none-eabi-g++ Project Template (ROM load)
	Startup script and vector table
	(C) 2023 Kumohakase
	You are free to destribute, modify without modifying this section.
	Please consider support me on kofi.com https://ko-fi.com/kumohakase
*/

#define ISR __attribute((interrupt, weak)) //Interrupt handler attributes

#include <stdint.h>

//Interrupt handler prototypes
void Reset_Handler();

/******  Cortex-M4 Processor Exceptions Numbers ****************************************************************/
ISR void NonMaskableInt_Handler();    /*!< 2 Non Maskable Interrupt                                          */
ISR void Hardfault_Handler();
ISR void MemoryManagement_Handler();    /*!< 4 Cortex-M4 Memory Management Interrupt                           */
ISR void BusFault_Handler();    /*!< 5 Cortex-M4 Bus Fault Interrupt                                   */
ISR void UsageFault_Handler();    /*!< 6 Cortex-M4 Usage Fault Interrupt                                 */
ISR void SVCall_Handler();     /*!< 11 Cortex-M4 SV Call Interrupt                                    */
ISR void DebugMonitor_Handler();     /*!< 12 Cortex-M4 Debug Monitor Interrupt                              */
ISR void PendSV_Handler();     /*!< 14 Cortex-M4 Pend SV Interrupt                                    */
ISR void SysTick_Handler();     /*!< 15 Cortex-M4 System Tick Interrupt                                */
/******  STM32 specific Interrupt Numbers **********************************************************************/
ISR void WWDG_Handler();      /*!< Window WatchDog Interrupt                                         */
ISR void PVD_Handler();      /*!< PVD through EXTI Line detection Interrupt                         */
ISR void TAMP_STAMP_Handler();      /*!< Tamper and TimeStamp interrupts through the EXTI line             */
ISR void RTC_WKUP_Handler();      /*!< RTC Wakeup interrupt through the EXTI line                        */
ISR void FLASH_Handler();      /*!< FLASH global Interrupt                                            */
ISR void RCC_Handler();      /*!< RCC global Interrupt                                              */
ISR void EXTI0_Handler();      /*!< EXTI Line0 Interrupt                                              */
ISR void EXTI1_Handler();      /*!< EXTI Line1 Interrupt                                              */
ISR void EXTI2_Handler();      /*!< EXTI Line2 Interrupt                                              */
ISR void EXTI3_Handler();      /*!< EXTI Line3 Interrupt                                              */
ISR void EXTI4_Handler();     /*!< EXTI Line4 Interrupt                                              */
ISR void DMA1_Stream0_Handler();     /*!< DMA1 Stream 0 global Interrupt                                    */
ISR void DMA1_Stream1_Handler();     /*!< DMA1 Stream 1 global Interrupt                                    */
ISR void DMA1_Stream2_Handler();     /*!< DMA1 Stream 2 global Interrupt                                    */
ISR void DMA1_Stream3_Handler();     /*!< DMA1 Stream 3 global Interrupt                                    */
ISR void DMA1_Stream4_Handler();     /*!< DMA1 Stream 4 global Interrupt                                    */
ISR void DMA1_Stream5_Handler();     /*!< DMA1 Stream 5 global Interrupt                                    */
ISR void DMA1_Stream6_Handler();     /*!< DMA1 Stream 6 global Interrupt                                    */
ISR void ADC_Handler();     /*!< ADC1, ADC2 and ADC3 global Interrupts                             */
ISR void CAN1_TX_Handler();     /*!< CAN1 TX Interrupt                                                 */
ISR void CAN1_RX0_Handler();     /*!< CAN1 RX0 Interrupt                                                */
ISR void CAN1_RX1_Handler();     /*!< CAN1 RX1 Interrupt                                                */
ISR void CAN1_SCE_Handler();     /*!< CAN1 SCE Interrupt                                                */
ISR void EXTI9_5_Handler();     /*!< External Line[9:5] Interrupts                                     */
ISR void TIM1_BRK_TIM9_Handler();     /*!< TIM1 Break interrupt and TIM9 global interrupt                    */
ISR void TIM1_UP_TIM10_Handler();     /*!< TIM1 Update Interrupt and TIM10 global interrupt                  */
ISR void TIM1_TRG_COM_TIM11_Handler();     /*!< TIM1 Trigger and Commutation Interrupt and TIM11 global interrupt */
ISR void TIM1_CC_Handler();     /*!< TIM1 Capture Compare Interrupt                                    */
ISR void TIM2_Handler();     /*!< TIM2 global Interrupt                                             */
ISR void TIM3_Handler();     /*!< TIM3 global Interrupt                                             */
ISR void TIM4_Handler();     /*!< TIM4 global Interrupt                                             */
ISR void I2C1_EV_Handler();     /*!< I2C1 Event Interrupt                                              */
ISR void I2C1_ER_Handler();     /*!< I2C1 Error Interrupt                                              */
ISR void I2C2_EV_Handler();     /*!< I2C2 Event Interrupt                                              */
ISR void I2C2_ER_Handler();     /*!< I2C2 Error Interrupt                                              */  
ISR void SPI1_Handler();     /*!< SPI1 global Interrupt                                             */
ISR void SPI2_Handler();     /*!< SPI2 global Interrupt                                             */
ISR void USART1_Handler();     /*!< USART1 global Interrupt                                           */
ISR void USART2_Handler();     /*!< USART2 global Interrupt                                           */
ISR void USART3_Handler();     /*!< USART3 global Interrupt                                           */
ISR void EXTI15_10_Handler();     /*!< External Line[15:10] Interrupts                                   */
ISR void RTC_Alarm_Handler();     /*!< RTC Alarm (A and B) through EXTI Line Interrupt                   */
ISR void OTG_FS_WKUP_Handler();     /*!< USB OTG FS Wakeup through EXTI line interrupt                     */    
ISR void TIM8_BRK_TIM12_Handler();     /*!< TIM8 Break Interrupt and TIM12 global interrupt                   */
ISR void TIM8_UP_TIM13_Handler();     /*!< TIM8 Update Interrupt and TIM13 global interrupt                  */
ISR void TIM8_TRG_COM_TIM14_Handler();     /*!< TIM8 Trigger and Commutation Interrupt and TIM14 global interrupt */
ISR void TIM8_CC_Handler();     /*!< TIM8 Capture Compare Interrupt                                    */
ISR void DMA1_Stream7_Handler();     /*!< DMA1 Stream7 Interrupt                                            */

#if defined (STM32F40XX) || defined (STM32F427X)
ISR void FSMC_Handler();     /*!< FSMC global Interrupt                                             */
#endif /* STM32F40XX || STM32F427X */

#if defined(STM32F429X)
ISR void FMC_Handler();     /*!< FMC global Interrupt                                              */
#endif /* STM32F429X */ 

ISR void SDIO_Handler();     /*!< SDIO global Interrupt                                             */
ISR void TIM5_Handler();     /*!< TIM5 global Interrupt                                             */
ISR void SPI3_Handler();     /*!< SPI3 global Interrupt                                             */
ISR void UART4_Handler();     /*!< UART4 global Interrupt                                            */
ISR void UART5_Handler();     /*!< UART5 global Interrupt                                            */
ISR void TIM6_DAC_Handler();     /*!< TIM6 global and DAC1&2 underrun error  interrupts                 */
ISR void TIM7_Handler();     /*!< TIM7 global interrupt                                             */
ISR void DMA2_Stream0_Handler();     /*!< DMA2 Stream 0 global Interrupt                                    */
ISR void DMA2_Stream1_Handler();     /*!< DMA2 Stream 1 global Interrupt                                    */
ISR void DMA2_Stream2_Handler();     /*!< DMA2 Stream 2 global Interrupt                                    */
ISR void DMA2_Stream3_Handler();     /*!< DMA2 Stream 3 global Interrupt                                    */
ISR void DMA2_Stream4_Handler();     /*!< DMA2 Stream 4 global Interrupt                                    */
ISR void ETH_Handler();     /*!< Ethernet global Interrupt                                         */
ISR void ETH_WKUP_Handler();     /*!< Ethernet Wakeup through EXTI line Interrupt                       */
ISR void CAN2_TX_Handler();     /*!< CAN2 TX Interrupt                                                 */
ISR void CAN2_RX0_Handler();     /*!< CAN2 RX0 Interrupt                                                */
ISR void CAN2_RX1_Handler();     /*!< CAN2 RX1 Interrupt                                                */
ISR void CAN2_SCE_Handler();     /*!< CAN2 SCE Interrupt                                                */
ISR void OTG_FS_Handler();     /*!< USB OTG FS global Interrupt                                       */
ISR void DMA2_Stream5_Handler();     /*!< DMA2 Stream 5 global interrupt                                    */
ISR void DMA2_Stream6_Handler();     /*!< DMA2 Stream 6 global interrupt                                    */
ISR void DMA2_Stream7_Handler();     /*!< DMA2 Stream 7 global interrupt                                    */
ISR void USART6_Handler();     /*!< USART6 global interrupt                                           */
ISR void I2C3_EV_Handler();     /*!< I2C3 event interrupt                                              */
ISR void I2C3_ER_Handler();     /*!< I2C3 error interrupt                                              */
ISR void OTG_HS_EP1_OUT_Handler();     /*!< USB OTG HS End Point 1 Out global interrupt                       */
ISR void OTG_HS_EP1_IN_Handler();     /*!< USB OTG HS End Point 1 In global interrupt                        */
ISR void OTG_HS_WKUP_Handler();     /*!< USB OTG HS Wakeup through EXTI interrupt                          */
ISR void OTG_HS_Handler();     /*!< USB OTG HS global interrupt                                       */
ISR void DCMI_Handler();     /*!< DCMI global interrupt                                             */
ISR void CRYP_Handler();     /*!< CRYP crypto global interrupt                                      */
ISR void HASH_RNG_Handler();     /*!< Hash and Rng global interrupt                                     */

#if defined(STM32F40XX)
ISR void FPU_Handler();     /*!< FPU global interrupt                                              */
#endif /* STM32F40XX */

#if defined (STM32F427X) 
ISR void FPU_Handler();     /*!< FPU global interrupt                                              */
ISR void UART7_Handler();     /*!< UART7 global interrupt                                            */
ISR void UART8_Handler();     /*!< UART8 global interrupt                                            */
ISR void SPI4_Handler();     /*!< SPI4 global Interrupt                                             */
ISR void SPI5_Handler();     /*!< SPI5 global Interrupt                                             */
ISR void SPI6_Handler();     /*!< SPI6 global Interrupt                                             */
#endif /* STM32F427X */

#if defined (STM32F429X)
ISR void FPU_Handler();     /*!< FPU global interrupt                                              */
ISR void UART7_Handler();     /*!< UART7 global interrupt                                            */
ISR void UART8_Handler();     /*!< UART8 global interrupt                                            */
ISR void SPI4_Handler();     /*!< SPI4 global Interrupt                                             */
ISR void SPI5_Handler();     /*!< SPI5 global Interrupt                                             */
ISR void SPI6_Handler();     /*!< SPI6 global Interrupt                                             */
ISR void SAI1_Handler();     /*!< SAI1 global Interrupt                                             */
ISR void LTDC_Handler();     /*!< LTDC global Interrupt                                              */
ISR void LTDC_ER_Handler();     /*!< LTDC Error global Interrupt                                        */
ISR void DMA2D_Handler();     /*!< DMA2D global Interrupt                                            */
#endif /* STM32F429X */  

typedef void (*IntHwnd_t)(); //Handler function pointer typedef
extern const uint32_t __stack; //Top of stack from flash.ld


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


//Called on Reset
void Reset_Handler() {
	extern const uint32_t __sidata, __sdata, __edata, __sbss, __ebss;
	uint8_t *src, *dst;
	//Copy initial data to RAM .data
	//Start address of initial data section preloaded in ROM
	src = (uint8_t*)&__sidata;
	dst = (uint8_t*)&__sdata; //Start address of RAM data section
	for(uint32_t i = 0; i < (uint8_t*)&__edata - dst; i++) {
		dst[i] = src[i];
	}
	//Fill bss with 0
	dst = (uint8_t*)&__sbss; //Start address of bss section
	for(uint32_t i = 0; i < (uint8_t*)&__ebss - dst; i++) {
		dst[i] = 0;
	}
	//Finally, call main()
	extern int main();
	main();
}
