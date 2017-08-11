
/****************************************************************************************************//**
 * @file     ATSAM4E16E.h
 *
 * @brief    CMSIS Cortex-M4 Peripheral Access Layer Header File for
 *           ATSAM4E16E from Atmel.
 *
 * @version  V0
 * @date     15. March 2017
 *
 * @note     Generated with SVDConv V2.85b 
 *           from CMSIS SVD File 'ATSAM4E16E.svd' Version 0,
 *******************************************************************************************************/



/** @addtogroup Atmel
  * @{
  */

/** @addtogroup ATSAM4E16E
  * @{
  */

#ifndef ATSAM4E16E_H
#define ATSAM4E16E_H

#ifdef __cplusplus
extern "C" {
#endif


/* -------------------------  Interrupt Number Definition  ------------------------ */

typedef enum {
/* -------------------  Cortex-M4 Processor Exceptions Numbers  ------------------- */
  Reset_IRQn                    = -15,              /*!<   1  Reset Vector, invoked on Power up and warm reset                 */
  NonMaskableInt_IRQn           = -14,              /*!<   2  Non maskable Interrupt, cannot be stopped or preempted           */
  HardFault_IRQn                = -13,              /*!<   3  Hard Fault, all classes of Fault                                 */
  MemoryManagement_IRQn         = -12,              /*!<   4  Memory Management, MPU mismatch, including Access Violation
                                                         and No Match                                                          */
  BusFault_IRQn                 = -11,              /*!<   5  Bus Fault, Pre-Fetch-, Memory Access Fault, other address/memory
                                                         related Fault                                                         */
  UsageFault_IRQn               = -10,              /*!<   6  Usage Fault, i.e. Undef Instruction, Illegal State Transition    */
  SVCall_IRQn                   =  -5,              /*!<  11  System Service Call via SVC instruction                          */
  DebugMonitor_IRQn             =  -4,              /*!<  12  Debug Monitor                                                    */
  PendSV_IRQn                   =  -2,              /*!<  14  Pendable request for system service                              */
  SysTick_IRQn                  =  -1,              /*!<  15  System Tick Timer                                                */
/* --------------------  ATSAM4E16E Specific Interrupt Numbers  ------------------- */
  ID_PMC_IRQn                   =   5,              /*!<   5  ID_PMC                                                           */
  ID_EFC_IRQn                   =   6,              /*!<   6  ID_EFC                                                           */
  ID_UART0_IRQn                 =   7,              /*!<   7  ID_UART0                                                         */
  ID_PIOA_IRQn                  =   9,              /*!<   9  ID_PIOA                                                          */
  ID_PIOB_IRQn                  =  10,              /*!<  10  ID_PIOB                                                          */
  ID_PIOC_IRQn                  =  11,              /*!<  11  ID_PIOC                                                          */
  ID_PIOD_IRQn                  =  12,              /*!<  12  ID_PIOD                                                          */
  ID_PIOE_IRQn                  =  13,              /*!<  13  ID_PIOE                                                          */
  ID_USART0_IRQn                =  14,              /*!<  14  ID_USART0                                                        */
  ID_USART1_IRQn                =  15,              /*!<  15  ID_USART1                                                        */
  ID_HSMCI_IRQn                 =  16,              /*!<  16  ID_HSMCI                                                         */
  ID_TWI0_IRQn                  =  17,              /*!<  17  ID_TWI0                                                          */
  ID_TWI1_IRQn                  =  18,              /*!<  18  ID_TWI1                                                          */
  ID_SPI_IRQn                   =  19,              /*!<  19  ID_SPI                                                           */
  ID_DMAC_IRQn                  =  20,              /*!<  20  ID_DMAC                                                          */
  ID_TC0_IRQn                   =  21,              /*!<  21  ID_TC0                                                           */
  ID_TC1_IRQn                   =  22,              /*!<  22  ID_TC1                                                           */
  ID_TC2_IRQn                   =  23,              /*!<  23  ID_TC2                                                           */
  ID_TC3_IRQn                   =  24,              /*!<  24  ID_TC3                                                           */
  ID_TC4_IRQn                   =  25,              /*!<  25  ID_TC4                                                           */
  ID_TC5_IRQn                   =  26,              /*!<  26  ID_TC5                                                           */
  ID_TC6_IRQn                   =  27,              /*!<  27  ID_TC6                                                           */
  ID_TC7_IRQn                   =  28,              /*!<  28  ID_TC7                                                           */
  ID_TC8_IRQn                   =  29,              /*!<  29  ID_TC8                                                           */
  ID_AFEC0_IRQn                 =  30,              /*!<  30  ID_AFEC0                                                         */
  ID_AFEC1_IRQn                 =  31,              /*!<  31  ID_AFEC1                                                         */
  ID_DACC_IRQn                  =  32,              /*!<  32  ID_DACC                                                          */
  ID_ACC_IRQn                   =  33,              /*!<  33  ID_ACC                                                           */
  ID_UDP_IRQn                   =  35,              /*!<  35  ID_UDP                                                           */
  ID_PWM_IRQn                   =  36,              /*!<  36  ID_PWM                                                           */
  ID_CAN0_IRQn                  =  37,              /*!<  37  ID_CAN0                                                          */
  ID_CAN1_IRQn                  =  38,              /*!<  38  ID_CAN1                                                          */
  ID_AES_IRQn                   =  39,              /*!<  39  ID_AES                                                           */
  ID_GMAC_IRQn                  =  44,              /*!<  44  ID_GMAC                                                          */
  ID_UART1_IRQn                 =  45               /*!<  45  ID_UART1                                                         */
} IRQn_Type;


/** @addtogroup Configuration_of_CMSIS
  * @{
  */


/* ================================================================================ */
/* ================      Processor and Core Peripheral Section     ================ */
/* ================================================================================ */

/* ----------------Configuration of the Cortex-M4 Processor and Core Peripherals---------------- */
#define __CM4_REV                 0x0000            /*!< Cortex-M4 Core Revision                                               */
#define __MPU_PRESENT                  0            /*!< MPU present or not                                                    */
#define __NVIC_PRIO_BITS               4            /*!< Number of Bits used for Priority Levels                               */
#define __Vendor_SysTickConfig         0            /*!< Set to 1 if different SysTick Config is used                          */
#define __FPU_PRESENT                  1            /*!< FPU present or not                                                    */
/** @} */ /* End of group Configuration_of_CMSIS */

#include "core_cm4.h"                               /*!< Cortex-M4 processor and core peripherals                              */
#include "system_SAM4E.h"                           /*!< ATSAM4E16E System                                                     */


/* ================================================================================ */
/* ================       Device Specific Peripheral Section       ================ */
/* ================================================================================ */


/** @addtogroup Device_Peripheral_Registers
  * @{
  */


/* -------------------  Start of section using anonymous unions  ------------------ */
#if defined(__CC_ARM)
  #pragma push
  #pragma anon_unions
#elif defined(__ICCARM__)
  #pragma language=extended
#elif defined(__GNUC__)
  /* anonymous unions are enabled by default */
#elif defined(__TMS470__)
/* anonymous unions are enabled by default */
#elif defined(__TASKING__)
  #pragma warning 586
#else
  #warning Not supported compiler type
#endif



/* ================================================================================ */
/* ================                       PWM                      ================ */
/* ================================================================================ */


/**
  * @brief Pulse Width Modulation Controller (PWM)
  */

typedef struct {                                    /*!< PWM Structure                                                         */
  __IO uint32_t  CLK;                               /*!< PWM Clock Register                                                    */
  __O  uint32_t  ENA;                               /*!< PWM Enable Register                                                   */
  __O  uint32_t  DIS;                               /*!< PWM Disable Register                                                  */
  __I  uint32_t  SR;                                /*!< PWM Status Register                                                   */
  __O  uint32_t  IER1;                              /*!< PWM Interrupt Enable Register 1                                       */
  __O  uint32_t  IDR1;                              /*!< PWM Interrupt Disable Register 1                                      */
  __I  uint32_t  IMR1;                              /*!< PWM Interrupt Mask Register 1                                         */
  __I  uint32_t  ISR1;                              /*!< PWM Interrupt Status Register 1                                       */
  __IO uint32_t  SCM;                               /*!< PWM Sync Channels Mode Register                                       */
  __I  uint32_t  RESERVED;
  __IO uint32_t  SCUC;                              /*!< PWM Sync Channels Update Control Register                             */
  __IO uint32_t  SCUP;                              /*!< PWM Sync Channels Update Period Register                              */
  __O  uint32_t  SCUPUPD;                           /*!< PWM Sync Channels Update Period Update Register                       */
  __O  uint32_t  IER2;                              /*!< PWM Interrupt Enable Register 2                                       */
  __O  uint32_t  IDR2;                              /*!< PWM Interrupt Disable Register 2                                      */
  __I  uint32_t  IMR2;                              /*!< PWM Interrupt Mask Register 2                                         */
  __I  uint32_t  ISR2;                              /*!< PWM Interrupt Status Register 2                                       */
  __IO uint32_t  OOV;                               /*!< PWM Output Override Value Register                                    */
  __IO uint32_t  OS;                                /*!< PWM Output Selection Register                                         */
  __O  uint32_t  OSS;                               /*!< PWM Output Selection Set Register                                     */
  __O  uint32_t  OSC;                               /*!< PWM Output Selection Clear Register                                   */
  __O  uint32_t  OSSUPD;                            /*!< PWM Output Selection Set Update Register                              */
  __O  uint32_t  OSCUPD;                            /*!< PWM Output Selection Clear Update Register                            */
  __IO uint32_t  FMR;                               /*!< PWM Fault Mode Register                                               */
  __I  uint32_t  FSR;                               /*!< PWM Fault Status Register                                             */
  __O  uint32_t  FCR;                               /*!< PWM Fault Clear Register                                              */
  __IO uint32_t  FPV1;                              /*!< PWM Fault Protection Value Register 1                                 */
  __IO uint32_t  FPE;                               /*!< PWM Fault Protection Enable Register                                  */
  __I  uint32_t  RESERVED1[3];
  __IO uint32_t  ELMR0;                             /*!< PWM Event Line 0 Mode Register                                        */
  __IO uint32_t  ELMR1;                             /*!< PWM Event Line 0 Mode Register                                        */
  __I  uint32_t  RESERVED2[7];
  __IO uint32_t  SSPR;                              /*!< PWM Spread Spectrum Register                                          */
  __O  uint32_t  SSPUP;                             /*!< PWM Spread Spectrum Update Register                                   */
  __I  uint32_t  RESERVED3[2];
  __IO uint32_t  SMMR;                              /*!< PWM Stepper Motor Mode Register                                       */
  __I  uint32_t  RESERVED4[3];
  __IO uint32_t  FPV2;                              /*!< PWM Fault Protection Value 2 Register                                 */
  __I  uint32_t  RESERVED5[8];
  __O  uint32_t  WPCR;                              /*!< PWM Write Protect Control Register                                    */
  __I  uint32_t  WPSR;                              /*!< PWM Write Protect Status Register                                     */
  __I  uint32_t  RESERVED6[7];
  __IO uint32_t  TPR;                               /*!< Transmit Pointer Register                                             */
  __IO uint32_t  TCR;                               /*!< Transmit Counter Register                                             */
  __I  uint32_t  RESERVED7[2];
  __IO uint32_t  TNPR;                              /*!< Transmit Next Pointer Register                                        */
  __IO uint32_t  TNCR;                              /*!< Transmit Next Counter Register                                        */
  __O  uint32_t  PTCR;                              /*!< Transfer Control Register                                             */
  __I  uint32_t  PTSR;                              /*!< Transfer Status Register                                              */
  __I  uint32_t  RESERVED8[2];
  __IO uint32_t  CMPV0;                             /*!< PWM Comparison 0 Value Register                                       */
  __O  uint32_t  CMPVUPD0;                          /*!< PWM Comparison 0 Value Update Register                                */
  __IO uint32_t  CMPM0;                             /*!< PWM Comparison 0 Mode Register                                        */
  __O  uint32_t  CMPMUPD0;                          /*!< PWM Comparison 0 Mode Update Register                                 */
  __IO uint32_t  CMPV1;                             /*!< PWM Comparison 1 Value Register                                       */
  __O  uint32_t  CMPVUPD1;                          /*!< PWM Comparison 1 Value Update Register                                */
  __IO uint32_t  CMPM1;                             /*!< PWM Comparison 1 Mode Register                                        */
  __O  uint32_t  CMPMUPD1;                          /*!< PWM Comparison 1 Mode Update Register                                 */
  __IO uint32_t  CMPV2;                             /*!< PWM Comparison 2 Value Register                                       */
  __O  uint32_t  CMPVUPD2;                          /*!< PWM Comparison 2 Value Update Register                                */
  __IO uint32_t  CMPM2;                             /*!< PWM Comparison 2 Mode Register                                        */
  __O  uint32_t  CMPMUPD2;                          /*!< PWM Comparison 2 Mode Update Register                                 */
  __IO uint32_t  CMPV3;                             /*!< PWM Comparison 3 Value Register                                       */
  __O  uint32_t  CMPVUPD3;                          /*!< PWM Comparison 3 Value Update Register                                */
  __IO uint32_t  CMPM3;                             /*!< PWM Comparison 3 Mode Register                                        */
  __O  uint32_t  CMPMUPD3;                          /*!< PWM Comparison 3 Mode Update Register                                 */
  __IO uint32_t  CMPV4;                             /*!< PWM Comparison 4 Value Register                                       */
  __O  uint32_t  CMPVUPD4;                          /*!< PWM Comparison 4 Value Update Register                                */
  __IO uint32_t  CMPM4;                             /*!< PWM Comparison 4 Mode Register                                        */
  __O  uint32_t  CMPMUPD4;                          /*!< PWM Comparison 4 Mode Update Register                                 */
  __IO uint32_t  CMPV5;                             /*!< PWM Comparison 5 Value Register                                       */
  __O  uint32_t  CMPVUPD5;                          /*!< PWM Comparison 5 Value Update Register                                */
  __IO uint32_t  CMPM5;                             /*!< PWM Comparison 5 Mode Register                                        */
  __O  uint32_t  CMPMUPD5;                          /*!< PWM Comparison 5 Mode Update Register                                 */
  __IO uint32_t  CMPV6;                             /*!< PWM Comparison 6 Value Register                                       */
  __O  uint32_t  CMPVUPD6;                          /*!< PWM Comparison 6 Value Update Register                                */
  __IO uint32_t  CMPM6;                             /*!< PWM Comparison 6 Mode Register                                        */
  __O  uint32_t  CMPMUPD6;                          /*!< PWM Comparison 6 Mode Update Register                                 */
  __IO uint32_t  CMPV7;                             /*!< PWM Comparison 7 Value Register                                       */
  __O  uint32_t  CMPVUPD7;                          /*!< PWM Comparison 7 Value Update Register                                */
  __IO uint32_t  CMPM7;                             /*!< PWM Comparison 7 Mode Register                                        */
  __O  uint32_t  CMPMUPD7;                          /*!< PWM Comparison 7 Mode Update Register                                 */
  __I  uint32_t  RESERVED9[20];
  __IO uint32_t  CMR0;                              /*!< PWM Channel Mode Register (ch_num = 0)                                */
  __IO uint32_t  CDTY0;                             /*!< PWM Channel Duty Cycle Register (ch_num = 0)                          */
  __O  uint32_t  CDTYUPD0;                          /*!< PWM Channel Duty Cycle Update Register (ch_num = 0)                   */
  __IO uint32_t  CPRD0;                             /*!< PWM Channel Period Register (ch_num = 0)                              */
  __O  uint32_t  CPRDUPD0;                          /*!< PWM Channel Period Update Register (ch_num = 0)                       */
  __I  uint32_t  CCNT0;                             /*!< PWM Channel Counter Register (ch_num = 0)                             */
  __IO uint32_t  DT0;                               /*!< PWM Channel Dead Time Register (ch_num = 0)                           */
  __O  uint32_t  DTUPD0;                            /*!< PWM Channel Dead Time Update Register (ch_num = 0)                    */
  __IO uint32_t  CMR1;                              /*!< PWM Channel Mode Register (ch_num = 1)                                */
  __IO uint32_t  CDTY1;                             /*!< PWM Channel Duty Cycle Register (ch_num = 1)                          */
  __O  uint32_t  CDTYUPD1;                          /*!< PWM Channel Duty Cycle Update Register (ch_num = 1)                   */
  __IO uint32_t  CPRD1;                             /*!< PWM Channel Period Register (ch_num = 1)                              */
  __O  uint32_t  CPRDUPD1;                          /*!< PWM Channel Period Update Register (ch_num = 1)                       */
  __I  uint32_t  CCNT1;                             /*!< PWM Channel Counter Register (ch_num = 1)                             */
  __IO uint32_t  DT1;                               /*!< PWM Channel Dead Time Register (ch_num = 1)                           */
  __O  uint32_t  DTUPD1;                            /*!< PWM Channel Dead Time Update Register (ch_num = 1)                    */
  __IO uint32_t  CMR2;                              /*!< PWM Channel Mode Register (ch_num = 2)                                */
  __IO uint32_t  CDTY2;                             /*!< PWM Channel Duty Cycle Register (ch_num = 2)                          */
  __O  uint32_t  CDTYUPD2;                          /*!< PWM Channel Duty Cycle Update Register (ch_num = 2)                   */
  __IO uint32_t  CPRD2;                             /*!< PWM Channel Period Register (ch_num = 2)                              */
  __O  uint32_t  CPRDUPD2;                          /*!< PWM Channel Period Update Register (ch_num = 2)                       */
  __I  uint32_t  CCNT2;                             /*!< PWM Channel Counter Register (ch_num = 2)                             */
  __IO uint32_t  DT2;                               /*!< PWM Channel Dead Time Register (ch_num = 2)                           */
  __O  uint32_t  DTUPD2;                            /*!< PWM Channel Dead Time Update Register (ch_num = 2)                    */
  __IO uint32_t  CMR3;                              /*!< PWM Channel Mode Register (ch_num = 3)                                */
  __IO uint32_t  CDTY3;                             /*!< PWM Channel Duty Cycle Register (ch_num = 3)                          */
  __O  uint32_t  CDTYUPD3;                          /*!< PWM Channel Duty Cycle Update Register (ch_num = 3)                   */
  __IO uint32_t  CPRD3;                             /*!< PWM Channel Period Register (ch_num = 3)                              */
  __O  uint32_t  CPRDUPD3;                          /*!< PWM Channel Period Update Register (ch_num = 3)                       */
  __I  uint32_t  CCNT3;                             /*!< PWM Channel Counter Register (ch_num = 3)                             */
  __IO uint32_t  DT3;                               /*!< PWM Channel Dead Time Register (ch_num = 3)                           */
  __O  uint32_t  DTUPD3;                            /*!< PWM Channel Dead Time Update Register (ch_num = 3)                    */
  __I  uint32_t  RESERVED10[96];
  __O  uint32_t  CMUPD0;                            /*!< PWM Channel Mode Update Register (ch_num = 0)                         */
  __IO uint32_t  CAE0;                              /*!< PWM Channel Additional Edge Register (ch_num = 0)                     */
  __O  uint32_t  CAEUPD0;                           /*!< PWM Channel Additional Edge Update Register (ch_num = 0)              */
  __I  uint32_t  RESERVED11[5];
  __O  uint32_t  CMUPD1;                            /*!< PWM Channel Mode Update Register (ch_num = 1)                         */
  __IO uint32_t  CAE1;                              /*!< PWM Channel Additional Edge Register (ch_num = 1)                     */
  __O  uint32_t  CAEUPD1;                           /*!< PWM Channel Additional Edge Update Register (ch_num = 1)              */
  __I  uint32_t  RESERVED12[5];
  __O  uint32_t  CMUPD2;                            /*!< PWM Channel Mode Update Register (ch_num = 2)                         */
  __IO uint32_t  CAE2;                              /*!< PWM Channel Additional Edge Register (ch_num = 2)                     */
  __O  uint32_t  CAEUPD2;                           /*!< PWM Channel Additional Edge Update Register (ch_num = 2)              */
  __I  uint32_t  RESERVED13[5];
  __O  uint32_t  CMUPD3;                            /*!< PWM Channel Mode Update Register (ch_num = 3)                         */
  __IO uint32_t  CAE3;                              /*!< PWM Channel Additional Edge Register (ch_num = 3)                     */
  __O  uint32_t  CAEUPD3;                           /*!< PWM Channel Additional Edge Update Register (ch_num = 3)              */
} PWM_Type;


/* ================================================================================ */
/* ================                       AES                      ================ */
/* ================================================================================ */


/**
  * @brief Advanced Encryption Standard (AES)
  */

typedef struct {                                    /*!< AES Structure                                                         */
  __O  uint32_t  CR;                                /*!< Control Register                                                      */
  __IO uint32_t  MR;                                /*!< Mode Register                                                         */
  __I  uint32_t  RESERVED[2];
  __O  uint32_t  IER;                               /*!< Interrupt Enable Register                                             */
  __O  uint32_t  IDR;                               /*!< Interrupt Disable Register                                            */
  __I  uint32_t  IMR;                               /*!< Interrupt Mask Register                                               */
  __I  uint32_t  ISR;                               /*!< Interrupt Status Register                                             */
  __O  uint32_t  KEYWR0;                            /*!< Key Word Register                                                     */
  __O  uint32_t  KEYWR1;                            /*!< Key Word Register                                                     */
  __O  uint32_t  KEYWR2;                            /*!< Key Word Register                                                     */
  __O  uint32_t  KEYWR3;                            /*!< Key Word Register                                                     */
  __O  uint32_t  KEYWR4;                            /*!< Key Word Register                                                     */
  __O  uint32_t  KEYWR5;                            /*!< Key Word Register                                                     */
  __O  uint32_t  KEYWR6;                            /*!< Key Word Register                                                     */
  __O  uint32_t  KEYWR7;                            /*!< Key Word Register                                                     */
  __O  uint32_t  IDATAR0;                           /*!< Input Data Register                                                   */
  __O  uint32_t  IDATAR1;                           /*!< Input Data Register                                                   */
  __O  uint32_t  IDATAR2;                           /*!< Input Data Register                                                   */
  __O  uint32_t  IDATAR3;                           /*!< Input Data Register                                                   */
  __I  uint32_t  ODATAR0;                           /*!< Output Data Register                                                  */
  __I  uint32_t  ODATAR1;                           /*!< Output Data Register                                                  */
  __I  uint32_t  ODATAR2;                           /*!< Output Data Register                                                  */
  __I  uint32_t  ODATAR3;                           /*!< Output Data Register                                                  */
  __O  uint32_t  IVR0;                              /*!< Initialization Vector Register                                        */
  __O  uint32_t  IVR1;                              /*!< Initialization Vector Register                                        */
  __O  uint32_t  IVR2;                              /*!< Initialization Vector Register                                        */
  __O  uint32_t  IVR3;                              /*!< Initialization Vector Register                                        */
} AES_Type;


/* ================================================================================ */
/* ================                      CAN0                      ================ */
/* ================================================================================ */


/**
  * @brief Controller Area Network 0 (CAN0)
  */

typedef struct {                                    /*!< CAN0 Structure                                                        */
  __IO uint32_t  MR;                                /*!< Mode Register                                                         */
  __O  uint32_t  IER;                               /*!< Interrupt Enable Register                                             */
  __O  uint32_t  IDR;                               /*!< Interrupt Disable Register                                            */
  __I  uint32_t  IMR;                               /*!< Interrupt Mask Register                                               */
  __I  uint32_t  SR;                                /*!< Status Register                                                       */
  __IO uint32_t  BR;                                /*!< Baudrate Register                                                     */
  __I  uint32_t  TIM;                               /*!< Timer Register                                                        */
  __I  uint32_t  TIMESTP;                           /*!< Timestamp Register                                                    */
  __I  uint32_t  ECR;                               /*!< Error Counter Register                                                */
  __O  uint32_t  TCR;                               /*!< Transfer Command Register                                             */
  __O  uint32_t  ACR;                               /*!< Abort Command Register                                                */
  __I  uint32_t  RESERVED[46];
  __IO uint32_t  WPMR;                              /*!< Write Protect Mode Register                                           */
  __I  uint32_t  WPSR;                              /*!< Write Protect Status Register                                         */
  __I  uint32_t  RESERVED1[69];
  __IO uint32_t  MMR0;                              /*!< Mailbox Mode Register (MB = 0)                                        */
  __IO uint32_t  MAM0;                              /*!< Mailbox Acceptance Mask Register (MB = 0)                             */
  __IO uint32_t  MID0;                              /*!< Mailbox ID Register (MB = 0)                                          */
  __I  uint32_t  MFID0;                             /*!< Mailbox Family ID Register (MB = 0)                                   */
  __I  uint32_t  MSR0;                              /*!< Mailbox Status Register (MB = 0)                                      */
  __IO uint32_t  MDL0;                              /*!< Mailbox Data Low Register (MB = 0)                                    */
  __IO uint32_t  MDH0;                              /*!< Mailbox Data High Register (MB = 0)                                   */
  __O  uint32_t  MCR0;                              /*!< Mailbox Control Register (MB = 0)                                     */
  __IO uint32_t  MMR1;                              /*!< Mailbox Mode Register (MB = 1)                                        */
  __IO uint32_t  MAM1;                              /*!< Mailbox Acceptance Mask Register (MB = 1)                             */
  __IO uint32_t  MID1;                              /*!< Mailbox ID Register (MB = 1)                                          */
  __I  uint32_t  MFID1;                             /*!< Mailbox Family ID Register (MB = 1)                                   */
  __I  uint32_t  MSR1;                              /*!< Mailbox Status Register (MB = 1)                                      */
  __IO uint32_t  MDL1;                              /*!< Mailbox Data Low Register (MB = 1)                                    */
  __IO uint32_t  MDH1;                              /*!< Mailbox Data High Register (MB = 1)                                   */
  __O  uint32_t  MCR1;                              /*!< Mailbox Control Register (MB = 1)                                     */
  __IO uint32_t  MMR2;                              /*!< Mailbox Mode Register (MB = 2)                                        */
  __IO uint32_t  MAM2;                              /*!< Mailbox Acceptance Mask Register (MB = 2)                             */
  __IO uint32_t  MID2;                              /*!< Mailbox ID Register (MB = 2)                                          */
  __I  uint32_t  MFID2;                             /*!< Mailbox Family ID Register (MB = 2)                                   */
  __I  uint32_t  MSR2;                              /*!< Mailbox Status Register (MB = 2)                                      */
  __IO uint32_t  MDL2;                              /*!< Mailbox Data Low Register (MB = 2)                                    */
  __IO uint32_t  MDH2;                              /*!< Mailbox Data High Register (MB = 2)                                   */
  __O  uint32_t  MCR2;                              /*!< Mailbox Control Register (MB = 2)                                     */
  __IO uint32_t  MMR3;                              /*!< Mailbox Mode Register (MB = 3)                                        */
  __IO uint32_t  MAM3;                              /*!< Mailbox Acceptance Mask Register (MB = 3)                             */
  __IO uint32_t  MID3;                              /*!< Mailbox ID Register (MB = 3)                                          */
  __I  uint32_t  MFID3;                             /*!< Mailbox Family ID Register (MB = 3)                                   */
  __I  uint32_t  MSR3;                              /*!< Mailbox Status Register (MB = 3)                                      */
  __IO uint32_t  MDL3;                              /*!< Mailbox Data Low Register (MB = 3)                                    */
  __IO uint32_t  MDH3;                              /*!< Mailbox Data High Register (MB = 3)                                   */
  __O  uint32_t  MCR3;                              /*!< Mailbox Control Register (MB = 3)                                     */
  __IO uint32_t  MMR4;                              /*!< Mailbox Mode Register (MB = 4)                                        */
  __IO uint32_t  MAM4;                              /*!< Mailbox Acceptance Mask Register (MB = 4)                             */
  __IO uint32_t  MID4;                              /*!< Mailbox ID Register (MB = 4)                                          */
  __I  uint32_t  MFID4;                             /*!< Mailbox Family ID Register (MB = 4)                                   */
  __I  uint32_t  MSR4;                              /*!< Mailbox Status Register (MB = 4)                                      */
  __IO uint32_t  MDL4;                              /*!< Mailbox Data Low Register (MB = 4)                                    */
  __IO uint32_t  MDH4;                              /*!< Mailbox Data High Register (MB = 4)                                   */
  __O  uint32_t  MCR4;                              /*!< Mailbox Control Register (MB = 4)                                     */
  __IO uint32_t  MMR5;                              /*!< Mailbox Mode Register (MB = 5)                                        */
  __IO uint32_t  MAM5;                              /*!< Mailbox Acceptance Mask Register (MB = 5)                             */
  __IO uint32_t  MID5;                              /*!< Mailbox ID Register (MB = 5)                                          */
  __I  uint32_t  MFID5;                             /*!< Mailbox Family ID Register (MB = 5)                                   */
  __I  uint32_t  MSR5;                              /*!< Mailbox Status Register (MB = 5)                                      */
  __IO uint32_t  MDL5;                              /*!< Mailbox Data Low Register (MB = 5)                                    */
  __IO uint32_t  MDH5;                              /*!< Mailbox Data High Register (MB = 5)                                   */
  __O  uint32_t  MCR5;                              /*!< Mailbox Control Register (MB = 5)                                     */
  __IO uint32_t  MMR6;                              /*!< Mailbox Mode Register (MB = 6)                                        */
  __IO uint32_t  MAM6;                              /*!< Mailbox Acceptance Mask Register (MB = 6)                             */
  __IO uint32_t  MID6;                              /*!< Mailbox ID Register (MB = 6)                                          */
  __I  uint32_t  MFID6;                             /*!< Mailbox Family ID Register (MB = 6)                                   */
  __I  uint32_t  MSR6;                              /*!< Mailbox Status Register (MB = 6)                                      */
  __IO uint32_t  MDL6;                              /*!< Mailbox Data Low Register (MB = 6)                                    */
  __IO uint32_t  MDH6;                              /*!< Mailbox Data High Register (MB = 6)                                   */
  __O  uint32_t  MCR6;                              /*!< Mailbox Control Register (MB = 6)                                     */
  __IO uint32_t  MMR7;                              /*!< Mailbox Mode Register (MB = 7)                                        */
  __IO uint32_t  MAM7;                              /*!< Mailbox Acceptance Mask Register (MB = 7)                             */
  __IO uint32_t  MID7;                              /*!< Mailbox ID Register (MB = 7)                                          */
  __I  uint32_t  MFID7;                             /*!< Mailbox Family ID Register (MB = 7)                                   */
  __I  uint32_t  MSR7;                              /*!< Mailbox Status Register (MB = 7)                                      */
  __IO uint32_t  MDL7;                              /*!< Mailbox Data Low Register (MB = 7)                                    */
  __IO uint32_t  MDH7;                              /*!< Mailbox Data High Register (MB = 7)                                   */
  __O  uint32_t  MCR7;                              /*!< Mailbox Control Register (MB = 7)                                     */
} CAN0_Type;


/* ================================================================================ */
/* ================                      CAN1                      ================ */
/* ================================================================================ */


/**
  * @brief Controller Area Network 1 (CAN1)
  */

typedef struct {                                    /*!< CAN1 Structure                                                        */
  __IO uint32_t  MR;                                /*!< Mode Register                                                         */
  __O  uint32_t  IER;                               /*!< Interrupt Enable Register                                             */
  __O  uint32_t  IDR;                               /*!< Interrupt Disable Register                                            */
  __I  uint32_t  IMR;                               /*!< Interrupt Mask Register                                               */
  __I  uint32_t  SR;                                /*!< Status Register                                                       */
  __IO uint32_t  BR;                                /*!< Baudrate Register                                                     */
  __I  uint32_t  TIM;                               /*!< Timer Register                                                        */
  __I  uint32_t  TIMESTP;                           /*!< Timestamp Register                                                    */
  __I  uint32_t  ECR;                               /*!< Error Counter Register                                                */
  __O  uint32_t  TCR;                               /*!< Transfer Command Register                                             */
  __O  uint32_t  ACR;                               /*!< Abort Command Register                                                */
  __I  uint32_t  RESERVED[46];
  __IO uint32_t  WPMR;                              /*!< Write Protect Mode Register                                           */
  __I  uint32_t  WPSR;                              /*!< Write Protect Status Register                                         */
  __I  uint32_t  RESERVED1[69];
  __IO uint32_t  MMR0;                              /*!< Mailbox Mode Register (MB = 0)                                        */
  __IO uint32_t  MAM0;                              /*!< Mailbox Acceptance Mask Register (MB = 0)                             */
  __IO uint32_t  MID0;                              /*!< Mailbox ID Register (MB = 0)                                          */
  __I  uint32_t  MFID0;                             /*!< Mailbox Family ID Register (MB = 0)                                   */
  __I  uint32_t  MSR0;                              /*!< Mailbox Status Register (MB = 0)                                      */
  __IO uint32_t  MDL0;                              /*!< Mailbox Data Low Register (MB = 0)                                    */
  __IO uint32_t  MDH0;                              /*!< Mailbox Data High Register (MB = 0)                                   */
  __O  uint32_t  MCR0;                              /*!< Mailbox Control Register (MB = 0)                                     */
  __IO uint32_t  MMR1;                              /*!< Mailbox Mode Register (MB = 1)                                        */
  __IO uint32_t  MAM1;                              /*!< Mailbox Acceptance Mask Register (MB = 1)                             */
  __IO uint32_t  MID1;                              /*!< Mailbox ID Register (MB = 1)                                          */
  __I  uint32_t  MFID1;                             /*!< Mailbox Family ID Register (MB = 1)                                   */
  __I  uint32_t  MSR1;                              /*!< Mailbox Status Register (MB = 1)                                      */
  __IO uint32_t  MDL1;                              /*!< Mailbox Data Low Register (MB = 1)                                    */
  __IO uint32_t  MDH1;                              /*!< Mailbox Data High Register (MB = 1)                                   */
  __O  uint32_t  MCR1;                              /*!< Mailbox Control Register (MB = 1)                                     */
  __IO uint32_t  MMR2;                              /*!< Mailbox Mode Register (MB = 2)                                        */
  __IO uint32_t  MAM2;                              /*!< Mailbox Acceptance Mask Register (MB = 2)                             */
  __IO uint32_t  MID2;                              /*!< Mailbox ID Register (MB = 2)                                          */
  __I  uint32_t  MFID2;                             /*!< Mailbox Family ID Register (MB = 2)                                   */
  __I  uint32_t  MSR2;                              /*!< Mailbox Status Register (MB = 2)                                      */
  __IO uint32_t  MDL2;                              /*!< Mailbox Data Low Register (MB = 2)                                    */
  __IO uint32_t  MDH2;                              /*!< Mailbox Data High Register (MB = 2)                                   */
  __O  uint32_t  MCR2;                              /*!< Mailbox Control Register (MB = 2)                                     */
  __IO uint32_t  MMR3;                              /*!< Mailbox Mode Register (MB = 3)                                        */
  __IO uint32_t  MAM3;                              /*!< Mailbox Acceptance Mask Register (MB = 3)                             */
  __IO uint32_t  MID3;                              /*!< Mailbox ID Register (MB = 3)                                          */
  __I  uint32_t  MFID3;                             /*!< Mailbox Family ID Register (MB = 3)                                   */
  __I  uint32_t  MSR3;                              /*!< Mailbox Status Register (MB = 3)                                      */
  __IO uint32_t  MDL3;                              /*!< Mailbox Data Low Register (MB = 3)                                    */
  __IO uint32_t  MDH3;                              /*!< Mailbox Data High Register (MB = 3)                                   */
  __O  uint32_t  MCR3;                              /*!< Mailbox Control Register (MB = 3)                                     */
  __IO uint32_t  MMR4;                              /*!< Mailbox Mode Register (MB = 4)                                        */
  __IO uint32_t  MAM4;                              /*!< Mailbox Acceptance Mask Register (MB = 4)                             */
  __IO uint32_t  MID4;                              /*!< Mailbox ID Register (MB = 4)                                          */
  __I  uint32_t  MFID4;                             /*!< Mailbox Family ID Register (MB = 4)                                   */
  __I  uint32_t  MSR4;                              /*!< Mailbox Status Register (MB = 4)                                      */
  __IO uint32_t  MDL4;                              /*!< Mailbox Data Low Register (MB = 4)                                    */
  __IO uint32_t  MDH4;                              /*!< Mailbox Data High Register (MB = 4)                                   */
  __O  uint32_t  MCR4;                              /*!< Mailbox Control Register (MB = 4)                                     */
  __IO uint32_t  MMR5;                              /*!< Mailbox Mode Register (MB = 5)                                        */
  __IO uint32_t  MAM5;                              /*!< Mailbox Acceptance Mask Register (MB = 5)                             */
  __IO uint32_t  MID5;                              /*!< Mailbox ID Register (MB = 5)                                          */
  __I  uint32_t  MFID5;                             /*!< Mailbox Family ID Register (MB = 5)                                   */
  __I  uint32_t  MSR5;                              /*!< Mailbox Status Register (MB = 5)                                      */
  __IO uint32_t  MDL5;                              /*!< Mailbox Data Low Register (MB = 5)                                    */
  __IO uint32_t  MDH5;                              /*!< Mailbox Data High Register (MB = 5)                                   */
  __O  uint32_t  MCR5;                              /*!< Mailbox Control Register (MB = 5)                                     */
  __IO uint32_t  MMR6;                              /*!< Mailbox Mode Register (MB = 6)                                        */
  __IO uint32_t  MAM6;                              /*!< Mailbox Acceptance Mask Register (MB = 6)                             */
  __IO uint32_t  MID6;                              /*!< Mailbox ID Register (MB = 6)                                          */
  __I  uint32_t  MFID6;                             /*!< Mailbox Family ID Register (MB = 6)                                   */
  __I  uint32_t  MSR6;                              /*!< Mailbox Status Register (MB = 6)                                      */
  __IO uint32_t  MDL6;                              /*!< Mailbox Data Low Register (MB = 6)                                    */
  __IO uint32_t  MDH6;                              /*!< Mailbox Data High Register (MB = 6)                                   */
  __O  uint32_t  MCR6;                              /*!< Mailbox Control Register (MB = 6)                                     */
  __IO uint32_t  MMR7;                              /*!< Mailbox Mode Register (MB = 7)                                        */
  __IO uint32_t  MAM7;                              /*!< Mailbox Acceptance Mask Register (MB = 7)                             */
  __IO uint32_t  MID7;                              /*!< Mailbox ID Register (MB = 7)                                          */
  __I  uint32_t  MFID7;                             /*!< Mailbox Family ID Register (MB = 7)                                   */
  __I  uint32_t  MSR7;                              /*!< Mailbox Status Register (MB = 7)                                      */
  __IO uint32_t  MDL7;                              /*!< Mailbox Data Low Register (MB = 7)                                    */
  __IO uint32_t  MDH7;                              /*!< Mailbox Data High Register (MB = 7)                                   */
  __O  uint32_t  MCR7;                              /*!< Mailbox Control Register (MB = 7)                                     */
} CAN1_Type;


/* ================================================================================ */
/* ================                      GMAC                      ================ */
/* ================================================================================ */


/**
  * @brief Gigabit Ethernet MAC (GMAC)
  */

typedef struct {                                    /*!< GMAC Structure                                                        */
  __IO uint32_t  NCR;                               /*!< Network Control Register                                              */
  __IO uint32_t  NCFGR;                             /*!< Network Configuration Register                                        */
  __I  uint32_t  NSR;                               /*!< Network Status Register                                               */
  __IO uint32_t  UR;                                /*!< User Register                                                         */
  __IO uint32_t  DCFGR;                             /*!< DMA Configuration Register                                            */
  __IO uint32_t  TSR;                               /*!< Transmit Status Register                                              */
  __IO uint32_t  RBQB;                              /*!< Receive Buffer Queue Base Address                                     */
  __IO uint32_t  TBQB;                              /*!< Transmit Buffer Queue Base Address                                    */
  __IO uint32_t  RSR;                               /*!< Receive Status Register                                               */
  __I  uint32_t  ISR;                               /*!< Interrupt Status Register                                             */
  __O  uint32_t  IER;                               /*!< Interrupt Enable Register                                             */
  __O  uint32_t  IDR;                               /*!< Interrupt Disable Register                                            */
  __I  uint32_t  IMR;                               /*!< Interrupt Mask Register                                               */
  __IO uint32_t  MAN;                               /*!< PHY Maintenance Register                                              */
  __I  uint32_t  RPQ;                               /*!< Received Pause Quantum Register                                       */
  __IO uint32_t  TPQ;                               /*!< Transmit Pause Quantum Register                                       */
  __I  uint32_t  RESERVED[16];
  __IO uint32_t  HRB;                               /*!< Hash Register Bottom [31:0]                                           */
  __IO uint32_t  HRT;                               /*!< Hash Register Top [63:32]                                             */
  __IO uint32_t  SAB1;                              /*!< Specific Address 1 Bottom [31:0] Register                             */
  __IO uint32_t  SAT1;                              /*!< Specific Address 1 Top [47:32] Register                               */
  __IO uint32_t  SAB2;                              /*!< Specific Address 2 Bottom [31:0] Register                             */
  __IO uint32_t  SAT2;                              /*!< Specific Address 2 Top [47:32] Register                               */
  __IO uint32_t  SAB3;                              /*!< Specific Address 3 Bottom [31:0] Register                             */
  __IO uint32_t  SAT3;                              /*!< Specific Address 3 Top [47:32] Register                               */
  __IO uint32_t  SAB4;                              /*!< Specific Address 4 Bottom [31:0] Register                             */
  __IO uint32_t  SAT4;                              /*!< Specific Address 4 Top [47:32] Register                               */
  __IO uint32_t  TIDM0;                             /*!< Type ID Match 1 Register                                              */
  __IO uint32_t  TIDM1;                             /*!< Type ID Match 1 Register                                              */
  __IO uint32_t  TIDM2;                             /*!< Type ID Match 1 Register                                              */
  __IO uint32_t  TIDM3;                             /*!< Type ID Match 1 Register                                              */
  __I  uint32_t  RESERVED1;
  __IO uint32_t  IPGS;                              /*!< IPG Stretch Register                                                  */
  __IO uint32_t  SVLAN;                             /*!< Stacked VLAN Register                                                 */
  __IO uint32_t  TPFCP;                             /*!< Transmit PFC Pause Register                                           */
  __IO uint32_t  SAMB1;                             /*!< Specific Address 1 Mask Bottom [31:0] Register                        */
  __IO uint32_t  SAMT1;                             /*!< Specific Address 1 Mask Top [47:32] Register                          */
  __I  uint32_t  RESERVED2[12];
  __I  uint32_t  OTLO;                              /*!< Octets Transmitted [31:0] Register                                    */
  __I  uint32_t  OTHI;                              /*!< Octets Transmitted [47:32] Register                                   */
  __I  uint32_t  FT;                                /*!< Frames Transmitted Register                                           */
  __I  uint32_t  BCFT;                              /*!< Broadcast Frames Transmitted Register                                 */
  __I  uint32_t  MFT;                               /*!< Multicast Frames Transmitted Register                                 */
  __I  uint32_t  PFT;                               /*!< Pause Frames Transmitted Register                                     */
  __I  uint32_t  BFT64;                             /*!< 64 Byte Frames Transmitted Register                                   */
  __I  uint32_t  TBFT127;                           /*!< 65 to 127 Byte Frames Transmitted Register                            */
  __I  uint32_t  TBFT255;                           /*!< 128 to 255 Byte Frames Transmitted Register                           */
  __I  uint32_t  TBFT511;                           /*!< 256 to 511 Byte Frames Transmitted Register                           */
  __I  uint32_t  TBFT1023;                          /*!< 512 to 1023 Byte Frames Transmitted Register                          */
  __I  uint32_t  TBFT1518;                          /*!< 1024 to 1518 Byte Frames Transmitted Register                         */
  __I  uint32_t  GTBFT1518;                         /*!< Greater Than 1518 Byte Frames Transmitted Register                    */
  __I  uint32_t  TUR;                               /*!< Transmit Under Runs Register                                          */
  __I  uint32_t  SCF;                               /*!< Single Collision Frames Register                                      */
  __I  uint32_t  MCF;                               /*!< Multiple Collision Frames Register                                    */
  __I  uint32_t  EC;                                /*!< Excessive Collisions Register                                         */
  __I  uint32_t  LC;                                /*!< Late Collisions Register                                              */
  __I  uint32_t  DTF;                               /*!< Deferred Transmission Frames Register                                 */
  __I  uint32_t  CSE;                               /*!< Carrier Sense Errors Register                                         */
  __I  uint32_t  ORLO;                              /*!< Octets Received [31:0] Received                                       */
  __I  uint32_t  ORHI;                              /*!< Octets Received [47:32] Received                                      */
  __I  uint32_t  FR;                                /*!< Frames Received Register                                              */
  __I  uint32_t  BCFR;                              /*!< Broadcast Frames Received Register                                    */
  __I  uint32_t  MFR;                               /*!< Multicast Frames Received Register                                    */
  __I  uint32_t  PFR;                               /*!< Pause Frames Received Register                                        */
  __I  uint32_t  BFR64;                             /*!< 64 Byte Frames Received Register                                      */
  __I  uint32_t  TBFR127;                           /*!< 65 to 127 Byte Frames Received Register                               */
  __I  uint32_t  TBFR255;                           /*!< 128 to 255 Byte Frames Received Register                              */
  __I  uint32_t  TBFR511;                           /*!< 256 to 511Byte Frames Received Register                               */
  __I  uint32_t  TBFR1023;                          /*!< 512 to 1023 Byte Frames Received Register                             */
  __I  uint32_t  TBFR1518;                          /*!< 1024 to 1518 Byte Frames Received Register                            */
  __I  uint32_t  TMXBFR;                            /*!< 1519 to Maximum Byte Frames Received Register                         */
  __I  uint32_t  UFR;                               /*!< Undersize Frames Received Register                                    */
  __I  uint32_t  OFR;                               /*!< Oversize Frames Received Register                                     */
  __I  uint32_t  JR;                                /*!< Jabbers Received Register                                             */
  __I  uint32_t  FCSE;                              /*!< Frame Check Sequence Errors Register                                  */
  __I  uint32_t  LFFE;                              /*!< Length Field Frame Errors Register                                    */
  __I  uint32_t  RSE;                               /*!< Receive Symbol Errors Register                                        */
  __I  uint32_t  AE;                                /*!< Alignment Errors Register                                             */
  __I  uint32_t  RRE;                               /*!< Receive Resource Errors Register                                      */
  __I  uint32_t  ROE;                               /*!< Receive Overrun Register                                              */
  __I  uint32_t  IHCE;                              /*!< IP Header Checksum Errors Register                                    */
  __I  uint32_t  TCE;                               /*!< TCP Checksum Errors Register                                          */
  __I  uint32_t  UCE;                               /*!< UDP Checksum Errors Register                                          */
  __I  uint32_t  RESERVED3[5];
  __IO uint32_t  TSSS;                              /*!< 1588 Timer Sync Strobe Seconds Register                               */
  __IO uint32_t  TSSN;                              /*!< 1588 Timer Sync Strobe Nanoseconds Register                           */
  __IO uint32_t  TS;                                /*!< 1588 Timer Seconds Register                                           */
  __IO uint32_t  TN;                                /*!< 1588 Timer Nanoseconds Register                                       */
  __O  uint32_t  TA;                                /*!< 1588 Timer Adjust Register                                            */
  __IO uint32_t  TI;                                /*!< 1588 Timer Increment Register                                         */
  __I  uint32_t  EFTS;                              /*!< PTP Event Frame Transmitted Seconds                                   */
  __I  uint32_t  EFTN;                              /*!< PTP Event Frame Transmitted Nanoseconds                               */
  __I  uint32_t  EFRS;                              /*!< PTP Event Frame Received Seconds                                      */
  __I  uint32_t  EFRN;                              /*!< PTP Event Frame Received Nanoseconds                                  */
  __I  uint32_t  PEFTS;                             /*!< PTP Peer Event Frame Transmitted Seconds                              */
  __I  uint32_t  PEFTN;                             /*!< PTP Peer Event Frame Transmitted Nanoseconds                          */
  __I  uint32_t  PEFRS;                             /*!< PTP Peer Event Frame Received Seconds                                 */
  __I  uint32_t  PEFRN;                             /*!< PTP Peer Event Frame Received Nanoseconds                             */
} GMAC_Type;


/* ================================================================================ */
/* ================                      CRCCU                     ================ */
/* ================================================================================ */


/**
  * @brief Cyclic Redundancy Check Calculation Unit (CRCCU)
  */

typedef struct {                                    /*!< CRCCU Structure                                                       */
  __IO uint32_t  DSCR;                              /*!< CRCCU Descriptor Base Register                                        */
  __I  uint32_t  RESERVED;
  __O  uint32_t  DMA_EN;                            /*!< CRCCU DMA Enable Register                                             */
  __O  uint32_t  DMA_DIS;                           /*!< CRCCU DMA Disable Register                                            */
  __I  uint32_t  DMA_SR;                            /*!< CRCCU DMA Status Register                                             */
  __O  uint32_t  DMA_IER;                           /*!< CRCCU DMA Interrupt Enable Register                                   */
  __O  uint32_t  DMA_IDR;                           /*!< CRCCU DMA Interrupt Disable Register                                  */
  __I  uint32_t  DMA_IMR;                           /*!< CRCCU DMA Interrupt Mask Register                                     */
  __I  uint32_t  DMA_ISR;                           /*!< CRCCU DMA Interrupt Status Register                                   */
  __I  uint32_t  RESERVED1[4];
  __O  uint32_t  CR;                                /*!< CRCCU Control Register                                                */
  __IO uint32_t  MR;                                /*!< CRCCU Mode Register                                                   */
  __I  uint32_t  SR;                                /*!< CRCCU Status Register                                                 */
  __O  uint32_t  IER;                               /*!< CRCCU Interrupt Enable Register                                       */
  __O  uint32_t  IDR;                               /*!< CRCCU Interrupt Disable Register                                      */
  __I  uint32_t  IMR;                               /*!< CRCCU Interrupt Mask Register                                         */
  __I  uint32_t  ISR;                               /*!< CRCCU Interrupt Status Register                                       */
} CRCCU_Type;


/* ================================================================================ */
/* ================                       SMC                      ================ */
/* ================================================================================ */


/**
  * @brief Static Memory Controller (SMC)
  */

typedef struct {                                    /*!< SMC Structure                                                         */
  __IO uint32_t  SETUP0;                            /*!< SMC Setup Register (CS_number = 0)                                    */
  __IO uint32_t  PULSE0;                            /*!< SMC Pulse Register (CS_number = 0)                                    */
  __IO uint32_t  CYCLE0;                            /*!< SMC Cycle Register (CS_number = 0)                                    */
  __IO uint32_t  MODE0;                             /*!< SMC Mode Register (CS_number = 0)                                     */
  __IO uint32_t  SETUP1;                            /*!< SMC Setup Register (CS_number = 1)                                    */
  __IO uint32_t  PULSE1;                            /*!< SMC Pulse Register (CS_number = 1)                                    */
  __IO uint32_t  CYCLE1;                            /*!< SMC Cycle Register (CS_number = 1)                                    */
  __IO uint32_t  MODE1;                             /*!< SMC Mode Register (CS_number = 1)                                     */
  __IO uint32_t  SETUP2;                            /*!< SMC Setup Register (CS_number = 2)                                    */
  __IO uint32_t  PULSE2;                            /*!< SMC Pulse Register (CS_number = 2)                                    */
  __IO uint32_t  CYCLE2;                            /*!< SMC Cycle Register (CS_number = 2)                                    */
  __IO uint32_t  MODE2;                             /*!< SMC Mode Register (CS_number = 2)                                     */
  __IO uint32_t  SETUP3;                            /*!< SMC Setup Register (CS_number = 3)                                    */
  __IO uint32_t  PULSE3;                            /*!< SMC Pulse Register (CS_number = 3)                                    */
  __IO uint32_t  CYCLE3;                            /*!< SMC Cycle Register (CS_number = 3)                                    */
  __IO uint32_t  MODE3;                             /*!< SMC Mode Register (CS_number = 3)                                     */
  __I  uint32_t  RESERVED[16];
  __IO uint32_t  OCMS;                              /*!< SMC OCMS MODE Register                                                */
  __O  uint32_t  KEY1;                              /*!< SMC OCMS KEY1 Register                                                */
  __O  uint32_t  KEY2;                              /*!< SMC OCMS KEY2 Register                                                */
  __I  uint32_t  RESERVED1[22];
  __IO uint32_t  WPMR;                              /*!< SMC Write Protect Mode Register                                       */
  __I  uint32_t  WPSR;                              /*!< SMC Write Protect Status Register                                     */
} SMC_Type;


/* ================================================================================ */
/* ================                      UART1                     ================ */
/* ================================================================================ */


/**
  * @brief Universal Asynchronous Receiver Transmitter 1 (UART1)
  */

typedef struct {                                    /*!< UART1 Structure                                                       */
  __O  uint32_t  CR;                                /*!< Control Register                                                      */
  __IO uint32_t  MR;                                /*!< Mode Register                                                         */
  __O  uint32_t  IER;                               /*!< Interrupt Enable Register                                             */
  __O  uint32_t  IDR;                               /*!< Interrupt Disable Register                                            */
  __I  uint32_t  IMR;                               /*!< Interrupt Mask Register                                               */
  __I  uint32_t  SR;                                /*!< Status Register                                                       */
  __I  uint32_t  RHR;                               /*!< Receive Holding Register                                              */
  __O  uint32_t  THR;                               /*!< Transmit Holding Register                                             */
  __IO uint32_t  BRGR;                              /*!< Baud Rate Generator Register                                          */
  __I  uint32_t  RESERVED[55];
  __IO uint32_t  RPR;                               /*!< Receive Pointer Register                                              */
  __IO uint32_t  RCR;                               /*!< Receive Counter Register                                              */
  __IO uint32_t  TPR;                               /*!< Transmit Pointer Register                                             */
  __IO uint32_t  TCR;                               /*!< Transmit Counter Register                                             */
  __IO uint32_t  RNPR;                              /*!< Receive Next Pointer Register                                         */
  __IO uint32_t  RNCR;                              /*!< Receive Next Counter Register                                         */
  __IO uint32_t  TNPR;                              /*!< Transmit Next Pointer Register                                        */
  __IO uint32_t  TNCR;                              /*!< Transmit Next Counter Register                                        */
  __O  uint32_t  PTCR;                              /*!< Transfer Control Register                                             */
  __I  uint32_t  PTSR;                              /*!< Transfer Status Register                                              */
} UART1_Type;


/* ================================================================================ */
/* ================                      HSMCI                     ================ */
/* ================================================================================ */


/**
  * @brief High Speed MultiMedia Card Interface (HSMCI)
  */

typedef struct {                                    /*!< HSMCI Structure                                                       */
  __O  uint32_t  CR;                                /*!< Control Register                                                      */
  __IO uint32_t  MR;                                /*!< Mode Register                                                         */
  __IO uint32_t  DTOR;                              /*!< Data Timeout Register                                                 */
  __IO uint32_t  SDCR;                              /*!< SD/SDIO Card Register                                                 */
  __IO uint32_t  ARGR;                              /*!< Argument Register                                                     */
  __O  uint32_t  CMDR;                              /*!< Command Register                                                      */
  __IO uint32_t  BLKR;                              /*!< Block Register                                                        */
  __IO uint32_t  CSTOR;                             /*!< Completion Signal Timeout Register                                    */
  __I  uint32_t  RSPR0;                             /*!< Response Register                                                     */
  __I  uint32_t  RSPR1;                             /*!< Response Register                                                     */
  __I  uint32_t  RSPR2;                             /*!< Response Register                                                     */
  __I  uint32_t  RSPR3;                             /*!< Response Register                                                     */
  __I  uint32_t  RDR;                               /*!< Receive Data Register                                                 */
  __O  uint32_t  TDR;                               /*!< Transmit Data Register                                                */
  __I  uint32_t  RESERVED[2];
  __I  uint32_t  SR;                                /*!< Status Register                                                       */
  __O  uint32_t  IER;                               /*!< Interrupt Enable Register                                             */
  __O  uint32_t  IDR;                               /*!< Interrupt Disable Register                                            */
  __I  uint32_t  IMR;                               /*!< Interrupt Mask Register                                               */
  __I  uint32_t  RESERVED1;
  __IO uint32_t  CFG;                               /*!< Configuration Register                                                */
  __I  uint32_t  RESERVED2[35];
  __IO uint32_t  WPMR;                              /*!< Write Protection Mode Register                                        */
  __I  uint32_t  WPSR;                              /*!< Write Protection Status Register                                      */
  __I  uint32_t  RESERVED3[5];
  __IO uint32_t  RPR;                               /*!< Receive Pointer Register                                              */
  __IO uint32_t  RCR;                               /*!< Receive Counter Register                                              */
  __IO uint32_t  TPR;                               /*!< Transmit Pointer Register                                             */
  __IO uint32_t  TCR;                               /*!< Transmit Counter Register                                             */
  __IO uint32_t  RNPR;                              /*!< Receive Next Pointer Register                                         */
  __IO uint32_t  RNCR;                              /*!< Receive Next Counter Register                                         */
  __IO uint32_t  TNPR;                              /*!< Transmit Next Pointer Register                                        */
  __IO uint32_t  TNCR;                              /*!< Transmit Next Counter Register                                        */
  __O  uint32_t  PTCR;                              /*!< Transfer Control Register                                             */
  __I  uint32_t  PTSR;                              /*!< Transfer Status Register                                              */
  __I  uint32_t  RESERVED4[54];
  __IO uint32_t  FIFO0;                             /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO1;                             /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO2;                             /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO3;                             /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO4;                             /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO5;                             /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO6;                             /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO7;                             /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO8;                             /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO9;                             /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO10;                            /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO11;                            /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO12;                            /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO13;                            /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO14;                            /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO15;                            /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO16;                            /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO17;                            /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO18;                            /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO19;                            /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO20;                            /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO21;                            /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO22;                            /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO23;                            /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO24;                            /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO25;                            /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO26;                            /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO27;                            /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO28;                            /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO29;                            /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO30;                            /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO31;                            /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO32;                            /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO33;                            /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO34;                            /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO35;                            /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO36;                            /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO37;                            /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO38;                            /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO39;                            /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO40;                            /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO41;                            /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO42;                            /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO43;                            /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO44;                            /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO45;                            /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO46;                            /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO47;                            /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO48;                            /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO49;                            /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO50;                            /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO51;                            /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO52;                            /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO53;                            /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO54;                            /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO55;                            /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO56;                            /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO57;                            /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO58;                            /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO59;                            /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO60;                            /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO61;                            /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO62;                            /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO63;                            /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO64;                            /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO65;                            /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO66;                            /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO67;                            /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO68;                            /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO69;                            /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO70;                            /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO71;                            /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO72;                            /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO73;                            /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO74;                            /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO75;                            /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO76;                            /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO77;                            /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO78;                            /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO79;                            /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO80;                            /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO81;                            /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO82;                            /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO83;                            /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO84;                            /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO85;                            /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO86;                            /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO87;                            /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO88;                            /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO89;                            /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO90;                            /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO91;                            /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO92;                            /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO93;                            /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO94;                            /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO95;                            /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO96;                            /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO97;                            /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO98;                            /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO99;                            /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO100;                           /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO101;                           /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO102;                           /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO103;                           /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO104;                           /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO105;                           /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO106;                           /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO107;                           /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO108;                           /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO109;                           /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO110;                           /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO111;                           /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO112;                           /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO113;                           /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO114;                           /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO115;                           /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO116;                           /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO117;                           /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO118;                           /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO119;                           /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO120;                           /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO121;                           /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO122;                           /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO123;                           /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO124;                           /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO125;                           /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO126;                           /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO127;                           /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO128;                           /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO129;                           /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO130;                           /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO131;                           /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO132;                           /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO133;                           /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO134;                           /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO135;                           /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO136;                           /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO137;                           /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO138;                           /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO139;                           /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO140;                           /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO141;                           /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO142;                           /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO143;                           /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO144;                           /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO145;                           /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO146;                           /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO147;                           /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO148;                           /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO149;                           /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO150;                           /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO151;                           /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO152;                           /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO153;                           /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO154;                           /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO155;                           /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO156;                           /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO157;                           /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO158;                           /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO159;                           /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO160;                           /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO161;                           /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO162;                           /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO163;                           /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO164;                           /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO165;                           /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO166;                           /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO167;                           /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO168;                           /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO169;                           /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO170;                           /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO171;                           /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO172;                           /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO173;                           /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO174;                           /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO175;                           /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO176;                           /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO177;                           /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO178;                           /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO179;                           /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO180;                           /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO181;                           /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO182;                           /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO183;                           /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO184;                           /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO185;                           /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO186;                           /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO187;                           /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO188;                           /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO189;                           /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO190;                           /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO191;                           /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO192;                           /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO193;                           /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO194;                           /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO195;                           /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO196;                           /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO197;                           /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO198;                           /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO199;                           /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO200;                           /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO201;                           /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO202;                           /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO203;                           /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO204;                           /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO205;                           /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO206;                           /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO207;                           /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO208;                           /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO209;                           /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO210;                           /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO211;                           /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO212;                           /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO213;                           /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO214;                           /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO215;                           /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO216;                           /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO217;                           /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO218;                           /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO219;                           /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO220;                           /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO221;                           /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO222;                           /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO223;                           /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO224;                           /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO225;                           /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO226;                           /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO227;                           /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO228;                           /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO229;                           /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO230;                           /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO231;                           /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO232;                           /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO233;                           /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO234;                           /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO235;                           /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO236;                           /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO237;                           /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO238;                           /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO239;                           /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO240;                           /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO241;                           /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO242;                           /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO243;                           /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO244;                           /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO245;                           /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO246;                           /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO247;                           /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO248;                           /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO249;                           /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO250;                           /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO251;                           /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO252;                           /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO253;                           /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO254;                           /*!< FIFO Memory Aperture0                                                 */
  __IO uint32_t  FIFO255;                           /*!< FIFO Memory Aperture0                                                 */
} HSMCI_Type;


/* ================================================================================ */
/* ================                       UDP                      ================ */
/* ================================================================================ */


/**
  * @brief USB Device Port (UDP)
  */

typedef struct {                                    /*!< UDP Structure                                                         */
  __I  uint32_t  FRM_NUM;                           /*!< Frame Number Register                                                 */
  __IO uint32_t  GLB_STAT;                          /*!< Global State Register                                                 */
  __IO uint32_t  FADDR;                             /*!< Function Address Register                                             */
  __I  uint32_t  RESERVED;
  __O  uint32_t  IER;                               /*!< Interrupt Enable Register                                             */
  __O  uint32_t  IDR;                               /*!< Interrupt Disable Register                                            */
  __I  uint32_t  IMR;                               /*!< Interrupt Mask Register                                               */
  __I  uint32_t  ISR;                               /*!< Interrupt Status Register                                             */
  __O  uint32_t  ICR;                               /*!< Interrupt Clear Register                                              */
  __I  uint32_t  RESERVED1;
  __IO uint32_t  RST_EP;                            /*!< Reset Endpoint Register                                               */
  __I  uint32_t  RESERVED2;
  
  union {
    __IO uint32_t  CSR0_ISOENDPT_ISOENDPT;          /*!< Endpoint Control and Status Register                                  */
    __IO uint32_t  CSR0;                            /*!< Endpoint Control and Status Register                                  */
  };
  __IO uint32_t  CSR1;                              /*!< Endpoint Control and Status Register                                  */
  __IO uint32_t  CSR2;                              /*!< Endpoint Control and Status Register                                  */
  __IO uint32_t  CSR3;                              /*!< Endpoint Control and Status Register                                  */
  __IO uint32_t  CSR4;                              /*!< Endpoint Control and Status Register                                  */
  __IO uint32_t  CSR5;                              /*!< Endpoint Control and Status Register                                  */
  __IO uint32_t  CSR6;                              /*!< Endpoint Control and Status Register                                  */
  __IO uint32_t  CSR7;                              /*!< Endpoint Control and Status Register                                  */
  __IO uint32_t  FDR0;                              /*!< Endpoint FIFO Data Register                                           */
  __IO uint32_t  FDR1;                              /*!< Endpoint FIFO Data Register                                           */
  __IO uint32_t  FDR2;                              /*!< Endpoint FIFO Data Register                                           */
  __IO uint32_t  FDR3;                              /*!< Endpoint FIFO Data Register                                           */
  __IO uint32_t  FDR4;                              /*!< Endpoint FIFO Data Register                                           */
  __IO uint32_t  FDR5;                              /*!< Endpoint FIFO Data Register                                           */
  __IO uint32_t  FDR6;                              /*!< Endpoint FIFO Data Register                                           */
  __IO uint32_t  FDR7;                              /*!< Endpoint FIFO Data Register                                           */
  __I  uint32_t  RESERVED3;
  __IO uint32_t  TXVC;                              /*!< Transceiver Control Register                                          */
} UDP_Type;


/* ================================================================================ */
/* ================                       SPI                      ================ */
/* ================================================================================ */


/**
  * @brief Serial Peripheral Interface (SPI)
  */

typedef struct {                                    /*!< SPI Structure                                                         */
  __O  uint32_t  CR;                                /*!< Control Register                                                      */
  __IO uint32_t  MR;                                /*!< Mode Register                                                         */
  __I  uint32_t  RDR;                               /*!< Receive Data Register                                                 */
  __O  uint32_t  TDR;                               /*!< Transmit Data Register                                                */
  __I  uint32_t  SR;                                /*!< Status Register                                                       */
  __O  uint32_t  IER;                               /*!< Interrupt Enable Register                                             */
  __O  uint32_t  IDR;                               /*!< Interrupt Disable Register                                            */
  __I  uint32_t  IMR;                               /*!< Interrupt Mask Register                                               */
  __I  uint32_t  RESERVED[4];
  __IO uint32_t  CSR0;                              /*!< Chip Select Register                                                  */
  __IO uint32_t  CSR1;                              /*!< Chip Select Register                                                  */
  __IO uint32_t  CSR2;                              /*!< Chip Select Register                                                  */
  __IO uint32_t  CSR3;                              /*!< Chip Select Register                                                  */
  __I  uint32_t  RESERVED1[41];
  __IO uint32_t  WPMR;                              /*!< Write Protection Control Register                                     */
  __I  uint32_t  WPSR;                              /*!< Write Protection Status Register                                      */
  __I  uint32_t  RESERVED2[5];
  __IO uint32_t  RPR;                               /*!< Receive Pointer Register                                              */
  __IO uint32_t  RCR;                               /*!< Receive Counter Register                                              */
  __IO uint32_t  TPR;                               /*!< Transmit Pointer Register                                             */
  __IO uint32_t  TCR;                               /*!< Transmit Counter Register                                             */
  __IO uint32_t  RNPR;                              /*!< Receive Next Pointer Register                                         */
  __IO uint32_t  RNCR;                              /*!< Receive Next Counter Register                                         */
  __IO uint32_t  TNPR;                              /*!< Transmit Next Pointer Register                                        */
  __IO uint32_t  TNCR;                              /*!< Transmit Next Counter Register                                        */
  __O  uint32_t  PTCR;                              /*!< Transfer Control Register                                             */
  __I  uint32_t  PTSR;                              /*!< Transfer Status Register                                              */
} SPI_Type;


/* ================================================================================ */
/* ================                       TC0                      ================ */
/* ================================================================================ */


/**
  * @brief Timer Counter 0 (TC0)
  */

typedef struct {                                    /*!< TC0 Structure                                                         */
  __O  uint32_t  CCR0;                              /*!< Channel Control Register (channel = 0)                                */
  
  union {
    __IO uint32_t  CMR0_WAVE_EQ_1_WAVE_EQ_1;        /*!< Channel Mode Register (channel = 0)                                   */
    __IO uint32_t  CMR0;                            /*!< Channel Mode Register (channel = 0)                                   */
  };
  __IO uint32_t  SMMR0;                             /*!< Stepper Motor Mode Register (channel = 0)                             */
  __I  uint32_t  RAB0;                              /*!< Register AB (channel = 0)                                             */
  __I  uint32_t  CV0;                               /*!< Counter Value (channel = 0)                                           */
  __IO uint32_t  RA0;                               /*!< Register A (channel = 0)                                              */
  __IO uint32_t  RB0;                               /*!< Register B (channel = 0)                                              */
  __IO uint32_t  RC0;                               /*!< Register C (channel = 0)                                              */
  __I  uint32_t  SR0;                               /*!< Status Register (channel = 0)                                         */
  __O  uint32_t  IER0;                              /*!< Interrupt Enable Register (channel = 0)                               */
  __O  uint32_t  IDR0;                              /*!< Interrupt Disable Register (channel = 0)                              */
  __I  uint32_t  IMR0;                              /*!< Interrupt Mask Register (channel = 0)                                 */
  __IO uint32_t  EMR0;                              /*!< Extended Mode Register (channel = 0)                                  */
  __I  uint32_t  RESERVED[3];
  __O  uint32_t  CCR1;                              /*!< Channel Control Register (channel = 1)                                */
  
  union {
    __IO uint32_t  CMR1_WAVE_EQ_1_WAVE_EQ_1;        /*!< Channel Mode Register (channel = 1)                                   */
    __IO uint32_t  CMR1;                            /*!< Channel Mode Register (channel = 1)                                   */
  };
  __IO uint32_t  SMMR1;                             /*!< Stepper Motor Mode Register (channel = 1)                             */
  __I  uint32_t  RAB1;                              /*!< Register AB (channel = 1)                                             */
  __I  uint32_t  CV1;                               /*!< Counter Value (channel = 1)                                           */
  __IO uint32_t  RA1;                               /*!< Register A (channel = 1)                                              */
  __IO uint32_t  RB1;                               /*!< Register B (channel = 1)                                              */
  __IO uint32_t  RC1;                               /*!< Register C (channel = 1)                                              */
  __I  uint32_t  SR1;                               /*!< Status Register (channel = 1)                                         */
  __O  uint32_t  IER1;                              /*!< Interrupt Enable Register (channel = 1)                               */
  __O  uint32_t  IDR1;                              /*!< Interrupt Disable Register (channel = 1)                              */
  __I  uint32_t  IMR1;                              /*!< Interrupt Mask Register (channel = 1)                                 */
  __IO uint32_t  EMR1;                              /*!< Extended Mode Register (channel = 1)                                  */
  __I  uint32_t  RESERVED1[3];
  __O  uint32_t  CCR2;                              /*!< Channel Control Register (channel = 2)                                */
  
  union {
    __IO uint32_t  CMR2_WAVE_EQ_1_WAVE_EQ_1;        /*!< Channel Mode Register (channel = 2)                                   */
    __IO uint32_t  CMR2;                            /*!< Channel Mode Register (channel = 2)                                   */
  };
  __IO uint32_t  SMMR2;                             /*!< Stepper Motor Mode Register (channel = 2)                             */
  __I  uint32_t  RAB2;                              /*!< Register AB (channel = 2)                                             */
  __I  uint32_t  CV2;                               /*!< Counter Value (channel = 2)                                           */
  __IO uint32_t  RA2;                               /*!< Register A (channel = 2)                                              */
  __IO uint32_t  RB2;                               /*!< Register B (channel = 2)                                              */
  __IO uint32_t  RC2;                               /*!< Register C (channel = 2)                                              */
  __I  uint32_t  SR2;                               /*!< Status Register (channel = 2)                                         */
  __O  uint32_t  IER2;                              /*!< Interrupt Enable Register (channel = 2)                               */
  __O  uint32_t  IDR2;                              /*!< Interrupt Disable Register (channel = 2)                              */
  __I  uint32_t  IMR2;                              /*!< Interrupt Mask Register (channel = 2)                                 */
  __IO uint32_t  EMR2;                              /*!< Extended Mode Register (channel = 2)                                  */
  __I  uint32_t  RESERVED2[3];
  __O  uint32_t  BCR;                               /*!< Block Control Register                                                */
  __IO uint32_t  BMR;                               /*!< Block Mode Register                                                   */
  __O  uint32_t  QIER;                              /*!< QDEC Interrupt Enable Register                                        */
  __O  uint32_t  QIDR;                              /*!< QDEC Interrupt Disable Register                                       */
  __I  uint32_t  QIMR;                              /*!< QDEC Interrupt Mask Register                                          */
  __I  uint32_t  QISR;                              /*!< QDEC Interrupt Status Register                                        */
  __IO uint32_t  FMR;                               /*!< Fault Mode Register                                                   */
  __I  uint32_t  RESERVED3[2];
  __IO uint32_t  WPMR;                              /*!< Write Protect Mode Register                                           */
  __I  uint32_t  RESERVED4[6];
  __IO uint32_t  RPR0;                              /*!< Receive Pointer Register (pdc = 0)                                    */
  __IO uint32_t  RCR0;                              /*!< Receive Counter Register (pdc = 0)                                    */
  __I  uint32_t  RESERVED5[2];
  __IO uint32_t  RNPR0;                             /*!< Receive Next Pointer Register (pdc = 0)                               */
  __IO uint32_t  RNCR0;                             /*!< Receive Next Counter Register (pdc = 0)                               */
  __I  uint32_t  RESERVED6[2];
  __O  uint32_t  PTCR0;                             /*!< Transfer Control Register (pdc = 0)                                   */
  __I  uint32_t  PTSR0;                             /*!< Transfer Status Register (pdc = 0)                                    */
  __I  uint32_t  RESERVED7[6];
  __IO uint32_t  RPR1;                              /*!< Receive Pointer Register (pdc = 1)                                    */
  __IO uint32_t  RCR1;                              /*!< Receive Counter Register (pdc = 1)                                    */
  __I  uint32_t  RESERVED8[2];
  __IO uint32_t  RNPR1;                             /*!< Receive Next Pointer Register (pdc = 1)                               */
  __IO uint32_t  RNCR1;                             /*!< Receive Next Counter Register (pdc = 1)                               */
  __I  uint32_t  RESERVED9[2];
  __O  uint32_t  PTCR1;                             /*!< Transfer Control Register (pdc = 1)                                   */
  __I  uint32_t  PTSR1;                             /*!< Transfer Status Register (pdc = 1)                                    */
  __I  uint32_t  RESERVED10[6];
  __IO uint32_t  RPR2;                              /*!< Receive Pointer Register (pdc = 2)                                    */
  __IO uint32_t  RCR2;                              /*!< Receive Counter Register (pdc = 2)                                    */
  __I  uint32_t  RESERVED11[2];
  __IO uint32_t  RNPR2;                             /*!< Receive Next Pointer Register (pdc = 2)                               */
  __IO uint32_t  RNCR2;                             /*!< Receive Next Counter Register (pdc = 2)                               */
  __I  uint32_t  RESERVED12[2];
  __O  uint32_t  PTCR2;                             /*!< Transfer Control Register (pdc = 2)                                   */
  __I  uint32_t  PTSR2;                             /*!< Transfer Status Register (pdc = 2)                                    */
} TC0_Type;


/* ================================================================================ */
/* ================                       TC1                      ================ */
/* ================================================================================ */


/**
  * @brief Timer Counter 1 (TC1)
  */

typedef struct {                                    /*!< TC1 Structure                                                         */
  __O  uint32_t  CCR0;                              /*!< Channel Control Register (channel = 0)                                */
  
  union {
    __IO uint32_t  CMR0_WAVE_EQ_1_WAVE_EQ_1;        /*!< Channel Mode Register (channel = 0)                                   */
    __IO uint32_t  CMR0;                            /*!< Channel Mode Register (channel = 0)                                   */
  };
  __IO uint32_t  SMMR0;                             /*!< Stepper Motor Mode Register (channel = 0)                             */
  __I  uint32_t  RAB0;                              /*!< Register AB (channel = 0)                                             */
  __I  uint32_t  CV0;                               /*!< Counter Value (channel = 0)                                           */
  __IO uint32_t  RA0;                               /*!< Register A (channel = 0)                                              */
  __IO uint32_t  RB0;                               /*!< Register B (channel = 0)                                              */
  __IO uint32_t  RC0;                               /*!< Register C (channel = 0)                                              */
  __I  uint32_t  SR0;                               /*!< Status Register (channel = 0)                                         */
  __O  uint32_t  IER0;                              /*!< Interrupt Enable Register (channel = 0)                               */
  __O  uint32_t  IDR0;                              /*!< Interrupt Disable Register (channel = 0)                              */
  __I  uint32_t  IMR0;                              /*!< Interrupt Mask Register (channel = 0)                                 */
  __IO uint32_t  EMR0;                              /*!< Extended Mode Register (channel = 0)                                  */
  __I  uint32_t  RESERVED[3];
  __O  uint32_t  CCR1;                              /*!< Channel Control Register (channel = 1)                                */
  
  union {
    __IO uint32_t  CMR1_WAVE_EQ_1_WAVE_EQ_1;        /*!< Channel Mode Register (channel = 1)                                   */
    __IO uint32_t  CMR1;                            /*!< Channel Mode Register (channel = 1)                                   */
  };
  __IO uint32_t  SMMR1;                             /*!< Stepper Motor Mode Register (channel = 1)                             */
  __I  uint32_t  RAB1;                              /*!< Register AB (channel = 1)                                             */
  __I  uint32_t  CV1;                               /*!< Counter Value (channel = 1)                                           */
  __IO uint32_t  RA1;                               /*!< Register A (channel = 1)                                              */
  __IO uint32_t  RB1;                               /*!< Register B (channel = 1)                                              */
  __IO uint32_t  RC1;                               /*!< Register C (channel = 1)                                              */
  __I  uint32_t  SR1;                               /*!< Status Register (channel = 1)                                         */
  __O  uint32_t  IER1;                              /*!< Interrupt Enable Register (channel = 1)                               */
  __O  uint32_t  IDR1;                              /*!< Interrupt Disable Register (channel = 1)                              */
  __I  uint32_t  IMR1;                              /*!< Interrupt Mask Register (channel = 1)                                 */
  __IO uint32_t  EMR1;                              /*!< Extended Mode Register (channel = 1)                                  */
  __I  uint32_t  RESERVED1[3];
  __O  uint32_t  CCR2;                              /*!< Channel Control Register (channel = 2)                                */
  
  union {
    __IO uint32_t  CMR2_WAVE_EQ_1_WAVE_EQ_1;        /*!< Channel Mode Register (channel = 2)                                   */
    __IO uint32_t  CMR2;                            /*!< Channel Mode Register (channel = 2)                                   */
  };
  __IO uint32_t  SMMR2;                             /*!< Stepper Motor Mode Register (channel = 2)                             */
  __I  uint32_t  RAB2;                              /*!< Register AB (channel = 2)                                             */
  __I  uint32_t  CV2;                               /*!< Counter Value (channel = 2)                                           */
  __IO uint32_t  RA2;                               /*!< Register A (channel = 2)                                              */
  __IO uint32_t  RB2;                               /*!< Register B (channel = 2)                                              */
  __IO uint32_t  RC2;                               /*!< Register C (channel = 2)                                              */
  __I  uint32_t  SR2;                               /*!< Status Register (channel = 2)                                         */
  __O  uint32_t  IER2;                              /*!< Interrupt Enable Register (channel = 2)                               */
  __O  uint32_t  IDR2;                              /*!< Interrupt Disable Register (channel = 2)                              */
  __I  uint32_t  IMR2;                              /*!< Interrupt Mask Register (channel = 2)                                 */
  __IO uint32_t  EMR2;                              /*!< Extended Mode Register (channel = 2)                                  */
  __I  uint32_t  RESERVED2[3];
  __O  uint32_t  BCR;                               /*!< Block Control Register                                                */
  __IO uint32_t  BMR;                               /*!< Block Mode Register                                                   */
  __O  uint32_t  QIER;                              /*!< QDEC Interrupt Enable Register                                        */
  __O  uint32_t  QIDR;                              /*!< QDEC Interrupt Disable Register                                       */
  __I  uint32_t  QIMR;                              /*!< QDEC Interrupt Mask Register                                          */
  __I  uint32_t  QISR;                              /*!< QDEC Interrupt Status Register                                        */
  __IO uint32_t  FMR;                               /*!< Fault Mode Register                                                   */
  __I  uint32_t  RESERVED3[2];
  __IO uint32_t  WPMR;                              /*!< Write Protect Mode Register                                           */
  __I  uint32_t  RESERVED4[6];
  __IO uint32_t  RPR0;                              /*!< Receive Pointer Register (pdc = 0)                                    */
  __IO uint32_t  RCR0;                              /*!< Receive Counter Register (pdc = 0)                                    */
  __I  uint32_t  RESERVED5[2];
  __IO uint32_t  RNPR0;                             /*!< Receive Next Pointer Register (pdc = 0)                               */
  __IO uint32_t  RNCR0;                             /*!< Receive Next Counter Register (pdc = 0)                               */
  __I  uint32_t  RESERVED6[2];
  __O  uint32_t  PTCR0;                             /*!< Transfer Control Register (pdc = 0)                                   */
  __I  uint32_t  PTSR0;                             /*!< Transfer Status Register (pdc = 0)                                    */
  __I  uint32_t  RESERVED7[6];
  __IO uint32_t  RPR1;                              /*!< Receive Pointer Register (pdc = 1)                                    */
  __IO uint32_t  RCR1;                              /*!< Receive Counter Register (pdc = 1)                                    */
  __I  uint32_t  RESERVED8[2];
  __IO uint32_t  RNPR1;                             /*!< Receive Next Pointer Register (pdc = 1)                               */
  __IO uint32_t  RNCR1;                             /*!< Receive Next Counter Register (pdc = 1)                               */
  __I  uint32_t  RESERVED9[2];
  __O  uint32_t  PTCR1;                             /*!< Transfer Control Register (pdc = 1)                                   */
  __I  uint32_t  PTSR1;                             /*!< Transfer Status Register (pdc = 1)                                    */
  __I  uint32_t  RESERVED10[6];
  __IO uint32_t  RPR2;                              /*!< Receive Pointer Register (pdc = 2)                                    */
  __IO uint32_t  RCR2;                              /*!< Receive Counter Register (pdc = 2)                                    */
  __I  uint32_t  RESERVED11[2];
  __IO uint32_t  RNPR2;                             /*!< Receive Next Pointer Register (pdc = 2)                               */
  __IO uint32_t  RNCR2;                             /*!< Receive Next Counter Register (pdc = 2)                               */
  __I  uint32_t  RESERVED12[2];
  __O  uint32_t  PTCR2;                             /*!< Transfer Control Register (pdc = 2)                                   */
  __I  uint32_t  PTSR2;                             /*!< Transfer Status Register (pdc = 2)                                    */
} TC1_Type;


/* ================================================================================ */
/* ================                       TC2                      ================ */
/* ================================================================================ */


/**
  * @brief Timer Counter 2 (TC2)
  */

typedef struct {                                    /*!< TC2 Structure                                                         */
  __O  uint32_t  CCR0;                              /*!< Channel Control Register (channel = 0)                                */
  
  union {
    __IO uint32_t  CMR0_WAVE_EQ_1_WAVE_EQ_1;        /*!< Channel Mode Register (channel = 0)                                   */
    __IO uint32_t  CMR0;                            /*!< Channel Mode Register (channel = 0)                                   */
  };
  __IO uint32_t  SMMR0;                             /*!< Stepper Motor Mode Register (channel = 0)                             */
  __I  uint32_t  RAB0;                              /*!< Register AB (channel = 0)                                             */
  __I  uint32_t  CV0;                               /*!< Counter Value (channel = 0)                                           */
  __IO uint32_t  RA0;                               /*!< Register A (channel = 0)                                              */
  __IO uint32_t  RB0;                               /*!< Register B (channel = 0)                                              */
  __IO uint32_t  RC0;                               /*!< Register C (channel = 0)                                              */
  __I  uint32_t  SR0;                               /*!< Status Register (channel = 0)                                         */
  __O  uint32_t  IER0;                              /*!< Interrupt Enable Register (channel = 0)                               */
  __O  uint32_t  IDR0;                              /*!< Interrupt Disable Register (channel = 0)                              */
  __I  uint32_t  IMR0;                              /*!< Interrupt Mask Register (channel = 0)                                 */
  __IO uint32_t  EMR0;                              /*!< Extended Mode Register (channel = 0)                                  */
  __I  uint32_t  RESERVED[3];
  __O  uint32_t  CCR1;                              /*!< Channel Control Register (channel = 1)                                */
  
  union {
    __IO uint32_t  CMR1_WAVE_EQ_1_WAVE_EQ_1;        /*!< Channel Mode Register (channel = 1)                                   */
    __IO uint32_t  CMR1;                            /*!< Channel Mode Register (channel = 1)                                   */
  };
  __IO uint32_t  SMMR1;                             /*!< Stepper Motor Mode Register (channel = 1)                             */
  __I  uint32_t  RAB1;                              /*!< Register AB (channel = 1)                                             */
  __I  uint32_t  CV1;                               /*!< Counter Value (channel = 1)                                           */
  __IO uint32_t  RA1;                               /*!< Register A (channel = 1)                                              */
  __IO uint32_t  RB1;                               /*!< Register B (channel = 1)                                              */
  __IO uint32_t  RC1;                               /*!< Register C (channel = 1)                                              */
  __I  uint32_t  SR1;                               /*!< Status Register (channel = 1)                                         */
  __O  uint32_t  IER1;                              /*!< Interrupt Enable Register (channel = 1)                               */
  __O  uint32_t  IDR1;                              /*!< Interrupt Disable Register (channel = 1)                              */
  __I  uint32_t  IMR1;                              /*!< Interrupt Mask Register (channel = 1)                                 */
  __IO uint32_t  EMR1;                              /*!< Extended Mode Register (channel = 1)                                  */
  __I  uint32_t  RESERVED1[3];
  __O  uint32_t  CCR2;                              /*!< Channel Control Register (channel = 2)                                */
  
  union {
    __IO uint32_t  CMR2_WAVE_EQ_1_WAVE_EQ_1;        /*!< Channel Mode Register (channel = 2)                                   */
    __IO uint32_t  CMR2;                            /*!< Channel Mode Register (channel = 2)                                   */
  };
  __IO uint32_t  SMMR2;                             /*!< Stepper Motor Mode Register (channel = 2)                             */
  __I  uint32_t  RAB2;                              /*!< Register AB (channel = 2)                                             */
  __I  uint32_t  CV2;                               /*!< Counter Value (channel = 2)                                           */
  __IO uint32_t  RA2;                               /*!< Register A (channel = 2)                                              */
  __IO uint32_t  RB2;                               /*!< Register B (channel = 2)                                              */
  __IO uint32_t  RC2;                               /*!< Register C (channel = 2)                                              */
  __I  uint32_t  SR2;                               /*!< Status Register (channel = 2)                                         */
  __O  uint32_t  IER2;                              /*!< Interrupt Enable Register (channel = 2)                               */
  __O  uint32_t  IDR2;                              /*!< Interrupt Disable Register (channel = 2)                              */
  __I  uint32_t  IMR2;                              /*!< Interrupt Mask Register (channel = 2)                                 */
  __IO uint32_t  EMR2;                              /*!< Extended Mode Register (channel = 2)                                  */
  __I  uint32_t  RESERVED2[3];
  __O  uint32_t  BCR;                               /*!< Block Control Register                                                */
  __IO uint32_t  BMR;                               /*!< Block Mode Register                                                   */
  __O  uint32_t  QIER;                              /*!< QDEC Interrupt Enable Register                                        */
  __O  uint32_t  QIDR;                              /*!< QDEC Interrupt Disable Register                                       */
  __I  uint32_t  QIMR;                              /*!< QDEC Interrupt Mask Register                                          */
  __I  uint32_t  QISR;                              /*!< QDEC Interrupt Status Register                                        */
  __IO uint32_t  FMR;                               /*!< Fault Mode Register                                                   */
  __I  uint32_t  RESERVED3[2];
  __IO uint32_t  WPMR;                              /*!< Write Protect Mode Register                                           */
} TC2_Type;


/* ================================================================================ */
/* ================                     USART0                     ================ */
/* ================================================================================ */


/**
  * @brief Universal Synchronous Asynchronous Receiver Transmitter 0 (USART0)
  */

typedef struct {                                    /*!< USART0 Structure                                                      */
  
  union {
    __O  uint32_t  CR_SPI_MODE_SPI_MODE;            /*!< Control Register                                                      */
    __O  uint32_t  CR;                              /*!< Control Register                                                      */
  };
  
  union {
    __IO uint32_t  MR_SPI_MODE_SPI_MODE;            /*!< Mode Register                                                         */
    __IO uint32_t  MR;                              /*!< Mode Register                                                         */
  };
  
  union {
    __O  uint32_t  IER_SPI_MODE_SPI_MODE;           /*!< Interrupt Enable Register                                             */
    __O  uint32_t  IER;                             /*!< Interrupt Enable Register                                             */
  };
  
  union {
    __O  uint32_t  IDR_SPI_MODE_SPI_MODE;           /*!< Interrupt Disable Register                                            */
    __O  uint32_t  IDR;                             /*!< Interrupt Disable Register                                            */
  };
  
  union {
    __I  uint32_t  IMR_SPI_MODE_SPI_MODE;           /*!< Interrupt Mask Register                                               */
    __I  uint32_t  IMR;                             /*!< Interrupt Mask Register                                               */
  };
  
  union {
    __I  uint32_t  CSR_SPI_MODE_SPI_MODE;           /*!< Channel Status Register                                               */
    __I  uint32_t  CSR;                             /*!< Channel Status Register                                               */
  };
  __I  uint32_t  RHR;                               /*!< Receiver Holding Register                                             */
  __O  uint32_t  THR;                               /*!< Transmitter Holding Register                                          */
  __IO uint32_t  BRGR;                              /*!< Baud Rate Generator Register                                          */
  __IO uint32_t  RTOR;                              /*!< Receiver Time-out Register                                            */
  __IO uint32_t  TTGR;                              /*!< Transmitter Timeguard Register                                        */
  __I  uint32_t  RESERVED[5];
  __IO uint32_t  FIDI;                              /*!< FI DI Ratio Register                                                  */
  __I  uint32_t  NER;                               /*!< Number of Errors Register                                             */
  __I  uint32_t  RESERVED1;
  __IO uint32_t  IF;                                /*!< IrDA Filter Register                                                  */
  __IO uint32_t  MAN;                               /*!< Manchester Encoder Decoder Register                                   */
  __I  uint32_t  RESERVED2[36];
  __IO uint32_t  WPMR;                              /*!< Write Protect Mode Register                                           */
  __I  uint32_t  WPSR;                              /*!< Write Protect Status Register                                         */
  __I  uint32_t  RESERVED3[5];
  __IO uint32_t  RPR;                               /*!< Receive Pointer Register                                              */
  __IO uint32_t  RCR;                               /*!< Receive Counter Register                                              */
  __IO uint32_t  TPR;                               /*!< Transmit Pointer Register                                             */
  __IO uint32_t  TCR;                               /*!< Transmit Counter Register                                             */
  __IO uint32_t  RNPR;                              /*!< Receive Next Pointer Register                                         */
  __IO uint32_t  RNCR;                              /*!< Receive Next Counter Register                                         */
  __IO uint32_t  TNPR;                              /*!< Transmit Next Pointer Register                                        */
  __IO uint32_t  TNCR;                              /*!< Transmit Next Counter Register                                        */
  __O  uint32_t  PTCR;                              /*!< Transfer Control Register                                             */
  __I  uint32_t  PTSR;                              /*!< Transfer Status Register                                              */
} USART0_Type;


/* ================================================================================ */
/* ================                     USART1                     ================ */
/* ================================================================================ */


/**
  * @brief Universal Synchronous Asynchronous Receiver Transmitter 1 (USART1)
  */

typedef struct {                                    /*!< USART1 Structure                                                      */
  
  union {
    __O  uint32_t  CR_SPI_MODE_SPI_MODE;            /*!< Control Register                                                      */
    __O  uint32_t  CR;                              /*!< Control Register                                                      */
  };
  
  union {
    __IO uint32_t  MR_SPI_MODE_SPI_MODE;            /*!< Mode Register                                                         */
    __IO uint32_t  MR;                              /*!< Mode Register                                                         */
  };
  
  union {
    __O  uint32_t  IER_SPI_MODE_SPI_MODE;           /*!< Interrupt Enable Register                                             */
    __O  uint32_t  IER;                             /*!< Interrupt Enable Register                                             */
  };
  
  union {
    __O  uint32_t  IDR_SPI_MODE_SPI_MODE;           /*!< Interrupt Disable Register                                            */
    __O  uint32_t  IDR;                             /*!< Interrupt Disable Register                                            */
  };
  
  union {
    __I  uint32_t  IMR_SPI_MODE_SPI_MODE;           /*!< Interrupt Mask Register                                               */
    __I  uint32_t  IMR;                             /*!< Interrupt Mask Register                                               */
  };
  
  union {
    __I  uint32_t  CSR_SPI_MODE_SPI_MODE;           /*!< Channel Status Register                                               */
    __I  uint32_t  CSR;                             /*!< Channel Status Register                                               */
  };
  __I  uint32_t  RHR;                               /*!< Receiver Holding Register                                             */
  __O  uint32_t  THR;                               /*!< Transmitter Holding Register                                          */
  __IO uint32_t  BRGR;                              /*!< Baud Rate Generator Register                                          */
  __IO uint32_t  RTOR;                              /*!< Receiver Time-out Register                                            */
  __IO uint32_t  TTGR;                              /*!< Transmitter Timeguard Register                                        */
  __I  uint32_t  RESERVED[5];
  __IO uint32_t  FIDI;                              /*!< FI DI Ratio Register                                                  */
  __I  uint32_t  NER;                               /*!< Number of Errors Register                                             */
  __I  uint32_t  RESERVED1;
  __IO uint32_t  IF;                                /*!< IrDA Filter Register                                                  */
  __IO uint32_t  MAN;                               /*!< Manchester Encoder Decoder Register                                   */
  __I  uint32_t  RESERVED2[36];
  __IO uint32_t  WPMR;                              /*!< Write Protect Mode Register                                           */
  __I  uint32_t  WPSR;                              /*!< Write Protect Status Register                                         */
  __I  uint32_t  RESERVED3[5];
  __IO uint32_t  RPR;                               /*!< Receive Pointer Register                                              */
  __IO uint32_t  RCR;                               /*!< Receive Counter Register                                              */
  __IO uint32_t  TPR;                               /*!< Transmit Pointer Register                                             */
  __IO uint32_t  TCR;                               /*!< Transmit Counter Register                                             */
  __IO uint32_t  RNPR;                              /*!< Receive Next Pointer Register                                         */
  __IO uint32_t  RNCR;                              /*!< Receive Next Counter Register                                         */
  __IO uint32_t  TNPR;                              /*!< Transmit Next Pointer Register                                        */
  __IO uint32_t  TNCR;                              /*!< Transmit Next Counter Register                                        */
  __O  uint32_t  PTCR;                              /*!< Transfer Control Register                                             */
  __I  uint32_t  PTSR;                              /*!< Transfer Status Register                                              */
} USART1_Type;


/* ================================================================================ */
/* ================                      TWI0                      ================ */
/* ================================================================================ */


/**
  * @brief Two-wire Interface 0 (TWI0)
  */

typedef struct {                                    /*!< TWI0 Structure                                                        */
  __O  uint32_t  CR;                                /*!< Control Register                                                      */
  __IO uint32_t  MMR;                               /*!< Master Mode Register                                                  */
  __IO uint32_t  SMR;                               /*!< Slave Mode Register                                                   */
  __IO uint32_t  IADR;                              /*!< Internal Address Register                                             */
  __IO uint32_t  CWGR;                              /*!< Clock Waveform Generator Register                                     */
  __I  uint32_t  RESERVED[3];
  __I  uint32_t  SR;                                /*!< Status Register                                                       */
  __O  uint32_t  IER;                               /*!< Interrupt Enable Register                                             */
  __O  uint32_t  IDR;                               /*!< Interrupt Disable Register                                            */
  __I  uint32_t  IMR;                               /*!< Interrupt Mask Register                                               */
  __I  uint32_t  RHR;                               /*!< Receive Holding Register                                              */
  __O  uint32_t  THR;                               /*!< Transmit Holding Register                                             */
  __I  uint32_t  RESERVED1[43];
  __IO uint32_t  WPROT_MODE;                        /*!< Protection Mode Register                                              */
  __I  uint32_t  WPROT_STATUS;                      /*!< Protection Status Register                                            */
  __I  uint32_t  RESERVED2[5];
  __IO uint32_t  RPR;                               /*!< Receive Pointer Register                                              */
  __IO uint32_t  RCR;                               /*!< Receive Counter Register                                              */
  __IO uint32_t  TPR;                               /*!< Transmit Pointer Register                                             */
  __IO uint32_t  TCR;                               /*!< Transmit Counter Register                                             */
  __IO uint32_t  RNPR;                              /*!< Receive Next Pointer Register                                         */
  __IO uint32_t  RNCR;                              /*!< Receive Next Counter Register                                         */
  __IO uint32_t  TNPR;                              /*!< Transmit Next Pointer Register                                        */
  __IO uint32_t  TNCR;                              /*!< Transmit Next Counter Register                                        */
  __O  uint32_t  PTCR;                              /*!< Transfer Control Register                                             */
  __I  uint32_t  PTSR;                              /*!< Transfer Status Register                                              */
} TWI0_Type;


/* ================================================================================ */
/* ================                      TWI1                      ================ */
/* ================================================================================ */


/**
  * @brief Two-wire Interface 1 (TWI1)
  */

typedef struct {                                    /*!< TWI1 Structure                                                        */
  __O  uint32_t  CR;                                /*!< Control Register                                                      */
  __IO uint32_t  MMR;                               /*!< Master Mode Register                                                  */
  __IO uint32_t  SMR;                               /*!< Slave Mode Register                                                   */
  __IO uint32_t  IADR;                              /*!< Internal Address Register                                             */
  __IO uint32_t  CWGR;                              /*!< Clock Waveform Generator Register                                     */
  __I  uint32_t  RESERVED[3];
  __I  uint32_t  SR;                                /*!< Status Register                                                       */
  __O  uint32_t  IER;                               /*!< Interrupt Enable Register                                             */
  __O  uint32_t  IDR;                               /*!< Interrupt Disable Register                                            */
  __I  uint32_t  IMR;                               /*!< Interrupt Mask Register                                               */
  __I  uint32_t  RHR;                               /*!< Receive Holding Register                                              */
  __O  uint32_t  THR;                               /*!< Transmit Holding Register                                             */
  __I  uint32_t  RESERVED1[43];
  __IO uint32_t  WPROT_MODE;                        /*!< Protection Mode Register                                              */
  __I  uint32_t  WPROT_STATUS;                      /*!< Protection Status Register                                            */
  __I  uint32_t  RESERVED2[5];
  __IO uint32_t  RPR;                               /*!< Receive Pointer Register                                              */
  __IO uint32_t  RCR;                               /*!< Receive Counter Register                                              */
  __IO uint32_t  TPR;                               /*!< Transmit Pointer Register                                             */
  __IO uint32_t  TCR;                               /*!< Transmit Counter Register                                             */
  __IO uint32_t  RNPR;                              /*!< Receive Next Pointer Register                                         */
  __IO uint32_t  RNCR;                              /*!< Receive Next Counter Register                                         */
  __IO uint32_t  TNPR;                              /*!< Transmit Next Pointer Register                                        */
  __IO uint32_t  TNCR;                              /*!< Transmit Next Counter Register                                        */
  __O  uint32_t  PTCR;                              /*!< Transfer Control Register                                             */
  __I  uint32_t  PTSR;                              /*!< Transfer Status Register                                              */
} TWI1_Type;


/* ================================================================================ */
/* ================                      AFEC0                     ================ */
/* ================================================================================ */


/**
  * @brief Analog-Front-End Controller 0 (AFEC0)
  */

typedef struct {                                    /*!< AFEC0 Structure                                                       */
  __O  uint32_t  CR;                                /*!< Control Register                                                      */
  __IO uint32_t  MR;                                /*!< Mode Register                                                         */
  __IO uint32_t  EMR;                               /*!< Extended Mode Register                                                */
  __IO uint32_t  SEQ1R;                             /*!< Channel Sequence 1 Register                                           */
  __IO uint32_t  SEQ2R;                             /*!< Channel Sequence 2 Register                                           */
  __O  uint32_t  CHER;                              /*!< Channel Enable Register                                               */
  __O  uint32_t  CHDR;                              /*!< Channel Disable Register                                              */
  __I  uint32_t  CHSR;                              /*!< Channel Status Register                                               */
  __I  uint32_t  LCDR;                              /*!< Last Converted Data Register                                          */
  __O  uint32_t  IER;                               /*!< Interrupt Enable Register                                             */
  __O  uint32_t  IDR;                               /*!< Interrupt Disable Register                                            */
  __I  uint32_t  IMR;                               /*!< Interrupt Mask Register                                               */
  __I  uint32_t  ISR;                               /*!< Interrupt Status Register                                             */
  __I  uint32_t  RESERVED[6];
  __I  uint32_t  OVER;                              /*!< Overrun Status Register                                               */
  __IO uint32_t  CWR;                               /*!< Compare Window Register                                               */
  __IO uint32_t  CGR;                               /*!< Channel Gain Register                                                 */
  __I  uint32_t  RESERVED1;
  __IO uint32_t  CDOR;                              /*!< Channel DC Offset Register                                            */
  __IO uint32_t  DIFFR;                             /*!< Channel Differential Register                                         */
  __I  uint32_t  CSELR;                             /*!< Channel Register Selection                                            */
  __I  uint32_t  CDR;                               /*!< Channel Data Register                                                 */
  __I  uint32_t  COCR;                              /*!< Channel Offset Compensation Register                                  */
  __IO uint32_t  TEMPMR;                            /*!< Temperature Sensor Mode Register                                      */
  __IO uint32_t  TEMPCWR;                           /*!< Temperature Compare Window Register                                   */
  __I  uint32_t  RESERVED2[7];
  __IO uint32_t  ACR;                               /*!< Analog Control Register                                               */
  __I  uint32_t  RESERVED3[19];
  __IO uint32_t  WPMR;                              /*!< Write Protect Mode Register                                           */
  __I  uint32_t  WPSR;                              /*!< Write Protect Status Register                                         */
  __I  uint32_t  RESERVED4[5];
  __IO uint32_t  RPR;                               /*!< Receive Pointer Register                                              */
  __IO uint32_t  RCR;                               /*!< Receive Counter Register                                              */
  __I  uint32_t  RESERVED5[2];
  __IO uint32_t  RNPR;                              /*!< Receive Next Pointer Register                                         */
  __IO uint32_t  RNCR;                              /*!< Receive Next Counter Register                                         */
  __I  uint32_t  RESERVED6[2];
  __O  uint32_t  PTCR;                              /*!< Transfer Control Register                                             */
  __I  uint32_t  PTSR;                              /*!< Transfer Status Register                                              */
} AFEC0_Type;


/* ================================================================================ */
/* ================                      AFEC1                     ================ */
/* ================================================================================ */


/**
  * @brief Analog-Front-End Controller 1 (AFEC1)
  */

typedef struct {                                    /*!< AFEC1 Structure                                                       */
  __O  uint32_t  CR;                                /*!< Control Register                                                      */
  __IO uint32_t  MR;                                /*!< Mode Register                                                         */
  __IO uint32_t  EMR;                               /*!< Extended Mode Register                                                */
  __IO uint32_t  SEQ1R;                             /*!< Channel Sequence 1 Register                                           */
  __IO uint32_t  SEQ2R;                             /*!< Channel Sequence 2 Register                                           */
  __O  uint32_t  CHER;                              /*!< Channel Enable Register                                               */
  __O  uint32_t  CHDR;                              /*!< Channel Disable Register                                              */
  __I  uint32_t  CHSR;                              /*!< Channel Status Register                                               */
  __I  uint32_t  LCDR;                              /*!< Last Converted Data Register                                          */
  __O  uint32_t  IER;                               /*!< Interrupt Enable Register                                             */
  __O  uint32_t  IDR;                               /*!< Interrupt Disable Register                                            */
  __I  uint32_t  IMR;                               /*!< Interrupt Mask Register                                               */
  __I  uint32_t  ISR;                               /*!< Interrupt Status Register                                             */
  __I  uint32_t  RESERVED[6];
  __I  uint32_t  OVER;                              /*!< Overrun Status Register                                               */
  __IO uint32_t  CWR;                               /*!< Compare Window Register                                               */
  __IO uint32_t  CGR;                               /*!< Channel Gain Register                                                 */
  __I  uint32_t  RESERVED1;
  __IO uint32_t  CDOR;                              /*!< Channel DC Offset Register                                            */
  __IO uint32_t  DIFFR;                             /*!< Channel Differential Register                                         */
  __I  uint32_t  CSELR;                             /*!< Channel Register Selection                                            */
  __I  uint32_t  CDR;                               /*!< Channel Data Register                                                 */
  __I  uint32_t  COCR;                              /*!< Channel Offset Compensation Register                                  */
  __IO uint32_t  TEMPMR;                            /*!< Temperature Sensor Mode Register                                      */
  __IO uint32_t  TEMPCWR;                           /*!< Temperature Compare Window Register                                   */
  __I  uint32_t  RESERVED2[7];
  __IO uint32_t  ACR;                               /*!< Analog Control Register                                               */
  __I  uint32_t  RESERVED3[19];
  __IO uint32_t  WPMR;                              /*!< Write Protect Mode Register                                           */
  __I  uint32_t  WPSR;                              /*!< Write Protect Status Register                                         */
  __I  uint32_t  RESERVED4[5];
  __IO uint32_t  RPR;                               /*!< Receive Pointer Register                                              */
  __IO uint32_t  RCR;                               /*!< Receive Counter Register                                              */
  __I  uint32_t  RESERVED5[2];
  __IO uint32_t  RNPR;                              /*!< Receive Next Pointer Register                                         */
  __IO uint32_t  RNCR;                              /*!< Receive Next Counter Register                                         */
  __I  uint32_t  RESERVED6[2];
  __O  uint32_t  PTCR;                              /*!< Transfer Control Register                                             */
  __I  uint32_t  PTSR;                              /*!< Transfer Status Register                                              */
} AFEC1_Type;


/* ================================================================================ */
/* ================                      DACC                      ================ */
/* ================================================================================ */


/**
  * @brief Digital-to-Analog Converter Controller (DACC)
  */

typedef struct {                                    /*!< DACC Structure                                                        */
  __O  uint32_t  CR;                                /*!< Control Register                                                      */
  __IO uint32_t  MR;                                /*!< Mode Register                                                         */
  __I  uint32_t  RESERVED[2];
  __O  uint32_t  CHER;                              /*!< Channel Enable Register                                               */
  __O  uint32_t  CHDR;                              /*!< Channel Disable Register                                              */
  __I  uint32_t  CHSR;                              /*!< Channel Status Register                                               */
  __I  uint32_t  RESERVED1;
  __O  uint32_t  CDR;                               /*!< Conversion Data Register                                              */
  __O  uint32_t  IER;                               /*!< Interrupt Enable Register                                             */
  __O  uint32_t  IDR;                               /*!< Interrupt Disable Register                                            */
  __I  uint32_t  IMR;                               /*!< Interrupt Mask Register                                               */
  __I  uint32_t  ISR;                               /*!< Interrupt Status Register                                             */
  __I  uint32_t  RESERVED2[24];
  __IO uint32_t  ACR;                               /*!< Analog Current Register                                               */
  __I  uint32_t  RESERVED3[19];
  __IO uint32_t  WPMR;                              /*!< Write Protect Mode register                                           */
  __I  uint32_t  WPSR;                              /*!< Write Protect Status register                                         */
  __I  uint32_t  RESERVED4[7];
  __IO uint32_t  TPR;                               /*!< Transmit Pointer Register                                             */
  __IO uint32_t  TCR;                               /*!< Transmit Counter Register                                             */
  __I  uint32_t  RESERVED5[2];
  __IO uint32_t  TNPR;                              /*!< Transmit Next Pointer Register                                        */
  __IO uint32_t  TNCR;                              /*!< Transmit Next Counter Register                                        */
  __O  uint32_t  PTCR;                              /*!< Transfer Control Register                                             */
  __I  uint32_t  PTSR;                              /*!< Transfer Status Register                                              */
} DACC_Type;


/* ================================================================================ */
/* ================                       ACC                      ================ */
/* ================================================================================ */


/**
  * @brief Analog Comparator Controller (ACC)
  */

typedef struct {                                    /*!< ACC Structure                                                         */
  __O  uint32_t  CR;                                /*!< Control Register                                                      */
  __IO uint32_t  MR;                                /*!< Mode Register                                                         */
  __I  uint32_t  RESERVED[7];
  __O  uint32_t  IER;                               /*!< Interrupt Enable Register                                             */
  __O  uint32_t  IDR;                               /*!< Interrupt Disable Register                                            */
  __I  uint32_t  IMR;                               /*!< Interrupt Mask Register                                               */
  __I  uint32_t  ISR;                               /*!< Interrupt Status Register                                             */
  __I  uint32_t  RESERVED1[24];
  __IO uint32_t  ACR;                               /*!< Analog Control Register                                               */
  __I  uint32_t  RESERVED2[19];
  __IO uint32_t  WPMR;                              /*!< Write Protect Mode Register                                           */
  __I  uint32_t  WPSR;                              /*!< Write Protect Status Register                                         */
} ACC_Type;


/* ================================================================================ */
/* ================                      DMAC                      ================ */
/* ================================================================================ */


/**
  * @brief DMA Controller (DMAC)
  */

typedef struct {                                    /*!< DMAC Structure                                                        */
  __IO uint32_t  GCFG;                              /*!< DMAC Global Configuration Register                                    */
  __IO uint32_t  EN;                                /*!< DMAC Enable Register                                                  */
  __IO uint32_t  SREQ;                              /*!< DMAC Software Single Request Register                                 */
  __IO uint32_t  CREQ;                              /*!< DMAC Software Chunk Transfer Request Register                         */
  __IO uint32_t  LAST;                              /*!< DMAC Software Last Transfer Flag Register                             */
  __I  uint32_t  RESERVED;
  __O  uint32_t  EBCIER;                            /*!< DMAC Error, Chained Buffer Transfer Completed Interrupt and
                                                         Buffer Transfer Completed Interrupt Enable register.                  */
  __O  uint32_t  EBCIDR;                            /*!< DMAC Error, Chained Buffer Transfer Completed Interrupt and
                                                         Buffer Transfer Completed Interrupt Disable register.                 */
  __I  uint32_t  EBCIMR;                            /*!< DMAC Error, Chained Buffer Transfer Completed Interrupt and
                                                         Buffer transfer completed Mask Register.                              */
  __I  uint32_t  EBCISR;                            /*!< DMAC Error, Chained Buffer Transfer Completed Interrupt and
                                                         Buffer transfer completed Status Register.                            */
  __O  uint32_t  CHER;                              /*!< DMAC Channel Handler Enable Register                                  */
  __O  uint32_t  CHDR;                              /*!< DMAC Channel Handler Disable Register                                 */
  __I  uint32_t  CHSR;                              /*!< DMAC Channel Handler Status Register                                  */
  __I  uint32_t  RESERVED1[2];
  __IO uint32_t  SADDR0;                            /*!< DMAC Channel Source Address Register (ch_num = 0)                     */
  __IO uint32_t  DADDR0;                            /*!< DMAC Channel Destination Address Register (ch_num = 0)                */
  __IO uint32_t  DSCR0;                             /*!< DMAC Channel Descriptor Address Register (ch_num = 0)                 */
  __IO uint32_t  CTRLA0;                            /*!< DMAC Channel Control A Register (ch_num = 0)                          */
  __IO uint32_t  CTRLB0;                            /*!< DMAC Channel Control B Register (ch_num = 0)                          */
  __IO uint32_t  CFG0;                              /*!< DMAC Channel Configuration Register (ch_num = 0)                      */
  __I  uint32_t  RESERVED2[4];
  __IO uint32_t  SADDR1;                            /*!< DMAC Channel Source Address Register (ch_num = 1)                     */
  __IO uint32_t  DADDR1;                            /*!< DMAC Channel Destination Address Register (ch_num = 1)                */
  __IO uint32_t  DSCR1;                             /*!< DMAC Channel Descriptor Address Register (ch_num = 1)                 */
  __IO uint32_t  CTRLA1;                            /*!< DMAC Channel Control A Register (ch_num = 1)                          */
  __IO uint32_t  CTRLB1;                            /*!< DMAC Channel Control B Register (ch_num = 1)                          */
  __IO uint32_t  CFG1;                              /*!< DMAC Channel Configuration Register (ch_num = 1)                      */
  __I  uint32_t  RESERVED3[4];
  __IO uint32_t  SADDR2;                            /*!< DMAC Channel Source Address Register (ch_num = 2)                     */
  __IO uint32_t  DADDR2;                            /*!< DMAC Channel Destination Address Register (ch_num = 2)                */
  __IO uint32_t  DSCR2;                             /*!< DMAC Channel Descriptor Address Register (ch_num = 2)                 */
  __IO uint32_t  CTRLA2;                            /*!< DMAC Channel Control A Register (ch_num = 2)                          */
  __IO uint32_t  CTRLB2;                            /*!< DMAC Channel Control B Register (ch_num = 2)                          */
  __IO uint32_t  CFG2;                              /*!< DMAC Channel Configuration Register (ch_num = 2)                      */
  __I  uint32_t  RESERVED4[4];
  __IO uint32_t  SADDR3;                            /*!< DMAC Channel Source Address Register (ch_num = 3)                     */
  __IO uint32_t  DADDR3;                            /*!< DMAC Channel Destination Address Register (ch_num = 3)                */
  __IO uint32_t  DSCR3;                             /*!< DMAC Channel Descriptor Address Register (ch_num = 3)                 */
  __IO uint32_t  CTRLA3;                            /*!< DMAC Channel Control A Register (ch_num = 3)                          */
  __IO uint32_t  CTRLB3;                            /*!< DMAC Channel Control B Register (ch_num = 3)                          */
  __IO uint32_t  CFG3;                              /*!< DMAC Channel Configuration Register (ch_num = 3)                      */
  __I  uint32_t  RESERVED5[70];
  __IO uint32_t  WPMR;                              /*!< DMAC Write Protect Mode Register                                      */
  __I  uint32_t  WPSR;                              /*!< DMAC Write Protect Status Register                                    */
} DMAC_Type;


/* ================================================================================ */
/* ================                      CMCC                      ================ */
/* ================================================================================ */


/**
  * @brief Cortex M Cache Controller (CMCC)
  */

typedef struct {                                    /*!< CMCC Structure                                                        */
  __I  uint32_t  TYPE;                              /*!< Cache Type Register                                                   */
  __IO uint32_t  CFG;                               /*!< Cache Configuration Register                                          */
  __O  uint32_t  CTRL;                              /*!< Cache Control Register                                                */
  __I  uint32_t  SR;                                /*!< Cache Status Register                                                 */
  __I  uint32_t  RESERVED[4];
  __O  uint32_t  MAINT0;                            /*!< Cache Maintenance Register 0                                          */
  __O  uint32_t  MAINT1;                            /*!< Cache Maintenance Register 1                                          */
  __IO uint32_t  MCFG;                              /*!< Cache Monitor Configuration Register                                  */
  __IO uint32_t  MEN;                               /*!< Cache Monitor Enable Register                                         */
  __O  uint32_t  MCTRL;                             /*!< Cache Monitor Control Register                                        */
  __I  uint32_t  MSR;                               /*!< Cache Monitor Status Register                                         */
} CMCC_Type;


/* ================================================================================ */
/* ================                     MATRIX                     ================ */
/* ================================================================================ */


/**
  * @brief AHB Bus Matrix (MATRIX)
  */

typedef struct {                                    /*!< MATRIX Structure                                                      */
  __IO uint32_t  MCFG0;                             /*!< Master Configuration Register                                         */
  __IO uint32_t  MCFG1;                             /*!< Master Configuration Register                                         */
  __IO uint32_t  MCFG2;                             /*!< Master Configuration Register                                         */
  __IO uint32_t  MCFG3;                             /*!< Master Configuration Register                                         */
  __IO uint32_t  MCFG4;                             /*!< Master Configuration Register                                         */
  __IO uint32_t  MCFG5;                             /*!< Master Configuration Register                                         */
  __IO uint32_t  MCFG6;                             /*!< Master Configuration Register                                         */
  __IO uint32_t  MCFG7;                             /*!< Master Configuration Register                                         */
  __IO uint32_t  MCFG8;                             /*!< Master Configuration Register                                         */
  __IO uint32_t  MCFG9;                             /*!< Master Configuration Register                                         */
  __IO uint32_t  MCFG10;                            /*!< Master Configuration Register                                         */
  __IO uint32_t  MCFG11;                            /*!< Master Configuration Register                                         */
  __IO uint32_t  MCFG12;                            /*!< Master Configuration Register                                         */
  __IO uint32_t  MCFG13;                            /*!< Master Configuration Register                                         */
  __IO uint32_t  MCFG14;                            /*!< Master Configuration Register                                         */
  __IO uint32_t  MCFG15;                            /*!< Master Configuration Register                                         */
  __IO uint32_t  SCFG0;                             /*!< Slave Configuration Register                                          */
  __IO uint32_t  SCFG1;                             /*!< Slave Configuration Register                                          */
  __IO uint32_t  SCFG2;                             /*!< Slave Configuration Register                                          */
  __IO uint32_t  SCFG3;                             /*!< Slave Configuration Register                                          */
  __IO uint32_t  SCFG4;                             /*!< Slave Configuration Register                                          */
  __IO uint32_t  SCFG5;                             /*!< Slave Configuration Register                                          */
  __IO uint32_t  SCFG6;                             /*!< Slave Configuration Register                                          */
  __IO uint32_t  SCFG7;                             /*!< Slave Configuration Register                                          */
  __IO uint32_t  SCFG8;                             /*!< Slave Configuration Register                                          */
  __IO uint32_t  SCFG9;                             /*!< Slave Configuration Register                                          */
  __IO uint32_t  SCFG10;                            /*!< Slave Configuration Register                                          */
  __IO uint32_t  SCFG11;                            /*!< Slave Configuration Register                                          */
  __IO uint32_t  SCFG12;                            /*!< Slave Configuration Register                                          */
  __IO uint32_t  SCFG13;                            /*!< Slave Configuration Register                                          */
  __IO uint32_t  SCFG14;                            /*!< Slave Configuration Register                                          */
  __IO uint32_t  SCFG15;                            /*!< Slave Configuration Register                                          */
  __IO uint32_t  PRAS0;                             /*!< Priority Register A for Slave 0                                       */
  __IO uint32_t  PRBS0;                             /*!< Priority Register B for Slave 0                                       */
  __IO uint32_t  PRAS1;                             /*!< Priority Register A for Slave 1                                       */
  __IO uint32_t  PRBS1;                             /*!< Priority Register B for Slave 1                                       */
  __IO uint32_t  PRAS2;                             /*!< Priority Register A for Slave 2                                       */
  __IO uint32_t  PRBS2;                             /*!< Priority Register B for Slave 2                                       */
  __IO uint32_t  PRAS3;                             /*!< Priority Register A for Slave 3                                       */
  __IO uint32_t  PRBS3;                             /*!< Priority Register B for Slave 3                                       */
  __IO uint32_t  PRAS4;                             /*!< Priority Register A for Slave 4                                       */
  __IO uint32_t  PRBS4;                             /*!< Priority Register B for Slave 4                                       */
  __IO uint32_t  PRAS5;                             /*!< Priority Register A for Slave 5                                       */
  __IO uint32_t  PRBS5;                             /*!< Priority Register B for Slave 5                                       */
  __IO uint32_t  PRAS6;                             /*!< Priority Register A for Slave 6                                       */
  __IO uint32_t  PRBS6;                             /*!< Priority Register B for Slave 6                                       */
  __IO uint32_t  PRAS7;                             /*!< Priority Register A for Slave 7                                       */
  __IO uint32_t  PRBS7;                             /*!< Priority Register B for Slave 7                                       */
  __IO uint32_t  PRAS8;                             /*!< Priority Register A for Slave 8                                       */
  __IO uint32_t  PRBS8;                             /*!< Priority Register B for Slave 8                                       */
  __IO uint32_t  PRAS9;                             /*!< Priority Register A for Slave 9                                       */
  __IO uint32_t  PRBS9;                             /*!< Priority Register B for Slave 9                                       */
  __IO uint32_t  PRAS10;                            /*!< Priority Register A for Slave 10                                      */
  __IO uint32_t  PRBS10;                            /*!< Priority Register B for Slave 10                                      */
  __IO uint32_t  PRAS11;                            /*!< Priority Register A for Slave 11                                      */
  __IO uint32_t  PRBS11;                            /*!< Priority Register B for Slave 11                                      */
  __IO uint32_t  PRAS12;                            /*!< Priority Register A for Slave 12                                      */
  __IO uint32_t  PRBS12;                            /*!< Priority Register B for Slave 12                                      */
  __IO uint32_t  PRAS13;                            /*!< Priority Register A for Slave 13                                      */
  __IO uint32_t  PRBS13;                            /*!< Priority Register B for Slave 13                                      */
  __IO uint32_t  PRAS14;                            /*!< Priority Register A for Slave 14                                      */
  __IO uint32_t  PRBS14;                            /*!< Priority Register B for Slave 14                                      */
  __IO uint32_t  PRAS15;                            /*!< Priority Register A for Slave 15                                      */
  __IO uint32_t  PRBS15;                            /*!< Priority Register B for Slave 15                                      */
  __IO uint32_t  MRCR;                              /*!< Master Remap Control Register                                         */
  __I  uint32_t  RESERVED[3];
  __IO uint32_t  SFR0;                              /*!< Special Function Register                                             */
  __IO uint32_t  SFR1;                              /*!< Special Function Register                                             */
  __IO uint32_t  SFR2;                              /*!< Special Function Register                                             */
  __IO uint32_t  SFR3;                              /*!< Special Function Register                                             */
  __IO uint32_t  SFR4;                              /*!< Special Function Register                                             */
  __IO uint32_t  SFR5;                              /*!< Special Function Register                                             */
  __IO uint32_t  SFR6;                              /*!< Special Function Register                                             */
  __IO uint32_t  SFR7;                              /*!< Special Function Register                                             */
  __IO uint32_t  SFR8;                              /*!< Special Function Register                                             */
  __IO uint32_t  SFR9;                              /*!< Special Function Register                                             */
  __IO uint32_t  SFR10;                             /*!< Special Function Register                                             */
  __IO uint32_t  SFR11;                             /*!< Special Function Register                                             */
  __IO uint32_t  SFR12;                             /*!< Special Function Register                                             */
  __IO uint32_t  SFR13;                             /*!< Special Function Register                                             */
  __IO uint32_t  SFR14;                             /*!< Special Function Register                                             */
  __IO uint32_t  SFR15;                             /*!< Special Function Register                                             */
  __I  uint32_t  RESERVED1[37];
  __IO uint32_t  WPMR;                              /*!< Write Protect Mode Register                                           */
  __I  uint32_t  WPSR;                              /*!< Write Protect Status Register                                         */
} MATRIX_Type;


/* ================================================================================ */
/* ================                       PMC                      ================ */
/* ================================================================================ */


/**
  * @brief Power Management Controller (PMC)
  */

typedef struct {                                    /*!< PMC Structure                                                         */
  __O  uint32_t  PMC_SCER;                          /*!< System Clock Enable Register                                          */
  __O  uint32_t  PMC_SCDR;                          /*!< System Clock Disable Register                                         */
  __I  uint32_t  PMC_SCSR;                          /*!< System Clock Status Register                                          */
  __I  uint32_t  RESERVED;
  __O  uint32_t  PMC_PCER0;                         /*!< Peripheral Clock Enable Register 0                                    */
  __O  uint32_t  PMC_PCDR0;                         /*!< Peripheral Clock Disable Register 0                                   */
  __I  uint32_t  PMC_PCSR0;                         /*!< Peripheral Clock Status Register 0                                    */
  __I  uint32_t  RESERVED1;
  __IO uint32_t  CKGR_MOR;                          /*!< Main Oscillator Register                                              */
  __IO uint32_t  CKGR_MCFR;                         /*!< Main Clock Frequency Register                                         */
  __IO uint32_t  CKGR_PLLAR;                        /*!< PLLA Register                                                         */
  __I  uint32_t  RESERVED2;
  __IO uint32_t  PMC_MCKR;                          /*!< Master Clock Register                                                 */
  __I  uint32_t  RESERVED3;
  __IO uint32_t  PMC_USB;                           /*!< USB Clock Register                                                    */
  __I  uint32_t  RESERVED4;
  __IO uint32_t  PMC_PCK0;                          /*!< Programmable Clock 0 Register                                         */
  __IO uint32_t  PMC_PCK1;                          /*!< Programmable Clock 0 Register                                         */
  __IO uint32_t  PMC_PCK2;                          /*!< Programmable Clock 0 Register                                         */
  __I  uint32_t  RESERVED5[5];
  __O  uint32_t  PMC_IER;                           /*!< Interrupt Enable Register                                             */
  __O  uint32_t  PMC_IDR;                           /*!< Interrupt Disable Register                                            */
  __I  uint32_t  PMC_SR;                            /*!< Status Register                                                       */
  __I  uint32_t  PMC_IMR;                           /*!< Interrupt Mask Register                                               */
  __IO uint32_t  PMC_FSMR;                          /*!< Fast Startup Mode Register                                            */
  __IO uint32_t  PMC_FSPR;                          /*!< Fast Startup Polarity Register                                        */
  __O  uint32_t  PMC_FOCR;                          /*!< Fault Output Clear Register                                           */
  __I  uint32_t  RESERVED6[26];
  __IO uint32_t  PMC_WPMR;                          /*!< Write Protect Mode Register                                           */
  __I  uint32_t  PMC_WPSR;                          /*!< Write Protect Status Register                                         */
  __I  uint32_t  RESERVED7[5];
  __O  uint32_t  PMC_PCER1;                         /*!< Peripheral Clock Enable Register 1                                    */
  __O  uint32_t  PMC_PCDR1;                         /*!< Peripheral Clock Disable Register 1                                   */
  __I  uint32_t  PMC_PCSR1;                         /*!< Peripheral Clock Status Register 1                                    */
  __I  uint32_t  RESERVED8;
  __IO uint32_t  PMC_OCR;                           /*!< Oscillator Calibration Register                                       */
} PMC_Type;


/* ================================================================================ */
/* ================                      UART0                     ================ */
/* ================================================================================ */


/**
  * @brief Universal Asynchronous Receiver Transmitter 0 (UART0)
  */

typedef struct {                                    /*!< UART0 Structure                                                       */
  __O  uint32_t  CR;                                /*!< Control Register                                                      */
  __IO uint32_t  MR;                                /*!< Mode Register                                                         */
  __O  uint32_t  IER;                               /*!< Interrupt Enable Register                                             */
  __O  uint32_t  IDR;                               /*!< Interrupt Disable Register                                            */
  __I  uint32_t  IMR;                               /*!< Interrupt Mask Register                                               */
  __I  uint32_t  SR;                                /*!< Status Register                                                       */
  __I  uint32_t  RHR;                               /*!< Receive Holding Register                                              */
  __O  uint32_t  THR;                               /*!< Transmit Holding Register                                             */
  __IO uint32_t  BRGR;                              /*!< Baud Rate Generator Register                                          */
  __I  uint32_t  RESERVED[55];
  __IO uint32_t  RPR;                               /*!< Receive Pointer Register                                              */
  __IO uint32_t  RCR;                               /*!< Receive Counter Register                                              */
  __IO uint32_t  TPR;                               /*!< Transmit Pointer Register                                             */
  __IO uint32_t  TCR;                               /*!< Transmit Counter Register                                             */
  __IO uint32_t  RNPR;                              /*!< Receive Next Pointer Register                                         */
  __IO uint32_t  RNCR;                              /*!< Receive Next Counter Register                                         */
  __IO uint32_t  TNPR;                              /*!< Transmit Next Pointer Register                                        */
  __IO uint32_t  TNCR;                              /*!< Transmit Next Counter Register                                        */
  __O  uint32_t  PTCR;                              /*!< Transfer Control Register                                             */
  __I  uint32_t  PTSR;                              /*!< Transfer Status Register                                              */
} UART0_Type;


/* ================================================================================ */
/* ================                     CHIPID                     ================ */
/* ================================================================================ */


/**
  * @brief Chip Identifier (CHIPID)
  */

typedef struct {                                    /*!< CHIPID Structure                                                      */
  __I  uint32_t  CIDR;                              /*!< Chip ID Register                                                      */
  __I  uint32_t  EXID;                              /*!< Chip ID Extension Register                                            */
} CHIPID_Type;


/* ================================================================================ */
/* ================                       EFC                      ================ */
/* ================================================================================ */


/**
  * @brief Embedded Flash Controller (EFC)
  */

typedef struct {                                    /*!< EFC Structure                                                         */
  __IO uint32_t  FMR;                               /*!< EEFC Flash Mode Register                                              */
  __O  uint32_t  FCR;                               /*!< EEFC Flash Command Register                                           */
  __I  uint32_t  FSR;                               /*!< EEFC Flash Status Register                                            */
  __I  uint32_t  FRR;                               /*!< EEFC Flash Result Register                                            */
} EFC_Type;


/* ================================================================================ */
/* ================                      PIOA                      ================ */
/* ================================================================================ */


/**
  * @brief Parallel Input/Output Controller A (PIOA)
  */

typedef struct {                                    /*!< PIOA Structure                                                        */
  __O  uint32_t  PER;                               /*!< PIO Enable Register                                                   */
  __O  uint32_t  PDR;                               /*!< PIO Disable Register                                                  */
  __I  uint32_t  PSR;                               /*!< PIO Status Register                                                   */
  __I  uint32_t  RESERVED;
  __O  uint32_t  OER;                               /*!< Output Enable Register                                                */
  __O  uint32_t  ODR;                               /*!< Output Disable Register                                               */
  __I  uint32_t  OSR;                               /*!< Output Status Register                                                */
  __I  uint32_t  RESERVED1;
  __O  uint32_t  IFER;                              /*!< Glitch Input Filter Enable Register                                   */
  __O  uint32_t  IFDR;                              /*!< Glitch Input Filter Disable Register                                  */
  __I  uint32_t  IFSR;                              /*!< Glitch Input Filter Status Register                                   */
  __I  uint32_t  RESERVED2;
  __O  uint32_t  SODR;                              /*!< Set Output Data Register                                              */
  __O  uint32_t  CODR;                              /*!< Clear Output Data Register                                            */
  __IO uint32_t  ODSR;                              /*!< Output Data Status Register                                           */
  __I  uint32_t  PDSR;                              /*!< Pin Data Status Register                                              */
  __O  uint32_t  IER;                               /*!< Interrupt Enable Register                                             */
  __O  uint32_t  IDR;                               /*!< Interrupt Disable Register                                            */
  __I  uint32_t  IMR;                               /*!< Interrupt Mask Register                                               */
  __I  uint32_t  ISR;                               /*!< Interrupt Status Register                                             */
  __O  uint32_t  MDER;                              /*!< Multi-driver Enable Register                                          */
  __O  uint32_t  MDDR;                              /*!< Multi-driver Disable Register                                         */
  __I  uint32_t  MDSR;                              /*!< Multi-driver Status Register                                          */
  __I  uint32_t  RESERVED3;
  __O  uint32_t  PUDR;                              /*!< Pull-up Disable Register                                              */
  __O  uint32_t  PUER;                              /*!< Pull-up Enable Register                                               */
  __I  uint32_t  PUSR;                              /*!< Pad Pull-up Status Register                                           */
  __I  uint32_t  RESERVED4;
  __IO uint32_t  ABCDSR0;                           /*!< Peripheral Select Register                                            */
  __IO uint32_t  ABCDSR1;                           /*!< Peripheral Select Register                                            */
  __I  uint32_t  RESERVED5[2];
  __O  uint32_t  IFSCDR;                            /*!< Input Filter Slow Clock Disable Register                              */
  __O  uint32_t  IFSCER;                            /*!< Input Filter Slow Clock Enable Register                               */
  __I  uint32_t  IFSCSR;                            /*!< Input Filter Slow Clock Status Register                               */
  __IO uint32_t  SCDR;                              /*!< Slow Clock Divider Debouncing Register                                */
  __O  uint32_t  PPDDR;                             /*!< Pad Pull-down Disable Register                                        */
  __O  uint32_t  PPDER;                             /*!< Pad Pull-down Enable Register                                         */
  __I  uint32_t  PPDSR;                             /*!< Pad Pull-down Status Register                                         */
  __I  uint32_t  RESERVED6;
  __O  uint32_t  OWER;                              /*!< Output Write Enable                                                   */
  __O  uint32_t  OWDR;                              /*!< Output Write Disable                                                  */
  __I  uint32_t  OWSR;                              /*!< Output Write Status Register                                          */
  __I  uint32_t  RESERVED7;
  __O  uint32_t  AIMER;                             /*!< Additional Interrupt Modes Enable Register                            */
  __O  uint32_t  AIMDR;                             /*!< Additional Interrupt Modes Disables Register                          */
  __I  uint32_t  AIMMR;                             /*!< Additional Interrupt Modes Mask Register                              */
  __I  uint32_t  RESERVED8;
  __O  uint32_t  ESR;                               /*!< Edge Select Register                                                  */
  __O  uint32_t  LSR;                               /*!< Level Select Register                                                 */
  __I  uint32_t  ELSR;                              /*!< Edge/Level Status Register                                            */
  __I  uint32_t  RESERVED9;
  __O  uint32_t  FELLSR;                            /*!< Falling Edge/Low Level Select Register                                */
  __O  uint32_t  REHLSR;                            /*!< Rising Edge/ High Level Select Register                               */
  __I  uint32_t  FRLHSR;                            /*!< Fall/Rise - Low/High Status Register                                  */
  __I  uint32_t  RESERVED10;
  __I  uint32_t  LOCKSR;                            /*!< Lock Status                                                           */
  __IO uint32_t  WPMR;                              /*!< Write Protect Mode Register                                           */
  __I  uint32_t  WPSR;                              /*!< Write Protect Status Register                                         */
  __I  uint32_t  RESERVED11[5];
  __IO uint32_t  SCHMITT;                           /*!< Schmitt Trigger Register                                              */
  __I  uint32_t  RESERVED12[3];
  __IO uint32_t  DELAYR;                            /*!< IO Delay Register                                                     */
  __I  uint32_t  RESERVED13[15];
  __IO uint32_t  PCMR;                              /*!< Parallel Capture Mode Register                                        */
  __O  uint32_t  PCIER;                             /*!< Parallel Capture Interrupt Enable Register                            */
  __O  uint32_t  PCIDR;                             /*!< Parallel Capture Interrupt Disable Register                           */
  __I  uint32_t  PCIMR;                             /*!< Parallel Capture Interrupt Mask Register                              */
  __I  uint32_t  PCISR;                             /*!< Parallel Capture Interrupt Status Register                            */
  __I  uint32_t  PCRHR;                             /*!< Parallel Capture Reception Holding Register                           */
  __IO uint32_t  RPR;                               /*!< Receive Pointer Register                                              */
  __IO uint32_t  RCR;                               /*!< Receive Counter Register                                              */
  __I  uint32_t  RESERVED14[2];
  __IO uint32_t  RNPR;                              /*!< Receive Next Pointer Register                                         */
  __IO uint32_t  RNCR;                              /*!< Receive Next Counter Register                                         */
  __I  uint32_t  RESERVED15[2];
  __O  uint32_t  PTCR;                              /*!< Transfer Control Register                                             */
  __I  uint32_t  PTSR;                              /*!< Transfer Status Register                                              */
} PIOA_Type;


/* ================================================================================ */
/* ================                      PIOB                      ================ */
/* ================================================================================ */


/**
  * @brief Parallel Input/Output Controller B (PIOB)
  */

typedef struct {                                    /*!< PIOB Structure                                                        */
  __O  uint32_t  PER;                               /*!< PIO Enable Register                                                   */
  __O  uint32_t  PDR;                               /*!< PIO Disable Register                                                  */
  __I  uint32_t  PSR;                               /*!< PIO Status Register                                                   */
  __I  uint32_t  RESERVED;
  __O  uint32_t  OER;                               /*!< Output Enable Register                                                */
  __O  uint32_t  ODR;                               /*!< Output Disable Register                                               */
  __I  uint32_t  OSR;                               /*!< Output Status Register                                                */
  __I  uint32_t  RESERVED1;
  __O  uint32_t  IFER;                              /*!< Glitch Input Filter Enable Register                                   */
  __O  uint32_t  IFDR;                              /*!< Glitch Input Filter Disable Register                                  */
  __I  uint32_t  IFSR;                              /*!< Glitch Input Filter Status Register                                   */
  __I  uint32_t  RESERVED2;
  __O  uint32_t  SODR;                              /*!< Set Output Data Register                                              */
  __O  uint32_t  CODR;                              /*!< Clear Output Data Register                                            */
  __IO uint32_t  ODSR;                              /*!< Output Data Status Register                                           */
  __I  uint32_t  PDSR;                              /*!< Pin Data Status Register                                              */
  __O  uint32_t  IER;                               /*!< Interrupt Enable Register                                             */
  __O  uint32_t  IDR;                               /*!< Interrupt Disable Register                                            */
  __I  uint32_t  IMR;                               /*!< Interrupt Mask Register                                               */
  __I  uint32_t  ISR;                               /*!< Interrupt Status Register                                             */
  __O  uint32_t  MDER;                              /*!< Multi-driver Enable Register                                          */
  __O  uint32_t  MDDR;                              /*!< Multi-driver Disable Register                                         */
  __I  uint32_t  MDSR;                              /*!< Multi-driver Status Register                                          */
  __I  uint32_t  RESERVED3;
  __O  uint32_t  PUDR;                              /*!< Pull-up Disable Register                                              */
  __O  uint32_t  PUER;                              /*!< Pull-up Enable Register                                               */
  __I  uint32_t  PUSR;                              /*!< Pad Pull-up Status Register                                           */
  __I  uint32_t  RESERVED4;
  __IO uint32_t  ABCDSR0;                           /*!< Peripheral Select Register                                            */
  __IO uint32_t  ABCDSR1;                           /*!< Peripheral Select Register                                            */
  __I  uint32_t  RESERVED5[2];
  __O  uint32_t  IFSCDR;                            /*!< Input Filter Slow Clock Disable Register                              */
  __O  uint32_t  IFSCER;                            /*!< Input Filter Slow Clock Enable Register                               */
  __I  uint32_t  IFSCSR;                            /*!< Input Filter Slow Clock Status Register                               */
  __IO uint32_t  SCDR;                              /*!< Slow Clock Divider Debouncing Register                                */
  __O  uint32_t  PPDDR;                             /*!< Pad Pull-down Disable Register                                        */
  __O  uint32_t  PPDER;                             /*!< Pad Pull-down Enable Register                                         */
  __I  uint32_t  PPDSR;                             /*!< Pad Pull-down Status Register                                         */
  __I  uint32_t  RESERVED6;
  __O  uint32_t  OWER;                              /*!< Output Write Enable                                                   */
  __O  uint32_t  OWDR;                              /*!< Output Write Disable                                                  */
  __I  uint32_t  OWSR;                              /*!< Output Write Status Register                                          */
  __I  uint32_t  RESERVED7;
  __O  uint32_t  AIMER;                             /*!< Additional Interrupt Modes Enable Register                            */
  __O  uint32_t  AIMDR;                             /*!< Additional Interrupt Modes Disables Register                          */
  __I  uint32_t  AIMMR;                             /*!< Additional Interrupt Modes Mask Register                              */
  __I  uint32_t  RESERVED8;
  __O  uint32_t  ESR;                               /*!< Edge Select Register                                                  */
  __O  uint32_t  LSR;                               /*!< Level Select Register                                                 */
  __I  uint32_t  ELSR;                              /*!< Edge/Level Status Register                                            */
  __I  uint32_t  RESERVED9;
  __O  uint32_t  FELLSR;                            /*!< Falling Edge/Low Level Select Register                                */
  __O  uint32_t  REHLSR;                            /*!< Rising Edge/ High Level Select Register                               */
  __I  uint32_t  FRLHSR;                            /*!< Fall/Rise - Low/High Status Register                                  */
  __I  uint32_t  RESERVED10;
  __I  uint32_t  LOCKSR;                            /*!< Lock Status                                                           */
  __IO uint32_t  WPMR;                              /*!< Write Protect Mode Register                                           */
  __I  uint32_t  WPSR;                              /*!< Write Protect Status Register                                         */
  __I  uint32_t  RESERVED11[5];
  __IO uint32_t  SCHMITT;                           /*!< Schmitt Trigger Register                                              */
  __I  uint32_t  RESERVED12[3];
  __IO uint32_t  DELAYR;                            /*!< IO Delay Register                                                     */
  __I  uint32_t  RESERVED13[15];
  __IO uint32_t  PCMR;                              /*!< Parallel Capture Mode Register                                        */
  __O  uint32_t  PCIER;                             /*!< Parallel Capture Interrupt Enable Register                            */
  __O  uint32_t  PCIDR;                             /*!< Parallel Capture Interrupt Disable Register                           */
  __I  uint32_t  PCIMR;                             /*!< Parallel Capture Interrupt Mask Register                              */
  __I  uint32_t  PCISR;                             /*!< Parallel Capture Interrupt Status Register                            */
  __I  uint32_t  PCRHR;                             /*!< Parallel Capture Reception Holding Register                           */
} PIOB_Type;


/* ================================================================================ */
/* ================                      PIOC                      ================ */
/* ================================================================================ */


/**
  * @brief Parallel Input/Output Controller C (PIOC)
  */

typedef struct {                                    /*!< PIOC Structure                                                        */
  __O  uint32_t  PER;                               /*!< PIO Enable Register                                                   */
  __O  uint32_t  PDR;                               /*!< PIO Disable Register                                                  */
  __I  uint32_t  PSR;                               /*!< PIO Status Register                                                   */
  __I  uint32_t  RESERVED;
  __O  uint32_t  OER;                               /*!< Output Enable Register                                                */
  __O  uint32_t  ODR;                               /*!< Output Disable Register                                               */
  __I  uint32_t  OSR;                               /*!< Output Status Register                                                */
  __I  uint32_t  RESERVED1;
  __O  uint32_t  IFER;                              /*!< Glitch Input Filter Enable Register                                   */
  __O  uint32_t  IFDR;                              /*!< Glitch Input Filter Disable Register                                  */
  __I  uint32_t  IFSR;                              /*!< Glitch Input Filter Status Register                                   */
  __I  uint32_t  RESERVED2;
  __O  uint32_t  SODR;                              /*!< Set Output Data Register                                              */
  __O  uint32_t  CODR;                              /*!< Clear Output Data Register                                            */
  __IO uint32_t  ODSR;                              /*!< Output Data Status Register                                           */
  __I  uint32_t  PDSR;                              /*!< Pin Data Status Register                                              */
  __O  uint32_t  IER;                               /*!< Interrupt Enable Register                                             */
  __O  uint32_t  IDR;                               /*!< Interrupt Disable Register                                            */
  __I  uint32_t  IMR;                               /*!< Interrupt Mask Register                                               */
  __I  uint32_t  ISR;                               /*!< Interrupt Status Register                                             */
  __O  uint32_t  MDER;                              /*!< Multi-driver Enable Register                                          */
  __O  uint32_t  MDDR;                              /*!< Multi-driver Disable Register                                         */
  __I  uint32_t  MDSR;                              /*!< Multi-driver Status Register                                          */
  __I  uint32_t  RESERVED3;
  __O  uint32_t  PUDR;                              /*!< Pull-up Disable Register                                              */
  __O  uint32_t  PUER;                              /*!< Pull-up Enable Register                                               */
  __I  uint32_t  PUSR;                              /*!< Pad Pull-up Status Register                                           */
  __I  uint32_t  RESERVED4;
  __IO uint32_t  ABCDSR0;                           /*!< Peripheral Select Register                                            */
  __IO uint32_t  ABCDSR1;                           /*!< Peripheral Select Register                                            */
  __I  uint32_t  RESERVED5[2];
  __O  uint32_t  IFSCDR;                            /*!< Input Filter Slow Clock Disable Register                              */
  __O  uint32_t  IFSCER;                            /*!< Input Filter Slow Clock Enable Register                               */
  __I  uint32_t  IFSCSR;                            /*!< Input Filter Slow Clock Status Register                               */
  __IO uint32_t  SCDR;                              /*!< Slow Clock Divider Debouncing Register                                */
  __O  uint32_t  PPDDR;                             /*!< Pad Pull-down Disable Register                                        */
  __O  uint32_t  PPDER;                             /*!< Pad Pull-down Enable Register                                         */
  __I  uint32_t  PPDSR;                             /*!< Pad Pull-down Status Register                                         */
  __I  uint32_t  RESERVED6;
  __O  uint32_t  OWER;                              /*!< Output Write Enable                                                   */
  __O  uint32_t  OWDR;                              /*!< Output Write Disable                                                  */
  __I  uint32_t  OWSR;                              /*!< Output Write Status Register                                          */
  __I  uint32_t  RESERVED7;
  __O  uint32_t  AIMER;                             /*!< Additional Interrupt Modes Enable Register                            */
  __O  uint32_t  AIMDR;                             /*!< Additional Interrupt Modes Disables Register                          */
  __I  uint32_t  AIMMR;                             /*!< Additional Interrupt Modes Mask Register                              */
  __I  uint32_t  RESERVED8;
  __O  uint32_t  ESR;                               /*!< Edge Select Register                                                  */
  __O  uint32_t  LSR;                               /*!< Level Select Register                                                 */
  __I  uint32_t  ELSR;                              /*!< Edge/Level Status Register                                            */
  __I  uint32_t  RESERVED9;
  __O  uint32_t  FELLSR;                            /*!< Falling Edge/Low Level Select Register                                */
  __O  uint32_t  REHLSR;                            /*!< Rising Edge/ High Level Select Register                               */
  __I  uint32_t  FRLHSR;                            /*!< Fall/Rise - Low/High Status Register                                  */
  __I  uint32_t  RESERVED10;
  __I  uint32_t  LOCKSR;                            /*!< Lock Status                                                           */
  __IO uint32_t  WPMR;                              /*!< Write Protect Mode Register                                           */
  __I  uint32_t  WPSR;                              /*!< Write Protect Status Register                                         */
  __I  uint32_t  RESERVED11[5];
  __IO uint32_t  SCHMITT;                           /*!< Schmitt Trigger Register                                              */
  __I  uint32_t  RESERVED12[3];
  __IO uint32_t  DELAYR;                            /*!< IO Delay Register                                                     */
  __I  uint32_t  RESERVED13[15];
  __IO uint32_t  PCMR;                              /*!< Parallel Capture Mode Register                                        */
  __O  uint32_t  PCIER;                             /*!< Parallel Capture Interrupt Enable Register                            */
  __O  uint32_t  PCIDR;                             /*!< Parallel Capture Interrupt Disable Register                           */
  __I  uint32_t  PCIMR;                             /*!< Parallel Capture Interrupt Mask Register                              */
  __I  uint32_t  PCISR;                             /*!< Parallel Capture Interrupt Status Register                            */
  __I  uint32_t  PCRHR;                             /*!< Parallel Capture Reception Holding Register                           */
} PIOC_Type;


/* ================================================================================ */
/* ================                      PIOD                      ================ */
/* ================================================================================ */


/**
  * @brief Parallel Input/Output Controller D (PIOD)
  */

typedef struct {                                    /*!< PIOD Structure                                                        */
  __O  uint32_t  PER;                               /*!< PIO Enable Register                                                   */
  __O  uint32_t  PDR;                               /*!< PIO Disable Register                                                  */
  __I  uint32_t  PSR;                               /*!< PIO Status Register                                                   */
  __I  uint32_t  RESERVED;
  __O  uint32_t  OER;                               /*!< Output Enable Register                                                */
  __O  uint32_t  ODR;                               /*!< Output Disable Register                                               */
  __I  uint32_t  OSR;                               /*!< Output Status Register                                                */
  __I  uint32_t  RESERVED1;
  __O  uint32_t  IFER;                              /*!< Glitch Input Filter Enable Register                                   */
  __O  uint32_t  IFDR;                              /*!< Glitch Input Filter Disable Register                                  */
  __I  uint32_t  IFSR;                              /*!< Glitch Input Filter Status Register                                   */
  __I  uint32_t  RESERVED2;
  __O  uint32_t  SODR;                              /*!< Set Output Data Register                                              */
  __O  uint32_t  CODR;                              /*!< Clear Output Data Register                                            */
  __IO uint32_t  ODSR;                              /*!< Output Data Status Register                                           */
  __I  uint32_t  PDSR;                              /*!< Pin Data Status Register                                              */
  __O  uint32_t  IER;                               /*!< Interrupt Enable Register                                             */
  __O  uint32_t  IDR;                               /*!< Interrupt Disable Register                                            */
  __I  uint32_t  IMR;                               /*!< Interrupt Mask Register                                               */
  __I  uint32_t  ISR;                               /*!< Interrupt Status Register                                             */
  __O  uint32_t  MDER;                              /*!< Multi-driver Enable Register                                          */
  __O  uint32_t  MDDR;                              /*!< Multi-driver Disable Register                                         */
  __I  uint32_t  MDSR;                              /*!< Multi-driver Status Register                                          */
  __I  uint32_t  RESERVED3;
  __O  uint32_t  PUDR;                              /*!< Pull-up Disable Register                                              */
  __O  uint32_t  PUER;                              /*!< Pull-up Enable Register                                               */
  __I  uint32_t  PUSR;                              /*!< Pad Pull-up Status Register                                           */
  __I  uint32_t  RESERVED4;
  __IO uint32_t  ABCDSR0;                           /*!< Peripheral Select Register                                            */
  __IO uint32_t  ABCDSR1;                           /*!< Peripheral Select Register                                            */
  __I  uint32_t  RESERVED5[2];
  __O  uint32_t  IFSCDR;                            /*!< Input Filter Slow Clock Disable Register                              */
  __O  uint32_t  IFSCER;                            /*!< Input Filter Slow Clock Enable Register                               */
  __I  uint32_t  IFSCSR;                            /*!< Input Filter Slow Clock Status Register                               */
  __IO uint32_t  SCDR;                              /*!< Slow Clock Divider Debouncing Register                                */
  __O  uint32_t  PPDDR;                             /*!< Pad Pull-down Disable Register                                        */
  __O  uint32_t  PPDER;                             /*!< Pad Pull-down Enable Register                                         */
  __I  uint32_t  PPDSR;                             /*!< Pad Pull-down Status Register                                         */
  __I  uint32_t  RESERVED6;
  __O  uint32_t  OWER;                              /*!< Output Write Enable                                                   */
  __O  uint32_t  OWDR;                              /*!< Output Write Disable                                                  */
  __I  uint32_t  OWSR;                              /*!< Output Write Status Register                                          */
  __I  uint32_t  RESERVED7;
  __O  uint32_t  AIMER;                             /*!< Additional Interrupt Modes Enable Register                            */
  __O  uint32_t  AIMDR;                             /*!< Additional Interrupt Modes Disables Register                          */
  __I  uint32_t  AIMMR;                             /*!< Additional Interrupt Modes Mask Register                              */
  __I  uint32_t  RESERVED8;
  __O  uint32_t  ESR;                               /*!< Edge Select Register                                                  */
  __O  uint32_t  LSR;                               /*!< Level Select Register                                                 */
  __I  uint32_t  ELSR;                              /*!< Edge/Level Status Register                                            */
  __I  uint32_t  RESERVED9;
  __O  uint32_t  FELLSR;                            /*!< Falling Edge/Low Level Select Register                                */
  __O  uint32_t  REHLSR;                            /*!< Rising Edge/ High Level Select Register                               */
  __I  uint32_t  FRLHSR;                            /*!< Fall/Rise - Low/High Status Register                                  */
  __I  uint32_t  RESERVED10;
  __I  uint32_t  LOCKSR;                            /*!< Lock Status                                                           */
  __IO uint32_t  WPMR;                              /*!< Write Protect Mode Register                                           */
  __I  uint32_t  WPSR;                              /*!< Write Protect Status Register                                         */
  __I  uint32_t  RESERVED11[5];
  __IO uint32_t  SCHMITT;                           /*!< Schmitt Trigger Register                                              */
  __I  uint32_t  RESERVED12[3];
  __IO uint32_t  DELAYR;                            /*!< IO Delay Register                                                     */
  __I  uint32_t  RESERVED13[15];
  __IO uint32_t  PCMR;                              /*!< Parallel Capture Mode Register                                        */
  __O  uint32_t  PCIER;                             /*!< Parallel Capture Interrupt Enable Register                            */
  __O  uint32_t  PCIDR;                             /*!< Parallel Capture Interrupt Disable Register                           */
  __I  uint32_t  PCIMR;                             /*!< Parallel Capture Interrupt Mask Register                              */
  __I  uint32_t  PCISR;                             /*!< Parallel Capture Interrupt Status Register                            */
  __I  uint32_t  PCRHR;                             /*!< Parallel Capture Reception Holding Register                           */
} PIOD_Type;


/* ================================================================================ */
/* ================                      PIOE                      ================ */
/* ================================================================================ */


/**
  * @brief Parallel Input/Output Controller E (PIOE)
  */

typedef struct {                                    /*!< PIOE Structure                                                        */
  __O  uint32_t  PER;                               /*!< PIO Enable Register                                                   */
  __O  uint32_t  PDR;                               /*!< PIO Disable Register                                                  */
  __I  uint32_t  PSR;                               /*!< PIO Status Register                                                   */
  __I  uint32_t  RESERVED;
  __O  uint32_t  OER;                               /*!< Output Enable Register                                                */
  __O  uint32_t  ODR;                               /*!< Output Disable Register                                               */
  __I  uint32_t  OSR;                               /*!< Output Status Register                                                */
  __I  uint32_t  RESERVED1;
  __O  uint32_t  IFER;                              /*!< Glitch Input Filter Enable Register                                   */
  __O  uint32_t  IFDR;                              /*!< Glitch Input Filter Disable Register                                  */
  __I  uint32_t  IFSR;                              /*!< Glitch Input Filter Status Register                                   */
  __I  uint32_t  RESERVED2;
  __O  uint32_t  SODR;                              /*!< Set Output Data Register                                              */
  __O  uint32_t  CODR;                              /*!< Clear Output Data Register                                            */
  __IO uint32_t  ODSR;                              /*!< Output Data Status Register                                           */
  __I  uint32_t  PDSR;                              /*!< Pin Data Status Register                                              */
  __O  uint32_t  IER;                               /*!< Interrupt Enable Register                                             */
  __O  uint32_t  IDR;                               /*!< Interrupt Disable Register                                            */
  __I  uint32_t  IMR;                               /*!< Interrupt Mask Register                                               */
  __I  uint32_t  ISR;                               /*!< Interrupt Status Register                                             */
  __O  uint32_t  MDER;                              /*!< Multi-driver Enable Register                                          */
  __O  uint32_t  MDDR;                              /*!< Multi-driver Disable Register                                         */
  __I  uint32_t  MDSR;                              /*!< Multi-driver Status Register                                          */
  __I  uint32_t  RESERVED3;
  __O  uint32_t  PUDR;                              /*!< Pull-up Disable Register                                              */
  __O  uint32_t  PUER;                              /*!< Pull-up Enable Register                                               */
  __I  uint32_t  PUSR;                              /*!< Pad Pull-up Status Register                                           */
  __I  uint32_t  RESERVED4;
  __IO uint32_t  ABCDSR0;                           /*!< Peripheral Select Register                                            */
  __IO uint32_t  ABCDSR1;                           /*!< Peripheral Select Register                                            */
  __I  uint32_t  RESERVED5[2];
  __O  uint32_t  IFSCDR;                            /*!< Input Filter Slow Clock Disable Register                              */
  __O  uint32_t  IFSCER;                            /*!< Input Filter Slow Clock Enable Register                               */
  __I  uint32_t  IFSCSR;                            /*!< Input Filter Slow Clock Status Register                               */
  __IO uint32_t  SCDR;                              /*!< Slow Clock Divider Debouncing Register                                */
  __O  uint32_t  PPDDR;                             /*!< Pad Pull-down Disable Register                                        */
  __O  uint32_t  PPDER;                             /*!< Pad Pull-down Enable Register                                         */
  __I  uint32_t  PPDSR;                             /*!< Pad Pull-down Status Register                                         */
  __I  uint32_t  RESERVED6;
  __O  uint32_t  OWER;                              /*!< Output Write Enable                                                   */
  __O  uint32_t  OWDR;                              /*!< Output Write Disable                                                  */
  __I  uint32_t  OWSR;                              /*!< Output Write Status Register                                          */
  __I  uint32_t  RESERVED7;
  __O  uint32_t  AIMER;                             /*!< Additional Interrupt Modes Enable Register                            */
  __O  uint32_t  AIMDR;                             /*!< Additional Interrupt Modes Disables Register                          */
  __I  uint32_t  AIMMR;                             /*!< Additional Interrupt Modes Mask Register                              */
  __I  uint32_t  RESERVED8;
  __O  uint32_t  ESR;                               /*!< Edge Select Register                                                  */
  __O  uint32_t  LSR;                               /*!< Level Select Register                                                 */
  __I  uint32_t  ELSR;                              /*!< Edge/Level Status Register                                            */
  __I  uint32_t  RESERVED9;
  __O  uint32_t  FELLSR;                            /*!< Falling Edge/Low Level Select Register                                */
  __O  uint32_t  REHLSR;                            /*!< Rising Edge/ High Level Select Register                               */
  __I  uint32_t  FRLHSR;                            /*!< Fall/Rise - Low/High Status Register                                  */
  __I  uint32_t  RESERVED10;
  __I  uint32_t  LOCKSR;                            /*!< Lock Status                                                           */
  __IO uint32_t  WPMR;                              /*!< Write Protect Mode Register                                           */
  __I  uint32_t  WPSR;                              /*!< Write Protect Status Register                                         */
  __I  uint32_t  RESERVED11[5];
  __IO uint32_t  SCHMITT;                           /*!< Schmitt Trigger Register                                              */
  __I  uint32_t  RESERVED12[3];
  __IO uint32_t  DELAYR;                            /*!< IO Delay Register                                                     */
  __I  uint32_t  RESERVED13[15];
  __IO uint32_t  PCMR;                              /*!< Parallel Capture Mode Register                                        */
  __O  uint32_t  PCIER;                             /*!< Parallel Capture Interrupt Enable Register                            */
  __O  uint32_t  PCIDR;                             /*!< Parallel Capture Interrupt Disable Register                           */
  __I  uint32_t  PCIMR;                             /*!< Parallel Capture Interrupt Mask Register                              */
  __I  uint32_t  PCISR;                             /*!< Parallel Capture Interrupt Status Register                            */
  __I  uint32_t  PCRHR;                             /*!< Parallel Capture Reception Holding Register                           */
} PIOE_Type;


/* ================================================================================ */
/* ================                      RSTC                      ================ */
/* ================================================================================ */


/**
  * @brief Reset Controller (RSTC)
  */

typedef struct {                                    /*!< RSTC Structure                                                        */
  __O  uint32_t  CR;                                /*!< Control Register                                                      */
  __I  uint32_t  SR;                                /*!< Status Register                                                       */
  __IO uint32_t  MR;                                /*!< Mode Register                                                         */
} RSTC_Type;


/* ================================================================================ */
/* ================                      SUPC                      ================ */
/* ================================================================================ */


/**
  * @brief Supply Controller (SUPC)
  */

typedef struct {                                    /*!< SUPC Structure                                                        */
  __O  uint32_t  CR;                                /*!< Supply Controller Control Register                                    */
  __IO uint32_t  SMMR;                              /*!< Supply Controller Supply Monitor Mode Register                        */
  __IO uint32_t  MR;                                /*!< Supply Controller Mode Register                                       */
  __IO uint32_t  WUMR;                              /*!< Supply Controller Wake Up Mode Register                               */
  __IO uint32_t  WUIR;                              /*!< Supply Controller Wake Up Inputs Register                             */
  __I  uint32_t  SR;                                /*!< Supply Controller Status Register                                     */
} SUPC_Type;


/* ================================================================================ */
/* ================                       RTT                      ================ */
/* ================================================================================ */


/**
  * @brief Real-time Timer (RTT)
  */

typedef struct {                                    /*!< RTT Structure                                                         */
  __IO uint32_t  MR;                                /*!< Mode Register                                                         */
  __IO uint32_t  AR;                                /*!< Alarm Register                                                        */
  __I  uint32_t  VR;                                /*!< Value Register                                                        */
  __I  uint32_t  SR;                                /*!< Status Register                                                       */
} RTT_Type;


/* ================================================================================ */
/* ================                       WDT                      ================ */
/* ================================================================================ */


/**
  * @brief Watchdog Timer (WDT)
  */

typedef struct {                                    /*!< WDT Structure                                                         */
  __O  uint32_t  CR;                                /*!< Control Register                                                      */
  __IO uint32_t  MR;                                /*!< Mode Register                                                         */
  __I  uint32_t  SR;                                /*!< Status Register                                                       */
} WDT_Type;


/* ================================================================================ */
/* ================                       RTC                      ================ */
/* ================================================================================ */


/**
  * @brief Real-time Clock (RTC)
  */

typedef struct {                                    /*!< RTC Structure                                                         */
  __IO uint32_t  CR;                                /*!< Control Register                                                      */
  __IO uint32_t  MR;                                /*!< Mode Register                                                         */
  __IO uint32_t  TIMR;                              /*!< Time Register                                                         */
  __IO uint32_t  CALR;                              /*!< Calendar Register                                                     */
  __IO uint32_t  TIMALR;                            /*!< Time Alarm Register                                                   */
  __IO uint32_t  CALALR;                            /*!< Calendar Alarm Register                                               */
  __I  uint32_t  SR;                                /*!< Status Register                                                       */
  __O  uint32_t  SCCR;                              /*!< Status Clear Command Register                                         */
  __O  uint32_t  IER;                               /*!< Interrupt Enable Register                                             */
  __O  uint32_t  IDR;                               /*!< Interrupt Disable Register                                            */
  __I  uint32_t  IMR;                               /*!< Interrupt Mask Register                                               */
  __I  uint32_t  VER;                               /*!< Valid Entry Register                                                  */
} RTC_Type;


/* ================================================================================ */
/* ================                      GPBR                      ================ */
/* ================================================================================ */


/**
  * @brief General Purpose Backup Register (GPBR)
  */

typedef struct {                                    /*!< GPBR Structure                                                        */
  __IO uint32_t  GPBR0;                             /*!< General Purpose Backup Register                                       */
  __IO uint32_t  GPBR1;                             /*!< General Purpose Backup Register                                       */
  __IO uint32_t  GPBR2;                             /*!< General Purpose Backup Register                                       */
  __IO uint32_t  GPBR3;                             /*!< General Purpose Backup Register                                       */
  __IO uint32_t  GPBR4;                             /*!< General Purpose Backup Register                                       */
  __IO uint32_t  GPBR5;                             /*!< General Purpose Backup Register                                       */
  __IO uint32_t  GPBR6;                             /*!< General Purpose Backup Register                                       */
  __IO uint32_t  GPBR7;                             /*!< General Purpose Backup Register                                       */
  __IO uint32_t  GPBR8;                             /*!< General Purpose Backup Register                                       */
  __IO uint32_t  GPBR9;                             /*!< General Purpose Backup Register                                       */
  __IO uint32_t  GPBR10;                            /*!< General Purpose Backup Register                                       */
  __IO uint32_t  GPBR11;                            /*!< General Purpose Backup Register                                       */
  __IO uint32_t  GPBR12;                            /*!< General Purpose Backup Register                                       */
  __IO uint32_t  GPBR13;                            /*!< General Purpose Backup Register                                       */
  __IO uint32_t  GPBR14;                            /*!< General Purpose Backup Register                                       */
  __IO uint32_t  GPBR15;                            /*!< General Purpose Backup Register                                       */
  __IO uint32_t  GPBR16;                            /*!< General Purpose Backup Register                                       */
  __IO uint32_t  GPBR17;                            /*!< General Purpose Backup Register                                       */
  __IO uint32_t  GPBR18;                            /*!< General Purpose Backup Register                                       */
  __IO uint32_t  GPBR19;                            /*!< General Purpose Backup Register                                       */
} GPBR_Type;


/* --------------------  End of section using anonymous unions  ------------------- */
#if defined(__CC_ARM)
  #pragma pop
#elif defined(__ICCARM__)
  /* leave anonymous unions enabled */
#elif defined(__GNUC__)
  /* anonymous unions are enabled by default */
#elif defined(__TMS470__)
  /* anonymous unions are enabled by default */
#elif defined(__TASKING__)
  #pragma warning restore
#else
  #warning Not supported compiler type
#endif




/* ================================================================================ */
/* ================              Peripheral memory map             ================ */
/* ================================================================================ */

#define PWM_BASE                        0x40000000UL
#define AES_BASE                        0x40004000UL
#define CAN0_BASE                       0x40010000UL
#define CAN1_BASE                       0x40014000UL
#define GMAC_BASE                       0x40034000UL
#define CRCCU_BASE                      0x40044000UL
#define SMC_BASE                        0x40060000UL
#define UART1_BASE                      0x40060600UL
#define HSMCI_BASE                      0x40080000UL
#define UDP_BASE                        0x40084000UL
#define SPI_BASE                        0x40088000UL
#define TC0_BASE                        0x40090000UL
#define TC1_BASE                        0x40094000UL
#define TC2_BASE                        0x40098000UL
#define USART0_BASE                     0x400A0000UL
#define USART1_BASE                     0x400A4000UL
#define TWI0_BASE                       0x400A8000UL
#define TWI1_BASE                       0x400AC000UL
#define AFEC0_BASE                      0x400B0000UL
#define AFEC1_BASE                      0x400B4000UL
#define DACC_BASE                       0x400B8000UL
#define ACC_BASE                        0x400BC000UL
#define DMAC_BASE                       0x400C0000UL
#define CMCC_BASE                       0x400C4000UL
#define MATRIX_BASE                     0x400E0200UL
#define PMC_BASE                        0x400E0400UL
#define UART0_BASE                      0x400E0600UL
#define CHIPID_BASE                     0x400E0740UL
#define EFC_BASE                        0x400E0A00UL
#define PIOA_BASE                       0x400E0E00UL
#define PIOB_BASE                       0x400E1000UL
#define PIOC_BASE                       0x400E1200UL
#define PIOD_BASE                       0x400E1400UL
#define PIOE_BASE                       0x400E1600UL
#define RSTC_BASE                       0x400E1800UL
#define SUPC_BASE                       0x400E1810UL
#define RTT_BASE                        0x400E1830UL
#define WDT_BASE                        0x400E1850UL
#define RTC_BASE                        0x400E1860UL
#define GPBR_BASE                       0x400E1890UL


/* ================================================================================ */
/* ================             Peripheral declaration             ================ */
/* ================================================================================ */

#define PWM                             ((PWM_Type                *) PWM_BASE)
#define AES                             ((AES_Type                *) AES_BASE)
#define CAN0                            ((CAN0_Type               *) CAN0_BASE)
#define CAN1                            ((CAN1_Type               *) CAN1_BASE)
#define GMAC                            ((GMAC_Type               *) GMAC_BASE)
#define CRCCU                           ((CRCCU_Type              *) CRCCU_BASE)
#define SMC                             ((SMC_Type                *) SMC_BASE)
#define UART1                           ((UART1_Type              *) UART1_BASE)
#define HSMCI                           ((HSMCI_Type              *) HSMCI_BASE)
#define UDP                             ((UDP_Type                *) UDP_BASE)
#define SPI                             ((SPI_Type                *) SPI_BASE)
#define TC0                             ((TC0_Type                *) TC0_BASE)
#define TC1                             ((TC1_Type                *) TC1_BASE)
#define TC2                             ((TC2_Type                *) TC2_BASE)
#define USART0                          ((USART0_Type             *) USART0_BASE)
#define USART1                          ((USART1_Type             *) USART1_BASE)
#define TWI0                            ((TWI0_Type               *) TWI0_BASE)
#define TWI1                            ((TWI1_Type               *) TWI1_BASE)
#define AFEC0                           ((AFEC0_Type              *) AFEC0_BASE)
#define AFEC1                           ((AFEC1_Type              *) AFEC1_BASE)
#define DACC                            ((DACC_Type               *) DACC_BASE)
#define ACC                             ((ACC_Type                *) ACC_BASE)
#define DMAC                            ((DMAC_Type               *) DMAC_BASE)
#define CMCC                            ((CMCC_Type               *) CMCC_BASE)
#define MATRIX                          ((MATRIX_Type             *) MATRIX_BASE)
#define PMC                             ((PMC_Type                *) PMC_BASE)
#define UART0                           ((UART0_Type              *) UART0_BASE)
#define CHIPID                          ((CHIPID_Type             *) CHIPID_BASE)
#define EFC                             ((EFC_Type                *) EFC_BASE)
#define PIOA                            ((PIOA_Type               *) PIOA_BASE)
#define PIOB                            ((PIOB_Type               *) PIOB_BASE)
#define PIOC                            ((PIOC_Type               *) PIOC_BASE)
#define PIOD                            ((PIOD_Type               *) PIOD_BASE)
#define PIOE                            ((PIOE_Type               *) PIOE_BASE)
#define RSTC                            ((RSTC_Type               *) RSTC_BASE)
#define SUPC                            ((SUPC_Type               *) SUPC_BASE)
#define RTT                             ((RTT_Type                *) RTT_BASE)
#define WDT                             ((WDT_Type                *) WDT_BASE)
#define RTC                             ((RTC_Type                *) RTC_BASE)
#define GPBR                            ((GPBR_Type               *) GPBR_BASE)


/** @} */ /* End of group Device_Peripheral_Registers */
/** @} */ /* End of group ATSAM4E16E */
/** @} */ /* End of group Atmel */

#ifdef __cplusplus
}
#endif


#endif  /* ATSAM4E16E_H */

