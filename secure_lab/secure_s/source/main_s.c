/*
 * Copyright 2018 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#if (__ARM_FEATURE_CMSE & 1) == 0
#error "Need ARMv8-M security extensions"
#elif (__ARM_FEATURE_CMSE & 2) == 0
#error "Compile with --cmse"
#endif


#include "fsl_spi.h"
#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "arm_cmse.h"
#include "board.h"
#include "veneer_table.h"
#include "tzm_config.h"

#include "pin_mux.h"
#include "clock_config.h"

#include "secure_gpio.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/

#define NON_SECURE_START          0x00010000
#define AHB_LAYERS_COUNT          19U

#define EXAMPLE_SPI_MASTER SPI8
#define EXAMPLE_SPI_MASTER_IRQ LSPI_HS_IRQn
#define EXAMPLE_SPI_MASTER_CLK_SRC kCLOCK_HsLspi
#define EXAMPLE_SPI_MASTER_CLK_FREQ CLOCK_GetFreq(kCLOCK_HsLspi)
#define EXAMPLE_SPI_SSEL 1
#define EXAMPLE_SPI_SPOL kSPI_SpolActiveAllLow

/* typedef for non-secure callback functions */
typedef void (*funcptr_ns) (void) __attribute__((cmse_nonsecure_call));

typedef union
{
    struct ahb_secure_fault_info
    {
        unsigned access_type:2;
        unsigned reserved0:2;
        unsigned master_sec_level:2;
        unsigned antipol_master_sec_level:2;
        unsigned master_number:4;
        unsigned reserved:20;
    } fault_info;
    unsigned value;
} ahb_secure_fault_info_t;

unsigned char NwkSkey[16] = { 0x44, 0x19, 0x4D, 0x01, 0xF3, 0x8C, 0x93, 0xCB, 0x19, 0x03, 0x51, 0x58, 0x03, 0x87, 0x7E, 0x67 };
unsigned char AppSkey[16] = { 0x78, 0x90, 0x38, 0x01, 0x20, 0xA7, 0x70, 0x33, 0xBA, 0xEC, 0x0B, 0x26, 0x75, 0xF0, 0xE3, 0x09 };
unsigned char DevAddr[4] = { 0x26, 0x02, 0x14, 0x2E };


/*******************************************************************************
 * Prototypes
 ******************************************************************************/
void HardFault_Handler (void);
uint32_t GetTestCaseNumber(int a);
uint32_t GetAppKey(void);



/*******************************************************************************
 * Global variables
 ******************************************************************************/
uint32_t testCaseNumber =  FAULT_NONE;
spi_master_config_t userConfig = {0};

/*******************************************************************************
 * Code
 ******************************************************************************/
/*!
 * @brief Application-specific implementation of the SystemInitHook() weak function.
 */
void SystemInitHook(void) 
{
/* The TrustZone should be configured as early as possible after RESET.
 * Therefore it is called from SystemInit() during startup. The SystemInitHook() weak function 
 * overloading is used for this purpose.
*/
    BOARD_InitTrustZone();
}


/*!
 * @brief Main function
 */
int main(void)
{
    funcptr_ns ResetHandler_ns;

    /* Init board hardware. */
    /* attach main clock divide to FLEXCOMM0 (debug console) */
    CLOCK_AttachClk(BOARD_DEBUG_UART_CLK_ATTACH);

    /* attach 12 MHz clock to SPI3 */
    CLOCK_AttachClk(kFRO12M_to_HSLSPI);

    /* reset FLEXCOMM for SPI */
    RESET_PeripheralReset(kHSLSPI_RST_SHIFT_RSTn);

    BOARD_InitPins();
    BOARD_BootClockFROHF96M();
    BOARD_InitDebugConsole();

    /* Initialize S1 as a Secure GPIO with Secure PINT */
    SecureGPIO_Init();

    PRINTF("\r\nHello from secure world!\r\n");

    /* Set non-secure main stack (MSP_NS) */
    __TZ_set_MSP_NS(*((uint32_t *)(NON_SECURE_START)));

    /* Set non-secure vector table */
    SCB_NS->VTOR = NON_SECURE_START;

    /* Get non-secure reset handler */
    ResetHandler_ns = (funcptr_ns)(*((uint32_t *)((NON_SECURE_START) + 4U)));

//    do
//    {
//    	PRINTF("\r\n---- Choose Secure Fault ----\r\n");
//    	PRINTF("0 Continue without fault \r\n");
//    	PRINTF("1 S ->NS  invalid transition \r\n");
//    	PRINTF("2 NS->S   invalid entry point \r\n");
//    	PRINTF("3 NS->S   illegal data access SAU Region 1 \r\n");
//    	PRINTF("4 NS->S   invalid data access Secure Bus (AHB MPC - Secure RAM1 ) \r\n");
//    	PRINTF("5 NS->NSC invalid input parameters  \r\n");
//    	testCaseNumber = GETCHAR();
//    	PUTCHAR(testCaseNumber);
//    }while(testCaseNumber<'0' || testCaseNumber>'5');
//    testCaseNumber -= '0';
    
    /* Call non-secure application */
    PRINTF("\r\nGoing to normal world.\r\n");

    /* Test 1 S->NS invalid transition
     * Direct jump to NS ResetHandler without
     * a. Cleaning core registers
     * b. NS most LSB address not cleared  */
    if (testCaseNumber == FAULT_INV_S_TO_NS_TRANS)
    {
        __asm("BXNS %0" : : "r" (ResetHandler_ns));
    }

    /* Jump to normal world (Correct way compare to Test 1) */
    ResetHandler_ns();

    while (1)
    {
        /* This point should never be reached */
    }
}

/*!
 * @brief This function returns the SecureFault test case number selected through UART. .
 */
uint32_t GetTestCaseNumber(int a)
{
	return DevAddr[a];
    //return testCaseNumber;
}


uint32_t GetAppKey()
{
    return AppSkey;
}

/*!
 * @brief HardFault handler. This handler can called from both normal and secure world
 */
void HardFault_Handler (void)
{
    uint32_t ahb_violation_status;
    uint32_t i;
    ahb_secure_fault_info_t ahb_violation_info;

    /* Handling SAU related secure faults */
    PRINTF("\r\nEntering HardFault interrupt!\r\n");
    if (SAU->SFSR != 0)
    {
        if (SAU->SFSR & SAU_SFSR_INVEP_Msk)
        {
            /* Invalid Secure state entry point */
            PRINTF("SAU->SFSR:INVEP fault: Invalid entry point to secure world.\r\n");
        }
        else
        if (SAU->SFSR & SAU_SFSR_AUVIOL_Msk)
        {
            /* AUVIOL: SAU violation  */
            PRINTF("SAU->SFSR:AUVIOL fault: SAU violation. Access to secure memory from normal world.\r\n");
        }
        else
        if (SAU->SFSR & SAU_SFSR_INVTRAN_Msk)
        {
            /* INVTRAN: Invalid transition from secure to normal world  */
            PRINTF("SAU->SFSR:INVTRAN fault: Invalid transition from secure to normal world.\r\n");
        }
        else
        {
            PRINTF("Another SAU error.\r\n");
        }
        if (SAU->SFSR & SAU_SFSR_SFARVALID_Msk)
        {
            /* SFARVALID: SFAR contain valid address that caused secure violation */
            PRINTF("Address that caused SAU violation is 0x%X.\r\n", SAU->SFAR);
        }
    }

    /* Handling secure bus related faults */
    if (SCB->CFSR != 0)
    {
        if (SCB->CFSR & SCB_CFSR_IBUSERR_Msk)
        {
            /* IBUSERR: Instruction bus error on an instruction prefetch */
            PRINTF("SCB->BFSR:IBUSERR fault: Instruction bus error on an instruction prefetch.\r\n");
        }
        else
        if (SCB->CFSR & SCB_CFSR_PRECISERR_Msk)
        {
            /* PRECISERR: Instruction bus error on an instruction prefetch */
            PRINTF("SCB->BFSR:PRECISERR fault: Precise data access error.\r\n");
        }
        else
        {
            PRINTF("Another secure bus error.\r\n");
        }
        if (SCB->CFSR & SCB_CFSR_BFARVALID_Msk)
        {
            /* BFARVALID: BFAR contain valid address that caused secure violation */
            PRINTF("Address that caused secure bus violation is 0x%X.\r\n", SCB->BFAR);
        }
    }

    /* Handling non-secure bus related faults */
    if (SCB_NS->CFSR != 0)
    {
        if (SCB_NS->CFSR & SCB_CFSR_IBUSERR_Msk)
        {
            /* IBUSERR: Instruction bus error on an instruction prefetch */
            PRINTF("SCB_NS->BFSR:IBUSERR fault: Instruction bus error on an instruction prefetch.\r\n");
        }
        else
        if (SCB_NS->CFSR & SCB_CFSR_PRECISERR_Msk)
        {
            /* PRECISERR: Data bus error on an data read/write */
            PRINTF("SCB_NS->BFSR:PRECISERR fault: Precise data access error.\r\n");
        }
        else
        {
            PRINTF("Another secure bus error.\r\n");
        }
        if (SCB_NS->CFSR & SCB_CFSR_BFARVALID_Msk)
        {
            /* BFARVALID: BFAR contain valid address that caused secure violation */
            PRINTF("Address that caused secure bus violation is 0x%X.\r\n", SCB_NS->BFAR);
        }
    }

    /* Handling AHB secure controller related faults.
     * AHB secure controller faults raise secure bus fault. Detail fault info
     * can be read from AHB secure controller violation registers */
    ahb_violation_status = AHB_SECURE_CTRL->SEC_VIO_INFO_VALID;
    if (ahb_violation_status != 0)
    {
        PRINTF("\r\nAdditional AHB secure controller error information:\r\n");
        for (i=0; i<AHB_LAYERS_COUNT;i++)
        {
            if (ahb_violation_status & 0x1U)
            {
                ahb_violation_info.value = AHB_SECURE_CTRL->SEC_VIO_MISC_INFO[i];
                PRINTF("Secure error at AHB layer %d.\r\n", i);
                PRINTF("Address that caused secure violation is 0x%X.\r\n", AHB_SECURE_CTRL->SEC_VIO_ADDR[i]);
                PRINTF("Secure error caused by bus master number %d.\r\n", ahb_violation_info.fault_info.master_number);
                PRINTF("Security level of master %d.\r\n", ahb_violation_info.fault_info.master_sec_level);
                PRINTF("Secure error happened during ");
                switch (ahb_violation_info.fault_info.access_type)
                {
                case 0:
                    PRINTF("read code access.\r\n");
                    break;
                case 2:
                    PRINTF("read data access.\r\n");
                    break;
                case 3:
                    PRINTF("read code access.\r\n");
                    break;
                default:
                    PRINTF("unknown access.\r\n");
                    break;
               }
           }
           ahb_violation_status = ahb_violation_status >> 1;
       }
    }
   /* Perform system RESET */
   SCB->AIRCR = ( SCB->AIRCR & ~SCB_AIRCR_VECTKEY_Msk ) | ( 0x05FAUL << SCB_AIRCR_VECTKEY_Pos ) | SCB_AIRCR_SYSRESETREQ_Msk;
}

