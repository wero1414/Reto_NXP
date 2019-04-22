/*
 * Copyright 2018 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#ifndef _VENEER_TABLE_H_
#define _VENEER_TABLE_H_

/*******************************************************************************
 * Definitions
 ******************************************************************************/
typedef  int (*callbackptr)(char const * s1, char const * s2);

/* NOTE: These defines are not related to veneer table. But since they are needed
 *       in both secure and non-secure project, they are placed here */
#define FAULT_NONE                  0
#define FAULT_INV_S_TO_NS_TRANS     1
#define FAULT_INV_S_ENTRY           2
#define FAULT_INV_NS_DATA_ACCESS    3
#define FAULT_INV_NS_DATA2_ACCESS   4
#define FAULT_INV_INPUT_PARAMS      5

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
/*!
 * @brief Entry function for debug PRINTF (DbgConsole_Printf)
 *
 * This function provides interface between secure and normal worlds
 * This function is called from normal world only
 *
 * @param s     String to be printed
 *
*/
void DbgConsole_Printf_NSE(char const * s);

/*!
 * @brief Entry function for two string comparison
 *
 * This function enables/disables Secure GPIO readings from normal world
 * This function is called from normal world only
 *
 * @param uint32_t     Secure GPIO mask
*/
void SecureGPIO_Mask_NSE(uint32_t iomask, _Bool secure);

/*!
 * @brief Entry function for GetTestCaseNumber(void)
 *
 * This function retuns number of actual fault test case
 * This function is called from normal world only
 *
 * @return             number of actual fault test case
*/
uint32_t GetTestCaseNumber_NSE(int a);

uint32_t GetTestCaseKey_NSE(void);

void Encrypt_Payload(unsigned char *Data, unsigned char Data_Length, unsigned int Frame_Counter, unsigned char Direction);
void Calculate_MIC(unsigned char *Data, unsigned char *Final_MIC, unsigned char Data_Length, unsigned int Frame_Counter, unsigned char Direction);
void AES_Encrypt(unsigned char *Data, unsigned char *Key);
#endif /* _VENEER_TABLE_H_ */
