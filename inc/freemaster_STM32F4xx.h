/******************************************************************************
*
* Copyright 2004-2016 NXP Semiconductor, Inc.
*
* This software is owned or controlled by NXP Semiconductor.
* Use of this software is governed by the NXP FreeMASTER License
* distributed with this Material.
* See the LICENSE file distributed for more details.
* 
***************************************************************************//*!
*
* @brief  FreeMASTER Driver hardware dependent stuff
*
******************************************************************************/

#ifndef __FREEMASTER_STM32F4xx_H
#define __FREEMASTER_STM32F4xx_H

/******************************************************************
 * Platform-specific default configuration
 *****************************************************************************/

/* Use 32-bit (EX) commands by default */
#ifndef FMSTR_USE_EX_CMDS
#define FMSTR_USE_EX_CMDS 1
#endif

/* Do not use 16-bit (no-EX) commands by default */
#ifndef FMSTR_USE_NOEX_CMDS
#define FMSTR_USE_NOEX_CMDS 1
#endif

/* At least one of EX or no-EX command handling must be enabled */
#if !FMSTR_USE_EX_CMDS && !FMSTR_USE_NOEX_CMDS
    #error At least one of EX or no-EX command handling must be enabled (please set FMSTR_USE_EX_CMDS)
    #undef  FMSTR_USE_EX_CMDS
    #define FMSTR_USE_EX_CMDS 1
#endif

/*****************************************************************************
* Board configuration information 
******************************************************************************/

#define FMSTR_PROT_VER           3      /* Protocol version 3 */
#define FMSTR_CFG_FLAGS         (FMSTR_CFGFLAG_LITTLEENDIAN | FMSTR_CFG_REC_LARGE_MODE) /* board info flags */
#define FMSTR_CFG_BUS_WIDTH       1     /* Data bus width */
#define FMSTR_GLOB_VERSION_MAJOR 2      /* Driver version */
#define FMSTR_GLOB_VERSION_MINOR 0
#define FMSTR_IDT_STRING  "STM32F4xx FreeMASTER Drv."
#define FMSTR_TSA_FLAGS          0

/******************************************************************************
* Platform-specific types
******************************************************************************/

typedef unsigned char  FMSTR_U8;        /* Smallest memory entity */
typedef unsigned short FMSTR_U16;       /* 16bit value */
typedef unsigned long  FMSTR_U32;       /* 32bit value */

typedef signed char    FMSTR_S8;        /* Signed 8bit value */
typedef signed short   FMSTR_S16;       /* Signed 16bit value */
typedef signed long    FMSTR_S32;       /* Signed 32bit value */

#if FMSTR_REC_FLOAT_TRIG
typedef float          FMSTR_FLOAT;     /* Float value */
#endif

typedef unsigned char  FMSTR_FLAGS;     /* Type to be union-ed with flags (at least 8 bits) */
typedef unsigned char  FMSTR_SIZE8;     /* One-byte size value */
typedef signed short   FMSTR_INDEX;     /* General for-loop index (must be signed) */

typedef unsigned char  FMSTR_BCHR;      /* Type of a single character in comm.buffer */
typedef unsigned char* FMSTR_BPTR;      /* Pointer within a communication buffer */

typedef unsigned char  FMSTR_SCISR;     /* Data type to store SCI status register */

/******************************************************************************
* Communication buffer access functions
******************************************************************************/

void FMSTR_CopyMemory(FMSTR_ADDR nDestAddr, FMSTR_ADDR nSrcAddr, FMSTR_SIZE8 nSize);
FMSTR_BPTR FMSTR_CopyToBuffer(FMSTR_BPTR pDestBuff, FMSTR_ADDR nSrcAddr, FMSTR_SIZE8 nSize);
FMSTR_BPTR FMSTR_CopyFromBuffer(FMSTR_ADDR nDestAddr, FMSTR_BPTR pSrcBuff, FMSTR_SIZE8 nSize);
void FMSTR_CopyFromBufferWithMask(FMSTR_ADDR nDestAddr, FMSTR_BPTR pSrcBuff, FMSTR_SIZE8 nSize);

/* Mixed EX and non-EX commands may occur */
#if (FMSTR_USE_EX_CMDS) && (FMSTR_USE_NOEX_CMDS) || (FMSTR_BYTE_BUFFER_ACCESS)
void FMSTR_SetExAddr(FMSTR_BOOL bNextAddrIsEx);
#else
/* Otherwise, we always know what addresses are used, (ignore FMSTR_SetExAddr) */
#define FMSTR_SetExAddr(bNextAddrIsEx) 
#endif

#if (FMSTR_BYTE_BUFFER_ACCESS)
FMSTR_BPTR FMSTR_ValueFromBuffer16(FMSTR_U16* pDest, FMSTR_BPTR pSrc);
FMSTR_BPTR FMSTR_ValueFromBuffer32(FMSTR_U32* pDest, FMSTR_BPTR pSrc);
FMSTR_BPTR FMSTR_ValueToBuffer16(FMSTR_BPTR pDest, FMSTR_U16 src);
FMSTR_BPTR FMSTR_ValueToBuffer32(FMSTR_BPTR pDest, FMSTR_U32 src);
#endif

/*********************************************************************************
* Communication buffer access functions. Most of them are trivial simple on S32XX
*********************************************************************************/

#define FMSTR_ValueFromBuffer8(pDest, pSrc) \
    ( (*((FMSTR_U8*)(pDest)) = *(FMSTR_U8*)(pSrc)), (((FMSTR_BPTR)(pSrc))+1) )

#if !(FMSTR_BYTE_BUFFER_ACCESS)
#define FMSTR_ValueFromBuffer16(pDest, pSrc) \
    ( (*((FMSTR_U16*)(pDest)) = *(FMSTR_U16*)(pSrc)), (((FMSTR_BPTR)(pSrc))+2) )

#define FMSTR_ValueFromBuffer32(pDest, pSrc) \
    ( (*((FMSTR_U32*)(pDest)) = *(FMSTR_U32*)(pSrc)), (((FMSTR_BPTR)(pSrc))+4) )
#endif

#define FMSTR_ValueToBuffer8(pDest, src) \
    ( (*((FMSTR_U8*)(pDest)) = (FMSTR_U8)(src)), (((FMSTR_BPTR)(pDest))+1) )

#if !(FMSTR_BYTE_BUFFER_ACCESS)
#define FMSTR_ValueToBuffer16(pDest, src) \
    ( (*((FMSTR_U16*)(pDest)) = (FMSTR_U16)(src)), (((FMSTR_BPTR)(pDest))+2) )

#define FMSTR_ValueToBuffer32(pDest, src) \
    ( (*((FMSTR_U32*)(pDest)) = (FMSTR_U32)(src)), (((FMSTR_BPTR)(pDest))+4) )
#endif

#define FMSTR_SkipInBuffer(pDest, nSize) \
    ( ((FMSTR_BPTR)(pDest)) + (nSize) )

#define FMSTR_ConstToBuffer8  FMSTR_ValueToBuffer8
#define FMSTR_ConstToBuffer16 FMSTR_ValueToBuffer16

/* EX address used only: fetching 32bit word */
#if (FMSTR_USE_EX_CMDS) && !(FMSTR_USE_NOEX_CMDS) && !(FMSTR_BYTE_BUFFER_ACCESS)
    #define FMSTR_AddressFromBuffer(pDest, pSrc) \
        FMSTR_ValueFromBuffer32(pDest, pSrc)
    #define FMSTR_AddressToBuffer(pDest, nAddr) \
        FMSTR_ValueToBuffer32(pDest, nAddr)

/* No-EX address used only: fetching 16bit word  */
#elif !(FMSTR_USE_EX_CMDS) && (FMSTR_USE_NOEX_CMDS) && !(FMSTR_BYTE_BUFFER_ACCESS)
    #define FMSTR_AddressFromBuffer(pDest, pSrc) \
        FMSTR_ValueFromBuffer16(pDest, pSrc)
    #define FMSTR_AddressToBuffer(pDest, nAddr) \
        FMSTR_ValueToBuffer16(pDest, nAddr)

/* Mixed addresses used, need to process it programatically */
#else
    FMSTR_BPTR FMSTR_AddressFromBuffer(FMSTR_ADDR* pAddr, FMSTR_BPTR pSrc);
    FMSTR_BPTR FMSTR_AddressToBuffer(FMSTR_BPTR pDest, FMSTR_ADDR nAddr);
#endif

#define FMSTR_GetS8(addr)  ( *(FMSTR_S8*)(addr) )
#define FMSTR_GetU8(addr)  ( *(FMSTR_U8*)(addr) )
#define FMSTR_GetS16(addr) ( *(FMSTR_S16*)(addr) )
#define FMSTR_GetU16(addr) ( *(FMSTR_U16*)(addr) )
#define FMSTR_GetS32(addr) ( *(FMSTR_S32*)(addr) )
#define FMSTR_GetU32(addr) ( *(FMSTR_U32*)(addr) )

#if FMSTR_REC_FLOAT_TRIG
#define FMSTR_GetFloat(addr) ( *(FMSTR_FLOAT*)(addr) )
#endif

/****************************************************************************************
* Other helper macros
*****************************************************************************************/

/* This macro assigns C pointer to FMSTR_ADDR-typed variable */
#define FMSTR_PTR2ADDR(tmpAddr,ptr) ( tmpAddr = (FMSTR_ADDR) (FMSTR_U8*) ptr )
#define FMSTR_ARR2ADDR FMSTR_PTR2ADDR

/****************************************************************************************
* Platform-specific configuration
*****************************************************************************************/


/****************************************************************************************
* General peripheral space access macros
*****************************************************************************************/

#define FMSTR_SETREG8(base, offset, value)      (*(volatile FMSTR_U8*)(((FMSTR_U32)(base))+(offset)) = value)
#define FMSTR_GETREG8(base, offset)             (*(volatile FMSTR_U8*)(((FMSTR_U32)(base))+(offset)))
#define FMSTR_SETBIT8(base, offset, bit)        (*(volatile FMSTR_U8*)(((FMSTR_U32)(base))+(offset)) |= bit)
#define FMSTR_CLRBIT8(base, offset, bit)        (*(volatile FMSTR_U8*)(((FMSTR_U32)(base))+(offset)) &= (FMSTR_U16)~((FMSTR_U16)(bit)))
#define FMSTR_SETREG16(base, offset, value)     (*(volatile FMSTR_U16*)(((FMSTR_U32)(base))+(offset)) = value)
#define FMSTR_GETREG16(base, offset)            (*(volatile FMSTR_U16*)(((FMSTR_U32)(base))+(offset)))
#define FMSTR_SETBIT16(base, offset, bit)       (*(volatile FMSTR_U16*)(((FMSTR_U32)(base))+(offset)) |= bit)
#define FMSTR_CLRBIT16(base, offset, bit)       (*(volatile FMSTR_U16*)(((FMSTR_U32)(base))+(offset)) &= (FMSTR_U16)~((FMSTR_U16)(bit)))
#define FMSTR_TSTBIT16(base, offset, bit)       (*(volatile FMSTR_U16*)(((FMSTR_U32)(base))+(offset)) & (bit))
#define FMSTR_SETREG32(base, offset, value)     (*(volatile FMSTR_U32*)(((FMSTR_U32)(base))+(offset)) = value)
#define FMSTR_GETREG32(base, offset)            (*(volatile FMSTR_U32*)(((FMSTR_U32)(base))+(offset)))
#define FMSTR_SETBIT32(base, offset, bit)       ((*(volatile FMSTR_U32*)(((FMSTR_U32)(base))+(offset))) |= bit)
#define FMSTR_CLRBIT32(base, offset, bit)       ((*(volatile FMSTR_U32*)(((FMSTR_U32)(base))+(offset))) &= ~(bit))
#define FMSTR_TSTBIT32(base, offset, bit)       (*(volatile FMSTR_U32*)(((FMSTR_U32)(base))+(offset)) & (bit))

/****************************************************************************************
* SCI (UART) module constants
*****************************************************************************************/

/* UART (SCI) module registers */
#define FMSTR_USART_SR_OFFSET  (FMSTR_U32)0x00000000
#define FMSTR_USART_CR1_OFFSET (FMSTR_U32)0x0000000C
#define FMSTR_USART_CR2_OFFSET (FMSTR_U32)0x00000010
#define FMSTR_USART_CR3_OFFSET (FMSTR_U32)0x00000014
#define FMSTR_USART_DR_OFFSET  (FMSTR_U32)0x00000004


/* UART (SCI) Control Register bits */
#define FMSTR_USART_TE        (FMSTR_U32)0x00000008
#define FMSTR_USART_RE        (FMSTR_U32)0x00000004
#define FMSTR_USART_TIE       (FMSTR_U32)0x00000080
#define FMSTR_USART_RIE       (FMSTR_U32)0x00000020
#define FMSTR_USART_TCIE      (FMSTR_U32)0x00000040
    
/* UART (SCI) Status registers bits */
#define FMSTR_SCISR_RDRF       (FMSTR_U32)0x00000020
#define FMSTR_SCISR_TDRE       (FMSTR_U32)0x00000080
#define FMSTR_SCISR_TC         (FMSTR_U32)0x00000040
#define FMSTR_SCISR_OR         (FMSTR_U32)0x00000008

/*******************************************************************************************
* SCI (UART) access macros
*****************************************************************************************/

/* Transmitter enable/disable */
#define FMSTR_SCI_TE() FMSTR_SETBIT32(FMSTR_SCI_BASE, FMSTR_USART_CR1_OFFSET, FMSTR_USART_TE)
#define FMSTR_SCI_TD() FMSTR_CLRBIT32(FMSTR_SCI_BASE, FMSTR_USART_CR1_OFFSET, FMSTR_USART_TE)

/* Receiver enable/disable */
#define FMSTR_SCI_RE() FMSTR_SETBIT32(FMSTR_SCI_BASE, FMSTR_USART_CR1_OFFSET, FMSTR_USART_RE)
#define FMSTR_SCI_RD() FMSTR_CLRBIT32(FMSTR_SCI_BASE, FMSTR_USART_CR1_OFFSET, FMSTR_USART_RE)

#define FMSTR_SCI_TE_RE() FMSTR_SETBIT32(FMSTR_SCI_BASE, FMSTR_USART_CR1_OFFSET, FMSTR_USART_RE | FMSTR_USART_TE)

/* Transmitter-empty interrupt enable/disable */
#define FMSTR_SCI_ETXI() FMSTR_SETBIT32(FMSTR_SCI_BASE, FMSTR_USART_CR1_OFFSET, FMSTR_USART_TIE)
#define FMSTR_SCI_CLRTC() FMSTR_CLRBIT32(FMSTR_SCI_BASE, FMSTR_USART_CR1_OFFSET, FMSTR_SCISR_TC)
#define FMSTR_SCI_DTXI() FMSTR_CLRBIT32(FMSTR_SCI_BASE, FMSTR_USART_CR1_OFFSET, FMSTR_USART_TIE)

/* Receiver-full interrupt enable/disable */
#define FMSTR_SCI_ERXI() FMSTR_SETBIT32(FMSTR_SCI_BASE, FMSTR_USART_CR1_OFFSET, FMSTR_USART_RIE)
#define FMSTR_SCI_DRXI() FMSTR_CLRBIT32(FMSTR_SCI_BASE, FMSTR_USART_CR1_OFFSET, FMSTR_USART_RIE)
#define FMSTR_SCI_ETCI() FMSTR_SETBIT32(FMSTR_SCI_BASE, FMSTR_USART_CR1_OFFSET, FMSTR_USART_TCIE)
/* Tranmsit character */
#define FMSTR_SCI_PUTCHAR(ch) FMSTR_SETREG8(FMSTR_SCI_BASE, FMSTR_USART_DR_OFFSET, ch)

/* Get received character */
#define FMSTR_SCI_GETCHAR() FMSTR_GETREG8(FMSTR_SCI_BASE, FMSTR_USART_DR_OFFSET);
                              

/* Read status register */
#define FMSTR_SCI_GETSR()   FMSTR_GETREG32(FMSTR_SCI_BASE, FMSTR_USART_SR_OFFSET)
#define FMSRT_SCI_CLRSR()   FMSTR_CLRBIT32(FMSTR_SCI_BASE, FMSTR_USART_SR_OFFSET,FMSTR_SCISR_RDRF)
/* Read & clear status register */
#define FMSTR_SCI_RDCLRSR() ((FMSTR_GETREG32(FMSTR_SCI_BASE, FMSTR_USART_SR_OFFSET));\
                              FMSTR_SETREG32(FMSTR_SCI_BASE, FMSTR_USART_SR_OFFSET, FMSTR_GETREG32(FMSTR_SCI_BASE, FMSTR_USART_SR_OFFSET));)


#endif /* __FREEMASTER_KEAXX_H */

