/***************************************************************************//*!
*
* @file   freemaster_cfg.h.example
*
* @brief  FreeMASTER Serial Communication Driver configuration file
*
*******************************************************************************/

#ifndef __FREEMASTER_CFG_H
#define __FREEMASTER_CFG_H

#define FMSTR_DISABLE          0    /* To disable all FreeMASTER functionalities */

/******************************************************************************
* Select interrupt or poll-driven serial communication
******************************************************************************/

#define FMSTR_LONG_INTR        0    /* complete message processing in interrupt */
#define FMSTR_SHORT_INTR       1    /* SCI FIFO-queuing done in interrupt */
#define FMSTR_POLL_DRIVEN      0    /* no interrupt needed, polling only */

/*****************************************************************************
* Select communication interface (SCI or JTAG)
******************************************************************************/

#define FMSTR_SCI_BASE         0x40004400U /* base address of UART register space on STM32F4xx  */
#define FMSTR_SCI_HAS_TXQUEUE  1
#define FMSTR_USE_SCI          1    /* To select SPI1 communication interface */

/******************************************************************************
* Input/output communication buffer size
******************************************************************************/

#define FMSTR_COMM_BUFFER_SIZE 254    /* set to 0 for "automatic" */

/******************************************************************************
* Receive FIFO queue size (use with FMSTR_SHORT_INTR only)
******************************************************************************/

#define FMSTR_COMM_RQUEUE_SIZE 254   /* set to 0 for "default" */

/*****************************************************************************
* Support for Application Commands 
******************************************************************************/

#define FMSTR_USE_APPCMD       1    /* enable/disable App.Commands support */
#define FMSTR_APPCMD_BUFF_SIZE 32   /* App.Command data buffer size */
#define FMSTR_MAX_APPCMD_CALLS 4    /* how many app.cmd callbacks? (0=disable) */

/*****************************************************************************
* Oscilloscope support
******************************************************************************/

#define FMSTR_USE_SCOPE        1    /* enable/disable scope support */
#define FMSTR_MAX_SCOPE_VARS   8    /* max. number of scope variables (2..8) */

/*****************************************************************************
* Recorder support
******************************************************************************/

#define FMSTR_USE_RECORDER     1    /* enable/disable recorder support */
#define FMSTR_MAX_REC_VARS     8    /* max. number of recorder variables (2..8) */
#define FMSTR_REC_OWNBUFF      0    /* use user-allocated rec. buffer (1=yes) */

/* built-in recorder buffer (use when FMSTR_REC_OWNBUFF is 0) */
#define FMSTR_REC_BUFF_SIZE    2048 /* built-in buffer size */

/* recorder time base, specifies how often the recorder is called in the user app. */
#define FMSTR_REC_TIMEBASE     FMSTR_REC_BASE_MICROSEC(64) /* 0 = "unknown" *//*64- 100us*/

/* Fast Recorder Control */
#define FMSTR_USE_FASTREC      0    /* Activate Fast Recorder driver */

#define FMSTR_REC_FLOAT_TRIG   0    /* enable/disable floating point triggering */

/*****************************************************************************
* Target-side address translation (TSA)
******************************************************************************/

#define FMSTR_USE_TSA          0    /* enable TSA functionality */
#define FMSTR_USE_TSA_SAFETY   1    /* enable access to TSA variables only */
#define FMSTR_USE_TSA_INROM    0    /* TSA tables declared as const (put to ROM) */

/*****************************************************************************
* Enable/Disable read/write memory commands
******************************************************************************/

#define FMSTR_USE_READMEM      1    /* enable read memory commands */
#define FMSTR_USE_WRITEMEM     1    /* enable write memory commands */
#define FMSTR_USE_WRITEMEMMASK 1    /* enable write memory bits commands */

/*****************************************************************************
* Enable/Disable read/write variable commands (a bit faster than Read Mem)
******************************************************************************/

#define FMSTR_USE_READVAR      1    /* enable read variable fast commands */
#define FMSTR_USE_WRITEVAR     1  /* enable write variable fast commands */
#define FMSTR_USE_WRITEVARMASK 1    /* enable write variable bits fast commands */

/*****************************************************************************
* Enable/Disable Lite version of FreeMASTER communication driver
******************************************************************************/

#define FMSTR_LIGHT_VERSION       0  /* enable code size reduction of FreeMASTER driver */ 
#define FMSTR_REC_STATIC_POSTTRIG 0  /* Set recorder post trigger samples manually */
#define FMSTR_REC_STATIC_DIVISOR  0  /* Set recorder divisor manually */
#define FMSTR_BYTE_BUFFER_ACCESS  0
#endif

