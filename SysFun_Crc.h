/**
  ******************************************************************************
 
  ******************************************************************************
  */ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SYSDEV_AT_H__
#define __SYSDEV_AT_H__

#include "config.h"
#include <string>
// #define __cplusplus
#ifdef __cplusplus
 extern "C" {
#endif 

// #include <main.h>



#define CRCLIB_USING_CRC16

#ifdef CRCLIB_USING_CRC16
//#define CRC16_USING_CONST_TABLE //using const table in flash memory, nodefined using table in ram
#ifndef CRC16_POLY
#define CRC16_POLY        0xA001 //crc16 polynome, Poly = x16+x15+x2+1 (IBM,SDLC)
//#define CRC16_POLY      0x8408 //crc16 polynome, Poly = x16+x12+x5+1 (CCITT,ISO,HDLC,ITUX25,PPP-FCS)
#endif
#ifndef CRC16_INIT_VAL
#define CRC16_INIT_VAL  0xFFFF
#endif
#endif

 
extern uint16_t crc16_table[256];//crc16计算
extern void crc16_table_init(void);
extern uint16_t crc16_cal(uint8_t *pdata,uint32_t len);

#ifdef __cplusplus
}
#endif
#endif  /* __SYSDEV_AT_H__*/

/*******************(C)COPYRIGHT 2011 STMicroelectronics *****END OF FILE******/