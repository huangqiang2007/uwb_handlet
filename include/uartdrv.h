#ifndef UARTDRV_H_
#define UARTDRV_H_

#include <stdint.h>
#include "mainctrl.h"

#define UARTFRAME_LEN_12B 12
/*
 * convert AD samples into RS422 frame
 * */
typedef struct {
	/*
	 * the frame of 12 bytes
	 * */
	uint8_t uartFrame[UARTFRAME_LEN_12B];
} UartFrame;

#define UARTCMD_LEN 22
typedef struct rcvMsg {
	uint8_t rcvBytes[UARTCMD_LEN];
	uint8_t len;
	bool searchHeadFlag;
} rcvMsg_t;

#define UPDATECMD_LEN 44
typedef struct rcvUpdateMsg {
	uint8_t rcvBytes[UPDATECMD_LEN];
	uint8_t len;
	bool searchHeadFlag;
} rcvUpdateMsg_t;

typedef struct uartCMD {
	uint8_t begin[5];
	uint8_t frameLen;
	uint8_t reserve[8];
	uint8_t uartCmd[2];
	uint8_t crc[2];
	uint8_t end[4];
} __attribute__((packed)) uartCMD_t;

rcvMsg_t g_rcvMessage;
rcvUpdateMsg_t g_rcvUpdateMessage;

/* Function prototypes */
extern void uartSetup(void);
extern void uartPutData(volatile uint8_t * dataPtr, uint32_t dataLen);
extern uint32_t uartGetData(uint8_t * dataPtr, uint32_t dataLen);
extern void    uartPutChar(uint8_t charPtr);
extern uint8_t uartGetChar(void);
extern uint32_t uartReadChar(uint8_t *data);
extern int UartFrameEnqueue(UartFrame *uFrame);
extern UartFrame* UartFrameDequeue(void);
extern uint32_t checkUartCMD(rcvMsg_t *rcvMessage);
extern uint32_t checkUpdateUartCMD(rcvUpdateMsg_t *rcvMessage);
void DMAInit(void);

#endif /* UARTDRV_H_ */
