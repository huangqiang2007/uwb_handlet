#include <stdbool.h>
#include <string.h>
#include "em_usart.h"
#include "em_cmu.h"
#include "em_gpio.h"
#include "em_timer.h"
#include "timer.h"
#include "uartdrv.h"
#include "mainctrl.h"
#include "libdw1000.h"
#include "libdw1000Types.h"
#include "main.h"

volatile uint8_t g_slaveStatus = 0;
uint16_t cnt[4];
//uint8_t frm_cnt[4];
uint8_t center_cnt;

extern volatile uint32_t g_Ticks;
//extern volatile uint32_t tx_start_times;

void globalInit(void)
{
	memset((void *)&g_mainCtrlFr, 0x00, sizeof(g_mainCtrlFr));
	memset((void *)&g_recvSlaveFr, 0x00, sizeof(g_recvSlaveFr));
	memset((void *)&g_dwDev , 0x00, sizeof(g_dwDev));
	memset((void *)&g_dwMacFrameSend, 0x00, sizeof(g_dwMacFrameSend));
	memset((void *)&g_dwMacFrameRecv, 0x00, sizeof(g_dwMacFrameRecv));
	memset(&cnt, 0x00, sizeof(cnt));
//	memset(&frm_cnt, 0x00, sizeof(frm_cnt));
	g_dataRecvDone = false;
	g_slaveWkup = false;
	g_cur_mode = DEFAULT_MODE;
}

void powerADandUWB(uint8_t master)
{
	if (master == 1) {
		GPIO_PinModeSet(gpioPortA, 0, gpioModePushPull, 1);
	} else {
		GPIO_PinModeSet(gpioPortA, 0, gpioModePushPull, 0);
	}
}

/*
 * all 5 bytes data add each other and get the 16bits sum.
 * low 8bits store into crc0, high 8bits store into crc1.
 * */
uint16_t CalFrameCRC(uint8_t data[], int len)
{
	int i = 0;
	uint16_t crc_sum = 0;

	for (; i < len; i++)
		crc_sum += data[i];

	return crc_sum;
}

void InitFrame(struct MainCtrlFrame *mainCtrlFr, uint8_t src, uint8_t slave,
		uint8_t type)
{
	uint16_t data_crc = 0;

	mainCtrlFr->head0 = 0xaa;
	mainCtrlFr->head1 = 0x55;
	mainCtrlFr->len = FRAME_LEN; //after frameType and before CRC
	mainCtrlFr->frameID = slave & 0x7;

	//mainCtrlFr->serial = g_recvSlaveFr.serial + 1;
	mainCtrlFr->serial = 0;

	mainCtrlFr->frameCtrl_blank[0] = 0xf3;
	mainCtrlFr->frameCtrl_blank[1] = 0x19;
	mainCtrlFr->frameCtrl_blank[2] = 0x01;
	mainCtrlFr->frameCtrl = (slave & 0x7) + src;
	mainCtrlFr->blank = 0x00;
	mainCtrlFr->frameType = type & 0xf;
	memset(mainCtrlFr->data, 0x00, FRAME_DATA_LEN);

	data_crc = CalFrameCRC(mainCtrlFr->data, FRAME_DATA_LEN);
	mainCtrlFr->crc0 = data_crc & 0xff;
//	mainCtrlFr->crc1 = (data_crc >> 8) & 0xff;
}

int checkSlaveWkup(struct MainCtrlFrame *mainCtrlFr, struct MainCtrlFrame *recvSlaveFr)
{
	/*
	 * send owner, slave ID and back token, all match, then it indicates
	 * slave is waken up.
	 * */
	if (((recvSlaveFr->frameCtrl & 0xff) == (mainCtrlFr->frameCtrl & 0xff))
		&& ((recvSlaveFr->frameType & 0x0f) == (mainCtrlFr->frameType & 0x0f) + 1))
		return 0;
	else
		return -1;
}

/*
 * send a frame to slave and poll the corresponding back token.
 * @src: frame source, main node, manual node or slave node
 * @slave: talk to which slave node
 * @type£º frame type
 *
 * @ret: -1: talk timeout; 0: talk successfully.
 * */
uint8_t TalktoSlave(dwDevice_t *dev, uint8_t src, uint8_t slave, uint8_t type)
{
	int8_t ret = -1;
//	uint16_t pan_id = PAN_ID1, dest_addr = SLAVE_ADDR1 + (slave - 1), source_addr = CENTER_ADDR1;

	InitFrame(&g_mainCtrlFr, src, slave, type);

	/*
	 * to do:  wireless send logic
	 * */
	g_dataRecvDone = false;

	g_cmd_feedback_timeout = g_Ticks + CMD_FEEDBACK_TIMEOUT;

//	dwTxBufferFrameEncode(&g_dwMacFrameSend, 1, 0, pan_id, dest_addr,
//		source_addr, (uint8_t *)&g_mainCtrlFr, sizeof(g_mainCtrlFr));
//	dwSendData(&g_dwDev, (uint8_t *)&g_dwMacFrameSend, sizeof(g_dwMacFrameSend));
	dwSendData(&g_dwDev, (uint8_t *)&g_mainCtrlFr, sizeof(g_mainCtrlFr));

	while (g_Ticks < g_cmd_feedback_timeout) {
		if (g_dataRecvDone == true) {
			ret = checkSlaveWkup(&g_mainCtrlFr, &g_recvSlaveFr);
			if (ret < 0)
				ret = -1;
			break;
		}
	}

	if (ret < 0)
		ret = -1;

	return ret;
}

/*
 * try to wake up all slave, woken slaves are marked in 'g_slaveStatus' var.
 * */
void WakeupSlave(dwDevice_t *dev)
{
	int i = 0, ret = -1;
	g_wakup_timeout = g_Ticks + WAKUP_DURATION;

	/*
	 * wake up duration is 10 minutes
	 * */
	while (g_Ticks < g_wakup_timeout) {
		for (i = 0; i < SLAVE_NUMS; i++) {
			/*
			 * reset command timeout
			 * */
			CMD_FEEDBACK_TIMEOUT = 3;
			ret = TalktoSlave(dev, MAIN_NODE_ID, i + 1, ENUM_SAMPLE_SET);
			if (ret == 0){
				g_slaveStatus |= (1 << i);
			}
			while (g_Ticks < g_cmd_feedback_timeout - 1) ;

		}

		/*
		 * all slaves are waken up
		 * */
		if ((g_slaveStatus & SLAVE_WKUP_MSK) == SLAVE_WKUP_MSK)
			break;

		if (checkSleepCMD(&g_rcvMessage) == 1000)
			return;
	}

	/*
	 * if it exists woken slaves, it can begin to fetch data from slaves.
	 * */
	if ((g_slaveStatus & SLAVE_WKUP_MSK) > 0) {
		g_slaveWkup = true;
		g_cur_mode = MAIN_SAMPLEMODE;
	}
}

//int time_start=0;
//int time_finish=0;
//int time=0;
/*
 * scan all slaves and fetch sample data.
 * */
void RecvFromSlave(dwDevice_t *dev)
{
	uint16_t crc_sum = 0;
	int i = 0, ret = -1;

//	memset(&g_RS422DataFr, 0xff, sizeof(struct RS422DataFrame));
//	g_RS422DataFr.head0 = 0x33;
//	g_RS422DataFr.head1 = 0xcc;
//	g_RS422DataFr.len = 0;

	/*
	 * scan each slave and receive sample data
	 * */
	for (i = 0; i < SLAVE_NUMS; i++) {
		if ((g_slaveStatus & (1 << i)) == (1 << i)) {
			/*
			 * reset command timeout
			 * */
			CMD_FEEDBACK_TIMEOUT = 4;
			ret = TalktoSlave(dev, MAIN_NODE_ID, i + 1, ENUM_SAMPLE_DATA);
			if (ret == 0) {
				cnt[i] = 0;
				crc_sum = CalFrameCRC(g_recvSlaveFr.data, FRAME_DATA_LEN);

				if (g_recvSlaveFr.head0 == 0xaa && g_recvSlaveFr.head1 == 0x55
					&& g_recvSlaveFr.crc0 == (crc_sum & 0xff) && g_recvSlaveFr.len == 0){

					//g_recvSlaveFr.serial = g_recvSlaveFr.serial - 1;
					continue;
				}

				//memcpy(&g_RS422DataFr.packets[g_RS422DataFr.len++], &g_recvSlaveFr, sizeof(g_recvSlaveFr));
				uartPutData((uint8_t *)&g_recvSlaveFr, sizeof(struct MainCtrlFrame));
			}
			else {
				cnt[i] += 1;
				if (cnt[i] > 500){
					cnt[i] = 0;
					g_slaveStatus &= ~(1 << i);
				}
				/*
				 * if all slave is offline, reset flag 'g_slaveWkup' to begin wakeup logic.
				 * */
				if ((g_slaveStatus & SLAVE_WKUP_MSK) == 0) {
					g_slaveWkup = false;
					g_cur_mode = MAIN_IDLEMODE;
				}
			}
		}
		else{
//			g_cmd_unwake_timeout = g_Ticks + UNWAKE_CMD_TIMEOUT;
//			while (g_Ticks < g_cmd_unwake_timeout) {
//				; //wait 2ms for non-waked salve
//			}
			CMD_FEEDBACK_TIMEOUT = 3;
			ret = TalktoSlave(dev, MAIN_NODE_ID, i + 1, ENUM_SAMPLE_SET);
			if (ret == 0){
				g_slaveStatus |= (1 << i);
			}
		}
	}

	/*
	 * if it exists valid sample data coming from slaves,
	 * calculate CRC and send them to control computer.
	 * */
//	for (i = 0; i < g_RS422DataFr.len; i++) {
//		uartPutData((uint8_t *)&g_RS422DataFr.packets[i], sizeof(struct MainCtrlFrame));
//	}
}

/*
 * scan all slaves and fetch sample data.
 * */
void sleepSlave(dwDevice_t *dev)
{
	uint16_t crc_sum = 0;
	int i = 0, ret = -1;
	g_wakup_timeout = g_Ticks + SLEEP_DURATION;
	g_slaveStatus = 0x0F;
	/*
	 * wake up duration is 10 minutes
	 * */
	while (g_Ticks < g_wakup_timeout) {
		/*
		 * scan each slave plus main node, and send sleep command to each one.
		 * */
		for (i = 0; i < SLAVE_NUMS; i++) {
			if ((g_slaveStatus & (1 << i)) == (1 << i)) {

				CMD_FEEDBACK_TIMEOUT = 4;
				ret = TalktoSlave(dev, MAIN_NODE_ID, i + 1, ENUM_SLAVE_SLEEP);
				if (ret == 0) {
					cnt[i] = 0;
					crc_sum = CalFrameCRC(g_recvSlaveFr.data, FRAME_DATA_LEN);

					g_cmd_wake_wait_time = g_Ticks + WAKE_CMD_WAIT_TIME;

					if (g_recvSlaveFr.head0 == 0xaa && g_recvSlaveFr.head1 == 0x55
						&& g_recvSlaveFr.crc0 == (crc_sum & 0xff) && g_recvSlaveFr.len == 0){
//							g_recvSlaveFr.serial = g_recvSlaveFr.serial - 1;
						continue;
					}

					g_slaveStatus &= ~(1 << i);
					uartPutData((uint8_t *)&g_recvSlaveFr, sizeof(struct MainCtrlFrame));
				}
			}
		}
	}

	/*
	 * if all slave is offline, reset flag 'g_slaveWkup' to begin wakeup logic.
	 * */
	if ((g_slaveStatus & SLAVE_WKUP_MSK) == 0) {
		g_slaveWkup = false;
//		g_cur_mode = MAIN_CENTERSLEEPMODE;
		g_cur_mode = DEFAULT_MODE;
		Delay_ms(100);
		return;
	}
}

/*
 * send sleep command to center node
 * */
void sleepCenter(dwDevice_t *dev)
{
	uint16_t crc_sum = 0;
	int ret = -1;
	g_sleep_timeout = g_Ticks + SLEEP_DURATION;
//
	while (g_Ticks < g_sleep_timeout){
		CMD_FEEDBACK_TIMEOUT = 4;
		ret = TalktoSlave(dev, 0, 0, ENUM_SLAVE_SLEEP);
		if (ret == 0) {
			center_cnt = 0;
			crc_sum = CalFrameCRC(g_recvSlaveFr.data, FRAME_DATA_LEN);

			g_cmd_wake_wait_time = g_Ticks + WAKE_CMD_WAIT_TIME;

			if (g_recvSlaveFr.head0 == 0xaa && g_recvSlaveFr.head1 == 0x55
				&& g_recvSlaveFr.crc0 == (crc_sum & 0xff) && g_recvSlaveFr.len == 0){
//				g_recvSlaveFr.serial = g_recvSlaveFr.serial - 1;
			}
			else{
				uartPutData((uint8_t *)&g_recvSlaveFr, sizeof(struct MainCtrlFrame));
				g_cur_mode = DEFAULT_MODE;
				dwIdle(&g_dwDev);
				return;
			}
		}
		else {
			center_cnt += 1;
			if (center_cnt > 200){
				center_cnt = 0;
				g_slaveWkup = false;
				g_cur_mode = DEFAULT_MODE;
				dwIdle(&g_dwDev);
				return;
			}
		}
	}
}
