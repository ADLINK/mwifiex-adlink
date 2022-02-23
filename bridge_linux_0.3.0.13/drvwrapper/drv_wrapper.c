/** @file drv_wrapper.cpp
 *
 *  @brief This file contains driver ioctl interface related code.
 *
* Copyright 2011 - 2021 NXP
 *
* NXP CONFIDENTIAL
 * The source code contained or described herein and all documents related to
* the source code ("Materials") are owned by NXP, its
* suppliers and/or its licensors. Title to the Materials remains with NXP,
* its suppliers and/or its licensors. The Materials contain
* trade secrets and proprietary and confidential information of NXP, its
* suppliers and/or its licensors. The Materials are protected by worldwide copyright
* and trade secret laws and treaty provisions. No part of the Materials may be
* used, copied, reproduced, modified, published, uploaded, posted,
* transmitted, distributed, or disclosed in any way without NXP's prior
* express written permission.
 *
 * No license under any patent, copyright, trade secret or other intellectual
 * property right is granted to or conferred upon you by disclosure or delivery
 * of the Materials, either expressly, by implication, inducement, estoppel or
 * otherwise. Any license under such intellectual property rights must be
* express and approved by NXP in writing.
 *
 */


#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <ctype.h>
#include <stdlib.h>
#include <poll.h>
#include <time.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/select.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <linux/if.h>
#include <linux/ioctl.h>
#include <linux/wireless.h>
#ifdef NONPLUG_SUPPORT
#ifndef _ANDROID_
#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/hci_lib.h>
#endif
#endif
#include "drv_wrapper.h"

#ifdef MFG_UPDATE

#include "mfgbridge.h"
#include "mfgdebug.h"

#endif

#ifdef RAWUR_BT_STACK
#include <termios.h>
#endif

//15_4
#include <time.h>
#ifdef SPI_SUPPORT
#include <linux/types.h>
#include <linux/spi/spidev.h>
#include <sys/ioctl.h>
#include <linux/gpio.h>
#include <pthread.h>
#endif

/********************************************************
        Local Variables
********************************************************/
#define IW_MAX_PRIV_DEF   128
#define MLAN_ETH_PRIV               (SIOCDEVPRIVATE + 14)

#define BRDG_OFFSET 4

#define SPEC_NFC_EVENT 0xf

//static int ethio_flag = 1;
extern int ethio_flag;
static int        raw_uart_fd = -1;

static int  zigbee_uart_fd = -1;

#ifdef SPI_SUPPORT
static int  zigbee_gpio_fd = -1;
static struct gpiohandle_request gpio_req;
//static struct gpioevent_request gpio_req;
static struct gpiohandle_data gpio_data;
static struct gpioevent_data gpio_event;
static int thread_ctl_flag = GPIO_THREAD_ST_IDLE;
static pthread_t idThread;
static int gpio_stat = GPIO_LOW;
#endif

extern char RAW_UART_PORT[128];      /* default hci Uart port */
extern char DUT_15_4_PORT[128] ;      /* default 15.4 port */

extern int DUT_15_4_If_Type;
extern int DUT_15_4_GPIO_Pin;		  /* default GPIO pin #*/
extern char DUT_15_4_GPIO_PORT[128];  /* default 15.4 GPIO port */
#define RSP_BUF_SIZE              16384
#define RSP_BUF_REFRESH_THRESHOLD 9000
#define RSP_BUF_PURGE_SIZE	 8000
char ziebee_rsp_buf[RSP_BUF_SIZE];

int zigbee_rx_per_enable = 0;
#define RX_BUF_SIZE              16384
#define RX_BUF_REFRESH_THRESHOLD 9000
#define RX_BUF_PURGE_SIZE	 8000
char ziebee_rx_per_buffer[RX_BUF_SIZE];

char  *ziebee_rx_per_buffer_ptr;
unsigned long  rx_per_readlen=0;
time_t time1, time2;

#define SPI_MSG_BUF_SIZE              8192
#define SPI_MSG_BUF_REFRESH_THRESHOLD 7000 
#define SPI_MSG_BUF_PURGE_SIZE		  6000
char spi_message_buf[SPI_MSG_BUF_SIZE];
int spi_buf_st = SPI_BUF_ST_READY;
int spi_buf_len = 0;

/** Private command structure */
typedef struct eth_priv_cmd
{
    /** Command buffer */
    char *buf;
    /** Used length */
    unsigned int used_len;
    /** Total length */
    unsigned int total_len;
} eth_cmd;

static struct ifreq userdata;
static struct iw_priv_args priv_args[IW_MAX_PRIV_DEF];
static int sockfd=0, ioctl_val, subioctl_val;
#ifdef NONPLUG_SUPPORT
static int hci_sock_char_nfc = 0;
static int hci_sock_char_fm = 0;
static int hci_sock_bz = 0;
static char *hci_addr = NULL;
static char addr[18];
static char *hci_intf = "hci0";
static int multi_hci_sock_bz[2]={0};
#ifndef _ANDROID_
static bdaddr_t  multi_hci_addr;
#endif
#endif

#ifdef MFG_UPDATE
static struct _drv_cb Driver;
struct _new_drv_cb *Driver1, *MultiDevPtr=NULL;
int  WiFidevicecnt=1;

static int BT_IF_MODE = HCI_Mode;
#endif // MFG_UPDATE
/********************************************************
        Local Functions
********************************************************/
/**
 *  @brief Get private info.
 *
 *  @param ifName   A pointer to net name
 *  @return         Number of private IOCTLs, -1 on failure
 */
static int
get_private_info_multi(const char *ifName, int sockfd_multi)
{
	struct iwreq iwReq;
	int status = 0;
	struct iw_priv_args *pPrivArgs = priv_args;

	memset(&iwReq, 0, sizeof(iwReq));
	strncpy(iwReq.ifr_name, ifName, IFNAMSIZ);
	iwReq.u.data.pointer = (caddr_t) pPrivArgs;
	iwReq.u.data.length = IW_MAX_PRIV_DEF;
	iwReq.u.data.flags = 0;

	if (ioctl(sockfd_multi, SIOCGIWPRIV, &iwReq) < 0) {
		perror("ioctl[SIOCGIWPRIV]");
		status = -1;
	} else {
		/* Return the number of private ioctls */
		status = iwReq.u.data.length;
	}

	return status;
}

/**
 *  @brief Get Sub command ioctl number
 *
 *  @param cmdIndex		command index
 *  @param privCnt   	Total number of private ioctls availabe in driver
 *  @param ioctlVal    	A pointer to return ioctl number
 *  @param subIoctlVal 	A pointer to return sub-ioctl number
 *  @return             0 on success, otherwise -1
 */
static int
get_subioctl_no(int cmdIndex, int privCnt, int *ioctlVal, int *subIoctlVal)
{
    int j;

	if (priv_args[cmdIndex].cmd >= SIOCDEVPRIVATE) {
		*ioctlVal = priv_args[cmdIndex].cmd;
		*subIoctlVal = 0;
        return 0;
    }

    j = -1;
	while ((++j < privCnt)
           && ((priv_args[j].name[0] != '\0') ||
		   (priv_args[j].set_args != priv_args[cmdIndex].set_args) ||
		   (priv_args[j].get_args != priv_args[cmdIndex].get_args))) {
    }

    /* If not found... */
	if (j == privCnt) {
        printf("Invalid private ioctl definition for: 0x%x\n",
		       priv_args[cmdIndex].cmd);
        return -1;
    }

    /* Save ioctl numbers */
	*ioctlVal = priv_args[j].cmd;
	*subIoctlVal = priv_args[cmdIndex].cmd;

    return 0;
}

/**
 *  @brief Get ioctl number
 *
 *  @param ifName       Interface name
 *  @param privCmd     	Private command name
 *  @param ioctlVal    	A pointer to return ioctl number
 *  @param subIoctlVal 	A pointer to return sub-ioctl number
 *  @return             0 on success, otherwise -1
 */
/*
static int
get_ioctl_no(const char *ifName,
	     const char *privCmd, int *ioctlVal, int *subIoctlVal)
{
	int i, privCnt, status = -1;

  printf("DEBUG>> get_ioctl_no=%s\n",ifName);
	//privCnt = get_private_info(ifName);
  printf("DEBUG>> privCnt=%d\n",privCnt);
	// Are there any private ioctls?
	if (privCnt <= 0 || privCnt > IW_MAX_PRIV_DEF) {
		// Could skip this message ?
		printf("%-8.8s  no private ioctls.\n", ifName);
        ethio_flag = 1;
		status = 0;
	} else {
		for (i = 0; i < privCnt; i++) {
            if (priv_args[i].name[0]
			    && !strcmp(priv_args[i].name, privCmd)) {
				status =
				    get_subioctl_no(i, privCnt, ioctlVal,
						    subIoctlVal);
				break;
			}
		}
	}
	return status;
}
*/
static int
get_ioctl_no_multi(const char *ifName,
	     const char *privCmd, int *ioctlVal, int *subIoctlVal, int sockfd)
{
	int i, privCnt, status = -1;

  printf("DEBUG>> get_ioctl_no::ifName=%s\n",ifName);
	privCnt = get_private_info_multi(ifName, sockfd);
  printf("DEBUG>> privCnt=%d\n",privCnt);
	// Are there any private ioctls?
	if (privCnt <= 0 || privCnt > IW_MAX_PRIV_DEF) {
		// Could skip this message ?
		printf("%-8.8s  no private ioctls.\n", ifName);
        ethio_flag = 1;
		status = 0;
	} else {
		for (i = 0; i < privCnt; i++) {
            if (priv_args[i].name[0]
			    && !strcmp(priv_args[i].name, privCmd)) {
				status =
				    get_subioctl_no(i, privCnt, ioctlVal,
						    subIoctlVal);
				break;
			}
		}
	}
	return status;
}

#ifdef NONPLUG_SUPPORT
static int
get_hci_dev_info(int s, int dev_id, long arg)
{
#ifndef _ANDROID_
    if (BT_IF_MODE==HCI_Mode)
    {
        struct hci_dev_info di = { dev_id:dev_id };

        if (ioctl(s, HCIGETDEVINFO, (void *)&di))
            return 0;

        if (!strcmp(di.name, hci_intf)) {
            ba2str(&di.bdaddr, addr);
            multi_hci_addr = di.bdaddr;
            hci_addr = addr;
        }
    }
#endif
    return 0;
}

/**
 *  @brief Get HCI driver info
 *
 *  @return  0 --- if already loadded, otherwise -1
 */
int
drv_wrapper_get_hci_info(char *cmdname)
{
    int ret = 0;
#ifndef _ANDROID_
    if (BT_IF_MODE==HCI_Mode)
    {
        if (!cmdname) {
            printf("Interface name is not present:%s\n",cmdname);
            return -1;
        }
        hci_intf = cmdname;
        hci_for_each_dev(HCI_UP, get_hci_dev_info, 0);

        ret = (hci_addr == NULL) ? -1 : 0;
    }
#endif
    return ret;
}
//------------------------------------

#ifdef SPI_SUPPORT
int set_spi_frame_hdr(  spi_frame_hdr *pspihdr, 
						unsigned char rst_flag, unsigned char crc_flag, unsigned char ccf_flag,
						unsigned short recv_len, unsigned short data_len)
{
	if(!pspihdr)
		return -1;
	memset(pspihdr, 0, sizeof(spi_frame_hdr));
	pspihdr->rst = rst_flag;
	pspihdr->crc = crc_flag;
	pspihdr->ccf = ccf_flag;
	pspihdr->bit1 = 1;
	pspihdr->bit0 = 0;
	pspihdr->recv_len = recv_len;
	pspihdr->data_len = data_len;

	return 0;
}
//
int read_message_from_spi_buf(char *outBuf, int bufSize)
{
int byte_read = 0;
    while(gpio_stat == GPIO_LOW)
    {
        usleep(20000);
    }
	// check ST flag
	if(spi_buf_st != SPI_BUF_ST_READY)
		return 0;
	spi_buf_st = SPI_BUF_ST_INUSE;

	// get data from spi_buf
	if(bufSize >= spi_buf_len){
		memcpy(outBuf, spi_message_buf, spi_buf_len);
		byte_read = spi_buf_len;

		// flush spi_buf
		memset(spi_message_buf, 0, SPI_MSG_BUF_SIZE);
		spi_buf_len = 0;
	}
	else{
		memcpy(outBuf, spi_message_buf, bufSize);
		byte_read = bufSize;
		spi_buf_len = spi_buf_len - bufSize;
		memcpy(spi_message_buf, &spi_message_buf[bufSize], spi_buf_len);
	}
	// return byte read
	spi_buf_st = SPI_BUF_ST_READY;
	
	return byte_read;
}

int check_and_purge_spi_msg()
{
	char *rx_str_per= NULL;

	// check and purge Rx test message
	if (zigbee_rx_per_enable)
	{
		rx_str_per = strstr(spi_message_buf, "Average");
		if (rx_str_per == NULL)
		{
			//printf("##INFO - Purge SPI message buffer.\n");
			memcpy(&spi_message_buf[0], &spi_message_buf[SPI_MSG_BUF_PURGE_SIZE], (spi_buf_len - SPI_MSG_BUF_PURGE_SIZE));
			spi_buf_len = spi_buf_len - SPI_MSG_BUF_PURGE_SIZE;
			spi_message_buf[spi_buf_len] = (char)0x0;
		}
		return 0;
	}
	// other test 
	return 0;
}
// 
int spi_rx_collection(int spi_fd)
{
    struct spi_ioc_transfer tr;
    int retIOCTL;
    char tr_txbuf[SPI_TR_DEFAULT_SIZE];
    char tr_rxbuf[SPI_TR_DEFAULT_SIZE];
    int rd_len = SPI_TR_DEFAULT_SIZE + 16 + SPI_HEADER_LENGTH;
    spi_frame_hdr *pspihdr;
    memset(&tr,0,sizeof(struct spi_ioc_transfer));

	// check spi_buf_st
	while(spi_buf_st != SPI_BUF_ST_READY){
//		printf("##ST - Buffer is used\n");
		usleep(500);
	}
	// set buffer state
	spi_buf_st = SPI_BUF_ST_READ;

	// prepare header
	memset(tr_txbuf, 0, SPI_TR_DEFAULT_SIZE);
	memset(tr_rxbuf, 0, SPI_TR_DEFAULT_SIZE);
	pspihdr = (spi_frame_hdr *)tr_txbuf;	
	// rst_flag,crc_flag, and ccf_flag set 0
	set_spi_frame_hdr(pspihdr, 0, 0, 0, (unsigned short)SPI_TR_DEFAULT_SIZE, 0);

	// read from SPI
    tr.tx_buf = (unsigned long)tr_txbuf;
    tr.rx_buf = (unsigned long)tr_rxbuf;
    tr.len = rd_len;
    retIOCTL = ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr);
    if (retIOCTL < 0)
    {
        printf("##ERR - Can't read message from SPI\n");
        perror("SPI ioctl: ");
        spi_buf_st = SPI_BUF_ST_READY;
        return retIOCTL;
    }
    // check payload from SPI IO and move to buffer
    pspihdr = (spi_frame_hdr *)tr_rxbuf;
    if(pspihdr->data_len > (SPI_TR_DEFAULT_SIZE - SPI_HEADER_LENGTH))
    {
//        printf("Slave has no TX data, transaction will be dropped\n");
        spi_buf_st = SPI_BUF_ST_READY;
        return 0;
    }
#ifdef _SPINEL_SUPPORT_    
    spi_buf_len = 0;
#endif
    if(pspihdr->data_len > 0)
    {
    	if( (pspihdr->data_len+spi_buf_len) <= SPI_MSG_BUF_SIZE )
    	{
			memcpy(&spi_message_buf[spi_buf_len], &tr_rxbuf[sizeof(spi_frame_hdr)], pspihdr->data_len);
    		spi_buf_len += pspihdr->data_len;
 		}
    }
	// If Rx/RSSI test was started, check and purge buffer
	if (spi_buf_len > SPI_MSG_BUF_REFRESH_THRESHOLD)
	{
		check_and_purge_spi_msg();
	}
	// release buffer
	spi_buf_st = SPI_BUF_ST_READY;

	return 0;
}

// 
int write_cmd_spi(int spi_fd, char *inbuf, char *outbuf, int pl_size)
{
    struct spi_ioc_transfer tr;
    int retIOCTL;
    char tr_txbuf[32];
    char tr_rxbuf[32];
    //int total_size = 0;
    spi_frame_hdr *pspihdr;
    memset(&tr,0,sizeof(struct spi_ioc_transfer));

	// check input
	if(!inbuf)
		return -1; 
	if(pl_size > 15)
		return CMD_15_4_TOO_LONG;

    while(spi_buf_st != SPI_BUF_ST_READY)
    {
//        printf("##RX in progress\n");
        usleep(5000);
    }
    spi_buf_st = SPI_BUF_ST_INUSE;

	// prepare buffer and header
	memset(tr_txbuf, 0, 32);
	memset(tr_rxbuf, 0, 32);
	pspihdr = (spi_frame_hdr *)tr_txbuf;	
//	total_size = ((pl_size+sizeof(spi_frame_hdr)+4)/5)*5;	// on multiple of 5 byte
	// rst_flag,crc_flag, and ccf_flag set 0
	set_spi_frame_hdr(pspihdr, 0, 0, 0, (unsigned char)pl_size, (unsigned char)pl_size);
	  
	// move inbuf CMD data to buffer
	memcpy(&tr_txbuf[sizeof(spi_frame_hdr)], inbuf, pl_size);
	
	// IOCTL call
    tr.tx_buf = (unsigned long)tr_txbuf;
    tr.rx_buf = (unsigned long)tr_rxbuf;
    tr.len = pl_size + 16 + SPI_HEADER_LENGTH;
    retIOCTL = ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr);

    if (retIOCTL < 0)
    {
        printf("##ERR - can't send spi message, error code = %d\n", retIOCTL);
		perror("SPI ioctl: ");
        spi_buf_st = SPI_BUF_ST_READY;
        return retIOCTL;
    }
	// check tr_rxbuf and collect data 
	pspihdr = (spi_frame_hdr *)tr_rxbuf;	
    if(pspihdr->data_len > 0)
    {
    	// read from SPI
    	if(outbuf){
			memcpy(outbuf, tr_rxbuf, 16);
    	}
    }
    spi_buf_st = SPI_BUF_ST_READY;
	return 0;
}
// GPIO IF init
int gpio_ifInit()
{
int retCode;

	/*  Open device: gpiochip5*/
	zigbee_gpio_fd = open(DUT_15_4_GPIO_PORT, O_RDWR);
    usleep(1000);  	

	if (zigbee_gpio_fd == -1) {
		printf("#ERR - Failed to open %s, errno = %d\n", -errno);
		return -errno;
	}
//	printf("##INFO - GPIO device %s opened\n", DUT_15_4_GPIO_PORT);

	/* request GPIO line */
	memset(&gpio_req, 0, sizeof(struct gpiohandle_request));
	gpio_req.lineoffsets[0] = DUT_15_4_GPIO_Pin;
	gpio_req.flags = GPIOHANDLE_REQUEST_INPUT;
	//gpio_req.eventflags = GPIOEVENT_REQUEST_FALLING_EDGE;
	memcpy(gpio_req.default_values, &gpio_data, sizeof(gpio_req.default_values));
	strcpy(gpio_req.consumer_label, "spi_int_gpio_a_15"); //Todo: Need to findout what to assign
	gpio_req.lines  = 1;

//	retCode = ioctl(zigbee_gpio_fd, GPIO_GET_LINEEVENT_IOCTL, &gpio_req);
	retCode = ioctl(zigbee_gpio_fd, GPIO_GET_LINEHANDLE_IOCTL, &gpio_req);

	if (retCode == -1) {
		printf("##ERR - Failed to issue GET LINEEVENT IOCTL (%d)\n", retCode);
	}
	else{
		printf("##INFO - Got GPIO LINEHANDLE request informaiton\n");
	}
	if (close(zigbee_gpio_fd) == -1){
		printf("##ERR - Failed to close GPIO device file\n");
		return -errno;
	}

	return 0;
}
// GPIO IF close
int gpio_ifClose()
{
	int retCode;
	if(gpio_req.fd){
		retCode = close(gpio_req.fd);
		if (retCode == -1) {
			printf("##ERR - Failed to close GPIO device LINEHANDLE, error = %d\n", -errno);
			return -errno;
		}
//		printf("##INFO - GPIO device LINEHANDLE closed\n");
	}
	return 0;
}


// GPIO thead
void* gpio_ifThread(void *arg)
{
//	struct timeval tv;
	int retCode;

//	printf("##INFO - GPIO thread started\n");

	while(thread_ctl_flag == GPIO_THREAD_ST_GO) {
		gpio_data.values[0] = !gpio_data.values[0];
		retCode = ioctl(gpio_req.fd, GPIOHANDLE_GET_LINE_VALUES_IOCTL, &gpio_data);
		if (retCode == -1) {
			retCode = -errno;
			fprintf(stderr, "Failed to issue %s (%d)\n",
				"GPIOHANDLE_SET_LINE_VALUES_IOCTL", retCode);
		}
		else
		{
			if(gpio_data.values[0] == 0)
			{
				gpio_stat = GPIO_LOW;
				spi_rx_collection(zigbee_uart_fd);
			}
			else
			{
				gpio_stat = GPIO_HIGH;
			}
			usleep(4000);
		}

	
	/*
		fd_set rfds;
		int maxFd = gpio_req.fd;

		FD_ZERO(&rfds);
		FD_SET(maxFd, &rfds);

		tv.tv_sec = 0;
		tv.tv_usec = 0;

		retCode = select(maxFd+1, &rfds, NULL, NULL, &tv);
		if (retCode == -1) {
			printf("##ERR - select() error\n");
			thread_ctl_flag == GPIO_THREAD_ST_STOP;
			continue;
		}
		if(FD_ISSET(gpio_req.fd, &rfds)){
           	retCode = ioctl(gpio_req.fd, GPIOHANDLE_GET_LINE_VALUES_IOCTL, &gpio_data);
			if (retCode == -1) {
				perror("##ERR - ioctl() error\n");
				thread_ctl_flag == GPIO_THREAD_ST_STOP;
				continue;
			}
    		read(gpio_req.fd, &gpio_event, sizeof(gpio_event));
			printf("##INFO - GPIO Interrupt %s\n",(gpio_data.values[0]==1)?"active high":"active low");
			// Call SPI message read function 
			spi_rx_collection(zigbee_uart_fd);
        }
        */
	}

//	printf("##INFO - GPIO thread stopped\n");
	return NULL;
}
#endif

int zigbee_rx_per_handle()
{
	long     readlen=0;
        unsigned char buf[8192]="";	
        char *rx_ptr;
	
	time(&time2);
	//if (differ_time <1)
	//	return 0;
	usleep(100000);
	
	if(DUT_15_4_If_Type == DUT_15_4_IF_TYPE_UART)//Uart
  	{
		readlen = read(zigbee_uart_fd, buf, 8192);
	}
#ifdef SPI_SUPPORT
	else // SPI
	{
		memset(buf, 0, 8192);
		readlen = read_message_from_spi_buf((char *)buf, 8192);
    }
#endif
	
	if (readlen)
        {
	 	
       	 	memcpy(&ziebee_rx_per_buffer[rx_per_readlen], buf, readlen); 
		rx_per_readlen += readlen;

		if(rx_per_readlen > RX_BUF_REFRESH_THRESHOLD)
		{
			// Check and clean up buffer
			rx_ptr = strstr(ziebee_rx_per_buffer, "Average");
			if (rx_ptr == NULL)
			{
			 	printf("Purge Rx PER message buffer.\n");
       	 			memcpy(&ziebee_rx_per_buffer[0], &ziebee_rx_per_buffer[RX_BUF_PURGE_SIZE], (rx_per_readlen-RX_BUF_PURGE_SIZE)); 
				rx_per_readlen = rx_per_readlen - RX_BUF_PURGE_SIZE;
				ziebee_rx_per_buffer[rx_per_readlen] = (char)0x0;
			}
		}
	 	//printf("rx_per_readlen>>%d\n", rx_per_readlen);
		if (rx_per_readlen > RX_BUF_SIZE) // Should not happen
		{
			 printf("buffer is full");
			 zigbee_rx_per_enable = 0; //Stop
			return -1;
		}
	}		
	time(&time1);
	return 0;

}
int reset_zigbee_MCU()
{
	 int zigbee_sock = zigbee_uart_fd;
	 char dummy_buf[2*1024]="";
	 char enterkey[] = "\r\n";
	 //int len = 0;
	 int readlen = 0;
	 int timeout = 0;
#ifdef SPI_SUPPORT
	 char cmdbuf[16], outbuf[16];
	 struct spi_ioc_transfer tr;
	 int retIOCTL;
#endif
	 
	    if(DUT_15_4_If_Type == DUT_15_4_IF_TYPE_UART)//Uart
  		{
			write(zigbee_sock,"!", 1); //Reset MCU
			usleep(500000);
		}
#ifdef SPI_SUPPORT
		else // SPI
		{
			int retCode;
			memset(cmdbuf, 0, 16);
			memset(outbuf, 0 ,16);
			memcpy(cmdbuf, "!", 1);
			//write_cmd_spi(zigbee_sock, cmdbuf, outbuf, 1);
			usleep(500000);
			// init thread
        	thread_ctl_flag = GPIO_THREAD_ST_GO;

			retCode = pthread_create(&idThread, NULL, &gpio_ifThread, NULL);
        	if (retCode != 0)
     	       printf("##ERR - Can't create thread\n");
        	else
            	printf("#INFO - Thread created successfully\n");
		}
#endif
		do {
	    	if(DUT_15_4_If_Type == DUT_15_4_IF_TYPE_UART)//Uart
  			{
				readlen = read(zigbee_sock, dummy_buf, 1024);
			}
#ifdef SPI_SUPPORT
			else // SPI
			{
				memset(dummy_buf, 0, 1024);
				//readlen = read_message_from_spi_buf(dummy_buf, 1024);
        	}
#endif
			timeout ++;
			if(readlen)
			{   
				//printf("%d:%s",readlen, dummy_buf);
				timeout =0;
			}
         readlen =0;
	}while(timeout<25);

	if(DUT_15_4_If_Type == DUT_15_4_IF_TYPE_UART)//Uart
  	{
   		write(zigbee_sock,enterkey, sizeof(enterkey)); //Enter key
   	}
#ifdef SPI_SUPPORT
   	else // SPI
   	{
			memset(cmdbuf, 0, 16);
			memset(outbuf, 0, 16);
			memcpy(cmdbuf, enterkey, sizeof(enterkey));
//     		write_cmd_spi(zigbee_sock, cmdbuf, outbuf, sizeof(enterkey));
   	}
#endif
   usleep(500000);
   do {
	    if(DUT_15_4_If_Type == DUT_15_4_IF_TYPE_UART)//Uart
  	    {
            readlen = read(zigbee_sock, dummy_buf, 1024);
	    }
#ifdef SPI_SUPPORT
	    else // SPI	
		{
			memset(dummy_buf, 0, 1024);
//			readlen = read_message_from_spi_buf(dummy_buf, 1024);
        }
#endif

		timeout ++;
		if(readlen)
		{   
			//printf("%d:%s",readlen, dummy_buf);
 			timeout =0;
		}
		
         readlen =0;
	}while(timeout<25);	
	
	return 0;
}

int raw_init_zigbee()
{
  struct termios ti;
  
  // open port for 15.4 DUT
  if(DUT_15_4_If_Type == DUT_15_4_IF_TYPE_UART)//Uart
  {
  	zigbee_uart_fd = open(DUT_15_4_PORT, O_RDWR | O_NOCTTY);

  }
#ifdef SPI_SUPPORT
  else{ // SPI 
  	zigbee_uart_fd = open(DUT_15_4_PORT, O_RDWR );
  }
#endif

  if(DUT_15_4_If_Type == DUT_15_4_IF_TYPE_UART)//Uart
  {
     
  	tcflush(zigbee_uart_fd, TCIOFLUSH);

  if (tcgetattr(zigbee_uart_fd, &ti) < 0)
  {
    perror("Can't get port settings");
    return -1;
  }
  
  cfmakeraw(&ti);
  ti.c_cflag |= CLOCAL;

  // Set 1 stop bit & no parity (8-bit data already handled by cfmakeraw)
  ti.c_cflag &= ~(CSTOPB | PARENB);

  ti.c_cflag |= CRTSCTS;

  //FOR READS:  set timeout time w/ no minimum characters needed (since we read only 1 at at time...)
    ti.c_cc[VMIN] = 0;
    //CRH ti.c_cc[VTIME] = 6 * 100;
     ti.c_cc[VTIME] = 0;	

    cfsetispeed(&ti,B115200);
    cfsetospeed(&ti,B115200);

  if (tcsetattr(zigbee_uart_fd, TCSANOW, &ti) < 0)
  {
    printf("Can't set port settings");
    return -1;
  }

  	tcflush(zigbee_uart_fd, TCIOFLUSH);
  }
#ifdef SPI_SUPPORT
  else // SPI mode
  {
  	int retCode;
  	unsigned char spi_mode = 0;
  	unsigned char bits_per_word = 8;
  	unsigned int spi_speed = 100000;
  	//SPI mode 0

		spi_mode = 0;
        retCode = ioctl(zigbee_uart_fd, SPI_IOC_WR_MODE, &spi_mode);
        if (retCode == -1) {
                printf("Can't set SPI WR mode");
                goto exit;
        }

	spi_mode = 0;
        retCode = ioctl(zigbee_uart_fd, SPI_IOC_RD_MODE, &spi_mode);
        if (retCode == -1) {
                printf("Can't set SPI RD mode");
                goto exit;
        }

        
    //Set bits per word
         
        bits_per_word = 8;
        retCode = ioctl(zigbee_uart_fd, SPI_IOC_WR_BITS_PER_WORD, &bits_per_word);
        if (retCode == -1) {
                printf("Can't set WR bits per word");
                goto exit;
        }

        retCode = ioctl(zigbee_uart_fd, SPI_IOC_RD_BITS_PER_WORD, &bits_per_word);
        if (retCode == -1) {
                printf("Can't set RD bits per word");
                goto exit;
        }

  	//CLK 1M
        spi_speed = 1000000;
        retCode = ioctl(zigbee_uart_fd, SPI_IOC_WR_MAX_SPEED_HZ, &spi_speed);
        if (retCode == -1) {
                printf("can't set RW max speed hz");
                goto exit;
        }

        retCode = ioctl(zigbee_uart_fd, SPI_IOC_RD_MAX_SPEED_HZ, &spi_speed);
        if (retCode == -1) {
                printf("Can't set RD max speed hz");
                goto exit;
        }
        // init GPIO
        gpio_ifInit();
#ifdef _SPINEL_SUPPORT_
		// init thread
        thread_ctl_flag = GPIO_THREAD_ST_GO;

		retCode = pthread_create(&idThread, NULL, &gpio_ifThread, NULL);
        if (retCode != 0)
            printf("##ERR - Can't create thread\n");
        else
            printf("#INFO - Thread created successfully\n");
#endif
  }
#endif
#ifndef _SPINEL_SUPPORT_
  reset_zigbee_MCU();
#endif

  return 0;

exit:
    // close fd
    printf("##FILE - close 15.4 interface file\n");
    close(zigbee_uart_fd);
    zigbee_uart_fd = 0;	
    return -1;
}

int drv_wrapper_deinit_zigbee()
{
    int ret = 0;

    printf("De-Initialize drvwrapper for 15_4 radio....\n");

    if (zigbee_uart_fd)
        close(zigbee_uart_fd);

    zigbee_uart_fd = 0;
#ifdef SPI_SUPPORT
 	if(DUT_15_4_If_Type == DUT_15_4_IF_TYPE_SPI)//SPI
 	{  
		thread_ctl_flag = GPIO_THREAD_ST_STOP;
    	gpio_ifClose();
   	}
#endif
    return ret;
}
int read_zigbee(unsigned char *buf, int buflen)
{
	char rcv_buf[2*1024]="";
	int readlen = 0;
      int zigbee_sock = zigbee_uart_fd;
	int timeout=0, recvcnt=0;
	//char *str1;
     
	
	memset(rcv_buf, 0x00, sizeof(rcv_buf)); //Set to 0xff

	if(DUT_15_4_If_Type == DUT_15_4_IF_TYPE_UART)//Uart
	{
        readlen = read(zigbee_sock, rcv_buf, 2*1024);

    //;str1 = strchr(rcv_buf, ">");
    //;printf("CRH::%d\n", str1);

    readlen = strlen(rcv_buf);
    for (timeout =0; timeout < sizeof(rcv_buf); timeout ++)
    {
        if (rcv_buf[timeout] == 0x0d)
            recvcnt ++;			
    }
    readlen = readlen + recvcnt;
	}
#ifdef SPI_SUPPORT
	else // SPI 
	{
		readlen = read_message_from_spi_buf(rcv_buf, 2*1024);
	}
#endif
	
	if(readlen < buflen)
		memcpy(buf, rcv_buf, readlen);
	else
		memcpy(buf, rcv_buf, buflen);
	
	//;printf("return %d\n", readlen);
        return readlen;
			
}

char * check_TxRspMesg(char *inpBuf, int inpLen)
{
	char *chkPtr = NULL;
	Wrapper_Spinel_CMD_Hdr *hdrPtr;

	if (!inpBuf)
		return inpBuf;

#ifdef _SPINEL_SUPPORT_
	// Check comand response from Spinel layer
	hdrPtr = (Wrapper_Spinel_CMD_Hdr *)inpBuf;
	//check FLG, SPINEL_CMD_NUM, length;
	if ((hdrPtr->FLG == WRAPPER_SPINEL_MFG_CMD_FLG) && (hdrPtr->SPINEL_CMD_NUM == WRAPPER_SPINEL_MFG_CMD_NUM))
	{
		if (hdrPtr->MFG_CMD_Payload_Length == (inpLen - sizeof(Wrapper_Spinel_CMD_Hdr)))
		{
			printf("##INFO - Get complete MFG respnse\n");
			return  inpBuf;
		}
	}
#else
	// Check ESMAC text message 
	chkPtr = strstr(inpBuf, "Tx DONE");
#endif
	return chkPtr;
}

/*
 * int drv_wrapper_send_hci_command(short ogf, short ocf,  unsigned char *in_buf, int in_buf_len,
 * unsigned char *out_buf, int *out_buf_len)
 */
int
drv_proc_ziebee_command(drv_cb * drv, unsigned char *buf, int *msglen, int buflen,int HciDeviceID)
{
#define EVENT_BUF_SIZE 400
    int status = 0;
    int hci_sock = zigbee_uart_fd;
	char  txbuf[16]="";
	long delay=0;
    zigbee_cmd_header   *cmd_ptr = (zigbee_cmd_header *) buf;
	zigbee_cmd_rsp_header *cmd_rsp_ptr = (zigbee_cmd_rsp_header *)ziebee_rsp_buf;
	int cmd_ptr_type =0 ;
	
    int readlen=0;
	int err=0;
    char *rx_str_per = NULL; 
    char *tx_str_per = NULL;
    char local_buf[10000];
#ifdef SPI_SUPPORT
    char outbuf[16];
#endif
    
    memcpy(txbuf, cmd_ptr->data, cmd_ptr->length);
    //printf("CMD len = %d\n", cmd_ptr->length);
    //printf("CMD type = %d\n", cmd_ptr->type);
    //printf("CMD control = %d\n", cmd_ptr->controldata);
    //printf("cmd_ptr->data %02x %02x %02x %02x\n", cmd_ptr->data[0], cmd_ptr->data[1],cmd_ptr->data[2],cmd_ptr->data[3]);

	if(DUT_15_4_If_Type == DUT_15_4_IF_TYPE_UART)//Uart
  	{
		write(hci_sock, txbuf,   cmd_ptr->length);  //Key
	}
#ifdef SPI_SUPPORT
	else // SPI
	{
		memset(outbuf, 0, 16);
        write_cmd_spi(hci_sock, txbuf, outbuf, cmd_ptr->length);
	}
#endif
    usleep(150000);

    if(cmd_ptr->type == delay_for_read_command_type)
    {
		//printf("##INFO - Delay type CMD\n");
        delay = (cmd_ptr->controldata)* 1600;
        //printf("Delay value=%d\n", delay);
        while(delay > 0)
        {
            readlen = read_zigbee((unsigned char *)local_buf, 10000);
            //tx_str_per =strstr(local_buf, "Tx DONE");
			tx_str_per = check_TxRspMesg((char *)local_buf, readlen);
            if(tx_str_per != NULL)
            {
                printf("Found key string\n");
                break;
            }
            usleep(100000);
            delay = delay - 100000;
        }
    }
    else if(cmd_ptr->type == rx_per_stop)
    {
        if(zigbee_rx_per_enable)
        {
            printf("*************rx_per_stop::\n");
            zigbee_rx_per_handle();
            zigbee_rx_per_enable = 0;
            //printf("Total length %d\n",rx_per_readlen);
            rx_str_per =strstr(ziebee_rx_per_buffer,"Average");
            if (rx_str_per != NULL)
            {
                printf("Found target Rx PER key string\n");
                readlen = rx_per_readlen - (rx_str_per - ziebee_rx_per_buffer);
                printf("Report len = %d\n", readlen);
                memcpy(cmd_rsp_ptr->data, rx_str_per, readlen);
                *msglen = readlen + 3;  // add zigbee_cmd_rsp_header header 	
                memcpy(buf, cmd_rsp_ptr, sizeof(zigbee_cmd_rsp_header));
                return 0;
            }
        }

    }
    else
    {
        cmd_ptr_type = cmd_ptr->type;
    }

    readlen = read_zigbee(cmd_rsp_ptr->data, buflen);
    *msglen = readlen + 3;  // add zigbee_cmd_rsp_header header

    if(cmd_ptr->type == delay_for_read_command_type){
        if(tx_str_per != NULL)
        {
            memcpy(cmd_rsp_ptr->data, tx_str_per, 7);
            readlen = 7;
            *msglen = readlen + 3;  // add zigbee_cmd_rsp_header header
        }
    }
 
    memcpy(buf, cmd_rsp_ptr, sizeof(zigbee_cmd_rsp_header));

    //;printf("2************%d\n",cmd_ptr->type);
    if(cmd_ptr_type == rx_per_start)
    {
        if(zigbee_rx_per_enable == 0)
        {
            printf("************rx_per_start\n");
            zigbee_rx_per_enable =1;
            rx_per_readlen = 0;
            memset (ziebee_rx_per_buffer, 0x00, sizeof (ziebee_rx_per_buffer));
            time(&time1);

//        if (err !=0)
//            printf("Create thread Fail!!");
//        else
//            printf("Create thread Success!!");
        }
    }
    return readlen;
}

//;#endif /* NONPLUG_SUPPORT */


//------------------------------------
#ifdef RAWUR_BT_STACK
/**
 *  @brief raw_init_uart
 *
 *  @param intfname Interface name
 *  @return  0 on success, otherwise -1
 */
int raw_init_uart(drv_config * drv_conf)
{
  struct termios ti;
  //raw_uart_fd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY);
  raw_uart_fd = open(RAW_UART_PORT, O_RDWR | O_NOCTTY);

  tcflush(raw_uart_fd, TCIOFLUSH);

  if (tcgetattr(raw_uart_fd, &ti) < 0)
  {
    perror("Can't get port settings");
    return -1;
  }

  cfmakeraw(&ti);
  ti.c_cflag |= CLOCAL;

  // Set 1 stop bit & no parity (8-bit data already handled by cfmakeraw)
  ti.c_cflag &= ~(CSTOPB | PARENB);

  ti.c_cflag |= CRTSCTS;

  //FOR READS:  set timeout time w/ no minimum characters needed (since we read only 1 at at time...)
    ti.c_cc[VMIN] = 0;
    ti.c_cc[VTIME] = 6 * 100;

    cfsetispeed(&ti,drv_conf->BAUDRATE);
    cfsetospeed(&ti,drv_conf->BAUDRATE);


  if (tcsetattr(raw_uart_fd, TCSANOW, &ti) < 0)
  {
    perror("Can't set port settings");
    return -1;
  }
  tcflush(raw_uart_fd, TCIOFLUSH);

  return raw_uart_fd;

}
#endif

/**
 *  @brief drv wrapper initialize HCI
 *
 *  @param intfname Interface name
 *  @return  0 on success, otherwise -1
 */
int
drv_wrapper_init_hci(char *cmdname, drv_config * drv_conf)
{
    int ret = 0;

    if(BT_IF_MODE == BTRAWUR_Mode)
    {
        raw_init_uart(drv_conf);
    }
#ifdef MARVELL_BT_STACK

	if(BT_IF_MODE==MBTCHAR_Mode)
	{
		printf("Initialize  drvwrapper for BT(mbtchar) ....\n");
		hci_sock_bz = open(cmdname, O_RDWR | O_NOCTTY);
		if (hci_sock_bz >= 0) {
			printf("Use device %s\n", cmdname);
		} else {
			printf("Cannot open device %s", cmdname);
			ret = -1;
		}
	}
#endif
#ifndef _ANDROID_
    if(BT_IF_MODE == HCI_Mode)
    {
		if (!cmdname) {
			printf("Interface name is not present\n");
			return -1;
		}
		hci_intf = cmdname;
		hci_addr = NULL;
		printf("Initialize  drvwrapper for BT(HCI) ....\n");
		hci_for_each_dev(HCI_UP, get_hci_dev_info, 0);
		if (hci_addr == NULL) {
			printf("BT interface is not present:%s\n",cmdname);
			ret = -1;
		} else {
			int hci_dev_id;
        hci_dev_id = hci_devid(hci_intf);
			hci_sock_bz = hci_open_dev(hci_dev_id);
			ret = 0;
		}
    }
#endif
    return ret;
}

/**
 *  @brief drv wrapper de-initialize HCI
 *
 *  @return  0 on success, otherwise -1
 */
int
drv_wrapper_deinit_hci()
{
    int ret = 0;

    printf("De-Initialize drvwrapper for BT....\n");
#ifdef RAWUR_BT_STACK
       close(raw_uart_fd);
#endif
    if (hci_sock_char_nfc)
        close(hci_sock_char_nfc);
    if (hci_sock_char_fm)
        close(hci_sock_char_fm);
    if (hci_sock_bz)
        close(hci_sock_bz);
    hci_addr = NULL;
    return ret;
}

/**
 *  @brief drv wrapper send HCI command
 *
 *  @return  0 on success, otherwise error code
 */
int
drv_wrapper_send_hci_command(short ogf, short ocf, unsigned char *in_buf,
                             int in_buf_len, unsigned char *out_buf,
                             int *out_buf_len, int HciDeviceID)
{
	return 0;
}
#endif /* NONPLUG_SUPPORT */

/**
 *  @brief Get driver info
 *
 *  @return  0 --- if already loadded, otherwise -1
 */
int
drv_wrapper_get_info(char *cmdname)
{
    struct ifreq ifr;
    int sk, ret = 0;

    if(!cmdname){
	printf("No cmdname provided\n");
	return -1;
    }
    printf("DEBUG>>drv_wrapper_get_info=%s\n",cmdname);
    /* Open socket to communicate with driver */
    sk = socket(AF_INET, SOCK_STREAM, 0);
    if (sk < 0) {
        printf("Failed to create socket\n");
        return -1;
    }
    strncpy(ifr.ifr_name, cmdname, IFNAMSIZ);
    if (ioctl(sk, SIOCGIFFLAGS, &ifr) < 0)
        ret = -1;
    close(sk);
    return ret;
}

/**
 *  @brief drv wrapper initialize
 *
 *  @return  0 on success, otherwise -1
 */
int
drv_wrapper_init(char *cmdname, int devid)
{
    int ret = 0;

    printf("Initialize drvwrapper ....\n");
     printf("DEBUG>>drv_wrapper_init =%s\n", cmdname);
    /* Open socket to communicate with driver */
    sockfd= socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0) {
        printf("Invalid sockfd, function return -1\n");
        return -1;
    }
  //;  if (get_ioctl_no(cmdname, "hostcmd", &ioctl_val, &subioctl_val)
  //;      == -1)
  {
        printf("No private IOCTL present for command \"hostcmd\"\n");
        close(sockfd);
        sockfd = 0;
        return -1;
    }

    return ret;
}

/**
 *  @brief drv wrapper initialize for multi WiF dev
 *
 *  @return  0 on success, otherwise -1
 */
int
drv_wrapper_init_multi(new_drv_cb *drv)
{
    int ret = 0;

    printf("Initialize drv_wrapper_init_multi ....\n");
     printf("DEBUG>>drv_wrapper_init =%s\n", &drv->wlan_ifname);
    /* Open socket to communicate with driver */
    drv->sockfd= socket(AF_INET, SOCK_STREAM, 0);
    printf("DEBUG>>sockfd=%d\n", drv->sockfd);
    if (drv->sockfd < 0) {
        printf("Invalid sockfd, function return -1\n");
        return -1;
    }
    if (get_ioctl_no_multi((char *)&drv->wlan_ifname, "hostcmd", &drv->ioctl_val, &drv->subioctl_val,drv->sockfd)
        == -1) {
        printf("No private IOCTL present for command \"hostcmd\"\n");
        close(drv->sockfd);

        drv->sockfd=0;
		printf("drv->sockfd=0\n");
        return -1;
    }

    printf("Debug>>get_ioctl_no_multi::drv->ioctl_val=0x%x, drv->subioctl_val=0x%x, drv->sockfd=0x%x\n",drv->ioctl_val, drv->subioctl_val, drv->sockfd);
    return ret;
}
/**
 *  @brief drv wrapper de-initialize
 *
 *  @return  0 on success, otherwise -1
 */
int
drv_wrapper_deinit()
{
    int ret = 0;

    printf("De-Initialize drvwrapper ....\n");
    if (sockfd)
        close(sockfd);
    if(sockfd)
    		close(sockfd);
    ioctl_val =0;
    subioctl_val =0;
    sockfd =0;

    return ret;
}
int drv_wrapper_deinit_multi()
{
    int ret = 0;
    int cnt=0;
    printf("De-Initialize drvwrapper multi....\n");
    MultiDevPtr = Driver1;
      for (cnt=0; cnt<WiFidevicecnt; cnt++)
      {
    		printf("DEBUG>>drv_wrapper_deinit_multi:: wlan_ifname=%s\n", &MultiDevPtr->wlan_ifname);
    		if (MultiDevPtr->sockfd)
        	close(MultiDevPtr->sockfd);
        MultiDevPtr->ioctl_val =0;
         MultiDevPtr->subioctl_val =0;
         MultiDevPtr->sockfd =0;
    		MultiDevPtr++;
    	}
    return ret;
}

#ifdef NONPLUG_SUPPORT

/*
 * int drv_wrapper_send_hci_command(short ogf, short ocf,  unsigned char *in_buf, int in_buf_len,
 * unsigned char *out_buf, int *out_buf_len)
 */
int
drv_proc_hci_command(drv_cb * drv, unsigned char *buf, int *msglen, int buflen,int HciDeviceID)
{
#define EVENT_BUF_SIZE 400
    int status = 0;
    int hci_sock = 0;
    int len;
#ifndef _ANDROID_
    struct hci_filter flt;
#endif
    int dummy_buf[512];
    int avail = 0;

    unsigned short ogf;
    unsigned short ocf;
    unsigned char *evntbuf = drv->cb_buf;

 if(BT_IF_MODE == MBTCHAR_Mode || BT_IF_MODE == BTRAWUR_Mode)
 {
#ifdef RAWUR_BT_STACK
	hci_sock = raw_uart_fd;
#else
   	hci_sock = multi_hci_sock_bz[0];
#endif
    len = write(hci_sock, buf, *msglen);
    if (len != *msglen) {
        printf("Failed to write %d bytes (written %d)\n", *msglen, len);
        status = -1;
    }
   }
#ifndef _ANDROID_
  else
	{
	    ogf = buf[2] >> 2;
		ocf = ((buf[2] & 0x03) << 8) + buf[1];

		hci_sock = multi_hci_sock_bz[HciDeviceID];
		printf("CRH_drv_proc_hci_command: hci_sock =%d\n",hci_sock);
		/* Setup filter */
		hci_filter_clear(&flt);
		hci_filter_set_ptype(HCI_EVENT_PKT, &flt);
		hci_filter_all_events(&flt);
		if (setsockopt(hci_sock, SOL_HCI, HCI_FILTER, &flt, sizeof(flt)) < 0) {
			mfg_dprint(DBG_ERROR, "HCI filter setup failed\n");
			return -1;
		}

		status = hci_send_cmd(hci_sock, ogf, ocf, *msglen - 4, buf + 4);

		if (status < 0) {
			mfg_dprint(DBG_ERROR,
					"Failed to send command (OGF = %04x OCF = %04x)\n",
					ogf, ocf);
			return status;
		}
	}
#endif
    do {
        *msglen = read(hci_sock, evntbuf, EVENT_BUF_SIZE);
        printf("Rx Event %X for %2.2X %2.2X \n", evntbuf[1], evntbuf[4],
               evntbuf[5]);
        printf("In Cmd  %2.2X %2.2X \n", buf[1], buf[2]);

    } while ((evntbuf[1] != 0xFF)
             && !(evntbuf[1] != 0xF && evntbuf[4] == buf[1]
                  && evntbuf[5] == buf[2]));

    if (evntbuf[1] == 0xFF && evntbuf[3] != 0xc0)
        avail = read(hci_sock, dummy_buf, EVENT_BUF_SIZE);

    memcpy(buf, evntbuf, *msglen);
    return status;
}

#endif /* NONPLUG_SUPPORT */

int
drv_config_default(drv_config * drv_conf)
{
    memset(drv_conf, 0, sizeof(drv_config));

#ifdef NONPLUG_SUPPORT
    sprintf(drv_conf->hci_port[0], "hci0");
	sprintf(drv_conf->hci_port[1], "hci1");
#endif
    sprintf(drv_conf->wlan_port, "mlan0");

    return 0;
}

int
drv_init(struct _bridge_cb *bridge, drv_config * drv_conf)
{
    drv_cb *drv;

    int cnt=0;

    bridge->drv = &Driver;
    drv = bridge->drv;
    BT_IF_MODE = drv_conf->BT_IF_Mode;

#ifdef NONPLUG_SUPPORT
    drv->hci_ifname_0 = drv_conf->hci_port[0];
	drv->hci_ifname_1 = drv_conf->hci_port[1];

    //
   printf("DEBUG>>drv_init>>HCI0=%s\n",drv->hci_ifname_0);
   printf("DEBUG>>drv_init>>HCI1=%s\n",drv->hci_ifname_1);
#endif

    drv->wlan_ifname = drv_conf->wlan_port;
    drv->load_script = drv_conf->load_script;
    drv->unload_script = drv_conf->unload_script;

    /** Initialize drvwrapper, if driver already loadded */
    /** Init WiFi */
/*
    if (!drv_wrapper_get_info(drv->wlan_ifname))
        drv_wrapper_init(drv->wlan_ifname, 0);
  */
    printf("MSG>>drv_init\n");
    MultiDevPtr = Driver1;
    for (cnt =0 ;cnt <WiFidevicecnt; cnt++)
    {

    	 if (!drv_wrapper_get_info((char *)&MultiDevPtr->wlan_ifname))
        drv_wrapper_init_multi(MultiDevPtr);
        MultiDevPtr++;

    }
#ifdef NONPLUG_SUPPORT
    /** Init BT */
	if (strstr(drv->hci_ifname_0,"/dev/mbtchar"))
	{
		BT_IF_MODE=MBTCHAR_Mode;
	}

#ifdef MARVELL_BT_STACK
	else
         {
	  	printf("BT Interface name is not present(must be /dev/mbtcharX):%s\n",drv->hci_ifname_0);
	 }
#endif //#ifdef MARVELL_BT_STACK
    if (!drv_wrapper_get_hci_info(drv->hci_ifname_0))
	{
        if (!drv_wrapper_init_hci(drv->hci_ifname_0, drv_conf))
		  multi_hci_sock_bz[0] =hci_sock_bz;
	   printf("DEBUG>>multi_hci_sock_bz[0]=0x%x\n",multi_hci_sock_bz[0]);
	}
   if(BT_IF_MODE !=MBTCHAR_Mode) //Current bridge support one btchar only.
   {
	if (!drv_wrapper_get_hci_info(drv->hci_ifname_1))
	{
			if (!drv_wrapper_init_hci(drv->hci_ifname_1, drv_conf))
			multi_hci_sock_bz[1] =hci_sock_bz;
		printf("DEBUG>>multi_hci_sock_bz[1]=0x%x\n",multi_hci_sock_bz[1]);
	}
   }
   //zigbee init
   {
		 raw_init_zigbee();
   }

#endif
    mfg_dprint(DBG_MINFO, "DRV:  driver is initialized.\n");
    return 0;
}

int
drv_proc_wlan_command(drv_cb * drv, unsigned char *buf, int *rsplen, int buflen)
{
    int status = -1;
    HostCmd_DS_Gen *hostcmd_hdr = (HostCmd_DS_Gen *) buf;
    char *ifname = drv->wlan_ifname;
    eth_cmd ether_cmd;

    int eth_header_len;
    char *buffer;
    mfg_Cmd_t *CMDPtr= (mfg_Cmd_t*) buf;

    {
     	printf("DEBUG>>drv_proc_wlan_command::DeviceId=0x%x\n",CMDPtr->deviceId);
		//Device ID check
		MultiDevPtr = Driver1;
		  if (abs(CMDPtr->deviceId) > WiFidevicecnt)
		{
			printf("DEBUG>>drv_proc_wlan_command::Fail!!Invalid DeviceId=%d(%d), force to Device 0\n", CMDPtr->deviceId, WiFidevicecnt);
			MultiDevPtr = Driver1;
		}
		else
			MultiDevPtr +=(CMDPtr->deviceId);
		if(MultiDevPtr->sockfd ==0)
		{
			printf("DEBUG>>MultiDevPtr->wlan_ifname =%s not exist, force command to ", MultiDevPtr->wlan_ifname);
			MultiDevPtr = Driver1;
			printf("%s\n", MultiDevPtr->wlan_ifname);
		}
     	printf("DEBUG>>drv_proc_wlan_command::wlan_ifname =%s\n", MultiDevPtr->wlan_ifname);
     	ifname = MultiDevPtr->wlan_ifname;
    }
    if (ethio_flag == 1) {
        struct ifreq ifr;
        char *p_cmd = "MRVL_CMDhostcmd";
        eth_header_len = strlen(p_cmd) + BRDG_OFFSET;
        eth_cmd *cmd = &ether_cmd;

        buffer = (char *) malloc(buflen + eth_header_len);
        if (buffer == NULL) {
            printf("can't allocate buffer \n");
            exit(0);
        }

        strncpy((char *) buffer, p_cmd, strlen(p_cmd));
        memcpy(buffer + eth_header_len, buf, buflen);

        /* Fill up buffer */
        cmd->buf = buffer;
        cmd->used_len = 0;
        cmd->total_len = buflen + eth_header_len;

        /* Perform IOCTL */
        memset(&ifr, 0, sizeof(struct ifreq));
        strncpy(ifr.ifr_ifrn.ifrn_name, ifname, strlen(ifname));
        ifr.ifr_ifru.ifru_data = (void *) cmd;
        //Check for device number
        //Needs to open
         printf("DEBUG>>drv_proc_wlan_command::MultiDevPtr->sockfd=%d\n", MultiDevPtr->sockfd);
         printf("DEBUG>>drv_proc_wlan_command::MultiDevPtr->wlan_ifname =%s\n", MultiDevPtr->wlan_ifname);
         printf("DEBUG>>drv_proc_wlan_command::MultiDevPtr->ioctl_val =%d\n", MultiDevPtr->ioctl_val);
        if (ioctl(MultiDevPtr->sockfd, MLAN_ETH_PRIV, &ifr)) {
			   printf("ether bridge: hostcmd fail\n");
		    }
		{
			//RSP
			mfg_Cmd_t *RspCMDPtr= (mfg_Cmd_t*) (cmd->buf+eth_header_len);
			printf("ether bridge MLAN_ETH_PRIV: RSP=0x%x \n", RspCMDPtr->header.len);
			*rsplen = RspCMDPtr->header.len;
		}
        memcpy(buf, cmd->buf + eth_header_len, *rsplen);
        free(buffer);
    } else {
        //Check for device number
        *rsplen = 0;
        printf("DEBUG>>drv_proc_wlan_command:ifname=%s\n", ifname);

        memset(&userdata, 0,  sizeof(struct ifreq));
        strncpy(userdata.ifr_name, ifname, strlen(ifname));
        userdata.ifr_data = (char *) buf;
        mfg_dprint(DBG_GINFO, "DRV:  send host cmd to ioctl\n");
         printf("DEBUG>>drv_proc_wlan_command::MultiDevPtr->sockfd=%d\n", MultiDevPtr->sockfd);
         printf("DEBUG>>drv_proc_wlan_command::MultiDevPtr->wlan_ifname =%s\n", MultiDevPtr->wlan_ifname);
         printf("DEBUG>>drv_proc_wlan_command::MultiDevPtr->ioctl_val =%d\n", MultiDevPtr->ioctl_val);
	printf("\n");
#if 0
        {
           int *data = (int *)&userdata;
           printf("DEBUG>>%d\n", sizeof(struct ifreq));

	   for(int i=0; i<sizeof(struct ifreq); i++)
             {
		printf("%x ", *data);
			data++;

                }

		printf("\n");
	}
#endif //#if 0
        status = ioctl(MultiDevPtr->sockfd, MultiDevPtr->ioctl_val, &userdata);
       printf("DEBUG>>drv_proc_wlan_command::Response Status =%d with MultiDevPtr->ioctl_val =%d\n", status, MultiDevPtr->ioctl_val);
       printf("DEBUG>>drv_proc_wlan_command::MultiDevPtr->wlan_ifname =%s\n", MultiDevPtr->wlan_ifname);
        *rsplen = hostcmd_hdr->size;

    }
    mfg_dprint(DBG_GINFO, "DRV:  host cmd is completed\n");
    return status;
}

int
drv_load_driver(struct _drv_cb *drv, int drv_if)
{
    char *command = drv->command;
    char *script = drv->load_script;

    if (!script) {
        printf("Load script is not provided\n");
        return -1;
    }

    if (!drv_wrapper_get_info(drv->wlan_ifname)) {
        printf("Driver already loaded\n");
    } else {
        sprintf(command, "sh %s", script);
        printf("Load driver ......\n");
        if (system(command) != 0) {
            printf("Failed to run the script\n");
            return -1;
        } else {
            drv_wrapper_init(drv->wlan_ifname, 1);
        }
    }
    return 0;
}

int
drv_unload_driver(struct _drv_cb *drv, int drv_if)
{

    int ret = -1;
    char *command = drv->command;
    char *script = drv->unload_script;

    if (!script) {
        printf("Unload script is not provided\n");
        return -1;
    }
    if (!drv_wrapper_get_info(drv->wlan_ifname)) {
        sprintf(command, "sh %s", script);
        drv_wrapper_deinit();
        printf("Unload driver ......\n");
        ret = system(command);
        if (ret) {
            printf("Failed to run the script\n");
            return -1;
        }
    } else {
        printf("No such device\n");
    }
    return 0;
}
