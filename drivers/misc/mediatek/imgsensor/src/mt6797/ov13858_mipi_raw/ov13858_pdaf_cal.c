#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>
//#include <linux/xlog.h> 
//#include <asm/system.h>

#include <linux/proc_fs.h> 


#include <linux/dma-mapping.h>

#include "kd_camera_hw.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"
#include "kd_camera_typedef.h"

#include "ov13858mipiraw_Sensor.h"
/****************************Modify Following Strings for Debug****************************/
#define PFX "OV13858_camera_pdaf"
//#define LOG_1 LOG_INF("OV13858,MIPI 4LANE\n")
//#define LOG_2 LOG_INF("preview 2096*1552@30fps,640Mbps/lane; video 4192*3104@30fps,1.2Gbps/lane; capture 13M@30fps,1.2Gbps/lane\n")
/****************************   Modify end    *******************************************/

#define LOG_INF(format, args...)    pr_debug(PFX "[%s] " format, __FUNCTION__, ##args)

//#include "ov13858_otp.h"
struct otp_pdaf_struct {
unsigned char pdaf_flag; //bit[7]--0:empty; 1:Valid
unsigned char data1[496];//output data1
unsigned char data2[806];//output data2
unsigned char data3[102];//output data3
unsigned char pdaf_checksum;//checksum of pd, SUM(0x0801~0x0D7C)%255+1

};




extern int iReadRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u8 * a_pRecvData, u16 a_sizeRecvData, u16 i2cId);
extern int iWriteRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u16 i2cId);
//extern void kdSetI2CSpeed(u16 i2cSpeed);
//#define EEPROM_READ_ID  0xA1
//#define EEPROM_WRITE_ID   0xA0
#define I2C_SPEED        400  //CAT24C512 can support 1Mhz

#define START_OFFSET     0x800

#define Delay(ms)  mdelay(ms)
//static unsigned char OV13858MIPI_WRITE_ID = (0xA0 >> 1);
/* HTC_START */
#if 0
#define EEPROM_READ_ID  0xA3
#define EEPROM_WRITE_ID   0xA2
#else
#define EEPROM_READ_ID  0xA1
#define EEPROM_WRITE_ID   0xA0
#endif
/* HTC_END */
#define MAX_OFFSET       0xd7b
#define DATA_SIZE 4096
//BYTE eeprom_data[DATA_SIZE]= {0};
static bool get_done = false;
static int last_size = 0;
static int last_offset = 0;
/*
kal_uint16 OV13858_R2A_read_i2c(kal_uint32 addr)
{
    kal_uint16 get_byte=0;
    iReadReg((u16) addr ,(u8*)&get_byte,EEPROM_READ_ID);
    return get_byte;
}



kal_uint16 OV13858_R2A_write_i2c(addr, para)
{
		iWriteReg((u16) addr , (u32) para , 1, EEPROM_WRITE_ID);
		return 1;
}




//read pdaf data
int read_otp_pdaf_data(kal_uint16 addr, BYTE* data, kal_uint32 size)
{
	int i = 0;
	LOG_INF("addr :%x, size:%d, \n", addr, size);
	addr = 0x800;
	for (i=0; i<1404;i++)
	{
		data[i] = OV13858_R2A_read_i2c(addr);
		addr++;
		LOG_INF("data[%d] :%x, addr:%x, \n", i, data[i], addr);
    }
	return 0;
}
*/

////

static bool OV13858_selective_read_eeprom(kal_uint16 addr, BYTE* data)
{
	char pu_send_cmd[2] = {(char)(addr >> 8) , (char)(addr & 0xFF) };
    if(addr > MAX_OFFSET)
        return false;
	//kdSetI2CSpeed(I2C_SPEED);

	if(iReadRegI2C(pu_send_cmd, 2, (u8*)data, 1, EEPROM_WRITE_ID)<0)
		return false;
    return true;
}

/* HTC_START */
/*
Module Information
    (Number)    (Sub Items)
    17          Module Info
    2           Module Info check sum

AF Positions
    (Number)    (Sub Items)
    4           AF DAC Values
    2           AF DAC check sum
*/
bool OV13858_read_eeprom_module_info(BYTE* data)
{
	int i = 0;
	int offset = 0x00;
	unsigned long checksum_count;

	/* Read all data out : 17+2+4+2=25 bytes */
	for(i = 0; i < 25; i++) {
		if(!OV13858_selective_read_eeprom(offset, &data[i])){
			//LOG_INF("read_eeprom 0x%0x %d fail \n",offset, data[i]);
			return false;
		}
		//LOG_INF("read_eeprom 0x%0x 0x%x\n",offset, data[i]);
		offset++;
	}

	/* Deal with : Module Info check sum */
	checksum_count = 0;
	for(i = 0; i < 17; i++) {
		checksum_count += data[i];
	}
	checksum_count = checksum_count & 0xFFFF;
	//LOG_INF("read_eeprom Module Info checksum_count=0x%x\n", (unsigned int)checksum_count);

	if ( (((checksum_count>>8)&0xFF) == data[17]) &&
		((checksum_count&0xFF) == data[18])
		) {
		//LOG_INF("read_eeprom Module Info checksum correct\n");
	} else {
		//LOG_INF("read_eeprom Module Info checksum error\n");
		return false;
	}

	/* Deal with : AF DAC check sum */
	checksum_count = 0;
	for(i = 19; i < 23; i++) {
		checksum_count += data[i];
	}
	checksum_count = checksum_count & 0xFFFF;
	//LOG_INF("read_eeprom AF DAC checksum_count=0x%x\n", (unsigned int)checksum_count);

	if ( (((checksum_count>>8)&0xFF) == data[23]) &&
		((checksum_count&0xFF) == data[24])
		) {
		//LOG_INF("read_eeprom AF DAC checksum correct\n");
	} else {
		//LOG_INF("read_eeprom AF DAC checksum error\n");
		return false;
	}

    return true;
}
/* HTC_END */

static bool OV13858_read_eeprom(kal_uint16 addr, BYTE* data, kal_uint32 size ){
	int i = 0;
	//int offset = addr;
	int offset = 0x800;
/* HTC_START */
	unsigned long pdaf_checksum_count = 0;
	BYTE pdaf_checksum_data[2];
/* HTC_END */

/* HTC_START */
#if 0
	//for(i = 0; i < 1404; i++) {
	for(i = 0; i < 1372; i++) {
		if(!OV13858_selective_read_eeprom(offset, &data[i])){
			LOG_INF("read_eeprom 0x%0x %d fail \n",offset, data[i]);
			return false;
		}
		LOG_INF("read_eeprom 0x%0x 0x%x\n",offset, data[i]);
		offset++;
	}
#else
	offset = 0x0019;

	/* PDAF Proc-1 and Proc-2 data */
	for(i = 0; i < 1372; i++) {
		if(!OV13858_selective_read_eeprom(offset, &data[i])){
			//LOG_INF("read_eeprom 0x%0x %d fail \n",offset, data[i]);
			return false;
		}
		LOG_INF("read_eeprom 0x%0x 0x%x\n",offset, data[i]);
		pdaf_checksum_count += data[i];
		offset++;
	}
	pdaf_checksum_count = pdaf_checksum_count & 0xFFFF;
	//LOG_INF("read_eeprom pdaf_checksum_count=0x%x\n", (unsigned int)pdaf_checksum_count);

	/* PDAF check sum */
	for(i = 0; i < 2; i++) {
		if(!OV13858_selective_read_eeprom(offset, &pdaf_checksum_data[i])){
			//LOG_INF("read_eeprom 0x%0x %d fail \n",offset, pdaf_checksum_data[i]);
			return false;
		}
		//LOG_INF("read_eeprom 0x%0x 0x%x\n",offset, pdaf_checksum_data[i]);
		offset++;
	}

	if ( (((pdaf_checksum_count>>8)&0xFF) == pdaf_checksum_data[0]) &&
		((pdaf_checksum_count&0xFF) == pdaf_checksum_data[1])
		) {
		//LOG_INF("read_eeprom PDAF checksum correct\n");
	} else {
		//LOG_INF("read_eeprom PDAF checksum error\n");
		return false;
	}

#endif
/* HTC_END */

	get_done = true;
	last_size = size;
	last_offset = addr;
    return true;
}

bool read_otp_pdaf_data( kal_uint16 addr, BYTE* data, kal_uint32 size){
	
	LOG_INF("read_otp_pdaf_data enter");
	if(!get_done || last_size != size || last_offset != addr) {
		//if(!_read_eeprom(addr, eeprom_data, size)){
		if(!OV13858_read_eeprom(addr, data, size)){
			get_done = 0;
            last_size = 0;
            last_offset = 0;
			LOG_INF("read_otp_pdaf_data fail");
			return false;
		}
	}
	//memcpy(data, eeprom_data, size);
	LOG_INF("read_otp_pdaf_data end");
    return true;
}

//
