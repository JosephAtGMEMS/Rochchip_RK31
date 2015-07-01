/* drivers/input/sensors/accel/gma30x.c
 *
 * Copyright (C) 2012-2015 ROCKCHIP.
 * Author: luowei <lw@rock-chips.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
//#define DEBUG	/* Enable gma->client->dev debug data .*/
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <linux/gpio.h>
#include <asm/uaccess.h>
#include <asm/atomic.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/freezer.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#include <linux/kthread.h>
#include <linux/syscalls.h>
#include <linux/sensor-dev.h>
#include <linux/average.h>
#include "gma30x.h"

#include <linux/rockchip/common.h>

#define GMA30x_ENABLE		1
#define GMA30x_PRECISION       13

static int gma_acc_calibration(struct i2c_client *client, int gAxis);
//char DIR_SENSOR[] = "/data/misc";				///< Need Create sensor folder
char GMA_Offset_TXT[] = "/data/misc/gsensor_offset.txt";	///< FILE offset.txt
static int GMA_WriteCalibration(struct sensor_private_data *gma, char * offset){
	char w_buf[20] = {0};
	struct file *fp;
	mm_segment_t fs;
	ssize_t ret;

	sprintf(w_buf,"%d %d %d", gma->offset.u.x, gma->offset.u.y, gma->offset.u.z);
	/* Set segment descriptor associated to kernel space */
	fp = filp_open(offset, O_RDWR | O_CREAT, 0666);
	if(IS_ERR(fp))
		dev_err(&gma->client->dev, "filp_open %s error!!.\n", offset);
	else{
		fs = get_fs();
		//set_fs(KERNEL_DS);
		set_fs(get_ds());
		dev_info(&gma->client->dev, "filp_open %s SUCCESS!!.\n", offset);
 		ret = fp->f_op->write(fp,w_buf,20,&fp->f_pos);
	}
	filp_close(fp,NULL);
	return 0;
}

void GMA_ReadCalibration(struct sensor_private_data *gma){
	unsigned int orgfs;
	char buffer[20];
	struct file *fp;/* *fp open offset.txt */
	orgfs = get_fs();
	/* Set segment descriptor associated to kernel space */
	set_fs(KERNEL_DS);
	fp = filp_open(GMA_Offset_TXT, O_RDWR , 0);
	if(IS_ERR(fp)){
		dev_err(&gma->client->dev, "Sorry,file open ERROR !\n");
#if AutoZeroZ
		//if(1) ABS(gma->axis.x) < LevelValueRange_2_0 && ABS(gma->axis.y) < LevelValueRange_2_0)
			gma_acc_calibration(gma->client, GRAVITY_ON_Z_NEGATIVE);
#endif
	}
	else{
		dev_dbg(&gma->client->dev, "filp_open %s SUCCESS!!.\n",GMA_Offset_TXT);
		fp->f_op->read( fp, buffer, 20, &fp->f_pos); // read offset.txt
		sscanf(buffer,"%d %d %d",&gma->offset.u.x,&gma->offset.u.y,&gma->offset.u.z);
		dev_info(&gma->client->dev, "offset.u.x/offset.u.y/offset.u.z = %d/%d/%d\n",
							gma->offset.u.x, gma->offset.u.y, gma->offset.u.z);
		filp_close(fp,NULL);
	}
	set_fs(orgfs);
}

int gma30x_init(struct sensor_private_data *gma){
	unsigned char buffer[5];
	msleep(30);
	/* 1. Powerdown reset */
	buffer[0] = GMA1302_REG_PD;
	buffer[1] = GMA1302_MODE_RESET;
	sensor_tx_data(gma->client, buffer, 2);
	/* 2. check GMA1302_REG_STADR(0x04) , check PID  */
	buffer[0] = GMA1302_REG_STADR;
	sensor_rx_data(gma->client, buffer, 1);
	if(buffer[0] == GMA30x_VAL_WMI)
		dev_info(&gma->client->dev, "%s: PID = 0x%x, line=%d GMA30x accelerometer\n", __func__, buffer[0],__LINE__);
	else{
	    dev_err(&gma->client->dev, "%s: PID = 0x%x, line=%d The device is not GlobalMems accelerometer.", __func__, buffer[0],__LINE__);
	    return -ENXIO;
	}
	/* 3. turn off the high-pass filter */
	buffer[0] = GMA1302_REG_CONTR1;
	buffer[1] = GMA1302_VAL_OFF;
	sensor_tx_data(gma->client, buffer, 2);
	/* 4. turn on the offset temperature compensation */
	buffer[0] = GMA1302_REG_CONTR3;
	buffer[1] = GMA1302_VAL_OFFSET_TC_ON;
	sensor_tx_data(gma->client, buffer, 2);
	/* 5. turn off the data ready interrupt and configure the INT pin to active high, push-pull type */
	buffer[0] = GMA1302_REG_INTCR;
	buffer[1] = GMA1302_VAL_OFF;//GMA1302_VAL_DATA_READY_ON;
	sensor_tx_data(gma->client, buffer, 2);
	/* 6. treshold set to max */
    buffer[0] = GMA1302_REG_MTHR;
    buffer[1] = GMA1302_VAL_TRESHOLD_MAX;
    sensor_tx_data(gma->client, buffer, 2);

	return 0;
}

/****************operate according to sensor chip:start************/
static int sensor_active(struct i2c_client *client, int enable, int rate)
{
	struct sensor_private_data *gma =
	    (struct sensor_private_data *) i2c_get_clientdata(client);	
	int result = 0;
	int status = 0;
	char buffer[5];
	gma->ops->ctrl_data = sensor_read_reg(client, gma->ops->ctrl_reg);
	//register setting according to chip datasheet		
	if(enable){	
		gma30x_init(gma);
		status = GMA30x_ENABLE;
		gma->ops->ctrl_data |= status;	
	}
	else{
		buffer[0] = GMA1302_REG_ACTR;
		buffer[1] = GMA1302_VAL_ACTR_CONTINUOUS;
		buffer[2] = GMA1302_VAL_ACTR_RESET;
		buffer[3] = 0x08;
		buffer[4] = GMA1302_VAL_ACTR_RESET;
		sensor_tx_data(gma->client, buffer, 5);
		
		status = ~GMA30x_ENABLE;
		gma->ops->ctrl_data &= status;
	}
	dev_dbg(&gma->client->dev, "reg=0x%x,reg_ctrl=0x%x,enable=%d\n", gma->ops->ctrl_reg, gma->ops->ctrl_data, enable);
	
	return result;

}

static int sensor_init(struct i2c_client *client)
{	
	struct sensor_private_data *gma =
	    (struct sensor_private_data *) i2c_get_clientdata(client);
	int i, err = 0;
	err = gma->ops->active(client,0,0);
	if(err){
		dev_err(&gma->client->dev, "%s:line=%d,error\n",__func__,__LINE__);
		return err;
	}
	
	//gma->status_cur = SENSOR_OFF;
	gma->status_cur = SENSOR_ON;

#ifdef SMA_FILTER
	for(i = 0; i < SENSOR_DATA_SIZE; ++i){
		gma->sum[i] = 0;
		for(j = 0; j < SMA_AVG; ++j)
			gma->bufferave[i][j] = 0;
	}
	gma->sma_filter = SMA_AVG;
	dev_info(&gma->client->dev, "GMA301 DEFAULT SMA FILTER: %d\n", gma->sma_filter);
#endif
	gma30x_init(gma);
#ifdef EWMA_FILTER
	ewma_init(&gma->average[0], EWMA_FACTOR, EWMA_WEIGHT_X);
	ewma_init(&gma->average[1], EWMA_FACTOR, EWMA_WEIGHT_Y);
	ewma_init(&gma->average[2], EWMA_FACTOR, EWMA_WEIGHT_Z);
	gma->ewma_filter[0] = EWMA_WEIGHT_X;
	gma->ewma_filter[1] = EWMA_WEIGHT_Y;
	gma->ewma_filter[2] = EWMA_WEIGHT_Z;
	dev_info(&gma->client->dev, "GMA301_EWMA_FILTER: %d %d %d\n", gma->ewma_filter[0], gma->ewma_filter[1], gma->ewma_filter[2]);
#endif
	//memset(&axis_average, 0, sizeof(struct sensor_axis_average));

	return err;
}

#ifdef SMA_FILTER
/* for Simple Moving Average */
static int SMA( struct sensor_private_data *gma, s16 *xyz){
	int i, j;
	static s8	pointer = -1;				/* last update data */

	/* init gma->sum */
	for(i = 0; i < SENSOR_DATA_SIZE; ++i)
		gma->sum[i] = 0;

	pointer++;
	pointer %= gma->sma_filter;
	for(i = 0; i < SENSOR_DATA_SIZE; ++i){
		gma->bufferave[i][pointer] = xyz[i];
	}

    for(i = 0; i < SENSOR_DATA_SIZE; ++i)
		for(j = 0; j < gma->sma_filter; ++j)
			gma->sum[i] += gma->bufferave[i][j];

	for(i = 0; i < SENSOR_DATA_SIZE; ++i)
		xyz[i] = gma->sum[i] / gma->sma_filter;

	return 0;
}
#endif

static int gma_acc_measure(struct sensor_private_data *gma, int *xyz_p){	
	//struct sensor_platform_data *pdata = gma->pdata;
	int i,ret = 0;
	s16 xyzTmp[SENSOR_DATA_SIZE];
	u8 buffer[gma->ops->read_len];
	for(i = 0; i < SENSOR_DATA_SIZE; ++i)
		xyzTmp[i] = 0;

	if(gma->ops->read_len < 3)
		return -1;
	
	memset(buffer, 0, 11);
	/* Data bytes from hardware xL, xH, yL, yH, zL, zH */	
	do {
		*buffer = gma->ops->read_reg;
		ret = sensor_rx_data(gma->client, buffer, gma->ops->read_len);
		if (ret < 0)
		return ret;
	} while (0);
	
	for(i = 0; i < SENSOR_DATA_SIZE; ++i){
	mutex_lock(&(gma->data_mutex) );
	/* merge xyz high/low bytes(13bit) & 1g = 512 *2 = DEFAULT_SENSITIVITY */
		xyzTmp[i] = ((int16_t)((buffer[2*(i+2)] << 8)) | buffer[2*(i+1)+1] ) << 1; // rawdata *2 becomes 1g = 1024
	mutex_unlock(&(gma->data_mutex) );
	}
	
	//dev_dbg(&gma->client->dev, "rawdata xyzTmp: %3d , %3d , %3d\n", xyzTmp[0]/4, xyzTmp[1]/4, xyzTmp[2]/4);
#ifdef SMA_FILTER
	if (xyzTmp[0] != 0 && xyzTmp[1] != 0 && xyzTmp[2] != 0)
		SMA( gma, (s16 *)&xyzTmp);
#endif
#ifdef EWMA_FILTER
	ewma_add(&gma->average[0], (unsigned long) (xyzTmp[0] + EWMA_POSITIVE));
	ewma_add(&gma->average[1], (unsigned long) (xyzTmp[1] + EWMA_POSITIVE));
	ewma_add(&gma->average[2], (unsigned long) (xyzTmp[2] + EWMA_POSITIVE));
	for(i = 0; i < SENSOR_DATA_SIZE; ++i)
		xyz_p[i] = (int) (ewma_read(&gma->average[i]) - EWMA_POSITIVE);
#else
	for(i = 0; i < SENSOR_DATA_SIZE; ++i)
			xyz_p[i] = xyzTmp[i];
#endif
	if (ret < 0)
		return ret;

	return 0;
}
#if 0//AutoZeroZ	//run time calibration AutoZeroZ 
static int check_horizontal_state(struct sensor_private_data *gma)
{
	static int MaxRange = LevelValueRange_0_0625;
	int i, MinRange = 0;
	raw_data offset; 		/* Record old offset */
	static int stable_flag =0; //for auto zero .Analyzing horizontal state
	/* 1.Condition: Analyzing horizontal state */
	if( ABS(gma->axis.x) < MaxRange && ABS(gma->axis.x) > MinRange && ABS(gma->axis.y) < MaxRange && ABS(gma->axis.y) > MinRange && stable_flag < 10)
		stable_flag++;
	else{
		if(stable_flag < 0)
			stable_flag = 0;
		else if(stable_flag < 10)
			stable_flag--;
		else if(stable_flag > 10)
			stable_flag=11;
		else
			stable_flag++;
	}
	//dev_dbg(&gma->client->dev, "a.stable_flag= %d, (rawdata - offset) x/y/z = %03d %03d %03d , MaxRange/981 =%d(LSB)\n", stable_flag, gma->axis.x/981, gma->axis.y/981, gma->axis.z/981, MaxRange/981);
	if(stable_flag == 10){
		//dev_dbg(&gma->client->dev, "stable_flag=%d\n", stable_flag);
		/* 2.Condition: Analyzing horizontal state check again*/
		if( ABS(gma->axis.x) <= LevelValueRange_0_015625  && ABS(gma->axis.y) <= LevelValueRange_0_015625 ){
		//dev_dbg(&gma->client->dev, "b.(rawdata - offset) x/y/z = %03d %03d %03d , MaxRange/981 =%d(LSB)\n", gma->axis.x/981, gma->axis.y/981, gma->axis.z/981, MaxRange/981);
			/* 3.record last time offset */
			for(i = 0; i < SENSOR_DATA_SIZE; ++i)
				offset.v[i] = gma->offset.v[i];
			/* 4.Calculate new offset */
			gma_acc_calibration(gma->client, GRAVITY_ON_Z_AUTO);
			/* 5.offset(X& Y): |new - last_time| < LevelValueRange_0_0078125 */
			if(ABS(gma->offset.u.x - offset.u.x) <= LevelValueRange_0_0078125 && ABS(gma->offset.u.y - offset.u.y) <= LevelValueRange_0_0078125 ){
				gma->offset.v[0] = offset.v[0];	/* (of.x) use last time offset */
				gma->offset.v[1] = offset.v[1];	/* (of.y) use last time offset */
				gma->offset.v[2] = offset.v[2];	/* (of.z) use last time offset */
				//dev_dbg(&gma->client->dev, "c.use old offset. gma->offset.v[0]/[1]/[2] =%d %d %d:%d\n", gma->offset.v[0],gma->offset.v[1],gma->offset.v[2],__LINE__);
			}
			else{
			/* 6.offset(Z): |last time| > 1g (Prevent calibration errors) */
				if(ABS(offset.u.z) > DEFAULT_SENSITIVITY)
					gma->offset.v[2] = offset.v[2];	/*(of.z) use last time offset */
					
				dev_dbg(&gma->client->dev, "d.gma->offset.v[2] =%d :%d\n", gma->offset.v[2], __LINE__);
				GMA_WriteCalibration(gma , GMA_Offset_TXT);
			}
			MaxRange /= 2;
			stable_flag = 0;
		}
	}
	
	return stable_flag;
}
#endif

static bool firsttime=true;
static int sensor_report_value(struct i2c_client *client)
{
	struct sensor_private_data *gma =
		(struct sensor_private_data *) i2c_get_clientdata(client);	
	struct sensor_platform_data *pdata = gma->pdata;
	int i, flag = 0, ret = 0;
	raw_data xyz;
		
	ret = gma_acc_measure(gma, (int *)&xyz.v);
	if (ret < 0)
		return ret;
	for(i = 0; i < SENSOR_DATA_SIZE; ++i)
		xyz.v[i] -= gma->offset.v[i];
	dev_dbg(&gma->client->dev, "xyz.v X/Y/Z = %04d %04d %04d\n", xyz.v[0], xyz.v[1], xyz.v[2]);
	dev_dbg(&gma->client->dev, "gma->offset.v X/Y/Z = %03d %03d %03d\n", gma->offset.v[0], gma->offset.v[1], gma->offset.v[2]);
	//Analyzing horizontal state 2015-05-16 add
	if( ABS(xyz.v[0]) < 42 && ABS(xyz.v[1]) < 42)
	{
		xyz.v[0] = 0;
		xyz.v[1] = 0;
	}

if(rockchip_boot_mode() != BOOT_MODE_RECOVERY)
	{
	if(firsttime){
		printk("zza rockchip_boot_mode=%d\n\n",rockchip_boot_mode());
		msleep(8000); /* wait the file system ready. read /data/misc/gsensor_offset.txt */
/*		ret = sys_mkdir(DIR_SENSOR , 01777);
		if(ret < 0)
			dev_err(&gma->client->dev, "fail to execute sys_mkdir,If the DIR exists, do not mind. ret = %d :%d\n"
					, ret, __LINE__);*/
		GMA_ReadCalibration(gma);
	 	firsttime=false;
	}
}
	/* 1g = (1024)*981 becomes 1g = 1004544 */
	gma->axis.x = (((pdata->orientation[0])*xyz.v[0] + (pdata->orientation[1])*xyz.v[1] + (pdata->orientation[2])*xyz.v[2]))*981;
	gma->axis.y = (((pdata->orientation[3])*xyz.v[0] + (pdata->orientation[4])*xyz.v[1] + (pdata->orientation[5])*xyz.v[2]))*981; 
	gma->axis.z = (((pdata->orientation[6])*xyz.v[0] + (pdata->orientation[7])*xyz.v[1] + (pdata->orientation[8])*xyz.v[2]))*981;

#if 0 //AutoZeroZ	//run time calibration AutoZeroZ
	if(flag != 11)
		flag = check_horizontal_state(gma);
	dev_dbg(&gma->client->dev, "flag= %d , gma->axis = %5d  %5d  %5d :%d\n", flag, gma->axis.x, gma->axis.y, gma->axis.z, __LINE__);
#endif
	/* Report acceleration sensor information */
	input_report_abs(gma->input_dev, ABS_X, -(gma->axis.y));
	input_report_abs(gma->input_dev, ABS_Y, (gma->axis.x));
	input_report_abs(gma->input_dev, ABS_Z, gma->axis.z);
	input_sync(gma->input_dev);
	return ret;
}
/* calculate delta offset */
static int gma_acc_calibration(struct i2c_client *client, int gAxis)
{
	struct sensor_private_data *gma =
		(struct sensor_private_data *) i2c_get_clientdata(client);	
	//struct sensor_platform_data *pdata = gma->pdata;
// add by Steve 20150202********
/*	bool compare_avg=false;
	int avg_Z=0;
	int cycle=0;
	int threshold=DEFAULT_SENSITIVITY/20;*/
// *****************************
	raw_data xyz, avg;
	long xyz_acc[SENSOR_DATA_SIZE];
	int i, j, ret = 0;
	/* initialize the offset value */
	for(i = 0; i < SENSOR_DATA_SIZE; ++i)
		gma->offset.v[i] = 0;
// add by Steve 20150202********
/*	for(cycle=0;cycle<3;cycle++)
	{		
// *****************************
		/* initialize the accumulation buffer */
	  	for(i = 0; i < SENSOR_DATA_SIZE; ++i)
			xyz_acc[i] = 0;
		/* get rawdata of AVG_NUM */
		for(i = 0; i < AVG_NUM; i++) {
			gma_acc_measure(gma, (int *)&xyz.v);
			for(j = 0; j < SENSOR_DATA_SIZE; ++j)
				xyz_acc[j] += xyz.v[j];
		}
		/* calculate averages */
		for(i = 0; i < SENSOR_DATA_SIZE; ++i)
			avg.v[i] = xyz_acc[i] / AVG_NUM;
		//dev_info(&gma->client->dev, "avg.u.z = %d , avg_Z = %d : %d\n",avg.u.z , avg_Z, __LINE__);
// add by Steve 20150202********
/*		if(abs(avg.u.z-avg_Z) > threshold)
		{
			dev_info(&gma->client->dev, "Sensor unstable cycle %d , try again automatically: %d\n",cycle , __LINE__);
			compare_avg=false;
			avg_Z=avg.u.z;	
		}			
		else
		{
			dev_info(&gma->client->dev, "Sensor Stable in comparsion cycle %d : %d\n",cycle , __LINE__);
			compare_avg=true;
			break;	
		}
	}//end of for(c_cycle=0;c_cycle<2;c_cycle++)r	
	if(compare_avg==false)
	{
		dev_info(&gma->client->dev, "Sensor unstable,Please try again %d : %d\n", gAxis, __LINE__);
		return -1;
	}
*/// *****************************
	/* calculate offset */
	switch(gAxis){
		case GRAVITY_ON_Z_NEGATIVE:
			gma->offset.u.x =  avg.v[0];
			gma->offset.u.y =  avg.v[1];
			gma->offset.u.z =  avg.v[2] - DEFAULT_SENSITIVITY;
			for(i = 0; i < SENSOR_DATA_SIZE; i++){
				dev_info(&gma->client->dev, "gAxis = %d, gma->offset.v[%d]=%4d , avg.v[%d]=%4d: %d\n", 
						gAxis, i, gma->offset.v[i], i, avg.v[i], __LINE__);
			}
			break;
		case GRAVITY_ON_X_POSITIVE:
			gma->offset.u.x =  avg.v[0] - DEFAULT_SENSITIVITY;    
			gma->offset.u.y =  avg.v[1];
			gma->offset.u.z =  avg.v[2];
			for(i = 0; i < SENSOR_DATA_SIZE; i++){
				dev_info(&gma->client->dev, "gAxis = %d, gma->offset.v[%d]=%4d , avg.v[%d]=%4d: %d\n", 
						gAxis, i, gma->offset.v[i], i, avg.v[i], __LINE__);
			}
		 	break;
		case GRAVITY_ON_Z_POSITIVE:
			gma->offset.u.x =  avg.v[0] ;
			gma->offset.u.y =  avg.v[1] ;
			gma->offset.u.z =  avg.v[2] + DEFAULT_SENSITIVITY;
			for(i = 0; i < SENSOR_DATA_SIZE; i++){
				dev_info(&gma->client->dev, "gAxis = %d, gma->offset.v[%d]=%4d , avg.v[%d]=%4d: %d\n", 
						gAxis, i, gma->offset.v[i], i, avg.v[i], __LINE__);
			}
		 	break;
		case GRAVITY_ON_X_NEGATIVE:
			gma->offset.u.x =  avg.v[0] + DEFAULT_SENSITIVITY;    
			gma->offset.u.y =  avg.v[1];
			gma->offset.u.z =  avg.v[2];
			for(i = 0; i < SENSOR_DATA_SIZE; i++){
				dev_info(&gma->client->dev, "gAxis = %d, gma->offset.v[%d]=%4d , avg.v[%d]=%4d: %d\n", 
						gAxis, i, gma->offset.v[i], i, avg.v[i], __LINE__);
			}
		 	break;
		case GRAVITY_ON_Y_NEGATIVE:
			gma->offset.u.x =  avg.v[0];    
			gma->offset.u.y =  avg.v[1] - DEFAULT_SENSITIVITY;
			gma->offset.u.z =  avg.v[2];
			for(i = 0; i < SENSOR_DATA_SIZE; i++){
				dev_info(&gma->client->dev, "gAxis = %d, gma->offset.v[%d]=%4d , avg.v[%d]=%4d: %d\n", 
						gAxis, i, gma->offset.v[i], i, avg.v[i], __LINE__);
			}
		 	break;
		case GRAVITY_ON_Y_POSITIVE:
			gma->offset.u.x =  avg.v[0];    
			gma->offset.u.y =  avg.v[1] + DEFAULT_SENSITIVITY;
			gma->offset.u.z =  avg.v[2];
			for(i = 0; i < SENSOR_DATA_SIZE; i++){
				dev_info(&gma->client->dev, "gAxis = %d, gma->offset.v[%d]=%4d , avg.v[%d]=%4d: %d\n", 
						gAxis, i, gma->offset.v[i], i, avg.v[i], __LINE__);
			}
		 	break;
		case GRAVITY_ON_X_AUTO:
			if(avg.v[0] < 0){
				gma->offset.u.x =  avg.v[0] + DEFAULT_SENSITIVITY;
				gma->offset.u.y =  avg.v[1];
				gma->offset.u.z =  avg.v[2];
			}
			else{
				gma->offset.u.x =  avg.v[0] - DEFAULT_SENSITIVITY;
				gma->offset.u.y =  avg.v[1];
				gma->offset.u.z =  avg.v[2];
			}
			for(i = 0; i < SENSOR_DATA_SIZE; i++){
				dev_info(&gma->client->dev, "gAxis = %d, gma->offset.v[%d]=%4d , avg.v[%d]=%4d: %d\n", 
						gAxis, i, gma->offset.v[i], i, avg.v[i], __LINE__);
			}
			break;
	    case GRAVITY_ON_Y_AUTO:
			if(avg.v[1] < 0){
				gma->offset.u.x =  avg.v[0];
				gma->offset.u.y =  avg.v[1] + DEFAULT_SENSITIVITY;
				gma->offset.u.z =  avg.v[2];
			}
			else{
				gma->offset.u.x =  avg.v[0];
				gma->offset.u.y =  avg.v[1] - DEFAULT_SENSITIVITY;
				gma->offset.u.z =  avg.v[2];
			}
			for(i = 0; i < SENSOR_DATA_SIZE; i++){
				dev_info(&gma->client->dev, "gAxis = %d, gma->offset.v[%d]=%4d , avg.v[%d]=%4d: %d\n", 
						gAxis, i, gma->offset.v[i], i, avg.v[i], __LINE__);
			}
			break;
		case GRAVITY_ON_Z_AUTO:
	
			if(avg.v[2] < 0){
				gma->offset.u.x =  avg.v[0];
				gma->offset.u.y =  avg.v[1];
				gma->offset.u.z =  avg.v[2] + DEFAULT_SENSITIVITY;
			}
			else{
				gma->offset.u.x =  avg.v[0];
				gma->offset.u.y =  avg.v[1];
				gma->offset.u.z =  avg.v[2] - DEFAULT_SENSITIVITY;
			}
			for(i = 0; i < SENSOR_DATA_SIZE; i++){
				dev_info(&gma->client->dev, "gAxis = %d, gma->offset.v[%d]=%4d , avg.v[%d]=%4d: %d\n", 
						gAxis, i, gma->offset.v[i], i, avg.v[i], __LINE__);
			}
			break;
		default:  
			return -ENOTTY;
	}

	GMA_WriteCalibration(gma , GMA_Offset_TXT);
	return 0;
}

struct sensor_operate gsensor_gma30x_ops = {
	.name				= GSENSOR_ID,
	.type				= SENSOR_TYPE_ACCEL,		//sensor type and it should be correct
	.id_i2c				= ACCEL_ID_GMA302,			//i2c id number
	.id_data 			= GMA30x_VAL_WMI,			//device id
	.read_reg			= GMA1302_REG_STADR,		//read data
	.read_len			= 13,						//data length
	.id_reg				= GMA1302_REG_STADR,//GMA1302_REG_PID,			//read device id from this register
	.precision			= GMA30x_PRECISION,			//13 bit
	.ctrl_reg 			= GMA1302_REG_PD,			//enable or disable 	
	.int_status_reg 	= GMA1302_REG_STADR,		//intterupt status register
	.range				= {-ABSMAX,ABSMAX},			//range
	.trig				= IRQF_TRIGGER_LOW|IRQF_ONESHOT,
	.active				= sensor_active,	
	.init				= sensor_init,
	.report 			= sensor_report_value,
	.calibration		= gma_acc_calibration,
};

/****************operate according to sensor chip:end************/
//function name should not be changed
static struct sensor_operate *gsensor_get_ops(void)
{
	return &gsensor_gma30x_ops;
}

static int __init gsensor_gma30x_init(void)
{
	struct sensor_operate *ops = gsensor_get_ops();
	int result = 0;
	int type = ops->type;
	result = sensor_register_slave(type, NULL, NULL, gsensor_get_ops);	
	return result;
}

static void __exit gsensor_gma30x_exit(void)
{
	struct sensor_operate *ops = gsensor_get_ops();
	int type = ops->type;
	sensor_unregister_slave(type, NULL, NULL, gsensor_get_ops);
}

module_init(gsensor_gma30x_init);
module_exit(gsensor_gma30x_exit);



