/*
 * Copyright (C) 2010 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/wait.h>
#include <linux/time.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/gpio.h>
#include <generated/autoconf.h>

#include "tpd_it7250_common.h"

#include "tpd.h"


#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>


#define TPD_SUPPORT_POINTS	2

struct i2c_client *i2c_client = NULL;
struct task_struct *thread = NULL;
static DECLARE_WAIT_QUEUE_HEAD(waiter);


static irqreturn_t tpd_eint_interrupt_handler(int irq, void *dev_id);


static int tpd_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int tpd_i2c_detect(struct i2c_client *client, struct i2c_board_info *info);
static int tpd_remove(struct i2c_client *client);
static int touch_event_handler(void *unused);
static void tpd_resume(struct device *h);
static void tpd_suspend(struct device *h);

static int tpd_flag;


unsigned int touch_irq = 0;
#define TPD_OK 0

#define TPD_RESET_ISSUE_WORKAROUND
#define TPD_MAX_RESET_COUNT	3


#define POWER_OFF	0
#define POWER_ON	1
#ifndef IT7250_KEEP_POWER
static int power_flag = POWER_OFF;
#endif
static u8 ite7258_is_true = 0;
static int tpd_halt=0;
static int FWversion;


#if defined(TPD_LEFT_RIGHT_SWAP)
static inline void tpd_swap_left_right(int *x, int *y)
{
	int temp = 0;
	temp = TPD_RES_X - *x;
	*x = temp;
}
#endif


#if defined(TPD_UP_DOWN_SWAP)
static inline void tpd_swap_up_down(int *x, int *y)
{
	int temp = 0;
	temp = TPD_RES_Y - *y;
	*y = temp;
}
#endif


#if defined(TPD_X_Y_SWAP)
static inline void tpd_swap_x_y(int *x, int *y)
{
	int temp = 0;
	
	temp = *x;
	*x = *y;
	*y = temp;
}
#endif


static const struct i2c_device_id it7250_tpd_id[] = {{"IT7250", 0}, {} };
static const struct of_device_id it7250_dt_match[] = {
	{.compatible = "mediatek,cap_touch"},
	{},
};
MODULE_DEVICE_TABLE(of, it7250_dt_match);

static struct i2c_driver tpd_i2c_driver = {
	.driver = {
		.of_match_table = of_match_ptr(it7250_dt_match),
		.name = "it7250",
	},
	.probe = tpd_probe,
	.remove = tpd_remove,
	.id_table = it7250_tpd_id,
	.detect = tpd_i2c_detect,
};

static int of_get_it7250_platform_data(struct device *dev)
{
	/*int ret, num;*/

	if (dev->of_node) {
		const struct of_device_id *match;
		match = of_match_device(of_match_ptr(it7250_dt_match), dev);
		if (!match) {
			TPD_DMESG("Error: No device match found\n");
			return -ENODEV;
		}
	}
	return 0;
}

static  void it7250_power_switch(int state)
{
#ifndef IT7250_KEEP_POWER
	int ret = 0;
	switch (state) {
		case POWER_ON:
			if(power_flag == POWER_OFF)
			{
				printk("Power switch on!\n");
				if(NULL == tpd->reg)
				{
				  	tpd->reg=regulator_get(tpd->tpd_dev,"vtouch"); // get pointer to regulator structure
					if (IS_ERR(tpd->reg)) {
						printk("touch panel regulator_get() failed!\n");
						return;
					}else
					{
						printk("regulator_get() Ok!\n");
					}
				}
	
				printk("regulator_set_voltage--begin\r\n");
				ret=regulator_set_voltage(tpd->reg, 2800000, 2800000);  // set 2.8v
				printk("regulator_set_voltage--end\r\n");
				if (ret)
					printk("regulator_set_voltage() failed!\n");
				ret=regulator_enable(tpd->reg);  //enable regulator
				if (ret)
					printk("regulator_enable() failed!\n");

				power_flag = POWER_ON;
			}
			else
			{
		  		printk("######Power already is on!#######\n");
		  	}
			break;
		case POWER_OFF:
			if(power_flag == POWER_ON)
			{
				printk("Power switch off!\n");
				if(!IS_ERR_OR_NULL(tpd->reg))
				{
					ret=regulator_disable(tpd->reg); //disable regulator
					if (ret)
						printk("regulator_disable() failed!\n");
					regulator_put(tpd->reg);
					tpd->reg = NULL;
					power_flag = POWER_OFF;
				}
			}
			else
			{
				printk("#######Power already is off!########\n");
			}
			break;
		  default:
			printk("Invalid power switch command!");
			break;
		}
#endif
} 

#define GTP_GPIO_AS_INT(pin) tpd_gpio_as_int(pin)
#define GTP_GPIO_OUTPUT(pin, level) tpd_gpio_output(pin, level)

static inline  int it7250_i2c_read(struct i2c_client *client, u8 command,u8 length, u8 *values)
{
	int res = 0;
	values[0] = command;
	client->addr &= I2C_MASK_FLAG;
	client->addr |= I2C_WR_FLAG;
	client->addr |= I2C_RS_FLAG;
	res = i2c_master_send(client, values,  (length << 8 | 1));
	client->addr &= I2C_MASK_FLAG;
	if (res < 0)
	{
		printk("[mtk-tpd] it7260 TPD i2c read fail (0x%x) (%d)\n",command,res);
	}
	return res;
}

static inline int it7250_i2c_write(struct i2c_client *client, u8 command,u8 length, u8 *values)
{
	int res = 0;

	client->addr &= I2C_MASK_FLAG;
	res = i2c_master_send(client, values, length);
	client->addr &= I2C_MASK_FLAG;
	if (res < 0)
	{
		printk("[mtk-tpd] it7260 TPD i2c write fail(%d) (%d)\n",length,res);
	}
	return res;
}

static struct device_attribute *it7250_attrs[] = {
};


static void tpd_down(int x, int y)
{

#if defined(TPD_LEFT_RIGHT_SWAP)
	tpd_swap_left_right(&x, &y);
#endif
	
#if defined(TPD_UP_DOWN_SWAP)
	tpd_swap_up_down(&x, &y);
#endif
	
#if defined(TPD_X_Y_SWAP)
	tpd_swap_x_y(&x, &y);
#endif	



#ifdef TPD_SOLVE_CHARGING_ISSUE
	if (0 != x) {
#else
	{
#endif
		TPD_DEBUG("%s x:%d y:%d \n", __func__, x, y);
		input_report_key(tpd->dev, BTN_TOUCH, 1);
		input_report_abs(tpd->dev, ABS_MT_TOUCH_MAJOR, 1);
		input_report_abs(tpd->dev, ABS_MT_POSITION_X, x);
		input_report_abs(tpd->dev, ABS_MT_POSITION_Y, y);
		input_mt_sync(tpd->dev);
	}
}

static void tpd_up(int x, int y)
{
#if defined(TPD_LEFT_RIGHT_SWAP)
	tpd_swap_left_right(&x, &y);
#endif
		
#if defined(TPD_UP_DOWN_SWAP)
	tpd_swap_up_down(&x, &y);
#endif
		
#if defined(TPD_X_Y_SWAP)
	tpd_swap_x_y(&x, &y);
#endif	

#ifdef TPD_SOLVE_CHARGING_ISSUE
	if (0 != x) {
#else
	{
#endif
		TPD_DEBUG("%s x:%d y:%d\n", __func__, x, y);
		input_report_key(tpd->dev, BTN_TOUCH, 0);
		input_mt_sync(tpd->dev);
	}
}



static int x[2] = { (int) -1, (int) -1 };
static int y[2] = { (int) -1, (int) -1 };
static bool finger[2] = { 0, 0 };
static bool flag = 0;

static int touch_event_handler(void *unused)
{
	int ret = 0;
	unsigned char pucPoint[14];
	struct sched_param param = { .sched_priority = 4 };
	sched_setscheduler(current, SCHED_RR, &param);

	do {
		set_current_state(TASK_INTERRUPTIBLE);
		wait_event_interruptible(waiter, tpd_flag != 0);

		tpd_flag = 0;

		set_current_state(TASK_RUNNING);

		TPD_DEBUG("touch_event_handler start\n");

		ret = it7250_i2c_read(i2c_client, 0x80, 1, pucPoint);
		if (!( pucPoint[0] & 0x80 || pucPoint[0] & 0x01 )){
			printk("[mtk-tpd] No point information\n");
			msleep(10);
			goto exit_work_func;
		}

		ret = it7250_i2c_read(i2c_client, 0xC0, 8, &pucPoint[6]);
		ret += it7250_i2c_read(i2c_client, 0xE0, 6, &pucPoint[0]);
		printk("[mtk-tpd] it7250 read touch point from touchpanel\n");

		if (ret == 0xE02) 
		{
			// gesture
		    if (pucPoint[0] & 0xF0) {
		    	{
		            TPD_DEBUG("[mtk-tpd] it's a button/gesture\n");
		            goto exit_work_func;
		        }
		    }
		    // palm
		    else if( pucPoint[1] & 0x01 ) {
		        TPD_DEBUG("[mtk-tpd] it's a palm\n");
				goto exit_work_func;
		    }
		    // no more data
		    else if (!(pucPoint[0] & 0x08)) {
				if( finger[0] ){
				    finger[0] = 0;
				    tpd_up(x[0], y[0]);
				    flag = 1;
				}
				if( finger[1] ){
				    finger[1] = 0;
				    tpd_up(x[1], y[1]);
				    flag = 1;
				}
				if( flag ){
				    flag = 0;
				    input_sync(tpd->dev);
				}
				TPD_DEBUG("[mtk-tpd] no more data\n");
				goto exit_work_func;
		    }
		    // 3 fingers
		    else if (pucPoint[0] & 0x04) {
		        TPD_DEBUG("[mtk-tpd] we don't support three fingers\n");
				goto exit_work_func;
		    }
			else{
				// finger 1
		        if (pucPoint[0] & 0x01) {
				    //char pressure_point;	
				    x[0] = ((pucPoint[3] & 0x0F) << 8) + pucPoint[2];
				    y[0] = ((pucPoint[3] & 0xF0) << 4) + pucPoint[4];
				    finger[0] = 1;
	//				printk("***********************x[0] = %d,y[0]=%d\n",x[0],y[0]);
				    tpd_down(x[0], y[0]);
					//printk("*******************tpd_down:x0=%d,y0=%d\n",x[0],y[0]);
					
				} 
				else if( finger[0] ){
				    tpd_up(x[0], y[0]);
				    finger[0] = 0;
				}
	
				// finger 2
				if (pucPoint[0] & 0x02) {
				    //char pressure_point;
		  		    x[1] = ((pucPoint[7] & 0x0F) << 8) + pucPoint[6];
				    y[1] = ((pucPoint[7] & 0xF0) << 4) + pucPoint[8];
				    finger[1] = 1;
				    tpd_down(x[1], y[1]);
					//printk("*******************tpd_down:x1=%d,y1=%d\n",x[1],y[1]);

				} else if (finger[1]){
				    tpd_up(x[1], y[1]);
				    finger[1] = 0;
				}
			input_sync(tpd->dev);
		    }
		}
		else
		{
		    TPD_DEBUG("[mtk-tpd] i2c read communcate error in getting pixels : 0x%x\n", ret);
		}

exit_work_func:
		enable_irq(touch_irq);
	} while (!kthread_should_stop());

	TPD_DEBUG("touch_event_handler exit\n");

	return 0;
}

static int tpd_i2c_detect(struct i2c_client *client, struct i2c_board_info *info)
{
	strcpy(info->type, TPD_DEVICE);

	return 0;
}

static irqreturn_t tpd_eint_interrupt_handler(int irq, void *dev_id)
{
	TPD_DEBUG("TPD interrupt has been triggered\n");
	disable_irq_nosync(touch_irq);
	tpd_flag = 1;
	wake_up_interruptible(&waiter);
	return IRQ_HANDLED;
}
static int tpd_irq_registration(void)
{
	struct device_node *node = NULL;
	int ret = 0;

	//node = of_find_compatible_node(NULL, NULL, "mediatek,cap_touch");
	node = of_find_matching_node(NULL, touch_of_match);
	if (node) {
		touch_irq = irq_of_parse_and_map(node, 0);
		printk("[mtk-tpd] it7250 touch_irq=%d\n",touch_irq);
		ret = request_irq(touch_irq, tpd_eint_interrupt_handler,
					IRQF_TRIGGER_FALLING, TPD_DEVICE, NULL);
		if (ret > 0)
			TPD_DMESG("tpd request_irq IRQ LINE NOT AVAILABLE!.");
	} else {
		TPD_DMESG("[%s] tpd request_irq can not find touch eint device node!.", __func__);
	}
	return 0;
}

static bool waitCommandDone(void)
{
	unsigned char ucQuery = 0xFF;
	unsigned int count = 0;

	do{
		ucQuery = 0xFF;
		it7250_i2c_read(i2c_client, 0x80, 1, &ucQuery);
		count++;
		//printk("[mtk-tpd] it7250 waitCommandDone read back:0x%x \n",ucQuery);
	}while(ucQuery & 0x01 && count < 500);

	if( !(ucQuery & 0x01) ){
	        return  true;
	}else{
		printk("XXX %s, %d\n", __FUNCTION__, __LINE__);
		return  false;
	}
}

static bool tpd_FW_version(struct i2c_client *client, int *values) {
	int sum = 0; 
	int ret_val = 0;
	uint8_t pucCommandBuffer[8];
	char verFw[4];
   
    printk("[mtk-tpd] enter tpd_print_version .\n");

	waitCommandDone();

	printk("[mtk-tpd] tpd_print_version.\n");
    pucCommandBuffer[0] = 0x20;
    pucCommandBuffer[1] = 0xE1;
    pucCommandBuffer[2] = 0x04;
    pucCommandBuffer[3] = 0x01;
    pucCommandBuffer[4] = 0x08;
    pucCommandBuffer[5] = 0x00;
    pucCommandBuffer[6] = 0x00;
    pucCommandBuffer[7] = 0xD8;
    ret_val = it7250_i2c_write(client, pucCommandBuffer[0], 8, pucCommandBuffer);
	printk("[mtk-tpd] IT7250 firmware write return value 201712301416:(%d)\n",ret_val);
	if(ret_val < 0)
	{
		printk("[mtk-tpd] IT7250 firmware version command write fial\n");
		return false;
	}
    msleep(20);  
    waitCommandDone();
    ret_val = it7250_i2c_read(client, 0xA0 , 4 , verFw);
	printk("[mtk-tpd] IT7250 firmware read return value:(%d)\n",ret_val);
	if(ret_val < 0 )
	{
		printk("[mtk-tpd] IT7250 firmware version command read fial\n");
		return false;
	}
    sum = (verFw[0] << 24) | (verFw[1] << 16) | (verFw[2] << 8) | (verFw[3]);
	*values = sum;
	printk("[mtk-tpd] IT7250 firmware version 0x%x 0x%x 0x%x 0x%x.\n",verFw[0], verFw[1], verFw[2], verFw[3] );
	return ret_val;
}


static void Get_IT7259ID(void)
{
	uint8_t rbuffer[4];
	uint8_t data_id[2];
	uint8_t ret;
	int len = 1;
	rbuffer[0] = 0x70;
	rbuffer[1] = 0x32;

	waitCommandDone();
	
    i2c_client->addr = (i2c_client->addr & I2C_MASK_FLAG) | I2C_WR_FLAG | I2C_RS_FLAG;
    ret = i2c_master_send(i2c_client, rbuffer, ((len << 8) | 2));   
	data_id[0] = rbuffer[0];

	
	rbuffer[0] = 0x70;
	rbuffer[1] = 0x33;
	i2c_client->addr = (i2c_client->addr & I2C_MASK_FLAG) | I2C_WR_FLAG | I2C_RS_FLAG;
    ret = i2c_master_send(i2c_client, rbuffer, ((len << 8) | 2));  
	data_id[1] = rbuffer[0];
	
	printk("\r\n data_id[0] = %2x,data_id[1] = %2x\r\n",data_id[0],data_id[1]);

	if((data_id[1] == 0x72)&&(data_id[0] == 0x59))
	{
		ite7258_is_true =0 ;
	}
	else
	{
		ite7258_is_true =1 ;
	}
	return;
}



static int tpd_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int retval = TPD_OK;
	char wrBuf[5];
	char buffer[3];

	i2c_client = client;

	of_get_it7250_platform_data(&client->dev);

	GTP_GPIO_OUTPUT(GTP_RST_PORT, 1);
	GTP_GPIO_AS_INT(GTP_INT_PORT);

	it7250_power_switch(POWER_ON);
	
	msleep(100);
	Get_IT7259ID();
	if (ite7258_is_true) //not 7259 and may be 7258
	{
		GTP_GPIO_OUTPUT(GTP_RST_PORT, 0);
		msleep(200);

		if(!tpd_FW_version(client,&FWversion))
		{
			goto probe_fail;
		}
		else
		{
			if(FWversion&0x05170000) //725
			{
				printk(" 7258 touch IC\n");
			}
			else
			{
				goto probe_fail;
			}
		}
	}
	else
	{
		GTP_GPIO_OUTPUT(GTP_RST_PORT, 0);
		msleep(20);
		GTP_GPIO_OUTPUT(GTP_RST_PORT, 1); 
		msleep(100);
		printk(" 7259 touch IC\n");
	}

	wrBuf[0] = 0x20; wrBuf[1] = 0x07;
	 if (ite7258_is_true)	
	{   
	    it7250_i2c_write(i2c_client, wrBuf[0], 2,wrBuf);   // Clean Queue {0x20, 0x07}
	    do{
			it7250_i2c_read(i2c_client, 0x80, 1, buffer);
	    }while( buffer[0] & 0x01 );

	    if(it7250_i2c_read(i2c_client, 0xA0, 2, wrBuf) < 0)
		{
			 TPD_DMESG("it7250 I2C probe try transfer error, line: %d\n", __LINE__);
			goto probe_fail; //ysong
		}
	}
	 
	GTP_GPIO_AS_INT(GTP_INT_PORT);
	msleep(50);
	tpd_irq_registration();
	
	/* tpd_load_status = 1; */
	thread = kthread_run(touch_event_handler, 0, TPD_DEVICE);
	if (IS_ERR(thread)) {
		retval = PTR_ERR(thread);
		TPD_DMESG(TPD_DEVICE " failed to create kernel thread: %d\n", retval);
	}
	
	tpd_load_status = 1;
	TPD_DMESG("Touch Panel Device Probe %s\n", (retval < TPD_OK) ? "FAIL" : "PASS");
	return 0;
probe_fail:
	it7250_power_switch(POWER_OFF);
	return -1;
}

static int tpd_remove(struct i2c_client *client)
{
	TPD_DEBUG("TPD removed\n");
	return 0;
}

static int tpd_local_init(void)
{
	TPD_DMESG("IT7250 I2C Touchscreen Driver...\n");
	
	if (i2c_add_driver(&tpd_i2c_driver) != 0) {
		TPD_DMESG("unable to add i2c driver.\n");
		return -1;
	}
     /* tpd_load_status = 1; */
	if (tpd_dts_data.use_tpd_button) {
		tpd_button_setting(tpd_dts_data.tpd_key_num, tpd_dts_data.tpd_key_local,
		tpd_dts_data.tpd_key_dim_local);
	}

#if (defined(TPD_WARP_START) && defined(TPD_WARP_END))
	TPD_DO_WARP = 1;
	memcpy(tpd_wb_start, tpd_wb_start_local, TPD_WARP_CNT * 4);
	memcpy(tpd_wb_end, tpd_wb_start_local, TPD_WARP_CNT * 4);
#endif

	TPD_DMESG("end %s, %d\n", __func__, __LINE__);
	tpd_type_cap = 1;
	
	return 0;
}

static void tpd_resume(struct device *h)
{

	if(ite7258_is_true) //7258
	{
		#ifndef IT7252_NO_POWER_OFF_IN_SLEEP
		it7250_power_switch(POWER_ON);
		msleep(100);
		GTP_GPIO_OUTPUT(GTP_RST_PORT, 1); 
		msleep(200);
	  	GTP_GPIO_OUTPUT(GTP_RST_PORT, 0); 
		#endif
	}
	else
	{
		#ifndef IT7257_NO_POWER_OFF_IN_SLEEP
		it7250_power_switch(POWER_ON);
		msleep(100);
		#endif
		GTP_GPIO_OUTPUT(GTP_RST_PORT, 1); 
		msleep(200);
	}

	waitCommandDone();
	enable_irq(touch_irq);
	tpd_halt = 0;
	tpd_up(0, 0);
	input_sync(tpd->dev);
	printk("it7250 tpd_resume end\n");
}

static void tpd_suspend(struct device *h)
{
	#ifdef IT7252_NO_POWER_OFF_IN_SLEEP
	unsigned char Wrbuf_sleep[4] = { 0x20, 0x04, 0x00, 0x02 };
	#endif
	TPD_DEBUG("TPD enter sleep\n");

	if (ite7258_is_true)
	{
	    printk("it7260 tpd_suspend start\n");
		{
			tpd_halt = 1;
			disable_irq(touch_irq);
		#ifndef IT7252_NO_POWER_OFF_IN_SLEEP
			it7250_power_switch(POWER_OFF);
		#else
			it7250_i2c_write(i2c_client, Wrbuf_sleep[0], 4,Wrbuf_sleep);
		#endif
	  	}
		printk("it7260 tpd_suspend end\n");
	}
	else
	{
		disable_irq(touch_irq);
		tpd_halt = 1;
	#ifndef IT7257_NO_POWER_OFF_IN_SLEEP
		it7250_power_switch(POWER_OFF);
	#endif
		GTP_GPIO_OUTPUT(GTP_RST_PORT, 0);
	}


}

static struct tpd_driver_t tpd_device_driver = {
	.tpd_device_name = "IT7250",
	.tpd_local_init = tpd_local_init,
	.suspend = tpd_suspend,
	.resume = tpd_resume,
	.attrs = {
		.attr = it7250_attrs,
		.num  = ARRAY_SIZE(it7250_attrs),
	},
};

/* called when loaded into kernel */
static int __init tpd_driver_init(void)
{
	TPD_DMESG("MediaTek IT7250 touch panel driver init\n");
	tpd_get_dts_info();
	if (tpd_driver_add(&tpd_device_driver) < 0)
		TPD_DMESG("add IT7250 driver failed\n");

	return 0;
}

/* should never be called */
static void __exit tpd_driver_exit(void)
{
	TPD_DMESG("MediaTek IT7250 touch panel driver exit\n");
	tpd_driver_remove(&tpd_device_driver);
}

module_init(tpd_driver_init);
module_exit(tpd_driver_exit);

