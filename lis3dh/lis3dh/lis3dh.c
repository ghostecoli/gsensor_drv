
#include <linux/kernel.h>
#include <linux/dmi.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/platform_device.h>
#include <linux/input-polldev.h>
#include <linux/delay.h>
#include <linux/semaphore.h>
#include <linux/wait.h>
#include <linux/poll.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/freezer.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#include <linux/pm_runtime.h>
#include <linux/atomic.h>
#include <linux/gpio.h>
#include <linux/irq.h>
#include <linux/fs.h>
#include <linux/of_device.h>

#include "lis3dh.h"

#define LIS3DH_DEVICE_NODE_NAME "lis3dh_acc"
#define SCALE_RANGE_16BIT (65536)

static struct lis3dh_priv *lis3dh_dev = NULL;
static DEFINE_SEMAPHORE(lis3dh_lock);
static DECLARE_WAIT_QUEUE_HEAD(lis3dh_waitq);

//*****************************************
// 配置gsensor工作模式

int lis3dh_set_power_mode(struct lis3dh_priv *lis3dh, LIS3DH_Mode_t mode)
{
	static unsigned int obr_old_value = 0;
	unsigned int value1 = 0;
	unsigned int value2 = 0;

	value1 = lis3dh->read(lis3dh, LIS3DH_CTRL_REG1);
	value2 = lis3dh->read(lis3dh, LIS3DH_CTRL_REG4);

	if(0 == (value1 & 0xF0))
	{
    	value1 = value1 | (obr_old_value & 0xF0); //if it comes from POWERDOWN
	}

	switch(mode)
	{
		case LIS3DH_POWER_DOWN:
		{
			obr_old_value = value1;
    		value1 &= 0x0F;
		}break;
		case LIS3DH_LOW_POWER:
		{
			value1 &= 0xF7;
		    value1 |=  (MEMS_SET << LIS3DH_LPEN);
		    value2 &= 0xF7;
		    value2 |= (MEMS_RESET << LIS3DH_HR); //reset HighResolution_BIT
		}break;
		case LIS3DH_NORMAL:
		{
			value1 &= 0xF7;
		    value1 |= (MEMS_RESET << LIS3DH_LPEN);
		    value2 &= 0xF7;
		    value2 |= (MEMS_SET << LIS3DH_HR);   //set HighResolution_BIT
		}break;
		default:
		{
			return -1;
		}break;
	}

	lis3dh->write(lis3dh, LIS3DH_CTRL_REG1, value1);
	lis3dh->write(lis3dh, LIS3DH_CTRL_REG4, value2);

	return 0;
}
EXPORT_SYMBOL_GPL(lis3dh_set_power_mode);

static inline void lis3dh_set_int_pin_mode(struct lis3dh_priv *lis3dh)
{
	// 只启动I1_IA1中断使能
	//lis3dh->update_bits(lis3dh, LIS3DH_CTRL_REG3, (0x1 << 6), (0x1 << 6));
	lis3dh->write(lis3dh, LIS3DH_CTRL_REG3, 0x40);

	// INT1 and INT2 pin polarity. Default value: 0
	// (0: active-high; 1: active-low)
	lis3dh->update_bits(lis3dh, LIS3DH_CTRL_REG6, 0x02, 0x00);
}

static inline int lis3dh_set_axis_int_config(struct lis3dh_priv *lis3dh, unsigned int int_mode)
{
	unsigned int cur_int_mode = lis3dh->read(lis3dh, LIS3DH_INT1_CFG);
	if (-1 == cur_int_mode)
	{
		return -1;
	}
	//todo,
	cur_int_mode &= 0xC0;
	cur_int_mode |= int_mode;

	return lis3dh->update_bits(lis3dh, LIS3DH_INT1_CFG, (0x3f << 0), cur_int_mode);
}

static inline int lis3dh_set_int_mode(struct lis3dh_priv *lis3dh, LIS3DH_INT_MODE_TYPE_t mode)
{
	unsigned int int_mode = 0;

	switch(mode)
	{
		case INT_MODE_OR_COMB:
		{
		}break;
		case INT_MODE_6D_MOVEMENT_REC:
		{
			int_mode |= (0x1 << 6);
		}break;
		case INT_MODE_AND_COMB:
		{
			int_mode |= (0x1 << 7);
		}break;
		case INT_MODE_6D_POSITION_REC:
		{
			int_mode |= (0x3 << 6);
		}break;
		default:
		{
			return -1;
		}break;
	}

	return lis3dh->update_bits(lis3dh, LIS3DH_INT1_CFG, (0x3 << 6), int_mode);
}

static inline int lis3dh_set_interrupt_threshold(struct lis3dh_priv *lis3dh, unsigned int threshold)
{
	if (threshold > 127)
	{
		return -1;
	}

	return lis3dh->write(lis3dh, LIS3DH_INT1_THS, threshold);
}

static inline int lis3dh_get_full_scale(struct lis3dh_priv *lis3dh, unsigned int *scale)
{
	unsigned int value = 0;
	value = lis3dh->read(lis3dh, LIS3DH_CTRL_REG4);
	if (-1 == value)
	{
		return -1;
	}
	value &= (0x3 << 4);
	value >>=  4;
	*scale = value;
	return 0;
}

static inline int lis3dh_set_full_scale(struct lis3dh_priv *lis3dh, LIS3DH_Fullscale_t scale)
{
	switch(scale)
	{
		case LIS3DH_FULLSCALE_2...LIS3DH_FULLSCALE_16:
		{
			lis3dh->cur_scale_range = scale;
			return lis3dh->update_bits(lis3dh, LIS3DH_CTRL_REG4, LIS3DH_FS_MASK, scale << LIS3DH_ODR_BIT);
		}break;
		default:break;
	}
	return -1;
}

static inline int lis3dh_set_axis(struct lis3dh_priv *lis3dh, LIS3DH_AXISenable_t enable)
{
	switch(enable)
	{
		case LIS3DH_X_ENABLE:
		{
			return lis3dh->update_bits(lis3dh, LIS3DH_CTRL_REG1, (0x1 << 0), 0x1);
		}break;
		case LIS3DH_X_DISABLE:
		{
			return lis3dh->update_bits(lis3dh, LIS3DH_CTRL_REG1, (0x1 << 0), 0x0);
		}break;
		case LIS3DH_Y_ENABLE:
		{
			return lis3dh->update_bits(lis3dh, LIS3DH_CTRL_REG1, (0x1 << 1), 0x1);
		}break;
		case LIS3DH_Y_DISABLE:
		{
			return lis3dh->update_bits(lis3dh, LIS3DH_CTRL_REG1, (0x1 << 1), 0x0);
		}break;
		case LIS3DH_Z_ENABLE:
		{
			return lis3dh->update_bits(lis3dh, LIS3DH_CTRL_REG1, (0x1 << 2), 0x1);
		}break;
		case LIS3DH_Z_DISABLE:
		{
			return lis3dh->update_bits(lis3dh, LIS3DH_CTRL_REG1, (0x1 << 2), 0x0);
		}break;
		case LIS3DH_X_ENABLE | LIS3DH_Y_ENABLE | LIS3DH_Z_ENABLE:
		{
			return lis3dh->update_bits(lis3dh, LIS3DH_CTRL_REG1, LIS3DH_3AXIS_EN_MASK, enable << 0);
		}break;
		default:break;
	}
	return -1;
}

static inline int lis3dh_set_odr(struct lis3dh_priv *lis3dh, LIS3DH_ODR_t odr)
{
	switch(odr)
	{
		case LIS3DH_ODR_1Hz...LIS3DH_ODR_1344Hz_NP_5367HZ_LP:
		{
			return lis3dh->update_bits(lis3dh, LIS3DH_CTRL_REG1, LIS3DH_ODR_BIT_MASK, odr << LIS3DH_ODR_BIT);
		}break;
		default:break;
	}
	return -1;
}

static int raw_mg_range_map[4] = {
	[LIS3DH_FULLSCALE_2] = 4000,			//+- 2G, 4000mg
	[LIS3DH_FULLSCALE_4] = 8000,			//+- 4G, 8000mg
	[LIS3DH_FULLSCALE_8] = 16000,			//+- 8G, 16000mg
	[LIS3DH_FULLSCALE_16] = 32000,			//+- 16G, 32000mg
};

static inline void convert_raw_data_to_mg(unsigned int scale_range, axis_raw_data_t *raw, axis_data_mg_t *dat_mg)
{
	//todo, 不太建议在内核态做这些转换
	dat_mg->x = (int)(raw->axis_x * raw_mg_range_map[scale_range] / SCALE_RANGE_16BIT);
	dat_mg->y = (int)(raw->axis_y * raw_mg_range_map[scale_range] / SCALE_RANGE_16BIT);
	dat_mg->z = (int)(raw->axis_z * raw_mg_range_map[scale_range] / SCALE_RANGE_16BIT);
}

static void lis3dh_get_data_mg(struct lis3dh_priv *lis3dh, axis_data_mg_t *dat_mg)
{
	axis_raw_data_t raw_dat = {0};
	lis3dh->read_raw_data(lis3dh, &raw_dat);

	convert_raw_data_to_mg(lis3dh->cur_scale_range, &raw_dat, dat_mg);
}

static int lis3dh_drv_open(struct inode *inode, struct file *file)
{
	//down(&lis3dh_lock);
	if (down_trylock(&lis3dh_lock))
			return -EBUSY;

	file->private_data = lis3dh_dev;
	enable_irq(lis3dh_dev->irq);

	return 0;
}

static int lis3dh_drv_close(struct inode *inode, struct file *file)
{
	struct lis3dh_priv *lis3dh = (struct lis3dh_priv *)file->private_data;
	up(&lis3dh_lock);
	disable_irq_nosync(lis3dh->irq);
	return 0;
}

static inline int lis3dh_zyx_data_is_ready(struct lis3dh_priv *lis3dh)
{
	unsigned int value = 0;
	value = lis3dh->read(lis3dh, LIS3DH_STATUS_REG);
	if (value & 0x00000008)
	{
		return 1;
	}
	return 0;
}

static ssize_t lis3dh_drv_read(struct file *file, char __user *buf, size_t size, loff_t *ppos)
{
	unsigned long ret = 0;
	size_t len = sizeof(axis_data_mg_t);
	axis_data_mg_t data;

	struct lis3dh_priv *lis3dh = (struct lis3dh_priv *)file->private_data;

	// while(!have_data)
	// {
	// 	if (file->f_flags & O_NONBLOCK)
	// 		return -EAGAIN;

	// 	wait_event_interruptible(lis3dh_waitq, lis3dh->int_data_rdy);
	// }

	if (file->f_flags & O_NONBLOCK)
	{
		// if (lis3dh->int_data_rdy)
		// 	return -EAGAIN;
	}
	else
	{
		// 阻塞读
		while(!lis3dh_zyx_data_is_ready(lis3dh))
		{
			usleep_range(100, 200);
		}

		lis3dh_get_data_mg(lis3dh, &data);
		if (len > size)
		{
			len = size;
		}
		ret = copy_to_user(buf, &data, len);
	}

	return len;
}

static unsigned int lis3dh_drv_poll(struct file *file, poll_table *wait)
{
	return 0;
}

static long lis3dh_drv_ioctl(struct file *file, unsigned int cmd, unsigned long value)
{
	long err = 0;
	struct lis3dh_priv *lis3dh = (struct lis3dh_priv *)file->private_data;
	int power_status = 0;
	int odr_status = 0;
	int scale_status = 0;
	int int_ths = 0;
	int int_mode = 0;
	int axis_en = 0;
	unsigned int scale = 0;
	unsigned int axis_int_status = 0;
	void __user *u_argp = (void __user *)value;

	if (_IOC_TYPE(cmd) != GSENSOR_IOCTL_BASE)
	{
        pr_err("%s command type [%c] error!\n", __func__, _IOC_TYPE(cmd));
        return -ENOTTY;
    }

	switch(cmd)
	{
		case GSENSOR_GET_MODEL_NAME:
		{
			if (copy_to_user(u_argp, LIS3DH_DEVICE_NODE_NAME, strlen(LIS3DH_DEVICE_NODE_NAME) + 1))
			{
	            printk(KERN_ERR "GSENSOR_GET_MODEL_NAME copy_to_user failed.\n");
	            err = -EFAULT;
        	}
		}break;
		case GSENSOR_SET_POWER_STATUS:
		{
			if (copy_from_user(&power_status, u_argp, sizeof(int)))
			{
	            printk(KERN_ERR "GSENSOR_SET_POWER_STATUS copy_from_user failed.\n");
	            err = -EFAULT;
	            goto ioctl_err_out;
        	}
        	err = lis3dh_set_power_mode(lis3dh, power_status);
		}break;
		case GSENSOR_SET_ODR_STATUS:
		{
			if (copy_from_user(&odr_status, u_argp, sizeof(int)))
			{
	            printk(KERN_ERR "GSENSOR_SET_ODR_STATUS copy_from_user failed.\n");
	            err = -EFAULT;
	            goto ioctl_err_out;
        	}
        	err = lis3dh_set_odr(lis3dh, odr_status);
		}break;
		case GSENSOR_SET_SCALE_STATUS:
		{
			if (copy_from_user(&scale_status, u_argp, sizeof(int)))
			{
	            printk(KERN_ERR "GSENSOR_SET_SCALE_STATUS copy_from_user failed.\n");
	            err = -EFAULT;
	            goto ioctl_err_out;
        	}
        	err = lis3dh_set_full_scale(lis3dh, scale_status);
		}break;
		case GSENSOR_GET_SCALE_STATUS:
		{
			lis3dh_get_full_scale(lis3dh, &scale);
			if (copy_to_user(u_argp, &scale, sizeof(scale)))
			{
	            printk(KERN_ERR "GSENSOR_GET_SCALE_STATUS copy_to_user failed.\n");
	            err = -EFAULT;
        	}
		}break;
		case GSENSOR_SET_INT_THRESHOLD:
		{
			if (copy_from_user(&int_ths, u_argp, sizeof(int)))
			{
	            printk(KERN_ERR "GSENSOR_SET_INT_THRESHOLD copy_from_user failed.\n");
	            err = -EFAULT;
	            goto ioctl_err_out;
        	}
        	err = lis3dh_set_interrupt_threshold(lis3dh, int_ths);
		}break;
		case GSENSOR_SET_INTERRUPT_MODE:
		{
			if (copy_from_user(&int_mode, u_argp, sizeof(int)))
			{
	            printk(KERN_ERR "GSENSOR_SET_INTERRUPT_MODE copy_from_user failed.\n");
	            err = -EFAULT;
	            goto ioctl_err_out;
        	}
        	err = lis3dh_set_int_mode(lis3dh, int_mode);
		}break;
		case GSENSOR_SET_AXIS_ENABLE:
		{
			if (copy_from_user(&axis_en, u_argp, sizeof(int)))
			{
	            printk(KERN_ERR "GSENSOR_SET_AXIS_ENABLE copy_from_user failed.\n");
	            err = -EFAULT;
	            goto ioctl_err_out;
        	}
        	err = lis3dh_set_axis(lis3dh, axis_en);
		}break;
		case GSENSOR_SET_AXIS_INTERRUPT:
		{
			if (copy_from_user(&axis_int_status, u_argp, sizeof(int)))
			{
	            printk(KERN_ERR "GSENSOR_SET_AXIS_INTERRUPT copy_from_user failed.\n");
	            err = -EFAULT;
	            goto ioctl_err_out;
        	}
        	err = lis3dh_set_axis_int_config(lis3dh, axis_int_status);
		}break;
		case GSENSOR_GET_INTTERRUPT_DATA:
		{
			wait_event_interruptible(lis3dh_waitq, lis3dh->int_data_rdy);

			//读中断时的数据，通知应用层用ioctl来获取中断时的数据
			lis3dh_get_data_mg(lis3dh, &lis3dh->int_axis_data);

			if (copy_to_user(u_argp, &lis3dh->int_axis_data, sizeof(axis_data_mg_t))) {
                pr_err("SENSOR_GET_INT_RAW_DATA copy_to_user failed.\n");
                err = -EFAULT;
            }

			lis3dh->int_data_rdy = 0;
		}break;
		default:break;
	}

ioctl_err_out:

	return err;
}

static int lis3dh_drv_fasync (int fd, struct file *filp, int on)
{
	struct lis3dh_priv *lis3dh = filp->private_data;

	return fasync_helper(fd, filp, on, &lis3dh->lis3dh_async);
}

//中断下半部读出此时的Gsensor数据
static void lis3dh_bottom_half_handler(unsigned long data)
{
	struct lis3dh_priv *lis3dh = (struct lis3dh_priv *)data;

	lis3dh->int_data_rdy = 1;
	wake_up_interruptible(&lis3dh_waitq);
	kill_fasync(&lis3dh->lis3dh_async, SIGIO, POLL_IN);	//异步IO通知

#if (LIS3DH_USE_INPUT_SUBSYS == 1)
	input_report_abs(lis3dh->idev, ABS_X, lis3dh->int_axis_data.axis_x);
    input_report_abs(lis3dh->idev, ABS_Y, lis3dh->int_axis_data.axis_x);
    input_report_abs(lis3dh->idev, ABS_Z, lis3dh->int_axis_data.axis_x);
    input_sync(lis3dh->idev);
#endif
    enable_irq(lis3dh->irq);
}

static irqreturn_t lis3dh_interrupt_isr(int irq, void *dev)
{
	struct lis3dh_priv *lis3dh = (struct lis3dh_priv *)dev;

//	printk(KERN_ERR "###[%s:%d] ISR Enter!\n", __func__, __LINE__);

	tasklet_init(&lis3dh->lis3dh_tasklet, lis3dh_bottom_half_handler, (unsigned long)lis3dh);
	tasklet_schedule(&lis3dh->lis3dh_tasklet);
	disable_irq_nosync(lis3dh->irq);

	return IRQ_RETVAL(IRQ_HANDLED);
}

static int lis3dh_work_init(struct lis3dh_priv *lis3dh)
{
	int err = 0;
	//1. set ODR (turn ON device)
	err = lis3dh_set_odr(lis3dh, LIS3DH_ODR_1Hz);
	if (err)
	{
		goto err_out;
	}

	//2. PowerMode
	err = lis3dh_set_power_mode(lis3dh, LIS3DH_NORMAL);
	if (err)
	{
		goto err_out;
	}

	//3. set Fullscale
	err = lis3dh_set_full_scale(lis3dh, LIS3DH_FULLSCALE_2);
	if (err)
	{
		goto err_out;
	}
	lis3dh->cur_scale_range = LIS3DH_FULLSCALE_2;

	lis3dh_set_int_pin_mode(lis3dh);
	//4. set Interrupt Threshold and INT. mode
	err = lis3dh_set_interrupt_threshold(lis3dh, 96);
	if (err)
	{
		goto err_out;
	}
	err = lis3dh_set_int_mode(lis3dh, INT_MODE_OR_COMB);
	if (err)
	{
		goto err_out;
	}

	//5. set axis Enable
	err = lis3dh_set_axis(lis3dh, LIS3DH_X_ENABLE | LIS3DH_Y_ENABLE | LIS3DH_Z_ENABLE);
	if (err)
	{
		goto err_out;
	}

	return 0;

err_out:
	pr_err("%s:%d lis3dh device init failed!!\n", __func__, __LINE__);
	return err;
}

static struct file_operations lis3dh_drv_fops = {
	.owner				= THIS_MODULE,
	.open				= lis3dh_drv_open,
	.read				= lis3dh_drv_read,
	.release			= lis3dh_drv_close,
	.poll 				= lis3dh_drv_poll,
	.unlocked_ioctl		= lis3dh_drv_ioctl,
	.fasync	 			= lis3dh_drv_fasync,
};


int lis3dh_init_device(struct lis3dh_priv *lis3dh)
{
	int ret = 0;
	int irq_flags = IRQF_TRIGGER_RISING;
	struct lis3dh_pdata *pdata = &lis3dh->pdata;

#if (LIS3DH_USE_INPUT_SUBSYS == 1)
	struct i2c_client *i2c = (struct i2c_client *)lis3dh->bus_priv;
#endif

	if( (ret = lis3dh_work_init(lis3dh)) )
	{
		goto err_out;
	}

	lis3dh->irq = gpio_to_irq(pdata->gsensor_int_gpio);

	ret = request_irq(lis3dh->irq, lis3dh_interrupt_isr,
				IRQF_SHARED | IRQF_ONESHOT | irq_flags,
				LIS3DH_DEVICE_NODE_NAME,
				lis3dh);
	disable_irq_nosync(lis3dh->irq);

	if (ret < 0) {
		pr_err("Request IRQ(%d) Failed!\n", lis3dh->irq);
		goto err_out;
	}
	enable_irq_wake(lis3dh->irq);


	lis3dh->miscdev.minor = MISC_DYNAMIC_MINOR;
	lis3dh->miscdev.name = LIS3DH_DEVICE_NODE_NAME;
	lis3dh->miscdev.fops = &lis3dh_drv_fops;

	//clc 注册一个misc设备，让上层应用可以select以获取lis3dh中断数据
	if ((ret = misc_register(&lis3dh->miscdev)))
		pr_err("misc_register failed\n");

#if (LIS3DH_USE_INPUT_SUBSYS == 1)

	lis3dh->idev = input_allocate_device();
	if (!lis3dh->idev)
	{
		pr_err("input_allocate_device failed\n");
		ret = -ENOMEM;
		goto err_out;
	}
	lis3dh->idev->name = "lis3dh_accel";
	lis3dh->idev->id.bustype = BUS_I2C;
	lis3dh->idev->evbit[0] = BIT_MASK(EV_ABS);
	lis3dh->idev->dev.parent = &i2c->dev;

	input_set_abs_params(lis3dh->idev, ABS_X, -0x7fff, 0x7fff, 0, 0);
    input_set_abs_params(lis3dh->idev, ABS_Y, -0x7fff, 0x7fff, 0, 0);
    input_set_abs_params(lis3dh->idev, ABS_Z, -0x7fff, 0x7fff, 0, 0);

    ret = input_register_device(lis3dh->idev);
    if (ret)
    {
    	pr_err("input_register_device failed\n");
    	goto input_err;
    }

#endif

	lis3dh_dev = lis3dh;

	return 0;

#if (LIS3DH_USE_INPUT_SUBSYS == 1)
input_err:
	input_free_device(lis3dh->idev);
#endif

err_out:

	return ret;
}
EXPORT_SYMBOL_GPL(lis3dh_init_device);

int lis3dh_remove_device(struct lis3dh_priv *lis3dh)
{
	if (NULL == lis3dh)
	{
		pr_err("%s:%d lis3dh is NULL!!\n", __func__, __LINE__);
		return -1;
	}
#if (LIS3DH_USE_INPUT_SUBSYS == 1)
	input_free_device(lis3dh->idev);
#endif
	free_irq(lis3dh->irq, lis3dh);
	misc_deregister(&lis3dh->miscdev);
	return 0;
}
EXPORT_SYMBOL_GPL(lis3dh_remove_device);


MODULE_AUTHOR("YAME");
MODULE_DESCRIPTION("G-sensor driver. Guangzhou YAME Information Technology Co,. LTD");
MODULE_LICENSE("GPL v2");
