#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/pm_runtime.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/of_device.h>
#include <linux/init.h>
#include <linux/pm.h>
#include <linux/regmap.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>

#include "lis3dh.h"

#define DRV_NAME	"lis3dh_i2c"

#define LIS3DH_REG_VAL_DEVICE_ID			0x33

//寄存器信息

static const struct reg_default lis3dh_reg[] = {
	{ 0x07, 0xff },
	{ 0x08, 0x00 },
	{ 0x09, 0x00 },
	{ 0x0a, 0x00 },
	{ 0x0b, 0x00 },
	{ 0x0c, 0x00 },
	{ 0x0d, 0x00 },
	{ 0x0e, 0x00 },
	{ 0x0f, 0x33 },
	{ 0x10, 0x8d },
	{ 0x11, 0x86 },
	{ 0x12, 0xb9 },
	{ 0x13, 0x01 },
	{ 0x14, 0x40 },
	{ 0x15, 0x34 },
	{ 0x16, 0x1f },
	{ 0x17, 0x1e },
	{ 0x18, 0x21 },
	{ 0x19, 0xb5 },
	{ 0x1a, 0x60 },
	{ 0x1b, 0x64 },
	{ 0x1c, 0xc0 },
	{ 0x1d, 0x00 },
	{ 0x1e, 0x50 },
	{ 0x1f, 0x00 },
	{ 0x20, 0x17 },
	{ 0x21, 0x00 },
	{ 0x22, 0x10 },
	{ 0x23, 0x08 },
	{ 0x24, 0x00 },
	{ 0x25, 0x00 },
	{ 0x26, 0x00 },
	{ 0x27, 0xff },
	{ 0x2e, 0x00 },
	{ 0x2f, 0x20 },
	{ 0x30, 0x00 },
	{ 0x31, 0x29 },
	{ 0x32, 0x01 },
	{ 0x33, 0x7f },
	{ 0x34, 0x00 },
	{ 0x35, 0x00 },
	{ 0x36, 0x00 },
	{ 0x37, 0x00 },
	{ 0x38, 0x00 },
	{ 0x39, 0x00 },
	{ 0x3a, 0x00 },
	{ 0x3b, 0x00 },
	{ 0x3c, 0x00 },
	{ 0x3d, 0x00 },
	{ 0x3e, 0x00 },
	{ 0x3f, 0x00 },
};

static bool lis3dh_volatile_register(struct device *dev, unsigned int reg)
{
	return true;
}

static bool lis3dh_readable_register(struct device *dev, unsigned int reg)
{
	switch (reg)
	{
		case 0x00:
		case 0x01:
		case 0x02:
		case 0x03:
		case 0x04:
		case 0x05:
		case 0x06:
		{
			return false;
		}break;
		default:
		{
			return true;
		}break;
	}
}

static bool lis3dh_writeable_register(struct device *dev, unsigned int reg)
{
	switch (reg)
	{
		case 0x1e:
		case 0x1f:
		case 0x20:
		case 0x21:
		case 0x22:
		case 0x23:
		case 0x24:
		case 0x25:
		case 0x26:
		case 0x2e:
		case 0x30:
		case 0x32:
		case 0x33:
		case 0x34:
		case 0x36:
		case 0x37:
		case 0x38:
		case 0x3a:
		case 0x3b:
		case 0x3c:
		case 0x3d:
		case 0x3e:
		case 0x3f:
		{
			return true;
		}break;
		default:
		{
			return false;
		}break;
	}
}

static int lis3dh_i2c_update_bits(struct lis3dh_priv *lis3dh, unsigned int reg,
	unsigned int mask, unsigned int new_val)
{
	return regmap_update_bits(lis3dh->regmap, reg, mask, new_val);
}

static int lis3dh_i2c_write(struct lis3dh_priv *lis3dh, unsigned int reg, unsigned int val)
{
	return regmap_write(lis3dh->regmap, reg, val);
}

static unsigned int lis3dh_i2c_read(struct lis3dh_priv *lis3dh, unsigned int reg)
{
	unsigned int val = 0;

	if(regmap_read(lis3dh->regmap, reg, &val))
	{
		val = -1;
	}

	return val;
}

static int lis3dh_i2c_read_raw_data(struct lis3dh_priv *lis3dh, axis_raw_data_t *raw)
{
	unsigned int val_l = 0;
	unsigned int val_h = 0;
	if (NULL == raw)
	{
		return -1;
	}

	// High-resolution mode (12-bit data output)
	regmap_read(lis3dh->regmap, LIS3DH_OUT_X_L, &val_l);
	regmap_read(lis3dh->regmap, LIS3DH_OUT_X_H, &val_h);
	raw->axis_x = ((val_l >> 4) | (val_h << 8))  & 0xffff;;

	regmap_read(lis3dh->regmap, LIS3DH_OUT_Y_L, &val_l);
	regmap_read(lis3dh->regmap, LIS3DH_OUT_Y_H, &val_h);
	raw->axis_y = ((val_l >> 4) | (val_h << 8))  & 0xffff;;

	regmap_read(lis3dh->regmap, LIS3DH_OUT_Z_L, &val_l);
	regmap_read(lis3dh->regmap, LIS3DH_OUT_Z_H, &val_h);
	raw->axis_z = ((val_l >> 4) | (val_h << 8)) & 0xffff;;

	return 0;
}

ssize_t lis3dh_reg_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct lis3dh_priv *lis3dh = i2c_get_clientdata(client);
	int i = 0;
	char *token = NULL;
	char *cookie_pot = (char *)buf;
	unsigned int value = 0, reg = 0;

	while((token = strsep(&cookie_pot,",")))
    {
        if (!i++)
        {
            reg = simple_strtol(token, NULL, 16);
        }
        else
        {
            value = simple_strtol(token, NULL, 16);
        }
    }
    dev_err(dev, "%s: reg = 0x%02x, value = 0x%02x\n", __func__, reg, value);
    lis3dh_i2c_write(lis3dh, reg, value);

	return count;
}

static ssize_t lis3dh_reg_show(struct device* dev, struct device_attribute *attr, char* buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct lis3dh_priv *lis3dh = i2c_get_clientdata(client);
	unsigned int val;
	int cnt = 0, i = 0;

	cnt += sprintf(buf, "LIS3DH codec register:\n");
	for (i = 0; i <= LIS3DH_ACT_DUR; i++)
	{
		if (cnt + 22 >= PAGE_SIZE)
			break;
		if (lis3dh_readable_register(dev, i))
		{
			val = lis3dh_i2c_read(lis3dh, i);

			cnt += snprintf(buf + cnt, 22,
					"%02x: %02x\n", i, val);
		}
	}

	if (cnt >= PAGE_SIZE)
		cnt = PAGE_SIZE - 1;
    return cnt;
}
static DEVICE_ATTR(lis3dh_reg, S_IRUGO | S_IWUSR, lis3dh_reg_show, lis3dh_reg_store);


#ifdef CONFIG_OF
static struct of_device_id lis3dh_i2c_dt_ids[] = {
	{ .compatible = "st,lis3dh" },
	{}
};
MODULE_DEVICE_TABLE(of, lis3dh_i2c_dt_ids);
#endif	// CONFIG_OF


static const struct i2c_device_id lis3lv02d_id[] = {
	{"lis3dh", LIS3DH},
	{}
};

MODULE_DEVICE_TABLE(i2c, lis3lv02d_id);


#ifdef CONFIG_PM_SLEEP
static int lis3dh_i2c_suspend(struct device *dev)
{
	// struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	// struct lis3dh_priv *lis3 = i2c_get_clientdata(client);


	return 0;
}

static int lis3dh_i2c_resume(struct device *dev)
{
	// struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	// struct lis3dh_priv *lis3 = i2c_get_clientdata(client);

	//todo, 电源恢复时上电


	return 0;
}
#endif /* CONFIG_PM_SLEEP */

#ifdef CONFIG_PM_RUNTIME
static int lis3_i2c_runtime_suspend(struct device *dev)
{
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	struct lis3dh_priv *lis3dh = i2c_get_clientdata(client);

	return lis3dh_set_power_mode(lis3dh, LIS3DH_POWER_DOWN);
}

static int lis3_i2c_runtime_resume(struct device *dev)
{
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	struct lis3dh_priv *lis3dh = i2c_get_clientdata(client);

	return lis3dh_set_power_mode(lis3dh, LIS3DH_NORMAL);
}
#endif /* CONFIG_PM_RUNTIME */


static const struct dev_pm_ops lis3_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(lis3dh_i2c_suspend,
				lis3dh_i2c_resume)
	SET_RUNTIME_PM_OPS(lis3_i2c_runtime_suspend,
			   lis3_i2c_runtime_resume,
			   NULL)
};

#if 0
static unsigned int use_default_i2c_method_read(struct i2c_client *i2c, unsigned int reg, unsigned int *value)
{
	unsigned int ret = 0;
	struct i2c_msg xfer[2];
	unsigned char _reg = reg;

	dev_err(&i2c->dev, "###clc i2c_transfer reg = %#02x\n", _reg);

	xfer[0].addr = i2c->addr;
	xfer[0].flags = 0;
	xfer[0].len = 1;
	xfer[0].buf = (void *)&_reg;

	xfer[1].addr = i2c->addr;
	xfer[1].flags = I2C_M_RD;
	xfer[1].len = 1;
	xfer[1].buf = (void *)value;

	ret = i2c_transfer(i2c->adapter, xfer, 2);
	if (ret == 2)
		return 0;
	else if (ret < 0)
		return ret;
	else
		return -EIO;
}
#endif

static const struct regmap_config lis3dh_regmap = {
	//.cache_type = REGCACHE_RBTREE,
	.cache_type = REGCACHE_NONE,
	//clc DS说都是8位的，但是重力加速度XYZ是16位的ADC，分两次读
	.reg_bits = 8,
	.val_bits = 8,
	.use_single_rw = true,

	.max_register = LIS3DH_ACT_DUR,
	.volatile_reg = lis3dh_volatile_register,
	.readable_reg = lis3dh_readable_register,
	.writeable_reg = lis3dh_writeable_register,
	.reg_defaults = lis3dh_reg,
	.num_reg_defaults = ARRAY_SIZE(lis3dh_reg),
};

static int lis3dh_i2c_probe(struct i2c_client *i2c, const struct i2c_device_id *id)
{
	int ret = 0;
	//dump_stack();		//clc debug call backtrace
	struct lis3dh_priv *lis3dh = NULL;
	unsigned int val = 0;
	struct device *dev = &i2c->dev;
	struct device_node *node = i2c->dev.of_node;
	enum of_gpio_flags flags;
	struct lis3dh_pdata *pdata = NULL;

	dev_err(dev, "### clc LIS3DH i2c Probe Enter!\n");

	lis3dh = devm_kzalloc(&i2c->dev, sizeof(struct lis3dh_priv), GFP_KERNEL);
	if (NULL == lis3dh)
		return -ENOMEM;

	memset(lis3dh, 0, sizeof(struct lis3dh_priv));

	lis3dh->int_data_rdy = 0;
	lis3dh->bus_priv = i2c;
	lis3dh->update_bits 	= lis3dh_i2c_update_bits;
	lis3dh->write 			= lis3dh_i2c_write;
	lis3dh->read 			= lis3dh_i2c_read;
	lis3dh->read_raw_data 	= lis3dh_i2c_read_raw_data;

	i2c_set_clientdata(i2c, lis3dh);
	pdata = &lis3dh->pdata;

	lis3dh->int_pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR_OR_NULL(lis3dh->int_pinctrl))
	{
        dev_err(dev, "%s: Cannot get gpio pinctrl:%ld\n", __func__, PTR_ERR(lis3dh->int_pinctrl));
        ret = PTR_ERR(lis3dh->int_pinctrl);
        goto err_pctrl_get;
    }

    lis3dh->int_pinctrl_sta = pinctrl_lookup_state(lis3dh->int_pinctrl, "gsensor_int");
    if (IS_ERR_OR_NULL(lis3dh->int_pinctrl_sta))
    {
        dev_err(dev, "%s: Cannot get gsensor_int pinctrl default state:%ld\n",
        					__func__, PTR_ERR(lis3dh->int_pinctrl_sta));
        ret = PTR_ERR(lis3dh->int_pinctrl_sta);
        goto err_lookup_state;
    }

    ret = pinctrl_select_state(lis3dh->int_pinctrl, lis3dh->int_pinctrl_sta);
    if (ret)
    {
        dev_err(dev, "%s: set gsensor_int gpio default state fail: %d\n", __func__, ret);
    }

    ret = of_get_named_gpio_flags(node, "gsensor_int_gpio", 0, &flags);
    if (ret < 0)
    {
    	dev_err(dev, "%s: get gsensor_int gpio fail: %d\n", __func__, ret);
    	goto err_get_gpio;
    }
    else
    {
    	pdata->gsensor_int_gpio = ret;
    	dev_err(dev, "%s: get gsensor interrupt gpio num: %d\n", __func__, pdata->gsensor_int_gpio);
    	//if( gpio_is_valid(pdata->gsensor_int_gpio) ) goto err_req_gpio;
    	ret = devm_gpio_request(dev, pdata->gsensor_int_gpio, "gsensor_int_gpio");
		if(ret < 0){
			dev_err(dev, "%s: devm_gpio_request gsensor_int_gpio request ERROR\n", __func__);
			goto err_req_gpio;
		}

		ret = gpio_direction_input(pdata->gsensor_int_gpio);
		if(ret < 0){
			dev_err(dev, "%s: gpio_direction_input gsensor_int_gpio set ERROR\n", __func__);
			goto err_req_gpio;
		}
    }

	lis3dh->regmap = devm_regmap_init_i2c(i2c, &lis3dh_regmap);
	if (IS_ERR(lis3dh->regmap))
	{
		ret = PTR_ERR(lis3dh->regmap);
		dev_err(&i2c->dev, "Failed to allocate register map: %d\n", ret);
		goto err_req_gpio;
	}
	regcache_cache_bypass(lis3dh->regmap, true);

	//clc 判断一下是不是对应的Gsensor设备
	ret =  regmap_read(lis3dh->regmap, LIS3DH_REG_ADDR_DEVICE_ID, &val);
	//ret = use_default_i2c_method_read(i2c, LIS3DH_REG_ADDR_DEVICE_ID, &val);
	if (ret)
	{
		dev_err(&i2c->dev, "###clc read WHO AM I reg. failed. val = %#x\n", val);
		ret = -ENODEV;
		goto err_req_gpio;
	}

	if (val != LIS3DH_REG_VAL_DEVICE_ID) {
		dev_err(&i2c->dev, "Device with ID register %x is not LIS3DH.\n", val);
		ret = -ENODEV;
		goto err_req_gpio;
	}
	dev_err(&i2c->dev, "Device with ID register %x is LIS3DH.\n", val);

	if ( (ret = lis3dh_init_device(lis3dh)) )
	{
		dev_err(&i2c->dev, "lis3dh_init_device FAILED!\n");
		goto err_req_gpio;
	}

	//debug info
	if ((ret = device_create_file(dev, &dev_attr_lis3dh_reg)) != 0) {
        dev_err(&i2c->dev,"### create lis3dh_reg attr file failed\n");
        goto err_req_gpio;
    }

	return 0;

err_req_gpio:
err_get_gpio:
err_lookup_state:
	devm_pinctrl_put(lis3dh->int_pinctrl);
	devm_gpio_free(dev, pdata->gsensor_int_gpio);

err_pctrl_get:
	devm_kfree(dev, lis3dh);

	return ret;
}

static int lis3dh_i2c_remove(struct i2c_client *i2c)
{
	struct lis3dh_priv *lis3dh = i2c_get_clientdata(i2c);
	lis3dh_remove_device(lis3dh);
	devm_pinctrl_put(lis3dh->int_pinctrl);
	devm_gpio_free(&i2c->dev, lis3dh->pdata.gsensor_int_gpio);
	devm_kfree(&i2c->dev, lis3dh);
	return 0;
}

static struct i2c_driver lis3dh_i2c_driver = {
	.driver	 = {
		.name   = DRV_NAME,
		.owner  = THIS_MODULE,
		.pm     = &lis3_pm_ops,
		.of_match_table = of_match_ptr(lis3dh_i2c_dt_ids),
	},
	.probe	= lis3dh_i2c_probe,
	.remove	= lis3dh_i2c_remove,
	.id_table = lis3lv02d_id,
};

module_i2c_driver(lis3dh_i2c_driver);

MODULE_AUTHOR("YAME");
MODULE_DESCRIPTION("lis3dh I2C interface. Guangzhou YAME Information Technology Co,. LTD");
MODULE_LICENSE("GPL v2");

