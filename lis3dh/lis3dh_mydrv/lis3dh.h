#ifndef __YM_LIS3HD_ACC_H__
#define __YM_LIS3HD_ACC_H__

#include <linux/platform_device.h>
#include <linux/input-polldev.h>
#include <linux/regulator/consumer.h>
#include <linux/miscdevice.h>
#include <linux/interrupt.h>

#define LIS3DH_USE_INPUT_SUBSYS 0
#define LIS3DH_USE_ISR_TASKLET 0

#define LIS3DH_REG_ADDR_DEVICE_ID     0x0F

#define MEMS_SET                                        0x01
#define MEMS_RESET                                      0x00


//ioctl命令构建
#define GSENSOR_IOCTL_BASE  'Y'
#define GSENSOR_GET_MODEL_NAME              _IOR(GSENSOR_IOCTL_BASE, 0, char *)
#define GSENSOR_SET_POWER_STATUS            _IOR(GSENSOR_IOCTL_BASE, 1, int)
#define GSENSOR_SET_ODR_STATUS              _IOR(GSENSOR_IOCTL_BASE, 3, int)
#define GSENSOR_SET_SCALE_STATUS            _IOR(GSENSOR_IOCTL_BASE, 5, int)
#define GSENSOR_GET_SCALE_STATUS            _IOR(GSENSOR_IOCTL_BASE, 6, int)
#define GSENSOR_SET_INT_THRESHOLD           _IOR(GSENSOR_IOCTL_BASE, 7, int)
#define GSENSOR_SET_INTERRUPT_MODE          _IOR(GSENSOR_IOCTL_BASE, 9, int)
#define GSENSOR_SET_AXIS_ENABLE             _IOR(GSENSOR_IOCTL_BASE, 11, int)
#define GSENSOR_SET_AXIS_INTERRUPT          _IOR(GSENSOR_IOCTL_BASE, 13, int)
#define GSENSOR_GET_INTTERRUPT_DATA         _IOR(GSENSOR_IOCTL_BASE, 15, int[3])


#undef BIT
#define BIT(x) ( (x) )
// CONTROL REGISTER 1
#define LIS3DH_CTRL_REG1        0x20
#define LIS3DH_ODR_BIT                BIT(4)
#define LIS3DH_ODR_BIT_MASK           (0xf << 4)
#define LIS3DH_3AXIS_EN_MASK      (0x7 << 0)
#define LIS3DH_LPEN         BIT(3)
#define LIS3DH_ZEN          BIT(2)
#define LIS3DH_YEN          BIT(1)
#define LIS3DH_XEN          BIT(0)

//CONTROL REGISTER 2
#define LIS3DH_CTRL_REG2        0x21
#define LIS3DH_HPM            BIT(6)
#define LIS3DH_HPCF         BIT(4)
#define LIS3DH_FDS          BIT(3)
#define LIS3DH_HPCLICK          BIT(2)
#define LIS3DH_HPIS2          BIT(1)
#define LIS3DH_HPIS1          BIT(0)

//CONTROL REGISTER 3
#define LIS3DH_CTRL_REG3        0x22
#define LIS3DH_I1_CLICK       BIT(7)
#define LIS3DH_I1_AOI1          BIT(6)
#define LIS3DH_I1_AOI2                BIT(5)
#define LIS3DH_I1_DRDY1       BIT(4)
#define LIS3DH_I1_DRDY2       BIT(3)
#define LIS3DH_I1_WTM         BIT(2)
#define LIS3DH_I1_ORUN          BIT(1)

//CONTROL REGISTER 6
#define LIS3DH_CTRL_REG6        0x25
#define LIS3DH_I2_CLICK       BIT(7)
#define LIS3DH_I2_INT1          BIT(6)
#define LIS3DH_I2_BOOT              BIT(4)
#define LIS3DH_H_LACTIVE        BIT(1)

//TEMPERATURE CONFIG REGISTER
#define LIS3DH_TEMP_CFG_REG       0x1F
#define LIS3DH_ADC_PD               BIT(7)
#define LIS3DH_TEMP_EN          BIT(6)

//CONTROL REGISTER 4
#define LIS3DH_CTRL_REG4        0x23
#define LIS3DH_BDU          BIT(7)
#define LIS3DH_BLE          BIT(6)
#define LIS3DH_FS         BIT(4)
#define LIS3DH_FS_MASK          (0x3 << 4)
#define LIS3DH_HR         BIT(3)
#define LIS3DH_ST               BIT(1)
#define LIS3DH_SIM          BIT(0)

//CONTROL REGISTER 5
#define LIS3DH_CTRL_REG5        0x24
#define LIS3DH_BOOT                                    BIT(7)
#define LIS3DH_FIFO_EN                                 BIT(6)
#define LIS3DH_LIR_INT1                                BIT(3)
#define LIS3DH_D4D_INT1                                BIT(2)

//REFERENCE/DATA_CAPTURE
#define LIS3DH_REFERENCE_REG                    0x26
#define LIS3DH_REF                      BIT(0)

//STATUS_REG_AXIES
#define LIS3DH_STATUS_REG       0x27
#define LIS3DH_ZYXOR                                   BIT(7)
#define LIS3DH_ZOR                                     BIT(6)
#define LIS3DH_YOR                                     BIT(5)
#define LIS3DH_XOR                                     BIT(4)
#define LIS3DH_ZYXDA                                   BIT(3)
#define LIS3DH_ZDA                                     BIT(2)
#define LIS3DH_YDA                                     BIT(1)
#define LIS3DH_XDA                                     BIT(0)

//STATUS_REG_AUX
#define LIS3DH_STATUS_AUX       0x07

//INTERRUPT 1 CONFIGURATION
#define LIS3DH_INT1_CFG       0x30
#define LIS3DH_ANDOR                                   BIT(7)
#define LIS3DH_INT_6D                                  BIT(6)
#define LIS3DH_ZHIE                                    BIT(5)
#define LIS3DH_ZLIE                                    BIT(4)
#define LIS3DH_YHIE                                    BIT(3)
#define LIS3DH_YLIE                                    BIT(2)
#define LIS3DH_XHIE                                    BIT(1)
#define LIS3DH_XLIE                                    BIT(0)

//FIFO CONTROL REGISTER
#define LIS3DH_FIFO_CTRL_REG                           0x2E
#define LIS3DH_FM                                      BIT(6)
#define LIS3DH_TR                                      BIT(5)
#define LIS3DH_FTH                                     BIT(0)

//CONTROL REG3 bit mask
#define LIS3DH_CLICK_ON_PIN_INT1_ENABLE                0x80
#define LIS3DH_CLICK_ON_PIN_INT1_DISABLE               0x00
#define LIS3DH_I1_INT1_ON_PIN_INT1_ENABLE              0x40
#define LIS3DH_I1_INT1_ON_PIN_INT1_DISABLE             0x00
#define LIS3DH_I1_INT2_ON_PIN_INT1_ENABLE              0x20
#define LIS3DH_I1_INT2_ON_PIN_INT1_DISABLE             0x00
#define LIS3DH_I1_DRDY1_ON_INT1_ENABLE                 0x10
#define LIS3DH_I1_DRDY1_ON_INT1_DISABLE                0x00
#define LIS3DH_I1_DRDY2_ON_INT1_ENABLE                 0x08
#define LIS3DH_I1_DRDY2_ON_INT1_DISABLE                0x00
#define LIS3DH_WTM_ON_INT1_ENABLE                      0x04
#define LIS3DH_WTM_ON_INT1_DISABLE                     0x00
#define LIS3DH_INT1_OVERRUN_ENABLE                     0x02
#define LIS3DH_INT1_OVERRUN_DISABLE                    0x00

//CONTROL REG6 bit mask
#define LIS3DH_CLICK_ON_PIN_INT2_ENABLE                0x80
#define LIS3DH_CLICK_ON_PIN_INT2_DISABLE               0x00
#define LIS3DH_I2_INT1_ON_PIN_INT2_ENABLE              0x40
#define LIS3DH_I2_INT1_ON_PIN_INT2_DISABLE             0x00
#define LIS3DH_I2_INT2_ON_PIN_INT2_ENABLE              0x20
#define LIS3DH_I2_INT2_ON_PIN_INT2_DISABLE             0x00
#define LIS3DH_I2_BOOT_ON_INT2_ENABLE                  0x10
#define LIS3DH_I2_BOOT_ON_INT2_DISABLE                 0x00
#define LIS3DH_INT_ACTIVE_HIGH                         0x00
#define LIS3DH_INT_ACTIVE_LOW                          0x02

//INT1_CFG bit mask
#define LIS3DH_INT1_AND                                0x80
#define LIS3DH_INT1_OR                                 0x00
#define LIS3DH_INT1_ZHIE_ENABLE                        0x20
#define LIS3DH_INT1_ZHIE_DISABLE                       0x00
#define LIS3DH_INT1_ZLIE_ENABLE                        0x10
#define LIS3DH_INT1_ZLIE_DISABLE                       0x00
#define LIS3DH_INT1_YHIE_ENABLE                        0x08
#define LIS3DH_INT1_YHIE_DISABLE                       0x00
#define LIS3DH_INT1_YLIE_ENABLE                        0x04
#define LIS3DH_INT1_YLIE_DISABLE                       0x00
#define LIS3DH_INT1_XHIE_ENABLE                        0x02
#define LIS3DH_INT1_XHIE_DISABLE                       0x00
#define LIS3DH_INT1_XLIE_ENABLE                        0x01
#define LIS3DH_INT1_XLIE_DISABLE                       0x00

//INT1_SRC bit mask
#define LIS3DH_INT1_SRC_IA                             0x40
#define LIS3DH_INT1_SRC_ZH                             0x20
#define LIS3DH_INT1_SRC_ZL                             0x10
#define LIS3DH_INT1_SRC_YH                             0x08
#define LIS3DH_INT1_SRC_YL                             0x04
#define LIS3DH_INT1_SRC_XH                             0x02
#define LIS3DH_INT1_SRC_XL                             0x01

//INT1 REGISTERS
#define LIS3DH_INT1_THS                                0x32
#define LIS3DH_INT1_DURATION                           0x33

//INTERRUPT 1 SOURCE REGISTER
#define LIS3DH_INT1_SRC       0x31

//FIFO Source Register bit Mask
#define LIS3DH_FIFO_SRC_WTM                            0x80
#define LIS3DH_FIFO_SRC_OVRUN                          0x40
#define LIS3DH_FIFO_SRC_EMPTY                          0x20

//INTERRUPT CLICK REGISTER
#define LIS3DH_CLICK_CFG        0x38
//INTERRUPT CLICK CONFIGURATION bit mask
#define LIS3DH_ZD_ENABLE                               0x20
#define LIS3DH_ZD_DISABLE                              0x00
#define LIS3DH_ZS_ENABLE                               0x10
#define LIS3DH_ZS_DISABLE                              0x00
#define LIS3DH_YD_ENABLE                               0x08
#define LIS3DH_YD_DISABLE                              0x00
#define LIS3DH_YS_ENABLE                               0x04
#define LIS3DH_YS_DISABLE                              0x00
#define LIS3DH_XD_ENABLE                               0x02
#define LIS3DH_XD_DISABLE                              0x00
#define LIS3DH_XS_ENABLE                               0x01
#define LIS3DH_XS_DISABLE                              0x00

//INTERRUPT CLICK SOURCE REGISTER
#define LIS3DH_CLICK_SRC                               0x39
//INTERRUPT CLICK SOURCE REGISTER bit mask
#define LIS3DH_IA                                      0x40
#define LIS3DH_DCLICK                                  0x20
#define LIS3DH_SCLICK                                  0x10
#define LIS3DH_CLICK_SIGN                              0x08
#define LIS3DH_CLICK_Z                                 0x04
#define LIS3DH_CLICK_Y                                 0x02
#define LIS3DH_CLICK_X                                 0x01

//Click-click Register
#define LIS3DH_CLICK_THS                               0x3A
#define LIS3DH_TIME_LIMIT                              0x3B
#define LIS3DH_TIME_LATENCY                            0x3C
#define LIS3DH_TIME_WINDOW                             0x3D
#define LIS3DH_ACT_THS                                 0x3E
#define LIS3DH_ACT_DUR                                 0x3F

//OUTPUT REGISTER
#define LIS3DH_OUT_X_L          0x28
#define LIS3DH_OUT_X_H          0x29
#define LIS3DH_OUT_Y_L          0x2A
#define LIS3DH_OUT_Y_H          0x2B
#define LIS3DH_OUT_Z_L          0x2C
#define LIS3DH_OUT_Z_H          0x2D

//AUX REGISTER
#define LIS3DH_OUT_1_L          0x08
#define LIS3DH_OUT_1_H          0x09
#define LIS3DH_OUT_2_L          0x0A
#define LIS3DH_OUT_2_H          0x0B
#define LIS3DH_OUT_3_L          0x0C
#define LIS3DH_OUT_3_H          0x0D

enum {
  LIS3DH = 0,
};

typedef enum {
  LIS3DH_ODR_1Hz                =   0x01,
  LIS3DH_ODR_10Hz                   =   0x02,
  LIS3DH_ODR_25Hz               =   0x03,
  LIS3DH_ODR_50Hz               =   0x04,
  LIS3DH_ODR_100Hz                =   0x05,
  LIS3DH_ODR_200Hz                =   0x06,
  LIS3DH_ODR_400Hz                =   0x07,
  LIS3DH_ODR_1620Hz_LP              =   0x08,
  LIS3DH_ODR_1344Hz_NP_5367HZ_LP        =   0x09
} LIS3DH_ODR_t;

typedef enum {
  LIS3DH_POWER_DOWN                         =   0x00,
  LIS3DH_LOW_POWER                      =   0x01,
  LIS3DH_NORMAL                         =   0x02
} LIS3DH_Mode_t;

typedef enum {
  LIS3DH_FULLSCALE_2                   =               0x00,
  LIS3DH_FULLSCALE_4                   =               0x01,
  LIS3DH_FULLSCALE_8                   =               0x02,
  LIS3DH_FULLSCALE_16                  =               0x03
} LIS3DH_Fullscale_t;

typedef enum {
  LIS3DH_X_ENABLE                      =               0x01,
  LIS3DH_X_DISABLE                     =               0x10,
  LIS3DH_Y_ENABLE                      =               0x02,
  LIS3DH_Y_DISABLE                     =               0x20,
  LIS3DH_Z_ENABLE                      =               0x04,
  LIS3DH_Z_DISABLE                     =               0x40,
  LIS3DH_XYZ_ENABLE                    =               LIS3DH_X_ENABLE | LIS3DH_Y_ENABLE | LIS3DH_Z_ENABLE,
} LIS3DH_AXISenable_t;


typedef enum {
  INT_MODE_OR_COMB                  =                 0x0, // 中断事件的OR组合
  INT_MODE_6D_MOVEMENT_REC          =                 0x1, // 6D 运动识别
  INT_MODE_AND_COMB                 =                 0x2, // 中断事件的AND组合
  INT_MODE_6D_POSITION_REC          =                 0x3, // 6D位置识别
} LIS3DH_INT_MODE_TYPE_t;

typedef struct _axis_raw_data {
  short axis_x;
  short axis_y;
  short axis_z;
} axis_raw_data_t;

typedef struct _axis_data_mg {
  int x;
  int y;
  int z;
} axis_data_mg_t;

struct lis3dh_pdata {
  unsigned int              gsensor_int_gpio;
};


struct lis3dh_priv {

  void                      *bus_priv;
  int                       int_data_rdy;
  int                       irq;
  unsigned int              cur_scale_range;

  struct lis3dh_pdata       pdata;
  struct fasync_struct      *lis3dh_async;
  struct regmap             *regmap;
  struct miscdevice         miscdev;

#if (LIS3DH_USE_INPUT_SUBSYS == 1)
  struct input_dev          *idev;
#endif

#if (LIS3DH_USE_ISR_TASKLET == 1)
  struct tasklet_struct     lis3dh_tasklet;
#endif
  struct delayed_work           work;

  struct pinctrl            *int_pinctrl;
  struct pinctrl_state      *int_pinctrl_sta;

  axis_data_mg_t           int_axis_data;

  int (*update_bits)(struct lis3dh_priv *lis3dh, unsigned int reg, unsigned int mask, unsigned int new_val);
  int (*write)(struct lis3dh_priv *lis3dh, unsigned int reg, unsigned int val);
  unsigned int (*read)(struct lis3dh_priv *lis3dh, unsigned int reg);
  int (*read_raw_data)(struct lis3dh_priv *lis3dh, axis_raw_data_t *raw);

};

int lis3dh_init_device(struct lis3dh_priv *lis3dh);
int lis3dh_remove_device(struct lis3dh_priv *lis3dh);
int lis3dh_set_power_mode(struct lis3dh_priv *lis3dh, LIS3DH_Mode_t mode);

#endif //__YM_LIS3HD_ACC_H__
