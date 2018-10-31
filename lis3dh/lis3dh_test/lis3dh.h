#ifndef __YM_LIS3HD_ACC_H__
#define __YM_LIS3HD_ACC_H__


#define GSENSOR_FIFO_NUM (32)
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
#define GSENSOR_GET_INTTERRUPT_DATA         _IOR(GSENSOR_IOCTL_BASE, 15, int[3*GSENSOR_FIFO_NUM])
#define GSENSOR_GET_RUNNING_STATUS          _IOR(GSENSOR_IOCTL_BASE, 16, unsigned int)

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

typedef enum {
  LIS3DH_ODR_1Hz		        		=		0x01,
  LIS3DH_ODR_10Hz               		=		0x02,
  LIS3DH_ODR_25Hz		        		=		0x03,
  LIS3DH_ODR_50Hz		        		=		0x04,
  LIS3DH_ODR_100Hz		        		=		0x05,
  LIS3DH_ODR_200Hz		        		=		0x06,
  LIS3DH_ODR_400Hz		        		=		0x07,
  LIS3DH_ODR_1620Hz_LP		        	=		0x08,
  LIS3DH_ODR_1344Hz_NP_5367HZ_LP       	=		0x09
} LIS3DH_ODR_t;

typedef enum {
  LIS3DH_POWER_DOWN                    =		0x00,
  LIS3DH_LOW_POWER 			=		0x01,
  LIS3DH_NORMAL			=		0x02
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
  INT_MODE_OR_COMB = 0x0,      // 中断事件的OR组合
  INT_MODE_6D_MOVEMENT_REC = 0x1, //6D 运动识别
  INT_MODE_AND_COMB = 0x2,      // 中断事件的AND组合
  INT_MODE_6D_POSITION_REC = 0x3, //6D位置识别
} LIS3DH_INT_MODE_TYPE_t;

// Gsensor 灵敏度
typedef enum {
  GSENSOR_SEN_LEV_LOW = 0x0,
  GSENSOR_SEN_LEV_MID = 0x1,
  GSENSOR_SEN_LEV_HIG = 0x2,
} gsensor_sensitivity_lev_t;


typedef struct _axis_data_mg {
  short x;
  short y;
  short z;
} axis_data_mg_t;


#endif //__YM_LIS3HD_ACC_H__
