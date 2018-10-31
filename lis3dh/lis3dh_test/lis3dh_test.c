#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <pthread.h>
#include <sys/ioctl.h>
#include <poll.h>
#include <signal.h>
#include <sys/types.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdlib.h>
#include <sys/time.h>

#include "lis3dh.h"
#include "option.h"

#define SCALE_RANGE_16BIT (65536)
#define GSENSOR_DEV ("/dev/lis3dh_acc")

#define ARRAY_SIZE(a) (sizeof(a) / sizeof(a[0]))
#define TIME_SHAKE_THRESHOLD_US (1000 * 1000)

typedef struct _gsensor_ctrl {
	int acc_fd;
	int scale_range;
	int sensitivity_level;
	int status;

	int vehicle_mode;
	struct timeval last_tv;
	axis_data_mg_t dat_mg[GSENSOR_FIFO_NUM];
} gsensor_ctrl_t;

gsensor_ctrl_t gsnsr_ctrl;


static int sen_to_ths_map[LIS3DH_FULLSCALE_16 + 1][GSENSOR_SEN_LEV_HIG + 1] = {
	[LIS3DH_FULLSCALE_2] = { [GSENSOR_SEN_LEV_LOW] = 0x0a, [GSENSOR_SEN_LEV_MID] = 0x10, [GSENSOR_SEN_LEV_HIG] = 0x16, },
	[LIS3DH_FULLSCALE_4] = { [GSENSOR_SEN_LEV_LOW] = 0x04, [GSENSOR_SEN_LEV_MID] = 0x07, [GSENSOR_SEN_LEV_HIG] = 0x0a, },
	[LIS3DH_FULLSCALE_8] = { [GSENSOR_SEN_LEV_LOW] = 0x02, [GSENSOR_SEN_LEV_MID] = 0x04, [GSENSOR_SEN_LEV_HIG] = 0x06, },
	[LIS3DH_FULLSCALE_16] = { [GSENSOR_SEN_LEV_LOW] = 0x01, [GSENSOR_SEN_LEV_MID] = 0x2, [GSENSOR_SEN_LEV_HIG] = 0x03, },
};

static const char *gsensor_scl_rng_desp[] = {
	"02G",
	"04G",
	"08G",
	"16G",
};

static const char *gsensor_sensitivity_desp[] = {
	"LOW",
	"MID",
	"HIG",
};

static const char *vehicle_desp[] = {
	"RUNNING",
	"STOP",
};



static int raw_mg_range_map[4] = {
	[LIS3DH_FULLSCALE_2] = 4000,			//+- 2G, 4000mg
	[LIS3DH_FULLSCALE_4] = 8000,			//+- 4G, 8000mg
	[LIS3DH_FULLSCALE_8] = 16000,			//+- 8G, 16000mg
	[LIS3DH_FULLSCALE_16] = 32000,			//+- 16G, 32000mg
};

void gsensor_convert_raw_data_to_mg(axis_data_mg_t *dat_mg)
{
	dat_mg->x = (short)(dat_mg->x * raw_mg_range_map[gsnsr_ctrl.scale_range] / SCALE_RANGE_16BIT);
	dat_mg->y = (short)(dat_mg->y * raw_mg_range_map[gsnsr_ctrl.scale_range] / SCALE_RANGE_16BIT);
	dat_mg->z = (short)(dat_mg->z * raw_mg_range_map[gsnsr_ctrl.scale_range] / SCALE_RANGE_16BIT);
}


static int interrupt_has_shake(void)
{
	int ret = 0;
	long long int time_interval = 0;
	struct timeval now_tv;
	gettimeofday(&now_tv, NULL);

	time_interval = ( now_tv.tv_sec - gsnsr_ctrl.last_tv.tv_sec ) * 1000000 +
								( now_tv.tv_usec - gsnsr_ctrl.last_tv.tv_usec );

	printf("time_interval = %lld\n", time_interval);

	if(time_interval < TIME_SHAKE_THRESHOLD_US)
	{
		ret = 1;
	}
	gettimeofday(&gsnsr_ctrl.last_tv, NULL);

	return ret;
}

void lis3dh_sig_hdl(int sig)
{
	int ret = 0;
	int i = 0;
	axis_data_mg_t _dat_mg[GSENSOR_FIFO_NUM];

	if (SIGIO == sig)
	{
		if (interrupt_has_shake())
		{
			printf("interrupt_has_shake!!!\n\n\n\n\n\n");
			return;
		}

		// 在这里做行车与安防业务逻辑处理
		ret = ioctl(gsnsr_ctrl.acc_fd, GSENSOR_GET_INTTERRUPT_DATA, _dat_mg);
		if (ret)
		{
			printf("%s:%d ioctl failed.\n", __func__, __LINE__);
			return;
		}
		for (i = 0; i < GSENSOR_FIFO_NUM; ++i)
		{
			gsensor_convert_raw_data_to_mg(&_dat_mg[i]);
			printf("[INTERRUPT][%d] ACCL_X: %08d mg, ACCL_Y: %08d mg, ACCL_Z: %08d mg\n", i, _dat_mg[i].x,
				_dat_mg[i].y, _dat_mg[i].z);
		}

		if (gsnsr_ctrl.vehicle_mode)
		{
			// 行车中
			printf("the vehicle running, nothing to do!\n");
		}
		else
		{
			// 停车中
			printf("the vehicle stop, something hint the vehicle!\n");
		}
	}
	else if(SIGINT == sig)
	{
		printf("[CTRL + C] be pressed , abort by user, program will exit!\n");
		close(gsnsr_ctrl.acc_fd);
		exit(0);
	}
}

inline static int convert_sensitivity_level_2_int_ths(void)
{
	return sen_to_ths_map[gsnsr_ctrl.scale_range][gsnsr_ctrl.sensitivity_level];
}

void* change_sys_config_thread(void* arg)
{
	char buff[64] = {0};
	int ret = 0;
	int value = 0;
	int i = 0;

	//debug
	return NULL;

	while(1)
	{
		readConfigTag(ACC_CONFIG_FILE_NAME, ACC_CONFIG_TAG_NAME_VEH_MODE, buff);
		if ( 0 == strncmp(buff, vehicle_desp[0], strlen(vehicle_desp[1])))
		{
			gsnsr_ctrl.vehicle_mode = 1;
		}
		else
		{
			gsnsr_ctrl.vehicle_mode = 0;
		}

		memset(buff, 0, sizeof(buff));
		readConfigTag(ACC_CONFIG_FILE_NAME, ACC_CONFIG_TAG_NAME_SEN_LEV, buff);
		for (i = 0; i < ARRAY_SIZE(gsensor_sensitivity_desp); ++i)
		{
			if ( 0 == strncmp(buff, gsensor_sensitivity_desp[i], strlen(gsensor_sensitivity_desp[i])))
			{
				break;
			}
		}
		if (i > ARRAY_SIZE(gsensor_sensitivity_desp) - 1)
		{
			sleep(1);
			continue;
		}

		gsnsr_ctrl.sensitivity_level = i;

		memset(buff, 0, sizeof(buff));
		readConfigTag(ACC_CONFIG_FILE_NAME, ACC_CONFIG_TAG_NAME_SCL_RNG, buff);
		for (i = 0; i < ARRAY_SIZE(gsensor_scl_rng_desp); ++i)
		{
			if ( 0 == strncmp(buff, gsensor_scl_rng_desp[i], strlen(gsensor_scl_rng_desp[i])))
			{
				break;
			}
		}
		if (i > ARRAY_SIZE(gsensor_scl_rng_desp) - 1)
		{
			sleep(1);
			continue;
		}
		gsnsr_ctrl.scale_range = i;

		value = gsnsr_ctrl.scale_range;
		ret = ioctl(gsnsr_ctrl.acc_fd, GSENSOR_SET_SCALE_STATUS, &value);
		if (ret)
		{
			printf("%s:%d ioctl failed.\n", __func__, __LINE__);
			sleep(1);
			continue;
		}

		value = convert_sensitivity_level_2_int_ths();
		ret = ioctl(gsnsr_ctrl.acc_fd, GSENSOR_SET_INT_THRESHOLD, &value);
		if (ret)
		{
			printf("%s:%d ioctl failed.\n", __func__, __LINE__);
		}
		printf("sen_level = [%s], scale_range = [%s], int_ths = %#x\n", gsensor_sensitivity_desp[gsnsr_ctrl.sensitivity_level],
															gsensor_scl_rng_desp[gsnsr_ctrl.scale_range],
															value);

		sleep(10);
	}

	return NULL;
}

int device_init_work(void)
{
	int ret = 0;
	int value = 0;
	char buff[36] = {0};

	ret = ioctl(gsnsr_ctrl.acc_fd, GSENSOR_GET_MODEL_NAME, buff);
	if (ret)
	{
		printf("%s:%d ioctl failed.\n", __func__, __LINE__);
		return -1;
	}
	printf("MODEL Name: %s\n", buff);

	// 设置电源找作模式
	value = LIS3DH_NORMAL;
	ret = ioctl(gsnsr_ctrl.acc_fd, GSENSOR_SET_POWER_STATUS, &value);
	if (ret)
	{
		printf("%s:%d ioctl failed.\n", __func__, __LINE__);
		return -1;
	}

	// 设置数据采样速率
	value = LIS3DH_ODR_200Hz;
	ret = ioctl(gsnsr_ctrl.acc_fd, GSENSOR_SET_ODR_STATUS, &value);
	if (ret)
	{
		printf("%s:%d ioctl failed.\n", __func__, __LINE__);
		return -1;
	}

	// 设置加速度计的量程
	value = LIS3DH_FULLSCALE_2;
	ret = ioctl(gsnsr_ctrl.acc_fd, GSENSOR_SET_SCALE_STATUS, &value);
	if (ret)
	{
		printf("%s:%d ioctl failed.\n", __func__, __LINE__);
		return -1;
	}

	// 获取设置的量程
	ret = ioctl(gsnsr_ctrl.acc_fd, GSENSOR_GET_SCALE_STATUS, &gsnsr_ctrl.scale_range);
	if (ret)
	{
		printf("%s:%d ioctl failed.\n", __func__, __LINE__);
		return -1;
	}
	printf("current scale_range: %u\n", gsnsr_ctrl.scale_range);

	// 设置加速度计中断阀值
	value = 0x7f;
	ret = ioctl(gsnsr_ctrl.acc_fd, GSENSOR_SET_INT_THRESHOLD, &value);
	if (ret)
	{
		printf("%s:%d ioctl failed.\n", __func__, __LINE__);
		return -1;
	}

	// 设置加速度计中断模式
	value = INT_MODE_OR_COMB;
	ret = ioctl(gsnsr_ctrl.acc_fd, GSENSOR_SET_INTERRUPT_MODE, &value);
	if (ret)
	{
		printf("%s:%d ioctl failed.\n", __func__, __LINE__);
		return -1;
	}

	// 使能加速度计XYZ三轴采集数据,可以组合
	value = LIS3DH_XYZ_ENABLE;
	ret = ioctl(gsnsr_ctrl.acc_fd, GSENSOR_SET_AXIS_ENABLE, &value);
	if (ret)
	{
		printf("%s:%d ioctl failed.\n", __func__, __LINE__);
		return -1;
	}

	// 使能加速度计三轴高8位寄存器中断
	value = LIS3DH_INT1_YHIE_ENABLE | LIS3DH_INT1_XHIE_ENABLE;
	ret = ioctl(gsnsr_ctrl.acc_fd, GSENSOR_SET_AXIS_INTERRUPT, &value);
	if (ret)
	{
		printf("%s:%d ioctl failed.\n", __func__, __LINE__);
		return -1;
	}

	printf("register configuare done!\n");

	return 0;
}

int main(int argc, char** argv)
{
	unsigned int cnt = 0;
	int ret = 0;
	int fd = 0;
	int flags = 0;
	int i = 0;
	char buffer[4096] = {0};

	pthread_t thread_id;
	pthread_attr_t tattr;

	signal(SIGIO, lis3dh_sig_hdl);
	signal(SIGINT, lis3dh_sig_hdl);

	pthread_attr_init (&tattr);
    pthread_attr_setdetachstate(&tattr, PTHREAD_CREATE_DETACHED);

	gsnsr_ctrl.acc_fd = open(GSENSOR_DEV, O_RDWR);
	if (gsnsr_ctrl.acc_fd < 0)
	{
		printf("open %s dev failed.\n", GSENSOR_DEV);
		return -1;
	}

	fcntl(gsnsr_ctrl.acc_fd, F_SETOWN, getpid());
	flags = fcntl(gsnsr_ctrl.acc_fd, F_GETFL);
	fcntl(gsnsr_ctrl.acc_fd, F_SETFL, flags | FASYNC);

	if (device_init_work())
	{
		return -1;
	}
	gettimeofday(&gsnsr_ctrl.last_tv, NULL);

	if (pthread_create(&thread_id, &tattr, change_sys_config_thread, NULL) != 0)
	{
        dbg_err("Create network data dispatch thread error.");
        return -1;
    }

#if 0

    fd = open("/sys/devices/78b6000.i2c/i2c-2/2-0018/lis3dh_reg", O_RDWR | O_NONBLOCK | O_NOCTTY , S_IRWXU | S_IRWXG);
    read(fd, buffer, 4096);
    printf("%s\n", buffer);
    return 0;
#endif

	while(1)
	{
#if 1
		ret = read(gsnsr_ctrl.acc_fd, gsnsr_ctrl.dat_mg, sizeof(gsnsr_ctrl.dat_mg));
		if (ret < 0)
		{
			printf("read failed.\n");
		}

		for (i = 0; i < GSENSOR_FIFO_NUM; ++i)
		{
			gsensor_convert_raw_data_to_mg(&gsnsr_ctrl.dat_mg[i]);
			printf("[%08u, num: %d] ACCL_X: %04d mg, ACCL_Y: %04d mg, ACCL_Z: %04d mg\n", cnt++, i,
											gsnsr_ctrl.dat_mg[i].x, gsnsr_ctrl.dat_mg[i].y, gsnsr_ctrl.dat_mg[i].z);
		}
#else
		ret = ioctl(gsnsr_ctrl.acc_fd, GSENSOR_GET_RUNNING_STATUS, &gsnsr_ctrl.status);
		if (ret)
		{
			printf("%s:%d ioctl failed.\n", __func__, __LINE__);
		}
		printf("current gsensor status %d\n", gsnsr_ctrl.status);
#endif
		usleep(5 * 1000);
	}
	printf("==========end==========\n");

	return ret;
}

