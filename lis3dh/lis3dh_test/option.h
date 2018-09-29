/*
 * =====================================================================================
 *
 *       Filename:  option.h
 *
 *    Description:  Head file of option function
 *
 *        Version:  1.0
 *        Created:  05/28/18 13:53:02
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Lin Hui (Link), linhui.568@163.com
 *        Company:  www.ecpark.cn
 *
 * =====================================================================================
 */

#ifndef __CZH_OPTION_H__
#define __CZH_OPTION_H__


#define ACC_CONFIG_FILE_NAME                "/ecpark/lis3dh_acc.conf"

#define ACC_CONFIG_TAG_NAME_SEN_LEV          "SEN_LEV"
#define ACC_CONFIG_TAG_NAME_SCL_RNG          "SCALE_RANGE"
#define ACC_CONFIG_TAG_NAME_VEH_MODE         "VEH_MODE"

#define dbg_err(format, ...) \
                printf("[ERROR][\033[31m%s\033[0m"":%d] >> "format"\n", __FUNCTION__, __LINE__, ##__VA_ARGS__)

int readConfigTag(char *filePath, char *tagName, char *tagValue);
int modifyConfigTag(char *filePath, char *tagName, char *tagValue);
int deleteConfigTag(char *filePath, char *tagName);
int czh_file_is_exists(const char *filename);

#endif

