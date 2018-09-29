/*
 * =====================================================================================
 *
 *       Filename:  option.c
 *
 *    Description:  system option configuration
 *
 *        Version:  1.0
 *        Created:  05/28/18 11:04:00
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Lin Hui (Link), linhui.568@163.com
 *        Company:  www.ecpark.cn
 *
 * =====================================================================================
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>


#include <unistd.h>
#include <pthread.h>
#include <fcntl.h>
#include <errno.h>

#include <execinfo.h>
#include <signal.h>

#include "option.h"


/*
 * Read tag value from config.ini file
 * Params:
 *  filePath:   config file path
 *  tagName:    name of tag
 *  tagValue:   value of tag
 * return:
 *  error: -1, success: 0
 * Note:
 *  "=" befor and after no content, ignore empty line.
 */
int readConfigTag(char *filePath, char *tagName, char *tagValue)
{
    char linebuf[256];
    char line_name[40];
    char tempBuf[256];
    char *config_sign = "=";
    char *leave_line;
    FILE *file;

    file = fopen(filePath, "r");
    if (file == NULL) {
        dbg_err("OPTION: open config file error");
        return -1;
    }

    fseek(file, 0, SEEK_SET);
    while (fgets(linebuf, 256, file) != NULL)
    {
        if ((strlen(linebuf) < 3) || (linebuf[0] == ';')) {   //judge empty line; skip ';' begin line.
            continue;
        }

        if (linebuf[strlen(linebuf) - 1] == '\n') {   //remove the last byte '\n'
            memset(tempBuf, 0, sizeof(tempBuf));
            strncpy(tempBuf, linebuf, strlen(linebuf) - 1);
            memset(linebuf, 0, sizeof(linebuf));
            strcpy(linebuf, tempBuf);
        }

        memset(line_name, 0, sizeof(line_name));
        leave_line = strstr(linebuf, config_sign);
        if ((leave_line == NULL)) {                   //skip line without '='
            continue;
        }

        int leave_num = leave_line - linebuf;
        strncpy(line_name, linebuf, leave_num);
        if (strcmp(line_name, tagName) == 0)
        {
            strncpy(tagValue, linebuf + (leave_num + 1), strlen(linebuf) - leave_num - 1);
            break;
        }
        if (fgetc(file) == EOF) {
            break;
        }

        fseek(file, -1, SEEK_CUR);
        memset(linebuf, 0, sizeof(linebuf));
    }
    fclose(file);

    return 0;
}

/*
 * Modify tag value of config.ini file, or add it if not exist
 * Params:
 *  filePath:   config file path
 *  tagName:    name of tag
 *  tagValue:   value of tag
 * return:
 *  error: -1, success: 0
 * Note:
 *  "=" befor and after no content, ignore empty line.
 */
int modifyConfigTag(char *filePath, char *tagName, char *tagValue)
{

    char linebuf[256];
    char line_name[40];
    char *config_sign = "=";
    char *leave_line;
    int  alter_sign = 0;
    char sum_buf[4096];

    FILE *file;
    file = fopen(filePath, "r+");
    if (file == NULL) {
        dbg_err("OPTION: open config file error");
        return -1;
    }

    fseek(file, 0, SEEK_END);
    int valueLen = strlen(tagValue);
    valueLen = valueLen + 5;

    memset(sum_buf, 0, sizeof(sum_buf));
    fseek(file, 0, SEEK_SET);
    while (fgets(linebuf, 256, file) != NULL)
    {
        if ((strlen(linebuf) < 3) || (linebuf[0] == ';')) {   //judge empty line; skip ';' begin line.
            strcat(sum_buf, linebuf);
            continue;
        }

        leave_line = NULL;
        leave_line = strstr(linebuf, config_sign); /* skip line without '=' */
        if (leave_line == NULL) {
            strcat(sum_buf, linebuf);
            continue;
        }

        int leave_num = leave_line - linebuf;
        memset(line_name, 0, sizeof(line_name));
        strncpy(line_name, linebuf, leave_num);
        if (strcmp(line_name, tagName) == 0) {
            strcat(sum_buf, tagName);
            strcat(sum_buf, "=");
            strcat(sum_buf, tagValue);
            strcat(sum_buf, "\n");
            alter_sign = 1;
        } else {
            strcat(sum_buf, linebuf);
        }

        if (fgetc(file) == EOF) {
            break;
        }

        fseek(file, -1, SEEK_CUR);
        memset(linebuf, 0, sizeof(linebuf));
    }
    if (alter_sign == 0)
    {
        strcat(sum_buf, tagName);
        strcat(sum_buf, "=");
        strcat(sum_buf, tagValue);
        strcat(sum_buf, "\n");
    }
    //dbg_debug("OPTION: =============>\n%s<=============", sum_buf);
    remove(filePath);
    fclose(file);

    FILE *fp;
    fp = fopen(filePath, "w+");
    if (fp == NULL) {
        dbg_err("OPEION: open config file error");
        return -1;
    }
    fseek(fp,0,SEEK_SET);
    fputs(sum_buf, fp);

    fclose(fp);
    return 0;
}


/*
 * Delete tag value of config.ini file
 * Params:
 *  filePath:   config file path
 *  tagName:    name of tag
 * return:
 *  error: -1, success: 0
 * Note:
 */
int deleteConfigTag(char *filePath, char *tagName)
{

    char linebuf[256];
    char line_name[40];
    char *config_sign = "=";
    char *leave_line;
    char sum_buf[4096];

    FILE *file;
    file = fopen(filePath, "r+");
    if (file == NULL) {
        dbg_err("OPTION: open config file error");
        return 0;
    }

    fseek(file, 0, SEEK_END);
    memset(sum_buf, 0, sizeof(sum_buf));
    fseek(file, 0, SEEK_SET);
    while (fgets(linebuf, 256, file) != NULL)
    {
        if ((strlen(linebuf) < 3) || (linebuf[0] == ';')) {   //judge empty line; skip ';' begin line.
            strcat(sum_buf,linebuf);
            continue;
        }

        leave_line = NULL;
        leave_line = strstr(linebuf, config_sign);
        if (leave_line == NULL) {
            strcat(sum_buf, linebuf);
            continue;
        }

        int leave_num = leave_line - linebuf;
        memset(line_name, 0, sizeof(line_name));
        strncpy(line_name, linebuf, leave_num);
        if (strcmp(line_name, tagName) == 0) {
            /* skip tagName */
        } else {
            strcat(sum_buf, linebuf);
        }

        if (fgetc(file) == EOF) {
            break;
        }

        fseek(file, -1, SEEK_CUR);
        memset(linebuf, 0, sizeof(linebuf));
    }

    //dbg_debug("OPTION: =============>\n%s<=============", sum_buf);
    remove(filePath);
    fclose(file);

    FILE *fp;
    fp = fopen(filePath, "w+");
    if(fp == NULL)
    {
        dbg_err("OPTION: open config file error");
        return 2;
    }
    fseek(fp,0,SEEK_SET);
    fputs(sum_buf, fp);

    fclose(fp);

    return 0;
}

int czh_file_is_exists(const char *filename)
{
   return (access(filename, 0) == 0);
}

