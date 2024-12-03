/********************************************************************************
 * @File name: ini.h
 * @Author: 石海新
 * @Version: 1.0
 * @Date: 2023.5.17
 * @Description: INI配置文件读写头文件
 ********************************************************************************/
#ifndef USER_INI_H
#define USER_INI_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

#include "sqlite3.h"
#include "track_common.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CARDB_DB_FILE       "/opt/carDB.db"
#define ADJUST_DB_FILE      "/home/root/pointdb.db"

bool JudgeBool(char *buff);

int ReadParams(sqlite3 *dtb, AdjustParams *adjustparams, SectionAndCoilRegionDef *sections, int *section_num);
int UpdateParams(sqlite3 *dtb, char *key, char *value);
void writeparams(char *key, char *value, AdjustParams *adjustparams);

int UpdateSections(sqlite3 *dtb, int id, char **section_value);
int InsertSections(sqlite3 *dtb, char **section_value);
int DeleteSections(sqlite3 *dtb, int id);

int UpdateCoils(sqlite3 *dtb, int id, int laneNum, char **coil_value);
int InsertCoils(sqlite3 *dtb, int id, char **coil_value);
int DeleteCoils(sqlite3 *dtb, int id, int laneNum);
int UpdateRoi(sqlite3 *dtb, char *name, char **roi_value);

int user_InsertPolygonRoi_func(sqlite3 *dtb, int id, char **coil_value);

#ifdef __cplusplus
}
#endif

#endif
