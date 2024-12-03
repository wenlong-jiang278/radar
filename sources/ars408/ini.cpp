/********************************************************************************
* @File name: ini.c
* @Author: 石海新
* @Version: 1.0
* @Date: 2023.5.17
* @Description: INI配置文件读写功能文件
********************************************************************************/
#include "ini.h"
#include <spdlog/spdlog.h>

extern int polyRoi_num;
extern UserPolyRegion polyRoi[];
extern UserOffsetParam gOffsetParam;

/********************************************************
* Function name ：ReadParams
* Description   ：实现对数据库的读操作
* Parameter     ：
* @dtb                   数据库指针
* @err_msg               错误信息指针
* @adjustparams          动态参数指针
* @sections              线圈和断面参数指针
* @section_num           断面数量指针
* Return                 状态信息
**********************************************************/
int ReadParams(sqlite3 *dtb, AdjustParams *adjustparams, SectionAndCoilRegionDef *sections, int *section_num)
{
    int ret = 0;
    int ret1 = 0;
    char **db_result;
    char **db_result1;
    int nrow, ncolumn;
    int nrow1, ncolumn1;
    char *err_msg = NULL;
    char sql_find_roi[512] = { 0 };
    char sql_find_coil[512] = { 0 };
    char sql_find_params[512] = { 0 };
    char sql_find_section[512] = { 0 };
    char sql_find_PolygonRoi[512] = { 0 };

    if ((adjustparams == NULL) || (sections == NULL) || (section_num == NULL)) {
        spdlog::error("ReadParams failed, invalid parameter");
        return -1;
    }

    sprintf(sql_find_params, "select * from configuration");
    sprintf(sql_find_roi, "select * from Roi");
    sprintf(sql_find_section, "select * from section");

    /* 读取参数 */
    ret = sqlite3_exec(dtb, "begin transaction", 0, 0, &err_msg);
    if (ret != SQLITE_OK) {
        sqlite3_free(err_msg);
    }

    ret = sqlite3_get_table(dtb, sql_find_params, &db_result, &nrow, &ncolumn, &err_msg);
    if (ret != SQLITE_OK) {
        sqlite3_close(dtb);
        sqlite3_free(err_msg);
        spdlog::error("get table sql:[{0}] failed", sql_find_params);
        return -1;
    }

    ret = sqlite3_exec(dtb, "commit transaction", 0, 0, &err_msg);
    if (ret != SQLITE_OK) {
        sqlite3_free(err_msg);
    }

    if (db_result != NULL) {
        for (int i = 1; i <= nrow; i++) {
            writeparams(db_result[i * ncolumn + 1], db_result[i * ncolumn + 2], adjustparams);
        }

        sqlite3_free_table(db_result);
    }

    /* 读取Roi */
    ret = sqlite3_exec(dtb, "begin transaction", 0, 0, &err_msg);
    if (ret != SQLITE_OK) {
        sqlite3_free(err_msg);
    }

    ret = sqlite3_get_table(dtb, sql_find_roi, &db_result, &nrow, &ncolumn, &err_msg);
    if (ret != SQLITE_OK) {
        sqlite3_close(dtb);
        sqlite3_free(err_msg);
        spdlog::error("get table sql:[{0}] failed", sql_find_roi);
        return -1;
    }

    ret = sqlite3_exec(dtb, "commit transaction", 0, 0, &err_msg);
    if (ret != SQLITE_OK) {
        sqlite3_free(err_msg);
    }

    if (db_result != NULL) {
        int roi_sum = 0;
        for (int i = 1; i <= nrow; i++) {
            ++roi_sum;

            strcpy(adjustparams->AdjustParams_ROI.PositionofROI[i - 1].name, db_result[i * ncolumn + 1]);
            adjustparams->AdjustParams_ROI.PositionofROI[i - 1].x1 = atof(db_result[i * ncolumn + 2]);
            adjustparams->AdjustParams_ROI.PositionofROI[i - 1].y1 = atof(db_result[i * ncolumn + 3]);
            adjustparams->AdjustParams_ROI.PositionofROI[i - 1].x2 = atof(db_result[i * ncolumn + 4]);
            adjustparams->AdjustParams_ROI.PositionofROI[i - 1].y2 = atof(db_result[i * ncolumn + 5]);
            adjustparams->AdjustParams_ROI.PositionofROI[i - 1].x3 = atof(db_result[i * ncolumn + 6]);
            adjustparams->AdjustParams_ROI.PositionofROI[i - 1].y3 = atof(db_result[i * ncolumn + 7]);
            adjustparams->AdjustParams_ROI.PositionofROI[i - 1].x4 = atof(db_result[i * ncolumn + 8]);
            adjustparams->AdjustParams_ROI.PositionofROI[i - 1].y4 = atof(db_result[i * ncolumn + 9]);
        }

        adjustparams->AdjustParams_ROI.UsingNumbofROI = roi_sum;

        spdlog::info("roi count:[{0}]", roi_sum);
        sqlite3_free_table(db_result);
    }

    /* 读取断面和线圈 */
    ret = sqlite3_exec(dtb, "begin transaction", 0, 0, &err_msg);
    if (ret != SQLITE_OK) {
        sqlite3_free(err_msg);
    }

    ret = sqlite3_get_table(dtb, sql_find_section, &db_result, &nrow, &ncolumn, &err_msg);
    if (ret != SQLITE_OK) {
        sqlite3_close(dtb);
        sqlite3_free(err_msg);
        spdlog::error("get table sql:[{0}] failed", sql_find_section);
        return -1;
    }

    ret = sqlite3_exec(dtb, "commit transaction", 0, 0, &err_msg);
    if (ret != SQLITE_OK) {
        sqlite3_free(err_msg);
    }

    if (db_result != NULL) {
        int section_sum = 0;
        for (int i = 1; i <= nrow; i++) {
            if (sections != NULL) {
                sections[section_sum].section_id = atoi(db_result[i * ncolumn + 0]);
                sections[section_sum].section_x = atof(db_result[i * ncolumn + 1]);
                sections[section_sum].detectionCycle = atof(db_result[i * ncolumn + 2]);
                sections[section_sum].section_status = atoi(db_result[i * ncolumn + 4]);
            }

            /* 遍历该断面的线圈 */
            sprintf(sql_find_coil, "select * from coil where Foreign_section_id = '%s'", db_result[i * ncolumn + 0]);
            int coil_sum = 0;

            ret1 = sqlite3_exec(dtb, "begin transaction", 0, 0, &err_msg);
            if (ret1 != SQLITE_OK) {
                sqlite3_free(err_msg);
            }

            ret1 = sqlite3_get_table(dtb, sql_find_coil, &db_result1, &nrow1, &ncolumn1, &err_msg);
            if (ret1 != SQLITE_OK) {
                sqlite3_close(dtb);
                sqlite3_free(err_msg);
                spdlog::error("get table sql:[{0}] failed", sql_find_coil);
                return -1;
            }

            ret = sqlite3_exec(dtb, "commit transaction", 0, 0, &err_msg);
            if (ret != SQLITE_OK) {
                sqlite3_free(err_msg);
            }

            if (db_result1 != NULL) {
                for (int j = 1; j <= nrow1; j++) {
                    if (sections != NULL) {
                        sections[section_sum].CoilPosition[coil_sum].laneNum = atoi(db_result1[j * ncolumn1 + 2]);
                        sections[section_sum].CoilPosition[coil_sum].coilX = atof(db_result1[j * ncolumn1 + 3]);
                        sections[section_sum].CoilPosition[coil_sum].coilY = atof(db_result1[j * ncolumn1 + 4]);
                        sections[section_sum].CoilPosition[coil_sum].coilWidth = atof(db_result1[j * ncolumn1 + 5]);
                        sections[section_sum].CoilPosition[coil_sum].coilLenth = atof(db_result1[j * ncolumn1 + 6]);
                        sections[section_sum].CoilPosition[coil_sum].coil_status = atof(db_result1[j * ncolumn1 + 7]);
                    }

                    ++coil_sum;
                }

                if (sections != NULL) {
                    sections[section_sum].coilNum = coil_sum;
                }
                ++section_sum;

                sqlite3_free_table(db_result1);
            }
        }

        if (section_num != NULL) {
            *section_num = section_sum;
        }

        spdlog::info("section count:[{0}]", section_sum);
        sqlite3_free_table(db_result);
    }

    sprintf(sql_find_PolygonRoi, "select * from PolygonRoi");
    ret1 = sqlite3_exec(dtb, "begin transaction", 0, 0, &err_msg);
    if (ret1 != SQLITE_OK) {
        sqlite3_free(err_msg);
    }

    ret1 = sqlite3_get_table(dtb, sql_find_PolygonRoi, &db_result1, &nrow1, &ncolumn1, &err_msg);
    if (ret1 != SQLITE_OK) {
        sqlite3_close(dtb);
        sqlite3_free(err_msg);
        spdlog::error("get table sql:[{0}] failed", sql_find_PolygonRoi);
        return -1;
    }

    ret = sqlite3_exec(dtb, "commit transaction", 0, 0, &err_msg);
    if (ret != SQLITE_OK) {
        sqlite3_free(err_msg);
    }

    int PolygonRoi_sum = 0;
    if (db_result1 != NULL) {
        for (int j = 1; j <= nrow1; j++) {
            polyRoi[PolygonRoi_sum].id = atoi(db_result1[j * ncolumn1 + 0]);
            polyRoi[PolygonRoi_sum].P1X = atof(db_result1[j * ncolumn1 + 1]);
            polyRoi[PolygonRoi_sum].P1Y = atof(db_result1[j * ncolumn1 + 2]);
            polyRoi[PolygonRoi_sum].P2X = atof(db_result1[j * ncolumn1 + 3]);
            polyRoi[PolygonRoi_sum].P2Y = atof(db_result1[j * ncolumn1 + 4]);
            polyRoi[PolygonRoi_sum].P3X = atof(db_result1[j * ncolumn1 + 5]);
            polyRoi[PolygonRoi_sum].P3Y = atof(db_result1[j * ncolumn1 + 6]);
            polyRoi[PolygonRoi_sum].P4X = atof(db_result1[j * ncolumn1 + 7]);
            polyRoi[PolygonRoi_sum].P4Y = atof(db_result1[j * ncolumn1 + 8]);

            ++PolygonRoi_sum;
            if (PolygonRoi_sum > (userMaxPloyRoiNum - 1)) {
                break;
            }
        }

        spdlog::info("polygon roi count:[{0}]", PolygonRoi_sum);
        sqlite3_free_table(db_result1);
    }
    polyRoi_num = PolygonRoi_sum;

    return 0;
}

/********************************************************
* Function name ：JudgeBool
* Description   ：实现对字符串转布尔操作
* Parameter     ：
* @buff          字符串
* Return         布尔
**********************************************************/
bool JudgeBool(char *buff)
{
    if (buff == NULL) {
        return false;
    }

    if (!strcmp(buff, "true")) {
        return true;
    }

    return false;
}

/********************************************************
* Function name ：UpdateParams
* Description   ：实现对数据库中参数表的修改操作
* Parameter     ：
* @dtb            数据库指针
* @err_msg        错误信息指针
* @key            更改的key信息
* @value          更改的value信息
* Return         状态信息
**********************************************************/
int UpdateParams(sqlite3 *dtb, char *key, char *value)
{
    int ret = 0;
    char *err_msg = NULL;
    char sql_update[512] = { 0 };

    if ((key == NULL) || (value == NULL)) {
        spdlog::error("UpdateParams key or value is NULL!");
        return -1;
    }

    sprintf(sql_update, "update configuration set value = '%s' where params_name = '%s'", value, key);
    ret = sqlite3_exec(dtb, sql_update, 0, 0, &err_msg);
    if (ret != SQLITE_OK) {
        sqlite3_close(dtb);
        sqlite3_free(err_msg);
        spdlog::error("Execute sql:[{0}] failed", sql_update);
        return -1;
    }

    return 0;
}

/********************************************************
* Function name ：UpdateRoi
* Description   ：实现对数据库中Roi表的修改、新增、删除操作
* Parameter     ：
* @dtb                   数据库指针
* @err_msg               错误信息指针
* @name                  更改的Roi区域名字
* @roi_value             更改的Roi坐标值
* Return                 状态信息
**********************************************************/
int UpdateRoi(sqlite3 *dtb, char *name, char **roi_value)
{
    /* 查询是否有这个屏蔽区 */
    int ret = 0;
    char **db_result;
    char *err_msg = NULL;
    int nrow = 0, ncolumn = 0;
    char sql_find[512] = { 0 };

    if (roi_value == NULL) {
        spdlog::error("ROI value is NULL.");
        return -1;
    }

    sprintf(sql_find, "select * from Roi where name = '%s'", name);
    ret = sqlite3_exec(dtb, "begin transaction", 0, 0, &err_msg);
    if (ret != SQLITE_OK) {
        sqlite3_free(err_msg);
    }

    ret = sqlite3_get_table(dtb, sql_find, &db_result, &nrow, &ncolumn, &err_msg);
    if (ret != SQLITE_OK) {
        sqlite3_close(dtb);
        sqlite3_free(err_msg);
        spdlog::error("get table sql:[{0}] failed", sql_find);
        return -1;
    }

    if (db_result != NULL) {
        sqlite3_free_table(db_result);
    }

    ret = sqlite3_exec(dtb, "commit transaction", 0, 0, &err_msg);
    if (ret != SQLITE_OK) {
        sqlite3_free(err_msg);
    }

    if (strlen(roi_value[0]) == 0) {
        /* 删除该屏蔽区 */
        if (nrow != 0 && ncolumn != 0) {
            /* 如果有这个屏蔽区则删除 */
            char sql_del[512] = { 0 };
            sprintf(sql_del, "delete from Roi where name = '%s'", name);
            ret = sqlite3_exec(dtb, "begin transaction", 0, 0, &err_msg);
            if (ret != SQLITE_OK) {
                sqlite3_free(err_msg);
            }

            ret = sqlite3_exec(dtb, sql_del, 0, 0, &err_msg);
            if (ret != SQLITE_OK) {
                sqlite3_close(dtb);
                sqlite3_free(err_msg);
                spdlog::error("exec sql:[{0}] failed", sql_del);
                return -1;
            }

            ret = sqlite3_exec(dtb, "commit transaction", 0, 0, &err_msg);
            if (ret != SQLITE_OK) {
                sqlite3_free(err_msg);
            }
        }
    } else {
        /* 想修改或新增屏蔽区 */
        if (nrow != 0 && ncolumn != 0) {
            /* 数据库中有同名则修改 */
            /* 修改屏蔽区的参数 */
            char sql_update[512] = { 0 };
            sprintf(sql_update, "update Roi set x1 = '%s', y1 = '%s', x2 = '%s', y2 = '%s', x3 = '%s', y3 = '%s', x4 = '%s', y4 = '%s' where name = '%s'", roi_value[0], roi_value[1], roi_value[2], roi_value[3], roi_value[4], roi_value[5], roi_value[6], roi_value[7], name);
            ret = sqlite3_exec(dtb, "begin transaction", 0, 0, &err_msg);
            if (ret != SQLITE_OK) {
                sqlite3_free(err_msg);
            }

            ret = sqlite3_exec(dtb, sql_update, 0, 0, &err_msg);
            if (ret != SQLITE_OK) {
                sqlite3_close(dtb);
                sqlite3_free(err_msg);
                spdlog::error("exec sql:[{0}] failed", sql_update);
                return -1;
            }

            ret = sqlite3_exec(dtb, "commit transaction", 0, 0, &err_msg);
            if (ret != SQLITE_OK) {
                sqlite3_free(err_msg);
            }
        } else {
            /* 数据库中没有则新增 */
            char sql_new[512] = { 0 };
            sprintf(sql_new, "insert into Roi(name, x1, y1, x2, y2, x3, y3, x4, y4) values('%s', '%s', '%s', '%s', '%s', '%s', '%s', '%s', '%s')", name, roi_value[0], roi_value[1], roi_value[2], roi_value[3], roi_value[4], roi_value[5], roi_value[6], roi_value[7]);

            ret = sqlite3_exec(dtb, sql_new, 0, 0, &err_msg);
            if (ret != SQLITE_OK) {
                sqlite3_close(dtb);
                sqlite3_free(err_msg);
                spdlog::error("exec sql:[{0}] failed", sql_new);
                return -1;
            }
        }
    }

    return 0;
}

/********************************************************
* Function name ：UpdateSections
* Description   ：实现对断面数据库的更新操作
* Parameter     ：
* @dtb                   数据库指针
* @err_msg               错误信息指针
* @id                    断面id
* @section_value         更新断面信息
* Return                 状态信息
**********************************************************/
int UpdateSections(sqlite3 *dtb, int id, char **section_value)
{
    int ret = 0;
    char *err_msg = NULL;
    char sql_update[512] = { 0 };

    if (section_value == NULL) {
        spdlog::error("UpdateSections error: section_value is null");
        return -1;
    }

    sprintf(sql_update, "update section set section_x = '%s', detectionCycle = '%s', status = '%s' where id = '%d'", section_value[0], section_value[1], section_value[2], id);
    ret = sqlite3_exec(dtb, "begin transaction", 0, 0, &err_msg);
    if (ret != SQLITE_OK) {
        sqlite3_free(err_msg);
    }

    ret = sqlite3_exec(dtb, sql_update, 0, 0, &err_msg);
    if (ret != SQLITE_OK) {
        sqlite3_close(dtb);
        sqlite3_free(err_msg);
        spdlog::error("exec sql:[{0}] failed", sql_update);
        return -1;
    }

    ret = sqlite3_exec(dtb, "commit transaction", 0, 0, &err_msg);
    if (ret != SQLITE_OK) {
        sqlite3_free(err_msg);
    }

    return 0;
}

/********************************************************
* Function name ：InsertSections
* Description   ：实现对断面数据库的插入操作
* Parameter     ：
* @dtb                   数据库指针
* @err_msg               错误信息指针
* @id                    断面id
* @section_value         更新断面信息
* Return                 状态信息
**********************************************************/
int InsertSections(sqlite3 *dtb, char **section_value)
{
    /* 新增断面 */
    int ret = 0;
    char *err_msg = NULL;
    char sql_new[512] = { 0 };

    if (section_value == NULL) {
        spdlog::error("new section value = NULL");
        return -1;
    }

    sprintf(sql_new, "insert into section(section_x, detectionCycle, coilNum, status) values('%s', '%s', '%s', '%s')", section_value[0], section_value[1], section_value[2], section_value[3]);
    ret = sqlite3_exec(dtb, sql_new, 0, 0, &err_msg);
    if (ret != SQLITE_OK) {
        sqlite3_close(dtb);
        sqlite3_free(err_msg);
        spdlog::error("exec sql:[{0}] failed", sql_new);
        return -1;
    }

    return 0;
}

/********************************************************
* Function name ：DeleteSections
* Description   ：实现对断面数据库的删除操作
* Parameter     ：
* @dtb                   数据库指针
* @err_msg               错误信息指针
* @id                    断面id
* Return                 状态信息
**********************************************************/
int DeleteSections(sqlite3 *dtb, int id)
{
    int ret = 0;
    char *err_msg = NULL;
    char sql_del[512] = { 0 };

    sprintf(sql_del, "delete from section where id = '%d'", id);
    ret = sqlite3_exec(dtb, "begin transaction", 0, 0, &err_msg);
    if (ret != SQLITE_OK) {
        sqlite3_free(err_msg);
    }

    ret = sqlite3_exec(dtb, sql_del, 0, 0, &err_msg);
    if (ret != SQLITE_OK) {
        sqlite3_close(dtb);
        sqlite3_free(err_msg);
        spdlog::error("exec sql:[{0}] failed", sql_del);
        return -1;
    }

    ret = sqlite3_exec(dtb, "commit transaction", 0, 0, &err_msg);
    if (ret != SQLITE_OK) {
        sqlite3_free(err_msg);
    }

    return 0;
}

/********************************************************
* Function name ：UpdateCoils
* Description   ：实现对线圈数据库的更新操作
* Parameter     ：
* @dtb                   数据库指针
* @err_msg               错误信息指针
* @id                    断面id
* @laneNum               线圈车道
* @coil_value            更新线圈信息
* Return                 状态信息
**********************************************************/
int UpdateCoils(sqlite3 *dtb, int id, int laneNum, char **coil_value)
{
    int ret = 0;
    char *err_msg = NULL;
    char sql_update[512] = { 0 };

    if (coil_value == NULL) {
        spdlog::error("coil value is NULL");
        return -1;
    }

    sprintf(sql_update, "update coil set coilX = '%s', coilY = '%s', coilWidth = '%s', coilLenth = '%s', status = '%s' where Foreign_section_id = '%d' AND laneNum = '%d'", coil_value[0], coil_value[1], coil_value[2], coil_value[3], coil_value[4], id, laneNum);
    ret = sqlite3_exec(dtb, "begin transaction", 0, 0, &err_msg);
    if (ret != SQLITE_OK) {
        sqlite3_free(err_msg);
    }

    ret = sqlite3_exec(dtb, sql_update, 0, 0, &err_msg);
    if (ret != SQLITE_OK) {
        sqlite3_close(dtb);
        sqlite3_free(err_msg);
        spdlog::error("exec sql:[{0}] failed", sql_update);
        return -1;
    }

    ret = sqlite3_exec(dtb, "commit transaction", 0, 0, &err_msg);
    if (ret != SQLITE_OK) {
        sqlite3_free(err_msg);
    }

    return 0;
}

/********************************************************
* Function name ：InsertCoilns
* Description   ：实现对线圈数据库的插入操作
* Parameter     ：
* @dtb                   数据库指针
* @err_msg               错误信息指针
* @id                    断面id
* @coil_value            更新线圈信息
* Return                 状态信息
**********************************************************/
int InsertCoils(sqlite3 *dtb, int id, char **coil_value)
{
    /* 新增线圈 */
    int ret = 0;
    char *err_msg = NULL;
    char sql_new[512] = { 0 };

    if (coil_value == NULL) {
        spdlog::error("The coil parameters is null! Cannot insert coil in database!");
        return -1;
    }

    sprintf(sql_new, "insert into coil(Foreign_section_id, laneNum, coilX, coilY, coilWidth, coilLenth, status) values('%d', '%s', '%s', '%s', '%s', '%s', '%s')", id, coil_value[0], coil_value[1], coil_value[2], coil_value[3], coil_value[4], coil_value[5]);
    ret = sqlite3_exec(dtb, sql_new, 0, 0, &err_msg);
    if (ret != SQLITE_OK) {
        sqlite3_close(dtb);
        sqlite3_free(err_msg);
        spdlog::error("exec sql:[{0}] failed", sql_new);
        return -1;
    }

    /* 更新断面中coilNum */
    char sql_find_coil[512] = { 0 };
    sprintf(sql_find_coil, "select * from coil where Foreign_section_id = '%d'", id);

    ret = sqlite3_exec(dtb, "begin transaction", 0, 0, &err_msg);
    if (ret != SQLITE_OK) {
        sqlite3_free(err_msg);
    }

    char **db_result1;
    int nrow1, ncolumn1;

    ret = sqlite3_get_table(dtb, sql_find_coil, &db_result1, &nrow1, &ncolumn1, &err_msg);
    if (ret != SQLITE_OK) {
        sqlite3_close(dtb);
        sqlite3_free(err_msg);
        spdlog::error("exec sql:[{0}] failed", sql_find_coil);
        return -1;
    }

    ret = sqlite3_exec(dtb, "commit transaction", 0, 0, &err_msg);
    if (ret != SQLITE_OK) {
        sqlite3_free(err_msg);
    }

    if (db_result1 != NULL) {
        sqlite3_free_table(db_result1);
    }

    /* 修改coilNum */
    char sql_update[512] = { 0 };
    sprintf(sql_update, "update section set coilNum = '%d' where id = '%d'", nrow1, id);

    ret = sqlite3_exec(dtb, "begin transaction", 0, 0, &err_msg);
    if (ret != SQLITE_OK) {
        sqlite3_free(err_msg);
    }

    ret = sqlite3_exec(dtb, sql_update, 0, 0, &err_msg);
    if (ret != SQLITE_OK) {
        sqlite3_close(dtb);
        sqlite3_free(err_msg);
        spdlog::error("exec sql:[{0}] failed", sql_update);
        return -1;
    }

    ret = sqlite3_exec(dtb, "commit transaction", 0, 0, &err_msg);
    if (ret != SQLITE_OK) {
        sqlite3_free(err_msg);
    }

    return 0;
}

/********************************************************
* Function name ：user_InsertPolygonRoi_func
* Description   ：插入区域事件检测的区域坐标
* Parameter     ：
* @dtb                   数据库指针
* @err_msg               错误信息指针
* @id                    区域id
* @coil_value            坐标信息
* Return                 状态信息
**********************************************************/
int user_InsertPolygonRoi_func(sqlite3 *dtb, int id, char **coil_value)
{
    int ret = 0;
    char *err_msg = NULL;
    char sql_new[512] = { 0 };

    if (coil_value == NULL) {
        spdlog::error("coil_value is NULL");
        return -1;
    }

    sprintf(sql_new, "insert into PolygonRoi(id, P1X, P1Y, P2X, P2Y, P3X, P3Y, P4X, P4Y) values('%d', '%s', '%s', '%s', '%s', '%s', '%s', '%s', '%s')", id, coil_value[0], coil_value[1], coil_value[2], coil_value[3], coil_value[4], coil_value[5], coil_value[6], coil_value[7]);
    ret = sqlite3_exec(dtb, sql_new, 0, 0, &err_msg);
    if (ret != SQLITE_OK) {
        sqlite3_close(dtb);
        sqlite3_free(err_msg);
        spdlog::error("exec sql:[{0}] failed", sql_new);
        return -1;
    }

    return 0;
}

/********************************************************
* Function name ：DeleteCoils
* Description   ：实现对线圈数据库的删除操作
* Parameter     ：
* @dtb                   数据库指针
* @err_msg               错误信息指针
* @id                    断面id
* @laneNum               线圈车道
* Return                 状态信息
**********************************************************/
int DeleteCoils(sqlite3 *dtb, int id, int laneNum)
{
    int ret = 0;
    char *err_msg = NULL;
    char sql_del[512] = { 0 };

    sprintf(sql_del, "delete from coil where Foreign_section_id = '%d' AND laneNum = '%d'", id, laneNum);
    ret = sqlite3_exec(dtb, "begin transaction", 0, 0, &err_msg);
    if (ret != SQLITE_OK) {
        sqlite3_free(err_msg);
    }

    ret = sqlite3_exec(dtb, sql_del, 0, 0, &err_msg);
    if (ret != SQLITE_OK) {
        sqlite3_close(dtb);
        sqlite3_free(err_msg);
        spdlog::error("exec sql:[{0}] failed", sql_del);
        return -1;
    }

    ret = sqlite3_exec(dtb, "commit transaction", 0, 0, &err_msg);
    if (ret != SQLITE_OK) {
        sqlite3_free(err_msg);
    }

    /* 更新断面中coilNum */
    char sql_find_coil[512] = { 0 };
    sprintf(sql_find_coil, "select * from coil where Foreign_section_id = '%d'", id);

    ret = sqlite3_exec(dtb, "begin transaction", 0, 0, &err_msg);
    if (ret != SQLITE_OK) {
        sqlite3_free(err_msg);
    }

    char **db_result1;
    int nrow1, ncolumn1;

    ret = sqlite3_get_table(dtb, sql_find_coil, &db_result1, &nrow1, &ncolumn1, &err_msg);
    if (ret != SQLITE_OK) {
        sqlite3_close(dtb);
        sqlite3_free(err_msg);
        spdlog::error("exec sql:[{0}] failed", sql_find_coil);
        return -1;
    }

    ret = sqlite3_exec(dtb, "commit transaction", 0, 0, &err_msg);
    if (ret != SQLITE_OK) {
        sqlite3_free(err_msg);
    }

    if (db_result1 != NULL) {
        sqlite3_free_table(db_result1);
    }

    /* 修改coilNum */
    char sql_update[512] = { 0 };
    sprintf(sql_update, "update section set coilNum = '%d' where id = '%d'", nrow1, id);

    ret = sqlite3_exec(dtb, "begin transaction", 0, 0, &err_msg);
    if (ret != SQLITE_OK) {
        sqlite3_free(err_msg);
    }

    ret = sqlite3_exec(dtb, sql_update, 0, 0, &err_msg);
    if (ret != SQLITE_OK)  {
        sqlite3_close(dtb);
        sqlite3_free(err_msg);
        spdlog::error("exec sql:[{0}] failed", sql_update);
        return -1;
    }

    ret = sqlite3_exec(dtb, "commit transaction", 0, 0, &err_msg);
    if (ret != SQLITE_OK) {
        sqlite3_free(err_msg);
    }

    return 0;
}

/********************************************************
* Function name ：writeparams
* Description   ：实现写入结构体
* Parameter     ：
* @key           数据库中的key
* @value         数据库中的value
* Return        ：
**********************************************************/
void writeparams(char *key, char *value, AdjustParams *adjustparams)
{
    if ((key == NULL) || (value == NULL) || (adjustparams == NULL)) {
        return;
    }

    if (!strcmp(key, "EnableTrajectoryManage")) {
        adjustparams->EnableTrajectoryManage = JudgeBool(value);
    } else if (!strcmp(key, "EnableExtra_Judge")) {
        adjustparams->EnableExtra_Judge = JudgeBool(value);
    } else if (!strcmp(key, "EnableSpeed_Dect")) {
        adjustparams->EnableSpeed_Dect = JudgeBool(value);
    } else if (!strcmp(key, "EnableRetrograde_Dect")) {
        adjustparams->EnableRetrograde_Dect = JudgeBool(value);
    } else if (!strcmp(key, "EnableStopCar_Dect")) {
        adjustparams->EnableStopCar_Dect = JudgeBool(value);
    } else if (!strcmp(key, "EnableLaneChange_Dect")) {
        adjustparams->EnableLaneChange_Dect = JudgeBool(value);
    } else if (!strcmp(key, "EnableCongest_Dect")) {
        adjustparams->EnableCongest_Dect = JudgeBool(value);
    } else if (!strcmp(key, "EnableLane_Judge")) {
        adjustparams->EnableLane_Judge = JudgeBool(value);
    } else if (!strcmp(key, "EnabletrafficFlow")) {
        adjustparams->EnabletrafficFlow = JudgeBool(value);
    } else if (!strcmp(key, "EnablelaneDividedTrafficFlow")) {
        adjustparams->EnablelaneDividedTrafficFlow = JudgeBool(value);
    } else if (!strcmp(key, "EnablesectionAverageSpeed")) {
        adjustparams->EnablesectionAverageSpeed = JudgeBool(value);
    } else if (!strcmp(key, "EnablelaneDividedAverageSpeed")) {
        adjustparams->EnablelaneDividedAverageSpeed = JudgeBool(value);
    } else if (!strcmp(key, "EnablelaneDividedTimeOccupancy")) {
        adjustparams->EnablelaneDividedTimeOccupancy = JudgeBool(value);
    } else if (!strcmp(key, "EnablelaneDividedAverageHeadway")) {
        adjustparams->EnablelaneDividedAverageHeadway = JudgeBool(value);
    } else if (!strcmp(key, "EnablelaneDividedAverageHeadwayGap")) {
        adjustparams->EnablelaneDividedAverageHeadwayGap = JudgeBool(value);
    } else if (!strcmp(key, "MaxLaneNum")) {
        int lanes = atoi(value);
        if (lanes <= 0) {
            lanes = 1;
        } else if (lanes >= 5) {
            lanes = 5;
        }

        adjustparams->MaxLaneNum = lanes;
    } else if (!strcmp(key, "AdjustParams_DisLongMin")) {
        adjustparams->AdjustParams_DisLongMin = atof(value);
    } else if (!strcmp(key, "AdjustParams_DisLongMax")) {
        adjustparams->AdjustParams_DisLongMax = atof(value);
    } else if (!strcmp(key, "AdjustParams_DisLatMin")) {
        adjustparams->AdjustParams_DisLatMin = atof(value);
    } else if (!strcmp(key, "AdjustParams_DisLatMax")) {
        adjustparams->AdjustParams_DisLatMax = atof(value);
    } else if (!strcmp(key, "AdjustParams_VrelLongMin")) {
        adjustparams->AdjustParams_VrelLongMin = atof(value);
    } else if (!strcmp(key, "AdjustParams_VrelLongMax")) {
        adjustparams->AdjustParams_VrelLongMax = atof(value);
    } else if (!strcmp(key, "AdjustParams_VrelLatMin")) {
        adjustparams->AdjustParams_VrelLatMin = atof(value);
    } else if (!strcmp(key, "AdjustParams_VrelLatMax")) {
        adjustparams->AdjustParams_VrelLatMax = atof(value);
    } else if (!strcmp(key, "AdjustParams_RCSMin")) {
        adjustparams->AdjustParams_RCSMin = atof(value);
    } else if (!strcmp(key, "AdjustParams_RCSMax")) {
        adjustparams->AdjustParams_RCSMax = atof(value);
    } else if (!strcmp(key, "AdjustParams_DBSCANMinPts")) {
        adjustparams->AdjustParams_DBSCANMinPts = atoi(value);
    } else if (!strcmp(key, "AdjustParams_DBSCANBIGCarEps_a")) {
        adjustparams->AdjustParams_DBSCANBIGCarEps_a = atof(value);
    } else if (!strcmp(key, "AdjustParams_DBSCANBIGCarEps_b")) {
        adjustparams->AdjustParams_DBSCANBIGCarEps_b = atof(value);
    } else if (!strcmp(key, "AdjustParams_DBSCANMEDIUMCarEps_a")) {
        adjustparams->AdjustParams_DBSCANMEDIUMCarEps_a = atof(value);
    } else if (!strcmp(key, "AdjustParams_DBSCANMEDIUMCarEps_b")) {
        adjustparams->AdjustParams_DBSCANMEDIUMCarEps_b = atof(value);
    } else if (!strcmp(key, "AdjustParams_DBSCANSMALLCarEps_a")) {
        adjustparams->AdjustParams_DBSCANSMALLCarEps_a = atof(value);
    } else if (!strcmp(key, "AdjustParams_DBSCANSMALLCarEps_b")) {
        adjustparams->AdjustParams_DBSCANSMALLCarEps_b = atof(value);
    } else if (!strcmp(key, "AdjustParams_EKFPairRNGTHR")) {
        adjustparams->AdjustParams_EKFPairRNGTHR = atof(value);
    } else if (!strcmp(key, "AdjustParams_EKFPairVELTHR")) {
        adjustparams->AdjustParams_EKFPairVELTHR = atof(value);
    } else if (!strcmp(key, "AdjustParams_EKFPairANGTHR")) {
        adjustparams->AdjustParams_EKFPairANGTHR = atof(value);
    } else if (!strcmp(key, "AdjustParams_EKFUpdatePreTypeThres")) {
        adjustparams->AdjustParams_EKFUpdatePreTypeThres = atoi(value);
    } else if (!strcmp(key, "AdjustParams_EKFUpdateValTypeThres")) {
        adjustparams->AdjustParams_EKFUpdateValTypeThres = atoi(value);
    } else if (!strcmp(key, "AdjustParams_EKFFilterTimeInt")) {
        adjustparams->AdjustParams_EKFFilterTimeInt = atof(value);
    } else if (!strcmp(key, "AdjustParams_EKFFilterACC")) {
        adjustparams->AdjustParams_EKFFilterACC = atof(value);
    } else if (!strcmp(key, "AdjustParams_EKFFilterR_RNG")) {
        adjustparams->AdjustParams_EKFFilterR_RNG = atof(value);
    } else if (!strcmp(key, "AdjustParams_EKFFilterR_VEL")) {
        adjustparams->AdjustParams_EKFFilterR_VEL = atof(value);
    } else if (!strcmp(key, "AdjustParams_EKFFilterR_ANG")) {
        adjustparams->AdjustParams_EKFFilterR_ANG = atof(value);
    } else if (!strcmp(key, "AdjustParams_SmallCarRCSThres")) {
        adjustparams->AdjustParams_SmallCarRCSThres = atof(value);
    } else if (!strcmp(key, "AdjustParams_MediumCarRCSThres")) {
        adjustparams->AdjustParams_MediumCarRCSThres = atof(value);
    } else if (!strcmp(key, "AdjustParams_BigCarRCSThres")) {
        adjustparams->AdjustParams_BigCarRCSThres = atof(value);
    } else if (!strcmp(key, "AdjustParams_V_Thr")) {
        adjustparams->AdjustParams_V_Thr = atof(value);
    } else if (!strcmp(key, "AdjustParams_Frame_car_list")) {
        adjustparams->AdjustParams_Frame_car_list = atof(value);
    } else if (!strcmp(key, "AdjustParams_Frame_car_stop")) {
        adjustparams->AdjustParams_Frame_car_stop = atof(value);
    } else if (!strcmp(key, "AdjustParams_X_Thr")) {
        adjustparams->AdjustParams_X_Thr = atof(value);
    } else if (!strcmp(key, "AdjustParams_Y_Thr")) {
        adjustparams->AdjustParams_Y_Thr = atof(value);
    } else if (!strcmp(key, "AdjustParams_MAX_SPEED")) {
        adjustparams->AdjustParams_MAX_SPEED = atof(value);
    } else if (!strcmp(key, "AdjustParams_LIMIT_SPEED")) {
        adjustparams->AdjustParams_LIMIT_SPEED = atof(value);
    } else if (!strcmp(key, "AdjustParams_LIMIT_DIST")) {
        adjustparams->AdjustParams_LIMIT_DIST = atof(value);
    }
}
