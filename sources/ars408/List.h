/********************************************************************************
* @File name: List.h
* @Author: 石海新
* @Version: 1.0
* @Date: 2023.6.7
* @Description: 链表头文件
********************************************************************************/
//#pragma once
#ifndef USER_LIST_H
#define USER_LIST_H

#include <stdio.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

struct list {
	void* data;			/* 存放数据的地址 */
	int id;				/* 存放id */
	int frame;
	struct list* next;  /* 下一个指针 */
};

void* List_Init(void* data);
void List_Add(struct list* head, void* data, int id, int frame);
int  List_Count(struct list* head);
void* List_GetNode(struct list* head, int index);
void* List_GetNode_Id(struct list* head, int id);
int List_Del(struct list* head, int index);
int List_Del_Frame(struct list* head, int num);
int List_Del_Id(struct list* head, int id);
void* List_Free(struct list* head);

#ifdef __cplusplus
}
#endif

#endif
