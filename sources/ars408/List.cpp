/********************************************************************************
* @File name: List.h
* @Author: 石海新
* @Version: 1.0
* @Date: 2023.6.7
* @Description: 链表实现文件
********************************************************************************/
#include "List.h"

/********************************************************
* Function name ：List_Init
* Description   ：实现链表的初始化
* Parameter     ：
* @data           头节点的数据(NULL)
* Return        ：头节点的指针
**********************************************************/
void *List_Init(void *data)
{
    struct list *head = (struct list *)malloc(sizeof(struct list));
    if (head == NULL) {
        exit(EXIT_FAILURE);
    }

    head->data = data;
    head->id = -1;
    head->frame = -1;
    head->next = NULL;
    return head;
}

/********************************************************
* Function name ：List_Add
* Description   ：实现链表的添加功能
* Parameter     ：
* @head           链表的头节点指针
* @data           要添加的数据
* @id             要添加的id
* @pNode          要添加的节点指针
* @frame          要添加的节点frame
* Return        ：
**********************************************************/
void List_Add(struct list *head, void *data, int id, int frame)
{
    struct list *p1 = head;
    struct list *pNode = (struct list *)malloc(sizeof(struct list));
    if (pNode == NULL) {
        exit(EXIT_FAILURE);
    }

    while (p1->next != NULL) {
        p1 = p1->next;  //遍历链表，找到最末尾的节点
    }

    p1->next = pNode;
    pNode->data = data;
    pNode->id = id;
    pNode->frame = frame;
    pNode->next = NULL;
}

/********************************************************
* Function name ：List_Count
* Description   ：获取链表的长度
* Parameter     ：
* @head           链表的头节点指针
* @nCount         链表的长度
* Return        ：链表的长度(INT)
**********************************************************/
int List_Count(struct list *head)
{
    int nCount = 0;
    struct list *p1 = head->next;

    while (p1 != NULL) {
        nCount++;
        p1 = p1->next;
    }

    return nCount;
}

/********************************************************
* Function name ：List_GetNode
* Description   ：获取链表的某个节点
* Parameter     ：
* @head           链表的头节点指针
* @index         节点的索引
* Return        ：节点的指针
**********************************************************/
void* List_GetNode(struct list *head, int index)
{
    int nCount = 0;
    struct list *p1 = head;

    while (p1->next != NULL) {
        p1 = p1->next;
        if (nCount == index) {
            return p1->data;
        }

        nCount++;
    }

    return NULL;
}

/********************************************************
* Function name ：LIST_GetNode_Id
* Description   ：通过id获取链表的某个节点
* Parameter     ：
* @head           链表的头节点指针
* @id             id
* Return        ：节点的指针
**********************************************************/
void *List_GetNode_Id(struct list *head, int id)
{
    struct list *p1 = head;
    while (p1->next != NULL) {
        if (p1->next->id == id) {
            return p1->next->data;
        }

        p1 = p1->next;
    }

    return NULL;
}

/********************************************************
* Function name ：List_Del
* Description   ：删除链表的某个节点
* Parameter     ：
* @head           链表的头节点指针
* @index          节点的索引
* @p2             要删除结点的临时结点
* Return        ：链表的新长度
**********************************************************/
int List_Del(struct list *head, int index)
{
    int nCount = 0;
    struct list *p1;
    struct list *p2;

    p1 = head;
    while (p1->next != NULL) {
        if (nCount == index) {
            p2 = p1->next;
            p1->next = p1->next->next;
            free(p2);
        }

        p1 = p1->next;
        nCount++;
    }

    return nCount;
}

/********************************************************
* Function name ：List_Del_Frame
* Description   ：删除链表中超过num帧的节点
* Parameter     ：
* @head           链表的头节点指针
* @p2             要删除结点的临时结点
* @num
* Return        ：链表的新长度
**********************************************************/
int List_Del_Frame(struct list *head, int num)
{
    int nCount = 0;
    struct list *p2;
    struct list *p1 = head;

    while (p1->next != NULL) {
        if (p1->next->frame > num) {
            p2 = p1->next;
            p1->next = p1->next->next;
            free(p2);
        } else {
            p1 = p1->next;
            nCount++;
        }
    }

    return nCount;
}

/********************************************************
* Function name ：List_Del_Id
* Description   ：删除链表中某一id的节点
* Parameter     ：
* @head           链表的头节点指针
* @p2             要删除结点的临时结点
* @num
* Return        ：链表的新长度
**********************************************************/
int List_Del_Id(struct list *head, int id)
{
    int nCount = 0;
    struct list *p1;
    struct list *p2;

    p1 = head;
    while (p1->next != NULL) {
        if (p1->next->id == id) {
            p2 = p1->next;
            p1->next = p1->next->next;
            free(p2);
        } else {
            p1 = p1->next;
            nCount++;
        }
    }

    return nCount;
}

/********************************************************
* Function name ：List_Free
* Description   ：释放链表
* Parameter     ：
* @head           链表的头节点指针
* Return        ：空的头节点指针
**********************************************************/
void* List_Free(struct list *head)
{
    struct list *ptr = head;

    while (ptr != NULL) {
        ptr = ptr->next;
        free(head->data);   // 先释放数据存储的内存空间
        free(head);         // 再释放链表节点的内存空间
        head = ptr;
    }

    return head;
}
