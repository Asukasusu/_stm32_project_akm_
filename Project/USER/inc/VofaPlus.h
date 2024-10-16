#ifndef __VOFA_PLUS_H__
#define __VOFA_PLUS_H__

#include <stdint.h>

#ifdef __C51__
#define inline
#define VOFA_USE_BIG_ENDIAN
#endif

// 定义浮点型占用大小
#define  FLOAT_SIZE    4


/* 通道信息结构体
 *    dat: 联合体 存储通道的具体数据状态
 *         fData: 浮点数据内容
 *         bData: 浮点转换后的整形数据数组
 *  *next: 链表后继指针
 */
typedef struct Vofa_Channel {
    union dat_u{
        /* 公用内存 */
        float    fData;
        uint8_t  bData[FLOAT_SIZE];
    } dat;
    
    struct Vofa_Channel* next;
}Vofa_Channel;


// Vofa 初始化
void Vofa_Init(void (*Vofa_UARTSendByteCallback)(uint8_t));
// Vofa 添加通道
void Vofa_AddChannel(struct Vofa_Channel* handle);
// Vofa 删除通道
void Vofa_DelChannel(struct Vofa_Channel* handle);
// Vofa 设置通道数据
void Vofa_SetChannelData(struct Vofa_Channel* handle, float fData);
// 通过 JustFloat 协议直接发送一个数据
void Vofa_JustFloatSendData(float fData);

void Vofa_JustFloatSendDataAll(float *fData, uint8_t num);
// 通过 JustFloat 协议发送一次指定通道的数据
void Vofa_JustFloatSendChannelData(struct Vofa_Channel* handle);
// 通过 JustFloat 协议一次发送所用通道的数据
void Vofa_JustFloatSendAllChannelData(void);

#endif
