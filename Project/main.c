#include "headfile.h"
#include <math.h>
// #include "FreeRTOS.h"
// #include "task.h"


/*
 * FLASH 256KB(代码存储部分 只读)  SRAM 48KB(堆(Heap)、栈(Stack)、静态区(Static Area)、全局区(Global Area))
 *
 *                   --------------------
 * 数据或变量 高地址 |栈      stack     | ↓ 存放函数局部变量和参数以及返回值 函数返回后  FIFO结构
 * 的存储位置        |                  |   由操作系统立即回收 使用不当容易栈溢出 大小固定 运行态
 *                   |                  |
 *                   |       ....       |
 *                   |                  |
 *                   |                  |   动态分配的内存段malloc calloc realloc申请空间 free释放
 *                   |堆      heap      | ↑ 调用malloc等函数 分配内存被动态添加到堆上扩张 不固定 运行态
 *                   |未初始化数据  .bss|   未初始化的全局和静态变量、或置零      大小固定  存储态和运行态
 *                   |数据段       .data|   被初始化的全局和静态变量 常量数据     大小固定  存储态和运行态
 *            低地址 |代码段       .text|   存放CPU执行的机器指令                 大小固定  存储态和运行态
 *                   --------------------
 *  数据类型  Code:    代码 编译产生的机器码汇编
 *            RO_data: Read Only  data 只读数据 存储在ROM(Flash)中 const关键字定义的变量 不可更改
 *            RW_data: Read Write data 读写数据 指初始化非零的数据 程序运行时 由FLASH搬运到RAM中常驻
 *            ZI_data: Zero Init  data 0初始化数据 运行中 性质与RW_data一样 由FLASH搬运到RAM中常驻
 * 
 *  程序烧录完成   Code + RO_data + RW_data（RO + RW）三种类型需要占用 Flash 空间 RAM不需要用到
 *  程序运行时     Flash不变,  Flash中的 RW_data(RW) 复制到RAM中  然后把 ZI_data 加载到 RAM中
 *  
 *  芯片的 Flash 大小 要大于 Code + RO-data + RW-data 的大小
 *  芯片的 RAM 大小 要大于 RW-data + ZI_data 的大小
 */

int main(void)
{
    __set_PRIMASK(1);
      System_init();
    __set_PRIMASK(0);
    // __enable_irq() ;

    for (;;) {

        TaskLoop_Run();

    }

}

