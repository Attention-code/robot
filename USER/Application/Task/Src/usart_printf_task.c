#include "main.h"
#include <stdio.h>
#include <stdarg.h>

#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "usart.h" // 确保包含了 huart7 的声明

// // 1. 强制分配在 D2 域的 SRAM (DMA 可直接访问)，并且强制 32 字节对齐以适配 Cache Line
// ALIGN_32BYTES(uint8_t tx_buf[256]) __attribute__((section(".dma_buffer")));
//
// // 2. 定义一个 RTOS 互斥锁句柄，防止多任务争抢串口
// static SemaphoreHandle_t printf_mutex = NULL;

// void usart_printf(const char *fmt, ...)
// {
//     // 【防御 1：中断上下文保护】如果在中断里调用（如串口回调），直接退出，防止 RTOS 崩溃
//     if (__get_IPSR() != 0) return;
//
//     // 【防御 2：互斥锁初始化与获取】
//     if (printf_mutex == NULL) {
//         printf_mutex = xSemaphoreCreateMutex();
//     }
//     xSemaphoreTake(printf_mutex, portMAX_DELAY);
//
//     // 【防御 3：等待上次 DMA 硬件搬运完毕，带超时自愈】
//     uint32_t timeout = 0;
//     while(huart7.gState != HAL_UART_STATE_READY) {
//         osDelay(1); // 让出 CPU，让 EKF 姿态结算等高优先级任务继续跑
//         timeout++;
//         if(timeout > 10) {
//             // 如果硬件受电磁干扰卡死，强行打断，防止底盘失控
//             HAL_UART_AbortTransmit(&huart7);
//             huart7.gState = HAL_UART_STATE_READY;
//             break;
//         }
//     }
//
//     // 此时串口绝对空闲，且被当前任务独占，开始安全打包字符串
//     va_list ap;
//     va_start(ap, fmt);
//     // 【防御 4：必须使用有符号 int 接收返回值】
//     int len = vsnprintf((char *)tx_buf, sizeof(tx_buf), fmt, ap);
//     va_end(ap);
//
//     // 【防御 5：拦截 NaN 导致的负数错误或缓存溢出】
//     if (len <= 0) {
//         xSemaphoreGive(printf_mutex);
//         return; // 数据异常，直接丢弃这一帧
//     }
//     if (len >= sizeof(tx_buf)) {
//         len = sizeof(tx_buf) - 1; // 强行截断，防止越界
//     }
//
//     // 【核心防御 6：D-Cache 32字节对齐刷新】
//     // CPU 刚刚把数据写进了 Cache，但在 H7 上，DMA 是直接去物理内存拿数据的。
//     // 必须把 Cache 里的数据“挤”到物理内存里，否则发出去的全是乱码或全 0。
//     uint32_t aligned_len = ((len + 31) / 32) * 32;
//     SCB_CleanDCache_by_Addr((uint32_t *)tx_buf, aligned_len);
//
//     // 【防御 7：清除硬件电磁干扰导致的静默死锁标志位】
//     __HAL_UART_CLEAR_FLAG(&huart7, UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_FEF);
//
//     // 启动 DMA 发送！硬件接管，CPU 瞬间解放去算 PID 和底盘运动学！
//     HAL_UART_Transmit_DMA(&huart7, tx_buf, (uint16_t)len);
//
//     // 【交出钥匙】：释放互斥锁，允许其他任务打印
//     xSemaphoreGive(printf_mutex);
// }
#define TX_BUF_SIZE  256

/* * ⭕ 正确做法：使用 section 属性指向链接脚本里的 .AXI_SRAM
 * 这样变量会被精准定位到 0x24000000 (D1域 / AXI SRAM)
 * 配合 MPU 关闭 Cache，完美解决 DMA 发送数据一致性问题
 */
uint8_t tx_buffer[TX_BUF_SIZE] __attribute__((section(".AXI_SRAM")));

void usart_printf(const char *format, ...)
{
    va_list args;
    uint32_t length;

    // 防止任务死锁
    if (huart7.gState == HAL_UART_STATE_BUSY_TX) {
        return;
    }

    va_start(args, format);
    length = vsnprintf((char *)tx_buffer, TX_BUF_SIZE, format, args);
    va_end(args);

    if (length > 0)
    {
        // 清除可能存在的干扰标志位
        __HAL_UART_CLEAR_FLAG(&huart7, UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_FEF);

        // 直接安全发送
        HAL_UART_Transmit_DMA(&huart7, tx_buffer, length);
    }
}