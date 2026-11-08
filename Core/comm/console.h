/* 串口命令行：s / x / m L R / t N / y / ? */
#ifndef __CONSOLE_H
#define __CONSOLE_H

void console_init(void);
void console_task(void);     /* 主循环调用，把收到的字节组成行并执行 */

#endif /* __CONSOLE_H */
