/* OLED 应用显示接口。 */
#ifndef __DISPLAY_H
#define __DISPLAY_H  /* 头文件保护宏，防止接口重复声明。 */

/* 显示开机页：车辆名称、模式、档位与按键提示。 */
void display_show_boot(void);

/* 运行开机配置页并等待用户启动。阻塞主流程，退出时复位运动状态并清空旧命令。 */
void display_boot_config(void);

/* 按当前工作模式刷新 OLED 状态页。主循环里调。 */
void display_task(void);

#endif
