/* OLED 应用显示接口。 */
#ifndef __APP_DISPLAY_H
#define __APP_DISPLAY_H  /* 头文件保护宏，防止接口重复声明。 */

/* 显示开机页：车辆名称、模式、档位与按键提示。 */
void AppDisplay_ShowBoot(void);

/* 运行开机配置页并等待用户启动。阻塞主流程，退出时复位运动状态并清空旧命令。 */
void AppDisplay_BootConfig(void);

/* 按当前工作模式刷新 OLED 状态页。主循环里调。 */
void AppDisplay_Task100ms(void);

#endif
