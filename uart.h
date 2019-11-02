#ifndef ANDROID_UART_INTERFACE_H
#define ANDROID_UART_INTERFACE_H

#include <stdint.h>
#include <sys/cdefs.h>
#include <sys/types.h>

#include <hardware/hardware.h>

__BEGIN_DECLS

#define UART_HARDWARE_MODULE_ID  "uart"
#define UART_HARDWARE_DEVICE_ID  "uart"
#define DEVICE_NAME "/dev/ttyS1"
#define MODULE_NAME "uart"
#define MODULE_AUTHOR "sylincom"

struct uart_module_t {
    struct hw_module_t common;	
};

struct uart_device_t {
	struct hw_device_t common;
	int fd;

	//void (*send)(struct uart_device_t* dev, unsigned char* buf, unsigned short len);
	//int (*recv)(struct uart_device_t* dev, unsigned char* buf, unsigned short len);
	int (*pwm)(struct uart_device_t* dev, int number, int val);
	int (*funspeed)(struct uart_device_t* dev, int number);
	int (*stepmotor)(struct uart_device_t* dev, int dir, int beats);
	int (*temp)(struct uart_device_t* dev);
	int (*brightness)(struct uart_device_t* dev);
};

typedef enum{
	CMD_PWM = 0x01,	//风扇pwm设置
	CMD_FUNSPEED,	//0x02，风扇转速获取
	CMD_STEPMOTOR,	//0x03，步进电机控制
	CMD_SLEEP,	//0x04，步进电机睡眠
	CMD_GET_TEMPE,	//0x05，获取温度
	CMD_GET_LIGHT_INTENSITY,	//0x06，设置光敏
	CMD_GET_ENV,	//0x07，获取环境变量
	CMD_SET_ENV,	//0x08，设置环境变量
	CMD_TEMPE_LIMIT,//0x09，温度阈值
	CMD_XFM,
}cmd_t;

//stm32与mstar串口通信协议
struct msg{
	cmd_t cmd:8;	//命令
	uint8_t sn;		//命令序列号
	uint8_t data[5];	//数据
	uint8_t check;	//简单校验：cmd,sn,data[5]求和，取低八位
};


__END_DECLS

#endif  // ANDROID_uart_INTERFACE_H
