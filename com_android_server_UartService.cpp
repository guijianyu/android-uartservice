#define LOG_TAG "UartService"

#include "jni.h"
#include "JNIHelp.h"
#include "android_runtime/AndroidRuntime.h"
#include <binder/IServiceManager.h>

#include <utils/misc.h>
#include <utils/Log.h>

#include <stdio.h>

#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <hardware/uart.h>

namespace android
{
static uart_device_t* uart_device;

static inline int uart_device_open(const hw_module_t* module, struct uart_device_t** device) {
		return module->methods->open(module, UART_HARDWARE_MODULE_ID, (struct hw_device_t**)device);
	}

jint uartOpen(JNIEnv *env, jobject cls)
{	
	jint err;
    uart_module_t* module;
	uart_device_t* device;

	ALOGI("native uartInit ...");		
  if(hw_get_module("uart", (hw_module_t const**)&module)==0)
  {		ALOGI("uart JNI OPEN: uart Stub found.");		
	    err =  uart_device_open(&(module->common), &device);
	    if (err == 0) {
	        uart_device = (uart_device_t *)device;
			return 0;
	    } else {
	        return -1;
    	}
  }		
		return -1;  
}

jint pwm(JNIEnv *env, jobject cls, int number, int val)
{
	ALOGI("\033[0;32m native jni  pwm \033[0;0m");
	return uart_device->pwm(uart_device, number, val);
}

jint funspeed(JNIEnv *env, jobject cls, int number)
{
	ALOGI("\033[0;32m native jni  funspeed \033[0;0m");
	return uart_device->funspeed(uart_device, number);
}

jint stepmotor(JNIEnv *env, jobject cls, int dir, int beats)
{
	ALOGI("\033[0;32m native jni  stepmotor \033[0;0m");
	return uart_device->stepmotor(uart_device, dir, beats);
}

jint temp(JNIEnv *env, jobject cls)
{
	ALOGI("\033[0;32m native jni  temp \033[0;0m");
	return uart_device->temp(uart_device);
}

jint brightness(JNIEnv *env, jobject cls)
{
	ALOGI("\033[0;32m native jni  brightness \033[0;0m");
	return uart_device->brightness(uart_device);
}
jint alarmtouser(JNIEnv *env, jobject cls)
{
	FILE *fp;
	char buffer[4];
	fp = popen("cat /sys/class/gpio/gpio2/value", "r");
	fgets(buffer, sizeof(buffer), fp);
	//ALOGI("atoi %d\n", atoi(buffer));
	pclose(fp);
	return atoi(buffer);
}
static const JNINativeMethod methods[] = {
	{"native_uartopen", "()I", (void *)uartOpen},
	{"native_pwm", "(II)I", (int *)pwm},
	{"native_funspeed", "(I)I", (int *)funspeed},
	{"native_stepmotor", "(II)I", (int *)stepmotor},
	{"native_temp", "()I", (int *)temp},
	{"native_brightness", "()I", (int *)brightness},
	{"native_alarmtouser", "()I", (int *)alarmtouser}
};

int register_android_server_UartService(JNIEnv *env)
{
    return jniRegisterNativeMethods(env, "com/android/server/UartService",
            methods, NELEM(methods));
}

}
