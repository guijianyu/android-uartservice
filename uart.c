#define LOG_TAG "UARTHAL"

#include <hardware/hardware.h>
#include <malloc.h>

#include <cutils/log.h>
#include <cutils/atomic.h>

#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <string.h>
#include <hardware/uart.h>

#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <utils/Log.h>
#include <time.h>
#include <termios.h>

static uint8_t sn = 0x00;

/******************************************************************* 
* 名称：                  uart_open 
* 功能：                打开串口并返回串口设备文件描述 
* 入口参数：        fd    :文件描述符     port :串口号(ttyS0,ttyS1,ttyS2) 
* 出口参数：        正确返回为1，错误返回为0 
*******************************************************************/  
static int uart_open(char* port)  
{  
	int fd;
	fd = open(port, O_RDWR|O_NOCTTY);  //|O_NDELAY
	if (false == fd)  
	{  
		perror("Can't Open Serial Port");  
		return(false);  
	}  
	//恢复串口为阻塞状态
	//if(fcntl(fd, F_SETFL, 0) < 0)
	//{  
	//	ALOGD("fcntl failed!\n");  
	//	return(false);  
	//}
	//else  
	//{
	//	ALOGD("fcntl=%d\n",fcntl(fd, F_SETFL,0));  
	//}  
	//测试是否为终端设备   
/*	
	if(0 == isatty(STDIN_FILENO))  
	{  
		ALOGD("standard input is not a terminal device\n");  
		return(false);  
	}  
	else  
	{  
		ALOGD("isatty success!\n");  
	}   
*/	
	ALOGD("fd->open=%d\n",fd);  
	return fd;  
}  

/******************************************************************* 
* 名称：                uart_close 
* 功能：                关闭串口并返回串口设备文件描述 
* 入口参数：        fd    :文件描述符     port :串口号(ttyS0,ttyS1,ttyS2) 
* 出口参数：        void 
*******************************************************************/  
   
static void uart_close(int fd)  
{  
	close(fd);  
}  
   
/******************************************************************* 
* 名称：                uart_set 
* 功能：                设置串口数据位，停止位和效验位 
* 入口参数：        fd        串口文件描述符 
*                              speed     串口速度 
*                              flow_ctrl   数据流控制 
*                           databits   数据位   取值为 7 或者8 
*                           stopbits   停止位   取值为 1 或者2 
*                           parity     效验类型 取值为N,E,O,,S 
*出口参数：          正确返回为1，错误返回为0 
*******************************************************************/  
static int uart_set(int fd, int speed,int flow_ctrl,int databits,int stopbits,int parity)  
{  
	unsigned int i;
	int   speed_arr[] = { B115200, B19200, B9600, B4800, B2400, B1200, B300};  
	int   name_arr[] = {115200,  19200,  9600,  4800,  2400,  1200,  300};  

	struct termios options;

	/*tcgetattr(fd,&options)得到与fd指向对象的相关参数，并将它们保存于options,该函数还可以测试配置是否正确，该串口是否可用等。若调用成功，函数返回值为0，若调用失败，函数返回值为1. 
    */  
	if( tcgetattr(fd,&options)!=0)  
	{
		ALOGI("SetupSerial 1");      
		return(false);
	}
    
    //设置串口输入波特率和输出波特率
	for ( i= 0;  i < sizeof(speed_arr) / sizeof(int);  i++)  
	{  
		if  (speed == name_arr[i])  
		{               
			if(cfsetispeed(&options, speed_arr[i])==0)
			{
				ALOGI("cfsetispeed success");    	
			}   
			if(cfsetospeed(&options, speed_arr[i])==0)
			{
				ALOGI("cfsetospeed success");   
			}
		}  
	}       
     
    //修改控制模式，保证程序不会占用串口  
    options.c_cflag |= CLOCAL;
    //修改控制模式，使得能够从串口中读取输入数据  
    options.c_cflag |= CREAD;
    
    //设置数据流控制  
    switch(flow_ctrl)  
    {  
        
		case 0 ://不使用流控制  
              options.c_cflag &= ~CRTSCTS;  
              break;     
        
		case 1 ://使用硬件流控制  
              options.c_cflag |= CRTSCTS;  
              break;  
		case 2 ://使用软件流控制  
              options.c_cflag |= IXON | IXOFF | IXANY;  
              break;  
    }  
    //设置数据位  
    //屏蔽其他标志位  
    options.c_cflag &= ~CSIZE;  
    switch (databits)  
    {    
		case 5    :  
                     options.c_cflag |= CS5;  
                     break;  
		case 6    :  
                     options.c_cflag |= CS6;  
                     break;  
		case 7    :      
                 options.c_cflag |= CS7;  
                 break;  
		case 8:      
                 options.c_cflag |= CS8;  
                 break;    
		default:     
                 ALOGD(stderr,"Unsupported data size\n");  
                 return (false);   
    }  
    //设置校验位  
    switch (parity)  
    {    
		case 'n':  
		case 'N': //无奇偶校验位。  
                 options.c_cflag &= ~PARENB;   
                 options.c_iflag &= ~INPCK;
                 break;   
		case 'o':    
		case 'O'://设置为奇校验      
                 options.c_cflag |= (PARODD | PARENB);   
                 options.c_iflag |= INPCK;               
                 break;   
		case 'e':   
		case 'E'://设置为偶校验    
                 options.c_cflag |= PARENB;         
                 options.c_cflag &= ~PARODD;         
                 options.c_iflag |= INPCK;        
                 break;  
		case 's':  
		case 'S': //设置为空格   
                 options.c_cflag &= ~PARENB;  
                 options.c_cflag &= ~CSTOPB;  
                 break;   
        default:    
                 ALOGD(stderr,"Unsupported parity\n");      
                 return (false);   
    }   
    // 设置停止位   
    switch (stopbits)  
    {    
		case 1:     
		//options.c_cflag &= ~CSTOPB; break;   
		options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
		options.c_oflag &= ~OPOST; 
		break; 

		case 2:     
                 options.c_cflag |= CSTOPB; 
				 break;  
		default:     
                       ALOGD(stderr,"Unsupported stop bits\n");   
                       return (false);  
    }  
     
	//修改输出模式，原始数据输入输出  
	options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);  
	options.c_oflag &= ~OPOST;
	options.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
     
    //设置等待时间和最小接收字符  
    options.c_cc[VTIME] = 1; /* 读取一个字符等待1*(1/10)s */    
    options.c_cc[VMIN] = 0; /* 读取字符的最少个数为1 */  
     /*
     VTIME      VMIN
     0		>0        一直阻塞到接收到VMIN个数据时read返回
     >0		0        普通超时
     >0		>0  	当接收到第一个字节时开始计算超时。
			如果超时时间未到但数据已经达到VMIN个read立即返回。
			如果超时时间到了就返回当前读到的个数。
     */
     
    //清空发送接收缓冲区
    tcflush(fd,TCIOFLUSH);
     
    //激活配置 (将修改后的termios数据设置到串口中）  
    if (tcsetattr(fd,TCSANOW,&options) != 0)    
	{  
		perror("com set error!\n");    
		return (false);   
	}
    return (true);   
}  
   
/******************************************************************* 
* 名称：                  uart_recv 
* 功能：                接收串口数据 
* 入口参数：        fd                  :文件描述符     
*                              rcv_buf     :接收串口中数据存入rcv_buf缓冲区中 
*                              data_len    :一帧数据的长度 
* 出口参数：        正确返回为1，错误返回为0 
*******************************************************************/  
static int uart_recv(int fd, unsigned char *rcv_buf,int data_len)  
{  
	int len,fs_sel;
    fd_set fs_read; 

    struct timeval time;  

    FD_ZERO(&fs_read);  
    FD_SET(fd,&fs_read);  
     
    time.tv_sec = 0;
    time.tv_usec = 500000;
     
    //使用select实现串口的多路通信  
    fs_sel = select(fd+1,&fs_read,NULL,NULL,&time);  

    if(fs_sel)
	{
		len = read(fd,rcv_buf,data_len);  
		ALOGD("uart_recv len:%d", len);
		return len;
	}
    else
	{
		ALOGD("nothing received");  
		return false;  
	}
}  

static void uart_send(int fd ,unsigned char *rcv_buf,unsigned short data_len)
{
	unsigned short i = 0;
	unsigned short size;
	
	//ALOGI("uart_send11  rcv_buf %02X %02X %02X %02X %02X %02X %02X %02X\n",rcv_buf[0],rcv_buf[1],rcv_buf[2],rcv_buf[3],rcv_buf[4],rcv_buf[5],rcv_buf[6],rcv_buf[7]);
	while(data_len){
		if(data_len > 10){
			data_len -= 10;
			size = 10;
		}else{
			size = data_len;
			data_len = 0;
		}
		write(fd, &rcv_buf[i], size);
		i += size;
		usleep(3000);
	}
}

static unsigned char sum(unsigned char* data, unsigned char len)
{
	unsigned short sum = 0, i;
	for(i = 0; i< len; i++){
		sum += data[i];
	}
	return (unsigned char)(sum & 0xFF);
}

static int uart_communicate(struct uart_device_t* dev, struct msg* t, struct msg* r)
{
	int res = true;
	t->check = sum((unsigned char*)t, sizeof(struct msg) - 1);
	ALOGI("fd:%d", dev->fd);
	ALOGI("uart_communicate send: %02X %02X %02X %02X %02X %02X %02X %02X\n",t->cmd,t->sn,t->data[0],t->data[1],t->data[2],t->data[3],t->data[4],t->check);
	tcflush(dev->fd,TCIOFLUSH);
	uart_send(dev->fd, (unsigned char*)t, sizeof(struct msg));
	uart_recv(dev->fd, (unsigned char*)r, sizeof(struct msg));
	ALOGI("uart_communicate receive: %02X %02X %02X %02X %02X %02X %02X %02X\n",r->cmd,r->sn,r->data[0],r->data[1],r->data[2],r->data[3],r->data[4],r->check);
	if((t->sn != r->sn) 
		|| (r->check != sum((unsigned char*)r, sizeof(struct msg) - 1)))
		res = false;
	return res;
}

static int pwm(struct uart_device_t* dev, int number, int val)
{
	struct msg t, r;
	ALOGI("\033[0;32m %s, %s \033[0;0m\n", __FILE__, __FUNCTION__);
	
	memset(&t, 0xFF, sizeof(struct msg));
	memset(&r, 0xFF, sizeof(struct msg));
	t.cmd = CMD_PWM;
	t.sn = sn++;
	t.data[0] = number;
	t.data[1] = val;
	if(uart_communicate(dev, &t, &r))
		return r.data[0]? false: true;
	else
		return false;
}

static int funspeed(struct uart_device_t* dev, int number)
{
	struct msg t, r;
	int speed;
	ALOGI("\033[0;32m %s, %s \033[0;0m\n", __FILE__, __FUNCTION__);
	
	memset(&t, 0xFF, sizeof(struct msg));
	memset(&r, 0xFF, sizeof(struct msg));
	t.cmd = CMD_FUNSPEED;
	t.sn = sn++;
	t.data[0] = number;

	if(uart_communicate(dev, &t, &r)){
		speed += ((int)r.data[1] & 0x000000FF)<<24;
		speed += ((int)r.data[2] & 0x000000FF)<<16;
		speed += ((int)r.data[3] & 0x000000FF)<<8;
		speed += (int)r.data[4];
	}else
		return false;

	return speed;
}

static int stepmotor(struct uart_device_t* dev, int dir, int beats)
{
	struct msg t, r;
	ALOGI("\033[0;32m %s, %s \033[0;0m\n", __FILE__, __FUNCTION__);
	
	memset(&t, 0xFF, sizeof(struct msg));
	memset(&r, 0xFF, sizeof(struct msg));
	t.cmd = CMD_STEPMOTOR;
	
	t.sn=sn++;
	t.data[0] = dir;
	t.data[1] = (beats & 0xFF00) >> 8;
	t.data[2] = beats;

	if(uart_communicate(dev, &t, &r)){
		return r.data[0];
	}else
		return false;
}

static int temp(struct uart_device_t* dev)
{
	struct msg t, r;
	int temper = ((int)(-100))<<24 | ((int)-100)<<8;

	ALOGI("\033[0;32m %s, %s \033[0;0m\n", __FILE__, __FUNCTION__);
	
	memset(&t, 0xFF, sizeof(struct msg));
	memset(&r, 0xFF, sizeof(struct msg));
	t.cmd = CMD_GET_TEMPE;
	t.sn = sn++;

	if(uart_communicate(dev, &t, &r)){
		temper = 0;
		temper += ((int)r.data[0] & 0x000000FF)<<24;
		temper += ((int)r.data[1] & 0x000000FF)<<16;
		temper += ((int)r.data[2] & 0x000000FF)<<8;
		temper += (int)r.data[3];
	
	} else {

	}
	return temper;
}

static int brightness(struct uart_device_t* dev)
{
	struct msg t, r;
	int intensity = -1;
	
	ALOGI("\033[0;32m %s, %s \033[0;0m\n", __FILE__, __FUNCTION__);
	
	memset(&t, 0xFF, sizeof(struct msg));
	memset(&r, 0xFF, sizeof(struct msg));
	t.cmd = CMD_GET_LIGHT_INTENSITY;
	t.sn = sn++;

	if(uart_communicate(dev, &t, &r)){
		intensity=0;
		intensity += ((int)r.data[0] & 0x000000FF)<<24;
		intensity += ((int)r.data[1] & 0x000000FF)<<16;
		intensity += ((int)r.data[2] & 0x000000FF)<<8;
		intensity += (int)r.data[3];
	}else
	{
		//null
	}

	return intensity;
}

static int uart_device_open(const struct hw_module_t* module, const char* id,
        struct hw_device_t** device)
{	
	struct uart_device_t* dev;
	dev = (struct uart_device_t*)malloc(sizeof(struct uart_device_t));
	
	if(!dev) {
		ALOGE("uart_device_t: failed to alloc space");
		return -EFAULT;
	}
	memset(dev, 0, sizeof(struct uart_device_t));
	dev->common.tag = HARDWARE_DEVICE_TAG;
	dev->common.version = 0;
	dev->common.module = (hw_module_t*)module;

	dev->pwm = pwm;
	dev->funspeed = funspeed;
	dev->stepmotor = stepmotor;
	dev->temp = temp;
	dev->brightness = brightness;
	
	if((dev->fd = uart_open(DEVICE_NAME)) == -1) {
		ALOGE("uart: failed to open %s ", DEVICE_NAME);
		free(dev);
		return -EFAULT;
	}else{
	    if (uart_set(dev->fd, 115200,0,8,1,'N') == false){
			ALOGI("uart_init failed");
			return false;
		}
	}

	ALOGI("uart_device_open fd : %d", dev->fd);
	*device = &(dev->common);
	
	return 0;
}


static struct hw_module_methods_t uart_module_methods = {
    .open = uart_device_open,
};

struct uart_module_t HAL_MODULE_INFO_SYM = {
	common: {
	tag: HARDWARE_MODULE_TAG,
	version_major: 1,
	version_minor: 0,
    id :UART_HARDWARE_MODULE_ID,
	name: MODULE_NAME,
	author: MODULE_AUTHOR,
    methods:&uart_module_methods,
	}
};
