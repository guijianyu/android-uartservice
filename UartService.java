package com.android.server;

import android.util.Slog;
import android.content.Context;
import android.os.IUartService;
import android.content.BroadcastReceiver;
import android.content.Intent;
import android.os.UserHandle;
import android.content.IntentFilter;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

public class UartService extends IUartService.Stub {
    private static final String TAG = "UartService";
	private static int mUartPtr = 0;
	private static Lock lock = new ReentrantLock();
	
	private final Context mContext;
	private	MyReceiver myReceiver;
	private IntentFilter intentFilter;
	public UartService(Context context) {
		mContext = context;
		mUartPtr = native_uartopen();
		if(mUartPtr < 0){
			Slog.e(TAG, "Failed to init uart service.");
		}
		myReceiver = new MyReceiver();
        intentFilter = new IntentFilter();
		intentFilter.addAction("android.intent.action.BOOT_COMPLETED");
		intentFilter.addAction(Intent.ACTION_USER_SYLINCOM_UART);
		mContext.registerReceiver(myReceiver, intentFilter);
		
		
	}
	
public class MyReceiver extends BroadcastReceiver{

	@Override
	public void onReceive(Context context, Intent intent) {
        if(intent.getAction().equals("android.intent.action.BOOT_COMPLETED")){
			new Thread(){
            @Override
            public void run() {
				try
				{
					while(true)
					{
					Slog.i(TAG, "wo shi zi xian cheng .");
					if(UartService.native_alarmtouser()==1)
					{
						//intent.putExtra("temp", 1);
						//mContext.sendBroadcast(intent);
						//mContext.sendBroadcastAsUser(intent,UserHandle.ALL);
						
						// Intent intent = new Intent(Intent.ACTION_USER_SYLINCOM_UART);
						 Intent intent = new Intent(Intent.ACTION_USER_SYLINCOM_UART);
						 
						//intent.addFlags(Intent.FLAG_RECEIVER_REPLACE_PENDING);
						//intent.putExtra("time-zone", zone.getID());
						mContext.sendBroadcastAsUser(intent, UserHandle.ALL);
						//mContext.sendStickvBroadcast(intent);
					}	
					Thread.sleep(3000);
					}
				}catch(Exception e)
				{
					Slog.i(TAG, "warning UartService sendBroadcast."+e.getMessage());
					e.printStackTrace();
					
				}
				
            }
        }.start();
		}
		else if (intent.getAction().equals(Intent.ACTION_USER_SYLINCOM_UART))
		{
			Slog.i(TAG, "ACTION_USER_SYLINCOM_UART received");
		}
	}
	
}
	public int pwm(int number, int val){
		int ret;
		
		if(mUartPtr < 0){
			Slog.e(TAG, "uart service is not initialized.");
		}
		
		Slog.i(TAG, "\033[0;32m uart service, pwm \033[0;0m\n");
		
		lock.lock();
		try {  
			ret = native_pwm(number, val);
	    }finally{  
	        lock.unlock();
	    }
		return ret;
	}
	
	public int funspeed(int number){
		int ret;
		
		if(mUartPtr < 0){
			Slog.e(TAG, "uart service is not initialized.");
		}
		
		Slog.i(TAG, "\033[0;32m uart service, funspeed \033[0;0m\n");
		
		lock.lock();
		try {  
			ret = native_funspeed(number);
	    }finally{  
	        lock.unlock();
	    }
		return ret;
	}

	public int stepmotor(int dir, int beats){
		int ret;
		if(mUartPtr < 0){
			Slog.e(TAG, "uart service is not initialized.");
		}
		
		Slog.i(TAG, "\033[0;32m uart service, stepmotor \033[0;0m\n");
		
		lock.lock();
		try {  
			ret = native_stepmotor(dir, beats);
	    }finally{  
	        lock.unlock();
	    }
		return ret;
	}

	public int temp(){
		int ret;
		if(mUartPtr < 0){
			Slog.e(TAG, "uart service is not initialized.");
		}
		
		Slog.i(TAG, "\033[0;32m uart service, temp \033[0;0m\n");
		
		lock.lock();
		try {  
			ret = native_temp();
	    }finally{  
	        lock.unlock();
	    }
		return ret;
	}

	public int brightness(){
		int ret;
		if(mUartPtr < 0){
			Slog.e(TAG, "uart service is not initialized.");
		}
		
		Slog.i(TAG, "\033[0;32m uart service, brightness \033[0;0m\n");
		
		lock.lock();
		try {  
			ret = native_brightness();
	    }finally{  
	        lock.unlock();
	    }
		return ret;
	}
	public int alarmtouser(){
			return native_alarmtouser();
	}
	public static native int native_uartopen();
	public static native int native_pwm(int number, int val);
	public static native int native_funspeed(int number);
	public static native int native_stepmotor(int dir, int beats);
	public static native int native_temp();
	public static native int native_brightness();
	public static native int native_alarmtouser();
}

