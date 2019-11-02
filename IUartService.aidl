package android.os;


/** {@hide} */
interface IUartService
{
  int pwm(int number, int val);
  int funspeed(int number);
  int stepmotor(int dir, int beats);
  int temp();
  int brightness();
  int alarmtouser();
}



