/*
 * control.c
 *
 *  Created on: Jun 14, 2022
 *      Author: lf
 */
#include "control.h"
#include "usart.h"
/**************************************************************************
Function: Control function
Input   : none
Output  : none
锟斤拷锟斤拷锟斤拷锟杰ｏ拷锟斤拷锟叫的匡拷锟狡达拷锟诫都锟斤拷锟斤拷锟斤拷锟斤拷
         5ms锟解部锟叫讹拷锟斤拷MPU6050锟斤拷INT锟斤拷锟脚达拷锟斤拷
         锟较革拷证锟斤拷锟斤拷锟斤拷锟斤拷锟捷达拷锟斤拷锟斤拷时锟斤拷同锟斤拷
锟斤拷诓锟斤拷锟斤拷锟斤拷锟 
锟斤拷锟斤拷  值锟斤拷锟斤拷
**************************************************************************/
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	static u8 Flag_Target;																//锟斤拷锟狡猴拷锟斤拷锟斤拷乇锟斤拷锟斤拷锟斤拷峁 10ms锟斤拷准
	int Encoder_Left,Encoder_Right;             					//锟斤拷锟揭憋拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟 
	int Balance_Pwm,Velocity_Pwm,Turn_Pwm;		  					//平锟解环PWM锟斤拷锟斤拷锟斤拷锟劫度伙拷PWM锟斤拷锟斤拷锟斤拷转锟斤拷PWM锟斤拷
	if(GPIO_Pin==MPU6050_INT_Pin)
	{
		Flag_Target=!Flag_Target;
		Get_Angle(Way_Angle);                     					//锟斤拷锟斤拷锟斤拷态锟斤拷5ms一锟轿ｏ拷锟斤拷锟竭的诧拷锟斤拷频锟绞匡拷锟皆革拷锟狡匡拷锟斤拷锟斤拷锟剿诧拷锟酵伙拷锟斤拷锟剿诧拷锟斤拷效锟斤拷
		Encoder_Left=-Read_Encoder(3);            					//锟斤拷取锟斤拷锟街憋拷锟斤拷锟斤拷锟斤拷值锟斤拷前锟斤拷为锟斤拷锟斤拷锟斤拷锟斤拷为锟斤拷
		Encoder_Right=-Read_Encoder(4);           					//锟斤拷取锟斤拷锟街憋拷锟斤拷锟斤拷锟斤拷值锟斤拷前锟斤拷为锟斤拷锟斤拷锟斤拷锟斤拷为锟斤拷
																												//锟斤拷锟斤拷A锟斤拷锟絋IM2_CH1,锟斤拷锟斤拷A锟斤拷锟絋IM4_CH2,锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟侥硷拷锟斤拷锟斤拷同
		Get_Velocity_Form_Encoder(Encoder_Left,Encoder_Right);//锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷转锟劫度ｏ拷mm/s锟斤拷
		Key();                                    					//扫锟借按锟斤拷状态 锟斤拷锟斤拷双锟斤拷锟斤拷锟皆改憋拷小锟斤拷锟斤拷锟斤拷状态
		Balance_Pwm=Balance(Angle_Balance,Gyro_Balance);    //平锟斤拷PID锟斤拷锟斤拷 Gyro_Balance平锟斤拷锟斤拷俣燃锟斤拷裕锟角帮拷锟轿拷锟斤拷锟斤拷锟斤拷锟轿拷锟 
		Velocity_Pwm=Velocity(Encoder_Left,Encoder_Right);  //锟劫度伙拷PID锟斤拷锟斤拷	锟斤拷住锟斤拷锟劫度凤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷小锟斤拷锟斤拷锟绞憋拷锟揭拷锟斤拷锟斤拷锟斤拷锟斤拷锟揭拷锟斤拷芸锟揭伙拷锟 
		Turn_Pwm=Turn(Gyro_Turn);														//转锟斤拷PID锟斤拷锟斤拷

		Motor_Left=Balance_Pwm+Velocity_Pwm+Turn_Pwm;       //锟斤拷锟斤拷锟斤拷锟街碉拷锟斤拷锟斤拷锟絇WM
		Motor_Right=Balance_Pwm+Velocity_Pwm-Turn_Pwm;      //锟斤拷锟斤拷锟斤拷锟街碉拷锟斤拷锟斤拷锟絇WM
																												//PWM值锟斤拷锟斤拷使小锟斤拷前锟斤拷锟斤拷锟斤拷锟斤拷使小锟斤拷锟斤拷锟斤拷
		Motor_Left=PWM_Limit(Motor_Left,6900,-6900);
		Motor_Right=PWM_Limit(Motor_Right,6900,-6900);			//PWM锟睫凤拷
		if(Turn_Off(Angle_Balance,Voltage)==0)     					//锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷斐 
			Set_Pwm(Motor_Left,Motor_Right);         					//锟斤拷值锟斤拷PWM锟侥达拷锟斤拷
		__HAL_GPIO_EXTI_CLEAR_IT(MPU6050_INT_Pin);					//锟斤拷锟斤拷卸媳锟街疚 
	 }
}

/**************************************************************************
Function: Vertical PD control
Input   : Angle:angle锟斤拷Gyro锟斤拷angular velocity
Output  : balance锟斤拷Vertical control PWM
锟斤拷锟斤拷锟斤拷锟杰ｏ拷直锟斤拷PD锟斤拷锟斤拷
锟斤拷诓锟斤拷锟斤拷锟紸ngle:锟角度ｏ拷Gyro锟斤拷锟斤拷锟劫讹拷
锟斤拷锟斤拷  值锟斤拷balance锟斤拷直锟斤拷锟斤拷锟斤拷PWM
**************************************************************************/
int Balance(float Angle,float Gyro)
{
   float Angle_bias,Gyro_bias;
	 int balance;
	 Angle_bias=Middle_angle-Angle;                       				//锟斤拷锟狡斤拷锟侥角讹拷锟斤拷值 锟酵伙拷械锟斤拷锟 
	 Gyro_bias=0-Gyro;
	 balance=-Balance_Kp/100*Angle_bias-Gyro_bias*Balance_Kd/100; //锟斤拷锟斤拷平锟斤拷锟斤拷频牡锟斤拷PWM  PD锟斤拷锟斤拷   kp锟斤拷P系锟斤拷 kd锟斤拷D系锟斤拷
	 return balance;
}

/**************************************************************************
Function: Speed PI control
Input   : encoder_left锟斤拷Left wheel encoder reading锟斤拷encoder_right锟斤拷Right wheel encoder reading
Output  : Speed control PWM
锟斤拷锟斤拷锟斤拷锟杰ｏ拷锟劫度匡拷锟斤拷PWM
锟斤拷诓锟斤拷锟斤拷锟絜ncoder_left锟斤拷锟斤拷锟街憋拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷encoder_right锟斤拷锟斤拷锟街憋拷锟斤拷锟斤拷锟斤拷锟斤拷
锟斤拷锟斤拷  值锟斤拷锟劫度匡拷锟斤拷PWM
**************************************************************************/
//锟睫革拷前锟斤拷锟斤拷锟斤拷锟劫度ｏ拷锟斤拷锟睫革拷Target_Velocity锟斤拷锟斤拷锟界，锟侥筹拷60锟酵比斤拷锟斤拷锟斤拷
int Velocity(int encoder_left,int encoder_right)
{
    static float velocity,Encoder_Least,Encoder_bias,Movement;
	  static float Encoder_Integral,Target_Velocity;
	  //================遥锟斤拷前锟斤拷锟斤拷锟剿诧拷锟斤拷====================//
		if(Flag_follow==1||Flag_avoid==1) Target_Velocity = 30; //锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷/锟斤拷锟斤拷模式,锟斤拷锟斤拷锟劫讹拷
		else 											        Target_Velocity = 50;
		if(Flag_front==1)    	Movement=Target_Velocity/Flag_velocity;	  //锟秸碉拷前锟斤拷锟脚猴拷
		else if(Flag_back==1)	Movement=-Target_Velocity/Flag_velocity;  //锟秸碉拷锟斤拷锟斤拷锟脚猴拷
	  else  Movement=0;

   //=============锟斤拷锟斤拷锟斤拷锟斤拷锟杰ｏ拷锟斤拷锟斤拷/锟斤拷锟较ｏ拷==================//
	  if(Flag_follow==1&&(Distance>200&&Distance<500)&&Flag_Left!=1&&Flag_Right!=1) //锟斤拷锟斤拷
			 Movement=Target_Velocity/Flag_velocity;
		if(Flag_follow==1&&Distance<200&&Flag_Left!=1&&Flag_Right!=1)
			 Movement=-Target_Velocity/Flag_velocity;
		if(Flag_avoid==1&&Distance<450&&Flag_Left!=1&&Flag_Right!=1)  //锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷
			 Movement=-Target_Velocity/Flag_velocity;

   //================锟劫讹拷PI锟斤拷锟斤拷锟斤拷=====================//
		Encoder_Least =0-(encoder_left+encoder_right);                    //锟斤拷取锟斤拷锟斤拷锟劫讹拷偏锟斤拷=目锟斤拷锟劫度ｏ拷锟剿达拷为锟姐）-锟斤拷锟斤拷锟劫度ｏ拷锟斤拷锟揭憋拷锟斤拷锟斤拷之锟酵ｏ拷
		Encoder_bias *= 0.84;		                                          //一锟阶碉拷通锟剿诧拷锟斤拷
		Encoder_bias += Encoder_Least*0.16;	                              //一锟阶碉拷通锟剿诧拷锟斤拷锟斤拷锟斤拷锟斤拷锟劫度变化
		Encoder_Integral +=Encoder_bias;                                  //锟斤拷锟街筹拷位锟斤拷 锟斤拷锟斤拷时锟戒：10ms
		Encoder_Integral=Encoder_Integral+Movement;                       //锟斤拷锟斤拷遥锟斤拷锟斤拷锟斤拷锟捷ｏ拷锟斤拷锟斤拷前锟斤拷锟斤拷锟斤拷
		if(Encoder_Integral>10000)  	Encoder_Integral=10000;             //锟斤拷锟斤拷锟睫凤拷
		if(Encoder_Integral<-10000)	  Encoder_Integral=-10000;            //锟斤拷锟斤拷锟睫凤拷
		velocity=-Encoder_bias*Velocity_Kp/100-Encoder_Integral*Velocity_Ki/100;     //锟劫度匡拷锟斤拷
		if(Turn_Off(Angle_Balance,Voltage)==1||Flag_Stop==1) Encoder_Integral=0;//锟斤拷锟斤拷乇蘸锟斤拷锟斤拷锟斤拷锟斤拷
	  return velocity;
}
/**************************************************************************
Function: Turn control
Input   : Z-axis angular velocity
Output  : Turn control PWM
锟斤拷锟斤拷锟斤拷锟杰ｏ拷转锟斤拷锟斤拷锟 
锟斤拷诓锟斤拷锟斤拷锟絑锟斤拷锟斤拷锟斤拷锟斤拷
锟斤拷锟斤拷  值锟斤拷转锟斤拷锟斤拷锟絇WM
锟斤拷    锟竭ｏ拷锟斤拷趣锟狡硷拷锟斤拷锟斤拷莞锟斤拷锟斤拷锟睫癸拷司
**************************************************************************/
int Turn(float gyro)
{
	 static float Turn_Target,turn,Turn_Amplitude=54;
	 float Kp=Turn_Kp,Kd;			//锟睫革拷转锟斤拷锟劫度ｏ拷锟斤拷锟睫革拷Turn_Amplitude锟斤拷锟斤拷
	//===================遥锟斤拷锟斤拷锟斤拷锟斤拷转锟斤拷锟斤拷=================//
	 if(1==Flag_Left)	        Turn_Target=-Turn_Amplitude/Flag_velocity;
	 else if(1==Flag_Right)	  Turn_Target=Turn_Amplitude/Flag_velocity;
	 else Turn_Target=0;
	 if(1==Flag_front||1==Flag_back)  Kd=Turn_Kd;
	 else Kd=0;   //转锟斤拷锟绞憋拷锟饺★拷锟斤拷锟斤拷锟斤拷堑木锟斤拷锟  锟叫碉拷模锟斤拷PID锟斤拷思锟斤拷
  //===================转锟斤拷PD锟斤拷锟斤拷锟斤拷=================//
	 turn=Turn_Target*Kp/100+gyro*Kd/100;//锟斤拷锟絑锟斤拷锟斤拷锟斤拷锟角斤拷锟斤拷PD锟斤拷锟斤拷
	 return turn;								 				 //转锟斤拷PWM锟斤拷转为锟斤拷锟斤拷锟斤拷转为锟斤拷
}

/**************************************************************************
Function: Assign to PWM register
Input   : motor_left锟斤拷Left wheel PWM锟斤拷motor_right锟斤拷Right wheel PWM
Output  : none
锟斤拷锟斤拷锟斤拷锟杰ｏ拷锟斤拷值锟斤拷PWM锟侥达拷锟斤拷
锟斤拷诓锟斤拷锟斤拷锟斤拷锟斤拷锟絇WM锟斤拷锟斤拷锟斤拷PWM
锟斤拷锟斤拷  值锟斤拷锟斤拷
**************************************************************************/
void Set_Pwm(int motor_left,int motor_right)
{
  if(motor_left>0)	   { AIN1_SET;			AIN2_RESET;} //前锟斤拷
  else           		{AIN1_RESET;			AIN2_SET; } //锟斤拷锟斤拷
  PWMB=myabs(motor_left);
  if(motor_right>0)			{BIN1_SET;			BIN2_RESET;}	//前锟斤拷
  else 	        			{BIN1_RESET;			BIN2_SET;}   //锟斤拷锟斤拷
  PWMA=myabs(motor_right);
}
/**************************************************************************
Function: PWM limiting range
Input   : IN锟斤拷Input  max锟斤拷Maximum value  min锟斤拷Minimum value
Output  : Output
锟斤拷锟斤拷锟斤拷锟杰ｏ拷锟斤拷锟斤拷PWM锟斤拷值
锟斤拷诓锟斤拷锟斤拷锟絀N锟斤拷锟斤拷锟斤拷锟斤拷锟   max锟斤拷锟睫凤拷锟斤拷锟街   min锟斤拷锟睫凤拷锟斤拷小值
锟斤拷锟斤拷  值锟斤拷锟睫凤拷锟斤拷锟街 
**************************************************************************/
int PWM_Limit(int IN,int max,int min)
{
	int OUT = IN;
	if(OUT>max) OUT = max;
	if(OUT<min) OUT = min;
	return OUT;
}
/**************************************************************************
Function: Press the key to modify the car running state
Input   : none
Output  : none
锟斤拷锟斤拷锟斤拷锟杰ｏ拷锟斤拷锟斤拷锟睫革拷小锟斤拷锟斤拷锟斤拷状态
锟斤拷诓锟斤拷锟斤拷锟斤拷锟 
锟斤拷锟斤拷  值锟斤拷锟斤拷
**************************************************************************/
void Key(void)
{
	u8 tmp,tmp2;
	tmp=click_N_Double(50);
	if(tmp==1)
	{
		Flag_Stop=!Flag_Stop;
	}		//锟斤拷锟斤拷锟斤拷锟斤拷小锟斤拷锟斤拷锟斤拷停
	tmp2=Long_Press();
  if(tmp2==1)
  {
	Flag_Show=!Flag_Show;	//锟斤拷锟斤拷锟斤拷锟狡斤拷锟斤拷锟斤拷位锟斤拷模式锟斤拷小锟斤拷锟斤拷锟斤拷示停止

	if(Flag_Show) RetargetInit(&huart1);
	else 		  RetargetInit(&huart3);
  }

}
/**************************************************************************
Function: If abnormal, turn off the motor
Input   : angle锟斤拷Car inclination锟斤拷voltage锟斤拷Voltage
Output  : 1锟斤拷abnormal锟斤拷0锟斤拷normal
锟斤拷锟斤拷锟斤拷锟杰ｏ拷锟届常锟截闭碉拷锟 
锟斤拷诓锟斤拷锟斤拷锟絘ngle锟斤拷小锟斤拷锟斤拷牵锟絭oltage锟斤拷锟斤拷压
锟斤拷锟斤拷  值锟斤拷1锟斤拷锟届常  0锟斤拷锟斤拷锟斤拷
**************************************************************************/
u8 Turn_Off(float angle, int voltage)
{
	u8 temp;
	if(angle<-40||angle>40||1==Flag_Stop)//锟斤拷氐锟窖癸拷锟斤拷锟 10V锟截闭碉拷锟 
	{	                                                 //锟斤拷谴锟斤拷锟 40锟饺关闭碉拷锟 
		temp=1;                                          //Flag_Stop锟斤拷1锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟狡关闭碉拷锟 
		AIN1_RESET;
		AIN2_RESET;
		BIN1_RESET;
		BIN2_RESET;
	}
	else
		temp=0;
	return temp;
}

///**************************************************************************
//Function: Get angle
//Input   : way：The algorithm of getting angle 1：DMP  2：kalman  3：Complementary filtering
//Output  : none
//函数功能：获取角度
//入口参数：way：获取角度的算法 1：DMP  2：卡尔曼 3：互补滤波
//返回  值：无
//**************************************************************************/
void Get_Angle(u8 way)
{
	short Accel_Y,Accel_Z,Accel_X,Accel_Angle_x,Accel_Angle_y,Gyro_X,Gyro_Z,Gyro_Y;
  	float gyro_x,gyro_y,accel_x,accel_y,accel_z;
	//Temperature=Read_Temperature();      //读取MPU6050内置温度传感器数据，近似表示主板温度。
	if(way==1)                           //DMP的读取在数据采集中断读取，严格遵循时序要求
	{
		Read_DMP();                      	 //读取加速度、角速度、倾角
		Angle_Balance=Pitch;             	 //更新平衡倾角,前倾为正，后倾为负
		Gyro_Balance=gyro[0];              //更新平衡角速度,前倾为正，后倾为负
		Gyro_Turn=gyro[2];                 //更新转向角速度
		Acceleration_Z=accel[2];           //更新Z轴加速度计
	}
	else
	{
		Gyro_X=(I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_XOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_XOUT_L);    //读取X轴陀螺仪
		Gyro_Y=(I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_YOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_YOUT_L);    //读取Y轴陀螺仪
		Gyro_Z=(I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_ZOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_ZOUT_L);    //读取Z轴陀螺仪
		Accel_X=(I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_XOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_XOUT_L); //读取X轴加速度计
		Accel_Y=(I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_YOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_YOUT_L); //读取X轴加速度计
		Accel_Z=(I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_ZOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_ZOUT_L); //读取Z轴加速度计
		Gyro_Balance=-Gyro_X;                            //更新平衡角速度
		accel_x=Accel_X/16384;                           //MPU6050初始化为加速度计为±2g，得到的原始数据为16位数据，并且最高位为符号位，
		accel_y=Accel_Y/16384;                           //所以读取到的数据位±32768，对应着2g，所以16384就是数据分辨率，原始数据除以16384得到数据为m/S^2
		accel_z=Accel_Z/16384;
		gyro_x=Gyro_X/16.4;                              //陀螺仪量程转换，量程±2000°/s对应灵敏度16.4，得到原始数据为整幅32768，对应±2000°/s
		gyro_y=Gyro_Y/16.4;                              //所以32768/2000 = 16.4，也可查看手册得到该数据
		if(Way_Angle==2)		  	
		{
			 Pitch= -Kalman_Filter_x(Accel_Angle_x,gyro_x);//卡尔曼滤波，单位为度
			 Roll = -Kalman_Filter_y(Accel_Angle_y,gyro_y);
		}
		else if(Way_Angle==3) 
		{  
			 Pitch = -Complementary_Filter_x(Accel_Angle_x,gyro_x);//互补滤波
			 Roll = -Complementary_Filter_y(Accel_Angle_y,gyro_y);
		}
		Angle_Balance=Pitch;                              //更新平衡倾角
		Gyro_Turn=Gyro_Z;                                 //更新转向角速度
		Acceleration_Z=Accel_Z;                           //更新Z轴加速度计
	}
}
/**************************************************************************
Function: Absolute value function
Input   : a锟斤拷Number to be converted
Output  : unsigned int
锟斤拷锟斤拷锟斤拷锟杰ｏ拷锟斤拷锟斤拷值锟斤拷锟斤拷
锟斤拷诓锟斤拷锟斤拷锟絘锟斤拷锟斤拷要锟斤拷锟斤拷锟斤拷锟街碉拷锟斤拷锟 
锟斤拷锟斤拷  值锟斤拷锟睫凤拷锟斤拷锟斤拷锟斤拷
**************************************************************************/
int myabs(int a)
{
	int temp;
	if(a<0)  temp=-a;
	else temp=a;
	return temp;
}
/**************************************************************************
Function: Check whether the car is picked up
Input   : Acceleration锟斤拷Z-axis acceleration锟斤拷Angle锟斤拷The angle of balance锟斤拷encoder_left锟斤拷Left encoder count锟斤拷encoder_right锟斤拷Right encoder count
Output  : 1锟斤拷picked up  0锟斤拷No action
锟斤拷锟斤拷锟斤拷锟杰ｏ拷锟斤拷锟叫★拷锟斤拷欠锟斤拷锟斤拷锟 
锟斤拷诓锟斤拷锟斤拷锟紸cceleration锟斤拷z锟斤拷锟斤拷俣龋锟紸ngle锟斤拷平锟斤拷慕嵌龋锟絜ncoder_left锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟絜ncoder_right锟斤拷锟揭憋拷锟斤拷锟斤拷锟斤拷锟斤拷
锟斤拷锟斤拷  值锟斤拷1:小锟斤拷锟斤拷锟斤拷锟斤拷  0锟斤拷小锟斤拷未锟斤拷锟斤拷锟斤拷
**************************************************************************/
int Pick_Up(float Acceleration,float Angle,int encoder_left,int encoder_right)
{
	 static u16 flag,count0,count1,count2;
	 if(flag==0)                                                      //锟斤拷一锟斤拷
	 {
			if(myabs(encoder_left)+myabs(encoder_right)<30)               //锟斤拷锟斤拷1锟斤拷小锟斤拷锟接斤拷锟斤拷止
			count0++;
			else
			count0=0;
			if(count0>10)
			flag=1,count0=0;
	 }
	 if(flag==1)                                                      //锟斤拷锟斤拷诙锟斤拷锟 
	 {
			if(++count1>200)       count1=0,flag=0;                       //锟斤拷时锟斤拷锟劫等达拷2000ms锟斤拷锟斤拷锟截碉拷一锟斤拷
			if(Acceleration>26000&&(Angle>(-20+Middle_angle))&&(Angle<(20+Middle_angle)))   //锟斤拷锟斤拷2锟斤拷小锟斤拷锟斤拷锟斤拷0锟饺革拷锟斤拷锟斤拷锟斤拷锟斤拷
			flag=2;
	 }
	 if(flag==2)                                                       //锟斤拷锟斤拷锟斤拷
	 {
		  if(++count2>100)       count2=0,flag=0;                        //锟斤拷时锟斤拷锟劫等达拷1000ms
	    if(myabs(encoder_left+encoder_right)>70)                       //锟斤拷锟斤拷3锟斤拷小锟斤拷锟斤拷锟斤拷胎锟斤拷为锟斤拷锟斤拷锟斤拷锟斤到锟斤拷锟斤拷转锟斤拷
      {
				flag=0;
				return 1;                                                    //锟斤拷獾叫★拷锟斤拷锟斤拷锟斤拷锟 
			}
	 }
	return 0;
}
/**************************************************************************
Function: Check whether the car is lowered
Input   : The angle of balance锟斤拷Left encoder count锟斤拷Right encoder count
Output  : 1锟斤拷put down  0锟斤拷No action
锟斤拷锟斤拷锟斤拷锟杰ｏ拷锟斤拷锟叫★拷锟斤拷欠癖环锟斤拷锟 
锟斤拷诓锟斤拷锟斤拷锟狡斤拷锟角度ｏ拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷冶锟斤拷锟斤拷锟斤拷锟斤拷锟 
锟斤拷锟斤拷  值锟斤拷1锟斤拷小锟斤拷锟斤拷锟斤拷   0锟斤拷小锟斤拷未锟斤拷锟斤拷
**************************************************************************/
int Put_Down(float Angle,int encoder_left,int encoder_right)
{
	 static u16 flag,count;
	 if(Flag_Stop==0)                     //锟斤拷止锟斤拷锟 
			return 0;
	 if(flag==0)
	 {
			if(Angle>(-10+Middle_angle)&&Angle<(10+Middle_angle)&&encoder_left==0&&encoder_right==0) //锟斤拷锟斤拷1锟斤拷小锟斤拷锟斤拷锟斤拷0锟饺革拷锟斤拷锟斤拷
			flag=1;
	 }
	 if(flag==1)
	 {
		  if(++count>50)                     //锟斤拷时锟斤拷锟劫等达拷 500ms
		  {
				count=0;flag=0;
		  }
	    if(encoder_left>3&&encoder_right>3&&encoder_left<40&&encoder_right<40) //锟斤拷锟斤拷2锟斤拷小锟斤拷锟斤拷锟斤拷胎锟斤拷未锟较碉拷锟绞憋拷锟斤拷锟轿拷锟 
      {
				flag=0;
				flag=0;
				return 1;                         //锟斤拷獾叫★拷锟斤拷锟斤拷锟斤拷锟 
			}
	 }
	return 0;
}
/**************************************************************************
Function: Encoder reading is converted to speed (mm/s)
Input   : none
Output  : none
锟斤拷锟斤拷锟斤拷锟杰ｏ拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷转锟斤拷为锟劫度ｏ拷mm/s锟斤拷
锟斤拷诓锟斤拷锟斤拷锟斤拷锟 
锟斤拷锟斤拷  值锟斤拷锟斤拷
**************************************************************************/
void Get_Velocity_Form_Encoder(int encoder_left,int encoder_right)
{
	float Rotation_Speed_L,Rotation_Speed_R;						//锟斤拷锟阶拷锟   转锟斤拷=锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷5ms每锟轿ｏ拷*锟斤拷取频锟斤拷/锟斤拷频锟斤拷/锟斤拷锟劫憋拷/锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷
	Rotation_Speed_L = encoder_left*Control_Frequency/EncoderMultiples/Reduction_Ratio/Encoder_precision;
	Velocity_Left = Rotation_Speed_L*PI*Diameter_67;		//锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷俣锟 =转锟斤拷*锟杰筹拷
	Rotation_Speed_R = encoder_right*Control_Frequency/EncoderMultiples/Reduction_Ratio/Encoder_precision;
	Velocity_Right = Rotation_Speed_R*PI*Diameter_67;		//锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷俣锟 =转锟斤拷*锟杰筹拷
}
/**************************************************************************
Function: Select car running mode
Input   : encoder_left锟斤拷Left wheel encoder reading锟斤拷encoder_right锟斤拷Right wheel encoder reading
Output  : none
锟斤拷锟斤拷锟斤拷锟杰ｏ拷选锟斤拷小锟斤拷锟斤拷锟斤拷模式
锟斤拷诓锟斤拷锟斤拷锟絜ncoder_left锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟   encoder_right锟斤拷锟揭憋拷锟斤拷锟斤拷锟斤拷锟斤拷
锟斤拷锟斤拷  值锟斤拷锟斤拷
**************************************************************************/
void Choose(int encoder_left,int encoder_right)
{
	static int count;
	if(Flag_Stop==0)
		count = 0;
	if((Flag_Stop==1)&&(encoder_left<10))	//锟斤拷时停止锟斤拷锟斤拷锟街诧拷锟斤拷
	{
		count += myabs(encoder_right);
		if(count>6&&count<180)	//锟斤拷通模式
		{
			Flag_follow = 0;
			Flag_avoid = 0;
		}
		if(count>180&&count<360)	//锟斤拷锟斤拷模式
		{
			Flag_avoid = 1;
			Flag_follow = 0;
		}
		if(count>360&&count<540)	//锟斤拷锟斤拷模式
		{
			Flag_avoid = 0;
			Flag_follow = 1;
		}
		if(count>540)
			count = 0;
	}
}



