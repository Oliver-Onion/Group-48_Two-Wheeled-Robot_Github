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
函数功能：控制函数，通过控制函数的调用来实现小车的各种运动状态，
         5ms进行一次调用，由MPU6050的INT引脚触发，
         严格保证运算周期，保证系统的实时性和稳定性
参数说明：无
返 回 值：无
**************************************************************************/
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	static u8 Flag_Target;																//标志位，用来确定读取编码器的目标，控制读取编码器的时机
	int Encoder_Left,Encoder_Right;             					//左右编码器的脉冲计数
	int Balance_Pwm,Velocity_Pwm,Turn_Pwm;		  					//平衡环PWM变量，速度环PWM变量，转向环PWM变量
	if(GPIO_Pin==MPU6050_INT_Pin)
	{
		Flag_Target=!Flag_Target;
		Get_Angle(Way_Angle);                     					//更新姿态，5ms一次，更新姿态的采样频率是200Hz，能够很好的满足系统需求
		Encoder_Left=-Read_Encoder(3);            					//读取左轮编码器的值，前进为负值，后退为正值
		Encoder_Right=-Read_Encoder(4);           					//读取右轮编码器的值，前进为负值，后退为正值
																												//这个编码器的读取方向是由编码器安装方向和电机转动方向决定的，在使用前需要结合实际情况调整
		Get_Velocity_Form_Encoder(Encoder_Left,Encoder_Right);//通过编码器计算车轮转速，单位：mm/s
		Key();                                    					//扫描按键状态，用来改变小车的运行状态
		Balance_Pwm=Balance(Angle_Balance,Gyro_Balance);    //平衡PID控制 Gyro_Balance平衡角速度极性：前倾为正，后倾为负
		Velocity_Pwm=Velocity(Encoder_Left,Encoder_Right);  //速度环PID控制	控制小车前进后退及其速度，通过控制期望速度来控制小车运动
		Turn_Pwm=Turn(Gyro_Turn);														//转向PID控制

		Motor_Left=Balance_Pwm+Velocity_Pwm+Turn_Pwm;       //计算左轮电机最终PWM
		Motor_Right=Balance_Pwm+Velocity_Pwm-Turn_Pwm;      //计算右轮电机最终PWM
																												//PWM值正负决定电机转向，PWM绝对值决定电机转速
		Motor_Left=PWM_Limit(Motor_Left,6900,-6900);
		Motor_Right=PWM_Limit(Motor_Right,6900,-6900);			//PWM限幅
		if(Turn_Off(Angle_Balance,Voltage)==0)     					//检查小车是否异常
			Set_Pwm(Motor_Left,Motor_Right);         					//赋值给PWM寄存器
		__HAL_GPIO_EXTI_CLEAR_IT(MPU6050_INT_Pin);					//清除中断标志位
	 }
}

/**************************************************************************
Function: Vertical PD control
Input   : Angle:angle，Gyro：angular velocity
Output  : balance：Vertical control PWM
函数功能：直立PD控制
入口参数：Angle:角度，Gyro：角速度
返 回 值：balance：直立控制PWM
**************************************************************************/
int Balance(float Angle,float Gyro)
{
   float Angle_bias,Gyro_bias;
	 int balance;
	 Angle_bias=Middle_angle-Angle;                       				//求出平衡的角度中值与机械中值的差
	 Gyro_bias=0-Gyro;
	 balance=-Balance_Kp/100*Angle_bias-Gyro_bias*Balance_Kd/100; //计算平衡控制的电机PWM  PD控制   kp为P系数 kd为D系数
	 return balance;
}

/**************************************************************************
Function: Speed PI control
Input   : encoder_left：Left wheel encoder reading，encoder_right：Right wheel encoder reading
Output  : Speed control PWM
函数功能：速度控制PWM
入口参数：encoder_left：左轮编码器读数，encoder_right：右轮编码器读数
返 回 值：速度控制PWM
**************************************************************************/
//这里利用前进的时候车轮速度和期望速度Target_Velocity做差，能比较好的实现控制效果
int Velocity(int encoder_left,int encoder_right)
{
    static float velocity,Encoder_Least,Encoder_bias,Movement;
	  static float Encoder_Integral,Target_Velocity;
	  //================遥控前进后退部分====================//
		if(Flag_follow==1||Flag_avoid==1) Target_Velocity = 30; //跟随/避障模式，速度较低
		else 											        Target_Velocity = 50;
		if(Flag_front==1)    	Movement=Target_Velocity/Flag_velocity;	  //前进标志位置1
		else if(Flag_back==1)	Movement=-Target_Velocity/Flag_velocity;  //后退标志位置1
	  else  Movement=0;

   //=============超声波跟随部分：跟随/避障==================//
	  if(Flag_follow==1&&(Distance>200&&Distance<500)&&Flag_Left!=1&&Flag_Right!=1) //跟随
			 Movement=Target_Velocity/Flag_velocity;
		if(Flag_follow==1&&Distance<200&&Flag_Left!=1&&Flag_Right!=1)
			 Movement=-Target_Velocity/Flag_velocity;
		if(Flag_avoid==1&&Distance<450&&Flag_Left!=1&&Flag_Right!=1)  //避障模式运行
			 Movement=-Target_Velocity/Flag_velocity;

   //================速度PI控制器=====================//
		Encoder_Least =0-(encoder_left+encoder_right);                    //获取最新速度偏差=目标速度（此处为零）-实际速度（左右编码器之和）
		Encoder_bias *= 0.84;		                                          //一阶低通滤波器
		Encoder_bias += Encoder_Least*0.16;	                              //一阶低通滤波器，减缓速度变化
		Encoder_Integral +=Encoder_bias;                                  //积分处理，注意采样时间：10ms
		Encoder_Integral=Encoder_Integral+Movement;                       //接收遥控器数据，直接控制前进后退
		if(Encoder_Integral>10000)  	Encoder_Integral=10000;             //积分限幅
		if(Encoder_Integral<-10000)	  Encoder_Integral=-10000;            //积分限幅
		velocity=-Encoder_bias*Velocity_Kp/100-Encoder_Integral*Velocity_Ki/100;     //速度控制
		if(Turn_Off(Angle_Balance,Voltage)==1||Flag_Stop==1) Encoder_Integral=0;//电机关闭后清除积分
	  return velocity;
}
/**************************************************************************
Function: Turn control
Input   : Z-axis angular velocity
Output  : Turn control PWM
函数功能：转向控制 
入口参数：Z轴角速度
返 回 值：转向控制PWM
备    注：转向控制调试相对简单，建议在掌握前两个控制环之后再调试
**************************************************************************/
int Turn(float gyro)
{
	 static float Turn_Target,turn,Turn_Amplitude=54;
	 float Kp=Turn_Kp,Kd;			//设置转向速度，设置转向幅度Turn_Amplitude
	//===================遥控左右转向部分=================//
	 if(1==Flag_Left)	        Turn_Target=-Turn_Amplitude/Flag_velocity;
	 else if(1==Flag_Right)	  Turn_Target=Turn_Amplitude/Flag_velocity;
	 else Turn_Target=0;
	 if(1==Flag_front||1==Flag_back)  Kd=Turn_Kd;
	 else Kd=0;   //转向的时候取消陀螺仪的纠正 实现原地转向的算法  只有前进后退的时候才PID纠正
  //===================转向PD控制器=================//
	 turn=Turn_Target*Kp/100+gyro*Kd/100;//结合Z轴陀螺仪进行PD控制
	 return turn;								 				 //转向PWM，正值为左转，负值为右转
}

/**************************************************************************
Function: Assign to PWM register
Input   : motor_left：Left wheel PWM，motor_right：Right wheel PWM
Output  : none
函数功能：赋值给PWM寄存器
入口参数：左轮PWM，右轮PWM
返 回 值：无
**************************************************************************/
void Set_Pwm(int motor_left,int motor_right)
{
  if(motor_left>0)	   { AIN1_SET;			AIN2_RESET;} //前进
  else           		{AIN1_RESET;			AIN2_SET; } //后退
  PWMB=myabs(motor_left);
  if(motor_right>0)			{BIN1_SET;			BIN2_RESET;}	//前进
  else 	        			{BIN1_RESET;			BIN2_SET;}   //后退
  PWMA=myabs(motor_right);
}
/**************************************************************************
Function: PWM limiting range
Input   : IN：Input  max：Maximum value  min：Minimum value
Output  : Output
函数功能：限制PWM赋值
入口参数：IN：输入参数   max：限幅最大值   min：限幅最小值
返 回 值：限幅后的值 
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
函数功能：按键修改小车运行状态
入口参数：无 
返 回 值：无
**************************************************************************/
void Key(void)
{
	u8 tmp,tmp2;
	tmp=click_N_Double(50);
	if(tmp==1)
	{
		Flag_Stop=!Flag_Stop;
	}		//单击控制小车的启动和停止
	tmp2=Long_Press();
  if(tmp2==1)
  {
	Flag_Show=!Flag_Show;	//长按控制OLED显示参数模式和小车运行显示停止

	if(Flag_Show) RetargetInit(&huart1);
	else 		  RetargetInit(&huart3);
  }

}
/**************************************************************************
Function: If abnormal, turn off the motor
Input   : angle：Car inclination，voltage：Voltage
Output  : 1：abnormal，0：normal
函数功能：异常关闭电机 
入口参数：angle：小车倾斜角，voltage：电压
返 回 值：1：异常  0：正常
**************************************************************************/
u8 Turn_Off(float angle, int voltage)
{
	u8 temp;
	if(angle<-40||angle>40||1==Flag_Stop)//电池电压低于10V关闭电机 
	{	                                                 //倾斜角度大于40度关闭电机 
		temp=1;                                          //Flag_Stop置1关闭电机
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
Input   : a：Number to be converted
Output  : unsigned int
函数功能：绝对值函数
入口参数：a：要取绝对值的数 
返 回 值：绝对值结果
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
Input   : Acceleration：Z-axis acceleration，Angle：The angle of balance，encoder_left：Left encoder count，encoder_right：Right encoder count
Output  : 1：picked up  0：No action
函数功能：检测小车是否被拿起 
入口参数：Acceleration：z轴加速度，Angle：平衡角度，encoder_left：左编码器读数，encoder_right：右编码器读数
返 回 值：1:小车被拿起  0：小车未被拿起
**************************************************************************/
int Pick_Up(float Acceleration,float Angle,int encoder_left,int encoder_right)
{
	 static u16 flag,count0,count1,count2;
	 if(flag==0)                                                      //第一步
	 {
			if(myabs(encoder_left)+myabs(encoder_right)<30)               //条件1：小车接近静止
			count0++;
			else
			count0=0;
			if(count0>10)
			flag=1,count0=0;
	 }
	 if(flag==1)                                                      //第二步 
	 {
			if(++count1>200)       count1=0,flag=0;                       //超时等待，超过2000ms回到第一步
			if(Acceleration>26000&&(Angle>(-20+Middle_angle))&&(Angle<(20+Middle_angle)))   //条件2：小车被拿起0度左右，Z轴加速度大
			flag=2;
	 }
	 if(flag==2)                                                       //第三步
	 {
		  if(++count2>100)       count2=0,flag=0;                        //超时等待，超过1000ms
	    if(myabs(encoder_left+encoder_right)>70)                       //条件3：小车车轮被拨动，车轮有明显转动
      {
				flag=0;
				return 1;                                                    //检测到小车被拿起 
			}
	 }
	return 0;
}
/**************************************************************************
Function: Check whether the car is lowered
Input   : The angle of balance，Left encoder count，Right encoder count
Output  : 1：put down  0：No action
函数功能：检测小车是否被放下 
入口参数：平衡角度，左编码器读数，右编码器读数 
返 回 值：1：小车被放下   0：小车未被放下
**************************************************************************/
int Put_Down(float Angle,int encoder_left,int encoder_right)
{
	 static u16 flag,count;
	 if(Flag_Stop==0)                     //没有关闭电机 
			return 0;
	 if(flag==0)
	 {
			if(Angle>(-10+Middle_angle)&&Angle<(10+Middle_angle)&&encoder_left==0&&encoder_right==0) //条件1：小车倾角接近0度且编码器无转动
			flag=1;
	 }
	 if(flag==1)
	 {
		  if(++count>50)                     //超时等待，超过 500ms
		  {
				count=0;flag=0;
		  }
	    if(encoder_left>3&&encoder_right>3&&encoder_left<40&&encoder_right<40) //条件2：小车车轮未被拨动，有微小转动即可 
      {
				flag=0;
				flag=0;
				return 1;                         //检测到小车被放下 
			}
	 }
	return 0;
}
/**************************************************************************
Function: Encoder reading is converted to speed (mm/s)
Input   : none
Output  : none
函数功能：编码器读数转换为速度（mm/s）
入口参数：无 
返 回 值：无
**************************************************************************/
void Get_Velocity_Form_Encoder(int encoder_left,int encoder_right)
{
	float Rotation_Speed_L,Rotation_Speed_R;						//转速   转速=编码器读数*采样频率5ms每次*读取频率/编码器倍频/减速比/编码器精度
	Rotation_Speed_L = encoder_left*Control_Frequency/EncoderMultiples/Reduction_Ratio/Encoder_precision;
	Velocity_Left = Rotation_Speed_L*PI*Diameter_67;		//车轮线速度 =转速*周长
	Rotation_Speed_R = encoder_right*Control_Frequency/EncoderMultiples/Reduction_Ratio/Encoder_precision;
	Velocity_Right = Rotation_Speed_R*PI*Diameter_67;		//车轮线速度 =转速*周长
}
/**************************************************************************
Function: Select car running mode
Input   : encoder_left：Left wheel encoder reading，encoder_right：Right wheel encoder reading
Output  : none
函数功能：选择小车运行模式
入口参数：encoder_left：左编码器读数   encoder_right：右编码器读数
返 回 值：无
**************************************************************************/
void Choose(int encoder_left,int encoder_right)
{
	static int count;
	if(Flag_Stop==0)
		count = 0;
	if((Flag_Stop==1)&&(encoder_left<10))	//停止且左轮基本不转
	{
		count += myabs(encoder_right);
		if(count>6&&count<180)	//普通模式
		{
			Flag_follow = 0;
			Flag_avoid = 0;
		}
		if(count>180&&count<360)	//避障模式
		{
			Flag_avoid = 1;
			Flag_follow = 0;
		}
		if(count>360&&count<540)	//跟随模式
		{
			Flag_avoid = 0;
			Flag_follow = 1;
		}
		if(count>540)
			count = 0;
	}
}



