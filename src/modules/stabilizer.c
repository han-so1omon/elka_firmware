/**
 * File that receives IMU data, gains, pilot inputs and update control signals to actuators
 * ELKA control software, UMD
 */
#include "stm32f4xx_conf.h"
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"
#include "math.h"
#include "semphr.h"

#include "stabilizer.h"
#include "sensfusion6.h"
#include "imu.h"
#include "motors.h"
#include <stdbool.h>
#include "queue.h"
#include "hw_config.h"
#include "mpu6050.h"


/**
 * Defines in what divided update rate should the attitude
 * control loop run relative the rate control loop.
 */
#define ATTITUDE_UPDATE_RATE_DIVIDER  1
#define FUSION_UPDATE_DT  (float)(1.0 / (IMU_UPDATE_FREQ / ATTITUDE_UPDATE_RATE_DIVIDER)) //(float)(ATTITUDE_UPDATE_RATE_DIVIDER/configTICK_RATE_HZ) //
#define PID_GAINS 5
#define SENSITIVITY 6
#define TRIM 4

#define PRIVATE

Axis3f gyro; // Gyro axis data in deg/s
Axis3f acc;  // Accelerometer axis data in mG
Axis3i16   gyroMpu;
Axis3i16   accelMpu;
int8_t pitchkp = 4;
int8_t pitchkd = 80;
int8_t rollkp = -4;
int8_t rollkd = 80;
int8_t yawkp = 100;
int8_t pitchki, rollki;
float f_actuatorThrust, f_actuatorRoll, f_actuatorPitch, f_actuatorYaw;

PRIVATE float eulerRollActual;
PRIVATE float eulerPitchActual;
PRIVATE float eulerYawActual;
PRIVATE float eulerRollDesired;
PRIVATE float eulerPitchDesired;
PRIVATE float eulerYawDesired;
PRIVATE float rollRateDesired;
PRIVATE float pitchRateDesired;
PRIVATE float yawRateDesired;
PRIVATE float fusionDt;

int16_t actuatorThrust = 1000;
int16_t  actuatorRoll = 0;
int16_t  actuatorPitch = 0;
int16_t  actuatorYaw =0;
int16_t actuatorLeftServo = 1500;
int16_t actuatorRightServo = 1500;
int16_t baseline = 0;//1000;

int16_t trimThrust = 900; //900
int16_t trimRoll = 0;
int16_t trimPitch = 0;
int16_t trimYaw = 0;
int16_t trimLeftServo = 1500;
int16_t trimRightServo = 1500;
float eulerPitchDesired = 0.0;
int8_t t_sens = 10;//10
int8_t r_sens = 1;//3
int8_t p_sens = 1;//3
int8_t y_sens = 30;//30;
float thrust_sens = 1.0;
float roll_sens = 1.0;
float yaw_sens = 1.0;
float pitch_sens = 1.0;


uint32_t leftmotor;
uint32_t rightmotor;
uint32_t tail;
uint32_t ls;
uint32_t rs;

int32_t motorPowerLeftfront;
int32_t motorPowerRightfront;
int32_t motorPowerLeftrear;
int32_t motorPowerRightrear;


float theta;

static bool isInit;

static void distributePower(const uint16_t thrust, const int16_t roll,
		const int16_t pitch, const int16_t yaw, const int16_t lefts, const int16_t rights );
static uint16_t limitThrust(int32_t value);
static void stabilizerTask(void* param);
static void imuinit();

xQueueHandle  tx1Queue, eulerqueue,spektrumqueue;
xSemaphoreHandle queuewritten;

/* Initialize Motors, IMU, Sensor Fusion and Stabilizer Task */
void stabilizerInit(void)
{
	//if(isInit)
	//return;

	motorsInit();
	imu6Init();
	sensfusion6Init();
	rollRateDesired = 0;
	pitchRateDesired = 0;
	yawRateDesired = 0;
	//vSemaphoreCreateBinary(queuewritten);


	xTaskCreate(stabilizerTask, (const signed char * const)"STABILIZER",
			configMINIMAL_STACK_SIZE, NULL, /*Piority*/tskIDLE_PRIORITY, NULL);

	//isInit = TRUE;
}

static void stabilizerTask(void* param)
{
	bool i;
	uint16_t ratio;
	uint8_t data;
	uint8_t buffer[14];
	imuinit();
	int16_t ax,ay,az,gx,gy,gz;
	uint32_t lastWakeTime;
	lastWakeTime = xTaskGetTickCount ();
	uint32_t attitudeCounter = 0;
	int helicnt = 0;
	uint16_t start = 0;
	int16_t delay = 0;
	uint16_t delay_temp = 0;
	eulerstruct euler;
	rxpacket test;
	static int k=0;

	int bias = 0;
    int biascount = 0;
    int16_t spektrumbias[4]={0,0,0,0};
    int ii = 0;


	while (1)
	{
		if (bias==0 & spektrumchannel[0]>100)
		{
			for (biascount = 0; biascount<=100;biascount++)
			{
				for (ii=0; ii<=3; ii++)
				{
					spektrumbias[ii] = spektrumchannel[ii];
				}
			}
			bias = 1;
		}




		if (helicnt > 10)
		{
			helicnt = 0;
		}
		helicnt++;
		vTaskDelayUntil(&lastWakeTime,2);//F2T(IMU_UPDATE_FREQ)
		//start = TIM4->CNT;
		imu6Read(&gyro, &acc);
		//mpu6050GetMotion6(&accelMpu.x, &accelMpu.y, &accelMpu.z, &gyroMpu.x, &gyroMpu.y, &gyroMpu.z);
		if (++attitudeCounter >= ATTITUDE_UPDATE_RATE_DIVIDER)
		{
			sensfusion6UpdateQ(gyro.x, gyro.y, gyro.z, acc.x, acc.y, acc.z, FUSION_UPDATE_DT);
			sensfusion6GetEulerRPY(&eulerRollActual, &eulerPitchActual, &eulerYawActual);
			attitudeCounter = 0;
			euler.data[0] = (int)(eulerRollActual*100);
			euler.data[1] = (int)((eulerPitchActual)*100);
			euler.data[2] = (int)(eulerPitchDesired*100);
			//vDebugPrintf("%i\r\n",attitudeCounter);
		}
		xQueueReceive(tx1Queue,&test,0);



		if (test.data[1]==PID_GAINS && test.data[2]==255 && test.data[3] == 255)// && k==0)
		{

			pitchkp = (test.data[4]<<8)+test.data[5];
			pitchki = (test.data[6]<<8)+test.data[7];
			pitchkd = (test.data[8]<<8)+test.data[9];
			rollkp = (test.data[10]<<8)+test.data[11];
			rollki = (test.data[12]<<8)+test.data[13];
			rollkd = (test.data[14]<<8)+test.data[15];
			yawkp = (test.data[16]<<8)+test.data[17];
			/*pitchkp = test.data[4];
			pitchki = test.data[5];
			pitchkd = test.data[6];
			rollkp = test.data[7];
			rollki = test.data[8];
			rollkd = test.data[9];
			yawkp = test.data[10];*/
			k=1;


		}

		else if (test.data[1] == SENSITIVITY && test.data[2]==255 && test.data[3] == 255)// && k==1)
		{
			t_sens = (test.data[4]<<8)+test.data[5];
			r_sens = (test.data[6]<<8)+test.data[7];
			p_sens = (test.data[8]<<8)+test.data[9];
			y_sens = (test.data[10]<<8)+test.data[11];
			/*t_sens = test.data[4];
			r_sens = test.data[5];
			p_sens = test.data[6];
			y_sens = test.data[7];*/
			k=2;

		}
		else if (test.data[1] == TRIM && test.data[2]==255 && test.data[3] == 255)// && k==2)
		{
			trimThrust = (test.data[4]<<8)+test.data[5];
			trimRoll = (test.data[6]<<8)+test.data[7];
			trimPitch = (test.data[8]<<8)+test.data[9];
			trimYaw = (test.data[10]<<8)+test.data[11];
			trimLeftServo = (test.data[12]<<8)+test.data[13];
			trimRightServo = (test.data[14]<<8)+test.data[15];
			//eulerPitchDesired = (test.data[16]<<8)+test.data[17];
			k=3;
		}



		eulerPitchDesired = p_sens*(spektrumchannel[2]-spektrumbias[2])/10 + trimPitch; //use pitch stick to command attitude angle instead of actuator inputs.
		eulerRollDesired = r_sens*(spektrumchannel[1]-spektrumbias[1])/10 + trimRoll;
		f_actuatorThrust =  t_sens*(spektrumchannel[0]-spektrumbias[0])/10 + trimThrust;
		f_actuatorRoll =    rollkd*gyro.x*0.017 - rollkp*(eulerRollActual - eulerRollDesired);
		f_actuatorPitch =  pitchkd*gyro.y*0.017 - pitchkp*(eulerPitchActual - eulerPitchDesired);
		f_actuatorYaw =   y_sens*(spektrumchannel[3]-spektrumbias[3])/10 + trimYaw - yawkp*gyro.z*0.017;

		if(helicnt==10)
		{
			//STM_EVAL_LEDToggle(LED3);
			//vDebugPrintf("%i\r\n",accelMpu.z);
		}

		actuatorThrust = (int)(f_actuatorThrust); //(int16_t)(10*spektrumchannel[0]*0.1 + trimThrust);
		actuatorRoll = (int)(f_actuatorRoll);
		actuatorPitch = (int)(f_actuatorPitch);
		actuatorYaw = (int)(f_actuatorYaw);

		euler.data[3] = (int)(actuatorThrust);
		euler.data[4] = (int)(actuatorRoll);
		euler.data[5] = (int)(actuatorPitch);
		euler.data[6] = (int)(actuatorYaw);



		/*euler.data[3] = (int)(spektrumchannel[0]);
	    euler.data[4] = (int)(spektrumchannel[1]);
		euler.data[5] = (int)(spektrumchannel[2]);
		euler.data[6] = (int)(spektrumchannel[3]);*/
		xQueueSend(eulerqueue,&euler,0);


		if (actuatorThrust>1100)
		{
			distributePower(actuatorThrust, actuatorRoll, actuatorPitch, actuatorYaw, 0, 0);
		}
		else if(actuatorThrust<=1100 | spektrumchannel[0]<=100)
		{
			distributePower(1000,0,0,0,0,0);
		}
		//distributePower(1500, 0, 0, 0, 0, 0);



		//vTaskDelay(LED[1]/portTICK_RATE_MS);
		// vTaskDelay(10/portTICK_RATE_MS);
		//vTaskDelay(1000);

	}
}

static void distributePower(const uint16_t thrust, const int16_t roll,
		const int16_t pitch, const int16_t yaw, const int16_t lefts, const int16_t rights)
{

	//roll = roll >> 1;
	//pitch = pitch >> 1;
	/*  motorPowerLeft =  limitThrust(thrust + roll + pitch - yaw);
  motorPowerRight = limitThrust(thrust - roll - pitch - yaw);
  motorPowerFront = limitThrust(thrust - roll + pitch + yaw);
  motorPowerRear =  limitThrust(thrust + roll - pitch + yaw);*/
	motorPowerLeftfront =  limitThrust(thrust + roll - pitch - yaw);
	motorPowerRightfront = limitThrust(thrust - roll - pitch + yaw);
	motorPowerLeftrear = limitThrust(thrust + roll + pitch + yaw);
	motorPowerRightrear =  limitThrust(thrust - roll + pitch - yaw);


	motorsSetRatio(MOTOR_LEFTFRONT, motorPowerLeftfront);
	motorsSetRatio(MOTOR_RIGHTFRONT, motorPowerRightfront);
	motorsSetRatio(MOTOR_LEFTREAR, motorPowerLeftrear);
	motorsSetRatio(MOTOR_RIGHTREAR, motorPowerRightrear);
}

static uint16_t limitThrust(int32_t value)
{
	if(value > 2000)
	{
		value = 2000;
	}
	else if(value < 1000)
	{
		value = 1000;
	}

	return (uint16_t)value;
}

void imuinit(void)
{
	static int i;
	mpu6050Reset();
	//vTaskDelay(M2T(50));
	for (i=0; i<10000; i++);
	// Activate MPU6050
	mpu6050SetSleepEnabled(FALSE);
	// Enable temp sensor
	mpu6050SetTempSensorEnabled(TRUE);
	// Disable interrupts
	mpu6050SetIntEnabled(FALSE);
	// Connect the HMC5883L to the main I2C bus
	mpu6050SetI2CBypassEnabled(TRUE);
	// Set x-axis gyro as clock source
	mpu6050SetClockSource(MPU6050_CLOCK_PLL_XGYRO);
	// Set gyro full scale range
	mpu6050SetFullScaleGyroRange( MPU6050_GYRO_FS_2000);
	// Set accelerometer full scale range
	mpu6050SetFullScaleAccelRange(MPU6050_ACCEL_FS_8);


	// Set output rate (1): 1000 / (1 + 1) = 500Hz
	mpu6050SetRate(0); //mpu6050SetRate(1);
	// Set digital low-pass bandwidth. Set to 100 Hz for now.
	mpu6050SetDLPFMode(MPU6050_DLPF_BW_98);
}

