/*
 \file		Robot101.c
 \author	${user}
 \date		${date}
 \brief		Simple Hello World! for the Ev3
*/

#include <ev3.h>

#define MAX_OBSTACLES   25     /*maximum number of obstacles*/


void RotateRobo(int deg,int speed)
{
	int motor_angle =(int) abs(deg) * 2.64;

	if(deg>0)
	{
		unsigned long req_time = motor_angle * 97 /speed;
		OnFwdReg(OUT_A,speed);
		OnFwdReg(OUT_B,-speed);
		OnFwdSync(OUT_A,speed);
		OnFwdSync(OUT_B,-speed);
		Wait(req_time);
		Off(OUT_A);
		Off(OUT_B);
//		int i =0;
//		int increment =10;
//		for(i=0;i<motor_angle;i+=increment)
//		{
//			RotateMotor(OUT_A,speed,increment);
//			RotateMotor(OUT_B,-speed,increment);
//		}
	}
	else
	{
		unsigned long req_time = motor_angle * 97 /speed;
		OnFwdReg(OUT_A,-speed);
		OnFwdReg(OUT_B,speed);
		OnFwdSync(OUT_A,-speed);
		OnFwdSync(OUT_B,speed);
		Wait(req_time);
		Off(OUT_A);
		Off(OUT_B);
	}
}


void DisplaceRobo(double distance, int speed)
{
	if(distance > 0.0)
	{
		unsigned long req_time = ((3.28 * 62000.0* fabs(distance))/speed);
		OnFwdReg(OUT_A,speed);
		OnFwdReg(OUT_B,speed);
		OnFwdSync(OUT_A,speed);
		OnFwdSync(OUT_B,speed);
		Wait(req_time);
	}
	else
	{
		unsigned long req_time = ((3.28 * 62000.0* fabs(distance))/speed);
		OnFwdReg(OUT_A,-speed);
		OnFwdReg(OUT_B,-speed);
		OnFwdSync(OUT_A,-speed);
		OnFwdSync(OUT_B,-speed);
		Wait(req_time);

	}

	Off(OUT_A);
	Off(OUT_B);

}



int main(void)
{
	//TODO Place here your variables
	int speed = 20;
	int num_obstacles = 13;                    /*number of obstacles*/
	double obstacle[MAX_OBSTACLES][2] =        /*obstacle locations*/{{0.61, 2.743},{0.915, 2.743},{1.219, 2.743},{1.829, 1.219},{1.829, 1.524},{ 1.829, 1.829}, {1.829, 2.134},{2.743, 0.305},{2.743, 0.61},{2.743, 0.915},{2.743, 2.743},{3.048, 2.743},{3.353, 2.743},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1}};
	double start[2] = {0.305, 1.219};          /*start location*/
	double goal[2] = {3.658, 1.829};           /*goal location*/

	double odometry[500][2]; // Stores position and orientation for each iteration.
	int i = 0;
	for(i= 0; i< 500;i++)
	{
		odometry[i][0]= 0;
		odometry[i][1]= 0;
	}


	InitEV3();

	//double distance = 0.2;
	//DisplaceRobo(distance,speed);
	RotateRobo(180,speed);




	Wait(SEC_1);

	FreeEV3();
	return 0;
}
