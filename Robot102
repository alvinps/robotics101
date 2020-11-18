/*
 \file		Project_2.c
 \author	${user}
 \date		${date}
 \brief		Simple Hello World! for the Ev3
*/

#include <ev3.h>

#define SPEED 20
#define BLUE 2
#define RED 4


bool isExitButtonPressed() {
    return ButtonIsDown(BTNEXIT);
}


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

int DisplaceRobo(double distance, int speed)
{
	int distance_mm = fabs(distance / 3.28) * 1000 ;
	TermPrintf(" MM (%d)",distance_mm);
	int i=0;

	for(i=0; i<= distance_mm; i+=5)
	{
		int color_read = ReadSensor(IN_1);

		if(color_read == BLUE )
		{
			return BLUE;
		}
		else if(color_read == RED )
		{
			return RED;
		}
		if(distance > 0.0)
			{
				unsigned long req_time = (( 62.0* 5)/speed);
				OnFwdReg(OUT_AB,speed);
				OnFwdSync(OUT_AB,speed);
				Wait(req_time);
				TermPrintf("Time (%f)",req_time);

			}
			else
			{
				unsigned long req_time =  (( 62000.0 * 3.28 /1000)/speed);
				OnFwdReg(OUT_AB,-speed);;
				OnFwdSync(OUT_AB,-speed);
				Wait(req_time);
			}
		Off(OUT_A);
		Off(OUT_B);
	}
	return ReadSensor(IN_1);
	}


//Write wall following behavior.
void wall_following(){
//rotate robo +90 degrees once function is called
// so that we are no longer facing the wall
RotateRobo(90,SPEED);
int color;
//int distance;

color = ReadSensor(IN_1);
bool on_wall = TRUE;
int i =0;
while (on_wall ){
	i++;
	DisplaceRobo(1,SPEED);
	// rotating back towards the wall to make sure we are still following it.
	RotateRobo(-90,10);
	Wait(1000);
	color = ReadSensor(IN_1);
	LcdPrintf(1,"Color - %d",color);
	// if our sensor does not detect that we are still following the wall then we will break from this while loop.
	if (color != 2|| color != 3)
	{
		on_wall = FALSE;
	}
//	//Rotate posotive 90 degrees so when the while loop runs again we will not go through the wall like the kool-aid man.
	RotateRobo(90,10);
	if(i==5)
	{
		break;
	}

}


}


void goal_finding(){
	int distance;
	int max_angle = 45;
	distance = ReadSensor(IN_2);
	int i;
	//203.2 in mm  is 8 inches
	//when we sense the red tape we are within 6 inches from the goal
	bool goal = FALSE;
	RotateRobo(-50,10);
	Wait(100);
	for (i = 0; i < 8 ;i++){
		RotateRobo(10,10);
		Wait(500);
		distance = ReadSensor(IN_2);
		if (distance < 203.2)
		{
			goal = TRUE;
			DisplaceRobo(1.5,10);
			break;
		}
	}

if (!goal){
	RotateRobo(50,10);
	for (i = 0; i < 8 ;i++){
			RotateRobo(-10,10);
			Wait(500);
			distance = ReadSensor(IN_2);
			if (distance < 203.2)
			{
				DisplaceRobo(1.5,10);
				break;
			}

	}
}



}





//Write wandering behavior for the robot.
//Wandering behavior looks for either a wall or the object and then calls the corresponding behavior functions.
int wandering()
{
	 int condition = -1;
     int color = ReadSensor(IN_1);
//	 distance = readSensor(IN_2);
//	 TermPrintf("Color - %d",color);
//	 TermPrintf("\n\nDistance - %d",distance);
//	 Wait(1000);
//	 LcdClean();

	 //while the sensor does not detect either blue or red
	 //2 = blue
	 //5 = red
     OnFwdReg(OUT_AB,15);
     		//OnFwdReg(OUT_AB,speed);
     OnFwdSync(OUT_AB,15);
     		//LcdPrintf("%c",MotorPower(OUT_ALL));
     while (color != 2 ||color != 3 || color !=5 )
	{
		color = ReadSensor(IN_1);

	if (color == 2 || color ==3 )
	{
		TermPrintf("ON THE WALL!!");
		Off(OUT_AB);
		condition = 0;

		break;


	}
	else if (color == 5)
	{
		TermPrintf("ON THE OBSTACLE!!");
		condition = 1;
		Off(OUT_AB);
		break;
		//call obstacle clearing behavior
	}

	}
return condition;
}

int main(void)
{
	InitEV3();
	SetAllSensorMode(COL_COLOR,US_DIST_MM,NO_SEN,NO_SEN);

	int goal_found = 0;
	DisplaceRobo(1,5);
//	while(goal_found != 1 )
//	{
//		wandering();
//		wall_following();
//
//
//
//
//	}
//	goal_finding();







	//Testing our sensors to make sure they work.
//	while (!isExitButtonPressed())
//	{

	FreeEV3();
	return 0;

}
