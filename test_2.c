/*
 \file		Project_2.c
 \author	${user}
 \date		${date}
 \brief		Simple Hello World! for the Ev3
*/

#include <ev3.h>

#define SPEED 20

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


void DisplaceRobo(double distance, int speed)
{
	if(distance > 0.0)
	{
		unsigned long req_time = (( 62000.0* fabs(distance))/speed);
		OnFwdReg(OUT_AB,speed);
		//OnFwdReg(OUT_AB,speed);
		OnFwdSync(OUT_AB,speed);
		OnFwdSync(OUT_AB,speed);
		//LcdPrintf("%c",MotorPower(OUT_ALL));
		Wait(req_time);
	}
	else
	{
		unsigned long req_time = (( 62000.0* fabs(distance))/speed);
		OnFwdReg(OUT_A,-speed);
		OnFwdReg(OUT_B,-speed);
		OnFwdSync(OUT_A,-speed);
		OnFwdSync(OUT_B,-speed);
		Wait(req_time);

	}
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
//Write wandering behavior for the robot.
//Wandering behavior looks for either a wall or the object and then calls the corresponding behavior functions.
void wandering()
{
	 SetLedPattern(LED_RED);
	 SetAllSensorMode(COL_COLOR,US_DIST_MM,NO_SEN,NO_SEN);
	 int color;
	 int distance;
     color = ReadSensor(IN_1);
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
		TermPrintf("Wandering");
		//Wait(1);
		LcdClean();
		//DisplaceRobo(1,SPEED);

	if (color == 2 || color ==3 )
	{
		TermPrintf("ON THE WALL!!");
		//Wait(1);
		Off(OUT_AB);
		break;
		//call wall_following behavior
	}
	else if (color == 5)
	{
		TermPrintf("ON THE OBSTACLE!!");
		//Wait(1);
		Off(OUT_AB);
		break;
		//call obstacle clearing behavior
	}

	}

}

int main(void)
{

	// INFO This code template works only with recent versions of the API. If TermPrintln is not found,
	//      please update the API or use the "Hello World EV3 Project Old API" as template.


	//TODO Place here your variables


	//TODO Place here your code
//	TermPrintln("Hello World!");
//	TermPrintf("Press ENTER to exit");
//	ButtonWaitForPress(BUTTON_ID_ENTER);

//	SetAllSensorMode(COL_COLOR,US_DIST_MM,NO_SEN,NO_SEN);
//	int color;
//	color = readSensor(IN_1);

//	TermPrintf("%d",color);


	InitEV3();
	//wandering();
	wall_following();
	//Testing our sensors to make sure they work.
//	while (!isExitButtonPressed())
//	{
//		    SetAllSensorMode(COL_COLOR,US_DIST_MM,NO_SEN,NO_SEN);
//			int color;
//			int distance;
//			color = readSensor(IN_1);
//			distance = readSensor(IN_2);
//			switch(color){
//			case 0 :
//				TermPrintf("Transparent");
//				break;
//			case 1 :
//				TermPrintf("Black");
//				break;
//			case 2 :
//				TermPrintf("Blue");
//				break;
//			case 3 :
//				TermPrintf("Green");
//				break;
//			case 4 :
//				TermPrintf("Yellow");
//				break;
//			case 5 :
//				TermPrintf("Red");
//				break;
//			case 6 :
//				TermPrintf("White");
//				break;
//			case 7 :
//				TermPrintf("Brown");
//				break;
//			default :
//				TermPrintf("No Default Color : %d",color);
//				break;
//			}
//			//TermPrintf("\n\nDistance - %d",distance);
//			Wait(1000);
//			LcdClean();
//
//	}


	FreeEV3();
	return 0;

}
