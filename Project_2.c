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


int precise_color(int data_count)
{
	int maxValue = 0, maxCount = 0, i, j;
	int measurement[data_count];
	for (i =0; i< data_count; i++)
	{
		measurement[i] = ReadSensor(IN_1);
		Wait(1);
	}

	   for (i = 0; i < data_count; ++i) {
	      int count = 0;

	      for (j = 0; j < data_count; ++j) {
	         if (measurement[j] == measurement[i])
	         ++count;
	      }

	      if (count > maxCount) {
	         maxCount = count;
	         maxValue = measurement[i];
	      }
	   }

	   return maxValue;
}


void goal_finding(){
	int distance;
	SetAllSensorMode(COL_COLOR,US_DIST_MM,NO_SEN,NO_SEN);

	SetLedPattern(LED_GREEN);

	int max_angle = 45;
	distance = readSensor(IN_2);
	int i;
	//203.2 in mm  is 8 inches
	//when we sense the red tape we are within 6 inches from the goal
	bool goal = FALSE;
	RotateRobo(-50,10);
	Wait(100);
	for (i = 0; i < 8 ;i++){
		RotateRobo(10,10);
		Wait(500);
		distance = readSensor(IN_2);
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
			distance = readSensor(IN_2);
			if (distance < 203.2)
			{
				DisplaceRobo(1.5,10);
				break;
			}

	}
}



}

//Write wall following behavior.
void wall_following(){
//rotate robo +90 degrees once function is called
// so that we are no longer facing the wall

//green = wall_following
SetLedPattern(LED_GREEN);
char color;
SetAllSensorMode(COL_COLOR,US_DIST_MM,NO_SEN,NO_SEN);
//int distance;



//rotate about our Original Angle.
//RotateRobo(90,SPEED);

while (!isExitButtonPressed()){


	Wait(100);
	color = precise_color(100);
	TermPrintf("Color - %d",color);
	// if our  color = blue then we know we are still on our wall
	if (color == 2 || color == 3)
	{
		Wait(1000);
		RotateRobo(90,10);
		LcdClean();
		TermPrintf("Color - %d",color);
		TermPrintf("Following the Wall");
		DisplaceRobo(1,SPEED);
		color = precise_color(100);
		//IF we hit a wall during our displacement we will turn to our original angle to continue along the corner.
		if  (color == 2 || color == 3){
			RotateRobo(90,10);
			DisplaceRobo(1,SPEED);
		}
	}
	else {
		DisplaceRobo(1,SPEED);
		RotateRobo(-90,10);
		wandering();

	}
	// rotating back towards the wall to make sure we are still following it.
	RotateRobo(-90,10);
}
//moves forward since there is no wall
//DisplaceRobo(1,SPEED);

}
//Write wandering behavior for the robot.
//Wandering behavior looks for either a wall or the object and then calls the corresponding behavior functions.
void wandering()
{
	 //BLACK = wandering
	 SetLedPattern(LED_BLACK);
	 SetAllSensorMode(COL_COLOR,US_DIST_MM,NO_SEN,NO_SEN);
	 int color;
	 int distance;
     color = precise_color(100);
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
     while (!isExitButtonPressed() )
	{

		TermPrintf("Wandering");
		Wait(1000);
		color = precise_color(100);
		LcdClean();
		//DisplaceRobo(1,SPEED);

	if (color == 2 || color ==3 )
	{
		TermPrintf("ON THE WALL!!");
		//Wait(1);
		Off(OUT_AB);
		wall_following();
	}
	else if (color == 5)
	{
		TermPrintf("ON THE OBSTACLE!!");
		//Wait(1);
		Off(OUT_AB);
		goal_finding();
		break;

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
//	char color;
//    color = readSensor(IN_1);

//	TermPrintf("%d",color);

	SetAllSensorMode(COL_COLOR,US_DIST_MM,NO_SEN,NO_SEN);


	InitEV3();
	wandering();

//	while (!isExitButtonPressed())
//    {
//		int color = precise_color(25);
////		TermPrintf("Color: %d",color);
//		switch(color){
//					case 0 :
//						TermPrintf("%d - Transparent",color);
//						break;
//					case 1 :
//						TermPrintf("%d - Black",color);
//						break;
//					case 2 :
//						TermPrintf("%d - Blue",color);
//						break;
//					case 3 :
//						TermPrintf("%d - Green",color);
//						break;
//					case 4 :
//						TermPrintf("%d - Yellow",color);
//						break;
//					case 5 :
//						TermPrintf("%d - Red", color);
//						break;
//					case 6 :
//						TermPrintf("%d - White", color);
//						break;
//					case 7 :
//						TermPrintf("%d - Brown",color);
//						break;
//					default :
//						TermPrintf("No Default Color : %d",color);
//						break;
//					}
//				Wait(1000);
//	         	LcdClean();
//    }


	//wall_following();
	//wandering();
	//wall_following();

	//goal_finding();


//	Testing our sensors to make sure they work.
//	while (!isExitButtonPressed())
//	{
//		    SetAllSensorMode(COL_COLOR,US_DIST_MM,NO_SEN,NO_SEN);
//			char color;
//			int distance;
//			color = readSensor(IN_1);
//			//LcdPrintf("Color : %c",color);
//			//TermPrintf("Color: %d",color);
//			distance = readSensor(IN_2);
//			//TermPrintf("Distance : %d",distance);
//		//	Wait(1000);
//		//	LcdClean();
//
//
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
