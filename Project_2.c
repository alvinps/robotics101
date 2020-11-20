#include <ev3.h>
#include <pthread.h>
#include <time.h>
//#include "robo_functions.c"


#define SPEED 10
#define BLUE 2
#define RED 5

#define WALL 2020
#define OBSTACLE 2021

int stop = 0;
int on_wall = 0;
int on_goal = 0;

void DisplaceRobo(double distance, int speed)
{
	if(distance > 0.0)
	{	// Positive Displacement of the robot. Forward
		unsigned long req_time = (( 62000.0* fabs(distance))/speed);
		OnFwdReg(OUT_AB,speed);
		OnFwdSync(OUT_AB,speed);
		Wait(req_time);
	}
	else
	{	// Negative Displacement of the robot. Backward
		unsigned long req_time = (( 62000.0* fabs(distance))/speed);
		OnFwdReg(OUT_AB,-speed);
		OnFwdSync(OUT_AB,-speed);
		Wait(req_time);
	}
	Off(OUT_A);
	Off(OUT_B);
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



bool isExitButtonPressed() {
    return ButtonIsDown(BTNEXIT);
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



void * color_reader(void* p )
{
	int time = *((int*) p) ;
	int i;
	clock_t start = clock();
	clock_t end ;

	int diff=0;
	while(1)
	{
		if(stop ==1)
			return NULL;

		int col = precise_color(25);
		if(col == RED || col == BLUE){
			Off(OUT_AB);
			if(col == RED) on_goal =1;

			stop = 1;
			return NULL;
		}
		diff =(int) (1000 * (clock() - start )/ CLOCKS_PER_SEC);
	}
	return NULL;
}


void *sonar_reader(void* p )
{
	int time = *((int*) p) ;
	ResetRotationCount(OUT_C);

		clock_t start = clock();

		int diff=0,distance , counter = 0, current = 0, previous = 0, flip =  0, add = 0;
		while(1)
		{
			if(stop == 1)
			{
				if(add > 0)
					RotateMotor(OUT_C,10,add);
				else
					RotateMotor(OUT_C,-10,-add);

				return NULL;
			}

			if(add > 120) flip = 1;
			if(add < -120) flip = 0;

			if(flip ==0)
			{
				add+=10;
				RotateMotor(OUT_C,-10,10);

			}
			else
			{
				add-=10;
				RotateMotor(OUT_C,10,10);
			}
			distance = ReadSensor(IN_4);
			if (distance < 500)
			{
				stop = 1 ;
				Off(OUT_AB);
				counter = MotorRotationCount(OUT_C);
				if(counter >0)
					RotateMotor(OUT_C,-10,counter);
				else
					RotateMotor(OUT_C,10,counter);
				RotateRobo(-counter,10);
				goalwandering();



				return NULL;
			}
			diff =(int) ( (clock() - start )/ CLOCKS_PER_SEC);
//			 TermPrintf("Diff - %d    %d\n",diff,time);
//			 Wait(1000);

		}

		if(add > 0)
			RotateMotor(OUT_C,10,add);
		else
			RotateMotor(OUT_C,-10,-add);
	return NULL;
}

void RotateRoboSens(int deg,int speed)
{
	pthread_t cid;
	int motor_angle =(int) abs(deg) * 2.64;

	if(deg>0)
	{
		unsigned long req_time = motor_angle * 97 /speed;
		int time = (int) req_time;
		pthread_create(&cid, NULL, color_reader, &time);
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
		int time = (int) req_time;
		pthread_create(&cid, NULL, color_reader, &time);
		OnFwdReg(OUT_A,-speed);
		OnFwdReg(OUT_B,speed);
		OnFwdSync(OUT_A,-speed);
		OnFwdSync(OUT_B,speed);
		Wait(req_time);
		Off(OUT_A);
		Off(OUT_B);
	}
	pthread_join(cid, NULL);

}




// Moves the robot and uses the sonar and color sensor at the same time.
void DisplaceRoboSens(double distance, int speed)
{
	pthread_t cid;
	pthread_t sid;
	if(distance > 0.0)
	{	// Positive Displacement of the robot. Forward
		unsigned long req_time = (( 62000.0* fabs(distance))/speed);
		int time = (int) req_time;
		pthread_create(&cid, NULL, color_reader, &time);
		pthread_create(&sid, NULL, sonar_reader, &time);
		OnFwdReg(OUT_AB,speed);
		OnFwdSync(OUT_AB,speed);
		Wait(req_time);
	}
	else
	{	// Negative Displacement of the robot. Backward
		unsigned long req_time = (( 62000.0* fabs(distance))/speed);
		int time = (int) req_time;
		pthread_create(&cid, NULL, color_reader, &time);
		pthread_create(&sid, NULL, sonar_reader, &time);
		OnFwdReg(OUT_AB,-speed);
		OnFwdSync(OUT_AB,-speed);
		Wait(req_time);
	}
	stop =1;
	Off(OUT_A);
	Off(OUT_B);
	pthread_join(cid, NULL);
	pthread_join(sid, NULL);
	stop =0;
}




int Sonar(int time)
{
	ResetRotationCount(OUT_C);

	clock_t start = clock();
	clock_t end ;
	int diff=0,distance, current = 0, previous = 0, flip =  0, add = 0;
	while(diff < time)
	{

		if(add > 120) flip = 1;
		if(add < -120) flip = 0;
		if(flip ==0)
		{
			add+=10;
			RotateMotor(OUT_C,-10,10);
		}
		else
		{
			add-=10;
			RotateMotor(OUT_C,10,10);
		}
		distance = ReadSensor(IN_4);
		if (distance < 700)
		{
			Off(OUT_AB);
			int counter = MotorRotationCount(OUT_C);
			RotateRobo(-counter,10);
			if(counter >0)
				RotateMotor(OUT_C,-10,counter);
			else
				RotateMotor(OUT_C,10,counter);
			break;
		}
		diff =(int) (1000 * (clock() - start )/ CLOCKS_PER_SEC);
	}
	return distance ;
}

void goal_finding(){
	int distance;
	SetLedPattern(LED_ORANGE);


	int max_angle = 45;
	distance = ReadSensor(IN_4);
	int i;
	//203.2 in mm  is 8 inches
	//when we sense the red tape we are within 6 inches from the goal
	bool goal = FALSE;
	RotateRobo(-50,10);
	Wait(100);
	for (i = 0; i < 8 ;i++){
		RotateRobo(10,10);
		Wait(500);
		distance = ReadSensor(IN_4);
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
			distance = ReadSensor(IN_4);
			if (distance < 203.2)
			{
				DisplaceRobo(1.5,10);
				break;
			}

	}
}

	SetLedPattern(LED_GREEN);
	PlaySound(TONE_C2);
	Wait(1000);
	FreeEV3();
	exit(0);
}

//Write wall following behavior.
//void wall_following(){
////rotate robo +90 degrees once function is called
//// so that we are no longer facing the wall
//
////green = wall_following
//SetLedPattern(LED_GREEN);
//char color;
//
////int distance;
//
//
//
////rotate about our Original Angle.
////RotateRobo(90,SPEED);
//
//while (!isExitButtonPressed()){
//
//
//	Wait(100);
//	color = precise_color(100);
//	TermPrintf("Color - %d",color);
//	// if our  color = blue then we know we are still on our wall
//	if (color == 2 || color == 3)
//	{
//		Wait(1000);
//		RotateRobo(90,10);
//		LcdClean();
//		TermPrintf("Color - %d",color);
//		TermPrintf("Following the Wall");
//		DisplaceRobo(1,SPEED);
//		color = precise_color(100);
//		//IF we hit a wall during our displacement we will turn to our original angle to continue along the corner.
//		if  (color == 2 || color == 3){
//			RotateRobo(90,10);
//			DisplaceRobo(1,SPEED);
//		}
//	}
//	else {
//		DisplaceRobo(1,SPEED);
//		RotateRobo(-90,10);
//		wandering();
//
//	}
//	// rotating back towards the wall to make sure we are still following it.
//	RotateRobo(-90,10);
//}
////moves forward since there is no wall
////DisplaceRobo(1,SPEED);
//
//}

//void wall_following(){
////rotate robo +90 degrees once function is called
//// so that we are no longer facing the wall
//
////green = wall_following
//SetLedPattern(LED_GREEN);
//char color;
//
////int distance;
//
//
//
////rotate about our Original Angle.
////RotateRobo(90,SPEED);
//
//while (!isExitButtonPressed()){
//
//
//	Wait(100);
//	color = precise_color(100);
//	TermPrintf("Color - %d",color);
//	// if our  color = blue then we know we are still on our wall
//	if (color == 2 || color == 3)
//	{
//		Wait(1000);
//		RotateRobo(90,10);
//		LcdClean();
//		TermPrintf("Color - %d",color);
//		TermPrintf("Following the Wall");
//		DisplaceRobo(1,SPEED);
//		color = precise_color(100);
//		//IF we hit a wall during our displacement we will turn to our original angle to continue along the corner.
//		if  (color == 2 || color == 3){
//			RotateRobo(90,10);
//			DisplaceRobo(1,SPEED);
//		}
//	}
//	else {
//		DisplaceRobo(1,SPEED);
//		RotateRobo(-90,10);
//		wandering();
//
//	}
//	// rotating back towards the wall to make sure we are still following it.
//	RotateRobo(-90,10);
//}
////moves forward since there is no wall
////DisplaceRobo(1,SPEED);
//
//}


//void wall_following()
//{
//	SetLedPattern(LED_GREEN);
//	while (!isExitButtonPressed()){
//
//		Wait(100);
//		int color = precise_color(100);
//		TermPrintf("Color - %d",color);
//		// if our  color = blue then we know we are still on our wall
//		if (color == 2 || color == 3)
//		{
//			Wait(1000);
//			RotateRobo(90,10);
//			LcdClean();
//			TermPrintf("Color - %d",color);
//			TermPrintf("Following the Wall");
//			if(precise_color(100)!=2 || precise_color(100)!=3 )
//				DisplaceRoboSens(1,SPEED);
//			else
//			{
//				RotateRobo(-90,10);
//				if(precise_sensor(100)!=2 || precise_color(100)!=3 )
//				{
//					DisplaceRobotSens(1.0,SPEED);
//				}
//				else
//				{
//					RotateRobo(90,10);
//					RotateRobo(90,10);
//					if(precise_color(100)!=2 || precise_color(100)!=3 )
//					{
//						DisplaceRoboSens(1,SPEED);
//					}
//					else
//					{
//						RotateRobo(90,10);
//						wandering();
//					}
//				}
//			}
//			color = precise_color(100);
//
//			//IF we hit a wall during our displacement we will turn to our original angle to continue along the corner.
//
//			if  (color == 2 || color == 3){
//				RotateRobo(-90,10);
//				DisplaceRobo(1,SPEED);
//			}
//		}
//		else {
//			DisplaceRoboSens(1,SPEED);
//			RotateRobo(90,10);
//		}
//		// rotating back towards the wall to make sure we are still following it.
//		RotateRobo(-90,10);
//	}
//	//moves forward since there is no wall
//	//DisplaceRobo(1,SPEED);
//}


void wall_following(){
//rotate robo +90 degrees once function is called
// so that we are no longer facing the wall
//green = wall_following
SetLedPattern(LED_GREEN);
char color;
//keeps track of the # of turns our robot has taken
int turns = 0;
//int distance;
//rotate about our Original Angle.
//RotateRobo(90,SPEED);
while (on_goal !=1 ){
	Wait(100);
	color = precise_color(100);
	TermPrintf("Color - %d",color);
	// if our  color = blue then we know we are still on our wall
	if (color == 2 || color == 3)
	{

		Wait(1000);
//		if (turns == 4){
//			turns = 0;
//			LcdClean();
//			TermPrintf("/n/n/n\t\t%d",turns);
//			Wait(1000);
//			LcdClean();
//		}
//		else{
			RotateRobo(90,10);
			turns++;
			LcdClean();
			TermPrintf("/n/n/n\t\t%d",turns);
		    wait(1000);
		    LcdClean();
//		}

		LcdClean();
		TermPrintf("Color - %d",color);
		TermPrintf("Following the Wall");
		//DisplaceRobo(1,SPEED);
		color = precise_color(100);
		//IF we hit a wall during our displacement we will turn to our original angle to continue along the corner.
		if  (color != 2 || color != 3){
			DisplaceRoboSens(1.0,SPEED);
			color = precise_color(100);
			if  (color != 2 || color != 3){
				RotateRobo(-90,10);
				}
			if  (color == 2 || color == 3){

				RotateRobo(90,10);
				RotateRobo(90,10);
				turns++;
				LcdClean();
				TermPrintf("/n/n/n\t\t%d",turns);
				wait(1000);
				LcdClean();
				wandering();
			}
		}

		else
		{
			RotateRobo(-90,10);
			RotateRobo(-90,10);
			wandering();

				}
	}
	else {
		//DisplaceRoboSens(10.0,SPEED);
		//RotateRobo(-90,10);
		wandering();
	}
	// rotating back towards the wall to make sure we are still following it.
}
//moves forward since there is no wall
//DisplaceRobo(1,SPEED);
}
void goalwandering()
{
	 //BLACK = wandering
	 SetLedPattern(LED_RED);
	 int color;
	 int distance;
     color = precise_color(100);

     int time = 50000;

     OnFwdReg(OUT_AB,SPEED);
     OnFwdSync(OUT_AB,SPEED);
     while (!isExitButtonPressed() || stop !=1)
	{

		color = precise_color(20);
		LcdClean();
	if (color == 2 || color ==3 )
	{
		TermPrintf("ON THE WALL!!");
		Off(OUT_AB);
		stop =1;
		break;
	}
	else if (color == 5)
	{
		TermPrintf("ON THE OBSTACLE!!");
		Off(OUT_AB);
		on_goal =1 ;
		stop =1;
		break;
	}
	}

     stop =1;
     Off(OUT_AB);
     stop =0;


    	 goal_finding();

     LcdClean();
}



//Write wandering behavior for the robot.
//Wandering behavior looks for either a wall or the object and then calls the corresponding behavior functions.
void wandering()
{
	 //BLACK = wandering
	 SetLedPattern(LED_RED);
	 int color;
	 int distance;
     color = precise_color(100);
     pthread_t sid;
     pthread_t cid;

     int time = 50000;

     pthread_create(&sid, NULL, sonar_reader, &time);
     pthread_create(&cid, NULL, sonar_reader, &time);
     OnFwdReg(OUT_AB,SPEED);
     OnFwdSync(OUT_AB,SPEED);
     while (!isExitButtonPressed() || stop !=1)
	{

		color = precise_color(20);
		LcdClean();
	if (color == 2 || color ==3 )
	{
		TermPrintf("ON THE WALL!!");
		Off(OUT_AB);
		stop =1;
		break;
	}
	else if (color == 5)
	{
		TermPrintf("ON THE OBSTACLE!!");
		Off(OUT_AB);
		on_goal =1 ;
		stop =1;
		break;
	}
	}

     stop =1;
     Off(OUT_AB);
     pthread_join(cid, NULL);
     pthread_join(sid, NULL);
     stop =0;

     if(on_goal ==1)
     {
    	 goalwandering;

     }
     LcdClean();
}


int main(void)
{
	InitEV3();
	SetAllSensorMode(COL_COLOR,NO_SEN,NO_SEN,US_DIST_MM);

	wandering();
	wall_following();
	goalwandering;
	goal_finding();


//	on_wall =1;
//	while(on_wall)
//	{
//		if( precise_color(100)== RED)
//			{
//				goal_finding();
//			}
//
//	}
	//wandering();

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
