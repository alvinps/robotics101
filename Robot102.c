#include <ev3.h>
#include <pthread.h>
#include <time.h>


#define SPEED 13
#define BLUE 2
#define RED 5

#define WALL 2021
#define OBSTACLE 2020

int stop = 0;
int on_wall = 0;
int on_goal = 0;
int on_goal_sonar = 0;
int angle_tracker = 0;
int displace_tracker = 0;

//Keeps track of the # of turns our robot has taken. 
int turns = 0;

//The angle our robot will always turn when it reaches a wall.
int angle = -90;

//Counter to alternate between -90 and 90 degree angle turns to get our robot out of
// infinite loops should it encounter one during its traversal.
int counter = 0;

void goal_wandering();


// This function simply displaces the robot to the given distance with the provided speed
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

// This function simply rotates the robot to the given angle with the provided speed
void RotateRobo(int deg,int speed)
{
	int motor_angle =(int) abs(deg) * 2.64;

	if(deg>0)
	{
        // forward movement
        // calculating total time for robot to rotatei in the given direction
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
        // reverse movement
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

// this function samples a set of color readings from the sensor and return the greatest color that occured in the sample;
int precise_color(int data_count)
{
	int maxValue = 0, maxCount = 0, i, j;
	int measurement[data_count];
	for (i =0; i< data_count; i++)
	{
		measurement[i] = ReadSensor(IN_1);
		// measures a sensor data and stores in a array
		Wait(1);
	}
		// takes the mode.
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


// This function is a threaded fucntion that works in parallel to the wall_following. 
// This detects a red or blue color and stops the motors immediately if RED or BLUE is detected.
// This also makes sure that the sonar thread is also exited and is at its initial position
void * color_reader(void* p )
{
	int time = *((int*) p) ;
	int i;
	clock_t start = clock();
	clock_t end ;

	int diff=0;
	while(1)
	{
        // exits if stop operation is called by any of the threaded functions
		if(stop ==1)
			return NULL;

		int col = precise_color(20);
		if(col == RED || col == BLUE){
            // on detectiong blue or red , it stops the motors and stops sonar function too.

			Off(OUT_AB);
			if(col == RED) on_goal =1;
			stop = 1;
			return NULL;
		}
		diff =(int) (1000 * (clock() - start )/ CLOCKS_PER_SEC);
	}
	return NULL;
}

// This function is a threaded fucntion that works in parallel to the wall_following. 
// This detects distance of an object/ obstacle from teh robot. When the threshold set is met,
// it aligns the robot to the direction of the obstacle and also stops the motion of the robot.
// It also stops the color thread. 
void *sonar_reader(void* p )
{
	int time = *((int*) p) ;
	ResetRotationCount(OUT_C);

		clock_t start = clock();

		int diff=0,distance , counter = 0, current = 0, previous = 0, flip =  1, add = 0;
		while(1)
		{
			if(stop == 1)
			{
                // rotates the sonar at an increment of 10 degree
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
			if (distance < 600)
			{
                // on detecting the obstacle, the motor wheels are turned off, and the whole robot is oriented towards he direction of the obstacle
				stop = 1 ;
				on_goal_sonar =1;
				Off(OUT_AB);
				counter = MotorRotationCount(OUT_C);
				if(counter >0)
					RotateMotor(OUT_C,-10,counter);
				else
					RotateMotor(OUT_C,10,counter);
				RotateRobo(-counter,10);
				goal_wandering();
				return NULL;
			}
			diff =(int) ( (clock() - start )/ CLOCKS_PER_SEC);

		}

		if(add > 0)
			RotateMotor(OUT_C,10,add);
		else
			RotateMotor(OUT_C,-10,-add);
	return NULL;
}

// THis is a multi threaded function that runs the motors, the color sensor and sonar sensor all at the same time
// Moves the robot and uses the sonar and color sensor at the same time.
void DisplaceRoboSens(double distance, int speed)
{
	pthread_t cid;
	pthread_t sid;
	if(distance > 0.0)
	{	// Positive Displacement of the robot. Forward
		unsigned long req_time = (( 62000.0* fabs(distance))/speed);
		int time = (int) req_time;

        // calling threaded function for sonar and color sensor at the same time.
		pthread_create(&cid, NULL, color_reader, &time);
		pthread_create(&sid, NULL, sonar_reader, &time);
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
	stop =1;
	Off(OUT_A);
	Off(OUT_B);
    
	pthread_join(cid, NULL);
	pthread_join(sid, NULL);
    // reseting sonar and color sensor function for next use
	stop =0;
}


void goal_finding(){
	int distance;
	DisplaceRobo(-0.1,5);
	SetLedPattern(LED_ORANGE);
	PlaySound(SOUND_UP);

	//trying several different angles to see which one is better
	//int max_angle = 50;

	int max_angle = 90;
	distance = ReadSensor(IN_4);
	int i;
	//304.8 in mm  is 12 inches
	//when we sense the red tape we are within 12 inches from the goal
	int max_distance = 350;
	bool goal = FALSE;
	RotateRobo(-max_angle,10);
	Wait(100);
	for (i = 0; i < 10 ;i++){

		Wait(500);
		distance = ReadSensor(IN_4);
		if (distance < max_distance)
		{
			goal = TRUE;
			PlaySound(SOUND_DOWN);
			DisplaceRobo(1.7,10);
			FreeEV3();
			exit(0);
			break;
		}
		RotateRobo(10,10);
	}

if (!goal){
	RotateRobo(max_angle,10);
	for (i = 0; i < 10 ;i++){

			Wait(500);
			distance = ReadSensor(IN_4);
			if (distance < max_distance)
			{
				goal = TRUE;
				PlaySound(SOUND_DOWN);
				DisplaceRobo(1.7,10);
				FreeEV3();
				exit(0);
				break;
			}
			RotateRobo(-10,10);
	}
}
	if(!goal)
	{
		PlaySound(SOUND_DOWN);
		DisplaceRobo(1.7,10);
	}
	Wait(1000);
	SetLedPattern(LED_GREEN);
	StopSound();

	FreeEV3();
	exit(0);
}

// This function is executed when the sonar detects our goal object, this function will simply displace
// the robo until the color sensor detects the color red, It then executes goal_finding.
// If a blue wall is hit during the time of the robots displacement, it simply returns back to wall following
void goal_wandering()
{
	 //BLACK = wandering
	 SetLedPattern(LED_ORANGE);
	 int color;
     color = precise_color(20);

     OnFwdReg(OUT_AB,SPEED);
     OnFwdSync(OUT_AB,SPEED);
     while (!isExitButtonPressed() || stop !=1)
	{
		color = precise_color(20);
		LcdClean();
	 if (color == 5)
	{
        
		Off(OUT_AB);
		on_goal =1 ;
		stop =1;
		break;
	}
	 if (color == 2 )
	 	{
             // if blue is detected, go into wall following instead of goal finding.
	 		Off(OUT_AB);
	 		on_goal =0 ;
	 		stop = 0;
	 		on_goal_sonar =0;
	 		wall_following();
	 		break;
	 	}
	}
     stop =1;
     Off(OUT_AB);

     stop =0;
     on_goal =1;

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

	 //Robot continues forward until the motors are turned off in the while loop.
     OnFwdReg(OUT_AB,SPEED);
     OnFwdSync(OUT_AB,SPEED);
	 //While runs until the exit button is pressed on the robot OR the stop variable = 1.
     while (!isExitButtonPressed() || stop !=1)
	{

		color = precise_color(20);
		LcdClean();
	// If color == 2(blue) then we know our sensor detects a wall and we can call
	// our wall following behavior.
	if (color == 2)
	{
		TermPrintf("ON THE WALL!!");

		Off(OUT_AB);
		stop =1;
		break;
	}
	//if color = 5 (RED) then the robot is on the obstacle and the goal finding behavior
	//can be called.
	else if (color == 5)
	{
		TermPrintf("ON THE OBSTACLE!!");
		Off(OUT_AB);
		on_goal =1 ;
		stop =1;
		DisplaceRobo(-0.1,5);
		goal_finding();
		break;
	}
	}
	//Turns off all of our motors on our robot.
     stop =1;
     Off(OUT_AB);
     pthread_join(sid, NULL);
     stop = 0;

     if(on_goal_sonar ==1)
     {
    	 goal_wandering();
     }
     if(on_goal ==1)
     {
    	 goal_finding();
     }
     LcdClean();
}



void wall_following(){
//rotate robo +90 degrees once function is called
// so that we are no longer facing the wall
SetLedPattern(LED_GREEN);
char color;


while (on_goal !=1 ){

	color = precise_color(100);
	//If the color sensor reads red then we have reached our goal and we can call the goal_finding
	//behavior.
	if(color == 5) goal_finding();

	// If our  color = blue then we know we are still on a wall.
	if (color == 2)
	{
		//If our robot has made 12 total turns and we STILL have not made it to the goal
		//then we can assume that our robot is stuck and that it needs to change its turning
		//angle to get out of the loop.
		if (turns >= 12){
			turns = 0;
			angle = 90;
			//Changes the angle again if we get stuck in another loop during the navigation process.
			if(counter % 2 != 0 ){
				angle = -90;
			}
			counter++;
		}
		//Since we are on the wall we need to turn our robot in order to not run into the wall. 
		RotateRobo(angle,10);
		turns++;
		TermPrintf("Following the Wall");
		
		color = precise_color(100);
		//If our color sensor does NOT detect a wall we are safe to move forward.
		if  (color != 2){
			DisplaceRoboSens(1.0,SPEED);
			color = precise_color(100);
			//Testing that we are still on the wall.
			if  (color != 2){
				RotateRobo(-angle,10);
				}
				color = precise_color(100);

			//If we are still on our wall then we can rotate again back to our previous
			//orientation and continue along the wall.
			if  (color == 2){

				RotateRobo(angle,10);
				DisplaceRoboSens(1.0,SPEED);
				// turns++;
			
			}
		}

		else
		{
			RotateRobo(-angle,10);
				}
	}
	else {
		
		wandering();
	}
	// rotating back towards the wall to make sure we are still following it.
}


//moves forward since there is no wall

}

int main(void)
{
	InitEV3();
	// initializing the sensors in the robot
	SetAllSensorMode(COL_COLOR,NO_SEN,NO_SEN,US_DIST_MM);
	
	wandering();
	wall_following();

	FreeEV3();
	return 0;

}
