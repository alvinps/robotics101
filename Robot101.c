/*
 \file		Robot101.c
 \author	${Alvin Poudel Sharma / Jared Kremer / Steven Adams}
 \date		${date}
 \brief		Simple Hello World! for the Ev3
*/

#include <ev3.h>
#include<math.h>

#define MAX_OBSTACLES   25     	/*maximum number of obstacles*/

#define MAX_X 13				/*Defines the boundary of robot's world */
#define MAX_Y 11				/*Defines the boundary of robot's world */

// Structure defined that will be used to propagate potentials throughout the grid
typedef struct points
{
	int x;
	int y;
	int w;
}points;

// A queue that will be used for calculating weights for all the cells in the grid
#define MAX 500
points qu[MAX];
int front =0;
int rear = -1;
int queue_size = 0;

// Inserts a coordinate and its weight in a queue
void qinsert(points  q)
{
	rear ++;
	qu[rear].x = q.x ;
	qu[rear].y = q.y ;
	qu[rear].w = q.w ;
	queue_size ++;
}

// Pops a coordinate and its weight from the queue
points qpop()
{
	points p;
	p.x = qu[front].x;
	p.y = qu[front].y;
	p.w = qu[front].w;
	front++;
	queue_size --;
	return p;
}

// Converts the points provided in meters into ft for easier calculation
int m_to_feet_idx(double m)
{
	return (int) round(m*3.28);
}

// Rotates the robot by the provided angles in degree with the provided speed.
void RotateRobo(int deg,int speed)
{
	int motor_angle =(int) abs(deg) * 2.64;

	if(deg>0)
	{	// Positive rotation of the robot.
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
	{	// Negative rotation of the robot.
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

// Displaces the robot in a linear fashion. Forward or backwards along the provided distance
// with the provided speed
void DisplaceRobo(double distance, int speed)
{
	if(distance > 0.0)
	{	// Positive Displacement of the robot. Forward
		unsigned long req_time = (( 62000.0* fabs(distance))/speed);
		OnFwdReg(OUT_AB,speed);
		//OnFwdReg(OUT_B,speed);
		OnFwdSync(OUT_AB,speed);
		//OnFwdSync(OUT_B,speed);
		Wait(req_time);
	}
	else
	{	// Negative Displacement of the robot. Backward
		unsigned long req_time = (( 62000.0* fabs(distance))/speed);
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
	int speed = 15;			// Defining the speed of the robot.
	int num_obstacles = 20;                         /* number of obstacles         */

	double obstacle[MAX_OBSTACLES][2] =             /* obstacle locations          */
	{{0.915, 0.305},{0.915, 0.61},{0.915, 0.915},{0.915, 1.219},
	 {0.915, 1.524},{0.915, 1.829}, {1.829, 1.524},{1.829, 1.829},
	 {1.829, 2.13},{1.829, 2.44},{1.829, 2.743},{1.829, 0.61},
	 {2.13, 0.61}, {2.44, 0.61}, {2.743, 0.61},{2.743, 0.305},
	 {2.743, 0.915},{2.743, 1.22},{2.743, 1.524},{2.743, 1.83},
	 {-1,-1},{-1,-1},{-1,-1}, {-1,-1},{-1,-1}};

	double start[2] = {0.305, 0.61};          /* start location              */
	double goal[2] = {3.356, 1.524};           /* goal location               */
	int Orientation = 0; // orientation  angle with +ve x-axis in degree 

	// Initializing the weight matrix with 9999 which will be later replaced
	// by their respective weight.
	int i,j;
	int weight[MAX_X][MAX_Y];
	for(i=0;i<MAX_X;i++)
	{
		for (j=0;j<MAX_Y;j++)
		{
			weight[i][j] = 9999;
		}
	}
	// Adding obstacles at the edge of the grid to prevent robot from colliding the edge
	// Assigning higher weight to the obstacles to increase their potential compared to
	// cells that are free of obstacles.
	for(i=0;i<MAX_X;i++)
		{
				weight[i][0] = 999;
				weight[i][10] = 999;
		}
	for(i=0;i<MAX_Y;i++)
		{
				weight[0][i] = 999;
				weight[12][i] = 999;
		}

// Assigning higher potential to obstacles at the provided points so that robot won't travel
// or collide the obstacle.
// Higher value of potential must be chosen if we are using higher grid. Possibly infinity.
	for(i=0;i<num_obstacles;i++)
	{
		if(((int)obstacle[i][0]!= -1)&& ((int)obstacle[i][1]!= -1))
		{
			int x_index = m_to_feet_idx(obstacle[i][0]);
			int y_index = m_to_feet_idx(obstacle[i][1]);
			weight[x_index][y_index] = 999;
		}
	}

	points current;
	current.x = m_to_feet_idx(goal[0]);
	current.y = m_to_feet_idx(goal[1]);
	current.w = 0;
	qinsert(current);
	
	// calculation of potential field per unit cell starts here
	// Propagating potential value starting from goal to its adjacent cells and so on.
	while(queue_size !=0)
	{
		current = qpop();
		
		if(weight[current.x][current.y] == 9999)
		{
			weight[current.x][current.y] = current.w;
			if((current.x + 1) < MAX_X && weight[current.x+1][current.y] == 9999 )
			{
				// Assigning potential value for (X+1,Y) coordinate
				points temp;
				temp.x = current.x +1;
				temp.y = current.y;
				temp.w = current.w +1;
				qinsert(temp);
			}
			if((current.x - 1) >= 0 && weight[current.x-1][current.y] == 9999)
			{
				// Assigning potential value for (X-1,Y) coordinate
				points temp;
				temp.x = current.x -1;
				temp.y = current.y;
				temp.w = current.w +1;
				qinsert(temp);
			}
			if((current.y + 1) < MAX_Y && weight[current.x][current.y+1] == 9999)
			{
				// Assigning potential value for (X,Y+1) coordinate
				points temp;
				temp.x = current.x ;
				temp.y = current.y +1;
				temp.w = current.w +1;
				qinsert(temp);
			}
			if((current.y - 1) >= 0 && weight[current.x][current.y-1] == 9999)
			{                    
				// Assigning potential value for (X,Y-1) coordinate
				points temp;
				temp.x = current.x ;
				temp.y = current.y -1;
				temp.w = current.w +1;
				qinsert(temp);
			}
		}
	}

	// initializing current locations and orientation for the robot to travel
	int currentX = m_to_feet_idx(start[0]) ;
	int currentY = m_to_feet_idx(start[1]) ;
	int goalX = m_to_feet_idx(goal[0]) ;
	int goalY = m_to_feet_idx(goal[1]) ;
	int currentA = Orientation;

	// Initializing EV3 vehicle
	InitEV3();

// Testing if potential field value are calculated correctly or not.

	//DisplaceRobo(distance,speed);
	//RotateRobo(90,speed);
//
//	  for(i=0;i<MAX_X;i++)
//    {
//        for (j=0;j<MAX_Y;j++)
//        {
//			LcdPrintf(1,"%4d|",weight[i][j]);
//        }
//        LcdPrintf(1,"\n");
//        for (j=0;j<MAX_Y;j++)
//        {
//            LcdPrintf(1,"%5s","-----");
//        }
//        LcdPrintf(1,"\n");
//    }

	SetLedPattern(LED_RED);

	// navigation of the robot starts here
	// The loop below will break when the robot reaches the goal point mentioned above.

	while((currentX != goalX) || (currentY != goalY) )
	{
		int min_weight = weight[currentX][currentY];
		int next_x = currentX;
		int next_y = currentY;
		int nextA = currentA;

		// A small check to make sure the robot doesn't rotate to much
		// Hence, defining its behavior to pick minimum potential at the current orientation
		// that prevents excessive rotation of the robot

		if (currentA == 0 || currentA == 180){

			if((currentX + 1) < MAX_X )
			{
				// Checking if (X+1, Y) has minimum weight or not
				if(weight[currentX+1][currentY] < min_weight )
				{
					min_weight = weight[currentX-1][currentY];
					next_x = currentX +1;
					next_y = currentY;
					nextA = 0;
				}

			}
			if((currentX - 1) >= 0 )
			{
				// Checking if (X-1, Y) has minimum weight or not
				if(weight[currentX-1][currentY] < min_weight )
				{
					min_weight = weight[currentX-1][currentY];
					next_x = currentX -1;
					next_y = currentY;
					nextA = 180;
				}

			}
			if((currentY + 1) < MAX_Y )
			{
				// Checking if (X, Y+1) has minimum weight or not
				if(weight[currentX][currentY+1] < min_weight )
				{
					min_weight = weight[currentX][currentY+1];
					next_x = currentX ;
					next_y = currentY +1;
					nextA = 90;
				}

			}
			if((currentY - 1) >= 0 )
			{
				// Checking if (X, Y-1) has minimum weight or not
				if(weight[currentX][currentY-1] < min_weight )
				{
					min_weight = weight[currentX][currentY-1];
					next_x = currentX ;
					next_y = currentY -1;
					nextA = -90;
				}

			}
		}

		if (currentA == -90 || currentA ==90 ){

			if((currentY + 1) < MAX_Y )
			{
				// Checking if (X, Y+1) has minimum weight or not
				if(weight[currentX][currentY+1] < min_weight )
				{
					min_weight = weight[currentX][currentY+1];
					next_x = currentX ;
					next_y = currentY +1;
					nextA = 90;
				}

			}
			if((currentY - 1) >= 0 )
			{
				// Checking if (X, Y-1) has minimum weight or not
				if(weight[currentX][currentY-1] < min_weight )
				{
					min_weight = weight[currentX][currentY-1];
					next_x = currentX ;
					next_y = currentY -1;
					nextA = -90;
				}

			}
			if((currentX + 1) < MAX_X )
			{
				// Checking if (X+1, Y) has minimum weight or not
				if(weight[currentX+1][currentY] < min_weight )
				{
					min_weight = weight[currentX-1][currentY];
					next_x = currentX +1;
					next_y = currentY;
					nextA = 0;
				}

			}
			if((currentX - 1) >= 0 )
			{
				// Checking if (X-1, Y) has minimum weight or not
				if(weight[currentX-1][currentY] < min_weight )
				{
					min_weight = weight[currentX-1][currentY];
					next_x = currentX -1;
					next_y = currentY;
					nextA = 180;
				}
			}
		}
		// Rotating the robot if there is any change in the orientation of robot
		// compared to the previous state.
		if(currentA != nextA)
		{
			RotateRobo(nextA- currentA, speed);
		}
		// Displacing the robot by one foot in the grid
		DisplaceRobo(1,speed);

		//LcdClean();
		//LcdPrintf(1,"Taking (%d,%d) at %d min %d\n",next_x,next_y, nextA, min_weight);
		//Wait(SEC_1);
		currentX = next_x;
		currentY = next_y;
		currentA = nextA;
	}

	// On reaching the goal the robot beeps and sets green led to show completing of the navigation
	if(currentX == goalX && currentY ==goalY)
	{
		SetLedPattern(LED_GREEN);
		PlaySound(SOUND_DOUBLE_BEEP);
		LcdClean();
	}

	Wait(SEC_1);
	StopSound();
	FreeEV3();
	return 0;
}
