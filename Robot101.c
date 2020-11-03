/*
 \file		Robot101.c
 \author	${Alvin Poudel Sharma / Jared Kremer / Steven Adams}
 \date		${date}
 \brief		Simple Hello World! for the Ev3
*/

#include <ev3.h>
#include<math.h>

#define MAX_OBSTACLES   25     /*maximum number of obstacles*/

#define MAX_X 13
#define MAX_Y 11


typedef struct points
{
	int x;
	int y;
	int w;
}points;
#define MAX 500
points qu[MAX];
int front =0;
int rear = -1;
int queue_size = 0;

void qinsert(points  q)
{

	rear ++;
	qu[rear].x = q.x ;
	qu[rear].y = q.y ;
	qu[rear].w = q.w ;


	queue_size ++;
}

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


int m_to_feet_idx(double m)
{
	return (int) round(m*3.28);
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
		OnFwdReg(OUT_A,speed);
		OnFwdReg(OUT_B,speed);
		OnFwdSync(OUT_A,speed);
		OnFwdSync(OUT_B,speed);
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
	int Orientation = 0; // orientation  angle with +ve x-axis in degree 

	int i,j;
	int weight[max_x][max_y];
	for(i=0;i<max_x;i++)
	{
		for (j=0;j<max_y;j++)
		{
			weight[i][j] = 9999;
		}
	}
	
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
	
	// calculation of potenital field per unit cell starts here
	while(queue_size !=0)
	{
		current = qpop();
		
		if(weight[current.x][current.y] == 9999)
		{
			weight[current.x][current.y] = current.w;
			if((current.x + 1) < max_x && weight[current.x+1][current.y] == 9999 )
			{                   
				points temp;
				temp.x = current.x +1;
				temp.y = current.y;
				temp.w = current.w +1;
				qinsert(temp);
			}
			if((current.x - 1) >= 0 && weight[current.x-1][current.y] == 9999)
			{
				points temp;
				temp.x = current.x -1;
				temp.y = current.y;
				temp.w = current.w +1;
				qinsert(temp);
			}
			if((current.y + 1) < max_y && weight[current.x][current.y+1] == 9999)
			{
				points temp;
				temp.x = current.x ;
				temp.y = current.y +1;
				temp.w = current.w +1;
				qinsert(temp);
			}
			if((current.y - 1) >= 0 && weight[current.x][current.y-1] == 9999)
			{                    
				points temp;
				temp.x = current.x ;
				temp.y = current.y -1;
				temp.w = current.w +1;
				qinsert(temp);
			}
		}
		
	}
	int currentX = m_to_feet_idx(start[0]) ;
	int currentY = m_to_feet_idx(start[1]) ;
	int goalX = m_to_feet_idx(goal[0]) ;
	int goalY = m_to_feet_idx(goal[1]) ;
	int currentA = Orientation;

	InitEV3();

	//DisplaceRobo(distance,speed);
	//RotateRobo(90,speed);

	  for(i=0;i<max_x;i++)
    {
        for (j=0;j<max_y;j++)
        {
			LcdPrintf(1,"%4d|",weight[i][j]);
        }
        LcdPrintf(1,"\n");
        for (j=0;j<max_y;j++)
        {
            LcdPrintf(1,"%5s","-----");
        }
        LcdPrintf(1,"\n");
    }












	SetLedPattern(LED_RED);

	// navigation starts here
	while((currentX != goalX) || (currentY != goalY) )
	{
		int min_weight = weight[currentX][currentY];
		int next_x = currentX;
		int next_y = currentY;
		int nextA = currentA;

		if (currentA == 0 || currentA == 180){


			if((currentX + 1) < MAX_X )
			{
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
				if(weight[currentX-1][currentY] < min_weight )
				{
					min_weight = weight[currentX-1][currentY];
					next_x = currentX -1;
					next_y = currentY;
					nextA = 180;
				}

			}
		}

		if(currentA != nextA)
		{
			RotateRobo(nextA- currentA, speed);
		}
		DisplaceRobo(1,speed);

		LcdPrintf(1,"Taking (%d,%d) at %d min %d\n",next_x,next_y, nextA, min_weight);

		
		currentX = next_x;
		currentY = next_y;
		currentA = nextA;
	}

	if(currentX == goalX && currentY ==goalY)
	{

		SetLedPattern(LED_GREEN);
		PlaySound(SOUND_DOUBLE_BEEP);
		LcdClean();
	}


	Wait(SEC_5);
	StopSound();
	FreeEV3();
	return 0;
}
