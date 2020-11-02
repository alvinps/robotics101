#include<stdio.h>
#include<stdlib.h>
#include<math.h>


#define MAX_OBSTACLES   25 

    int max_x  = 13;
    int max_y = 11;


int m_to_feet_idx(double m)
{
    return (int) round(m*3.28);
}



void weight_calc( int x_index, int y_index,int previous_weight,int weight[max_x][max_y],double obstacle[MAX_OBSTACLES][2] )
{

    int i;
    for(i=0;i<MAX_OBSTACLES;i++)
    {
        if(x_index == m_to_feet_idx(obstacle[i][0]) && y_index == m_to_feet_idx(obstacle[i][1]))
        {
            weight[x_index][y_index] = 999;

            printf("Weight [ %d , %d ]-> %d \n",x_index ,y_index , 999 );
            return;
        }
    }
    
    printf("Weight [ %d , %d ]-> %d \n",x_index ,y_index , previous_weight+1 );
    if((x_index + 1) < max_x && weight[x_index+1][y_index] == -1)
    {
        weight[x_index+1][y_index] = previous_weight +1;
        weight_calc(x_index+1,y_index,previous_weight+1,weight, obstacle);
    }
    if((x_index - 1) >= 0 && weight[x_index-1][y_index] == -1)
    {
        weight[x_index-1][y_index] = previous_weight +1;
        weight_calc(x_index-1,y_index,previous_weight+1,weight, obstacle);
    }
    if((y_index + 1) < max_y && weight[x_index][y_index+1] == -1)
    {
        weight[x_index][y_index+1] = previous_weight +1;
        weight_calc(x_index,y_index+1,previous_weight+1,weight, obstacle);
    }
    if((y_index - 1) >= 0 && weight[x_index][y_index-1] == -1)
    {
        weight[x_index][y_index-1] = previous_weight +1;
        weight_calc(x_index,y_index-1,previous_weight+1,weight, obstacle);
    }
    
    return ;
}

int main()
{

	int speed = 20;
	int num_obstacles = 13;                    /*number of obstacles*/
	double obstacle[MAX_OBSTACLES][2] =        /*obstacle locations*/{{0.61, 2.743},{0.915, 2.743},{1.219, 2.743},{1.829, 1.219},{1.829, 1.524},{ 1.829, 1.829}, {1.829, 2.134},{2.743, 0.305},{2.743, 0.61},{2.743, 0.915},{2.743, 2.743},{3.048, 2.743},{3.353, 2.743},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1}};
	double start[2] = {0.305, 1.219};          /*start location*/
	double goal[2] = {3.658, 1.829};           /*goal location*/






    int i,j;
    int weight[12][10];
    for(i=0;i<max_x;i++)
    {
        for (j=0;j<max_y;j++)
        {
            weight[i][j] = -1;
        }
      }



    int x_gindex = m_to_feet_idx(goal[0]);    
    int y_gindex = m_to_feet_idx(goal[1]);
    printf("Weight [ %d , %d ]-> %d \n",x_gindex ,y_gindex , 0 );

    weight[x_gindex ][y_gindex] = 0;

    if((x_gindex + 1) < max_x && weight[x_gindex+1][y_gindex] == -1 )
    {
        weight[x_gindex+1 ][y_gindex] = 0 +1;
        weight_calc(x_gindex+1,y_gindex,1,weight, obstacle);
    }
    if((x_gindex - 1) >= 0 && weight[x_gindex-1][y_gindex] == -1)
    {
        weight[x_gindex-1 ][y_gindex] = 0 +1;
        weight_calc(x_gindex-1,y_gindex,1,weight, obstacle);
    }
    if((y_gindex + 1) < max_y && weight[x_gindex][y_gindex+1] == -1)
    {
        weight[x_gindex ][y_gindex +1] = 0 +1;
        weight_calc(x_gindex,y_gindex+1,1,weight, obstacle);
    }
    if((y_gindex - 1) >= 0 && weight[x_gindex][y_gindex-1] == -1)
    {
        weight[x_gindex ][y_gindex-1] = 0 +1;        
        weight_calc(x_gindex,y_gindex-1,1,weight, obstacle);
    }
    
    for(i=0;i<max_x;i++)
    {
        for (j=0;j<max_y;j++)
        {
            printf("%4d",weight[i][j]);
        }
        printf("\n");
    }

    return 0;
}