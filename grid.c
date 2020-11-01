#include <stdio.h>

#define X 12  //MAX WORKSPACE ROWS
#define Y 10  //MAX WORKSPACE COLUMNS
#define MAX_LENGTH 4.88 // MAX LENGTH IN METERS
#define MAX_WIDTH 3.05 // MAX WIDTH IN METERS

#define MAX_OBSTACLES 25 /* maximum number of obstacles */
int num_obstacles = 13; /* number of obstacles */
double obstacle[MAX_OBSTACLES][2] = /* obstacle locations */
{{0.61, 2.743},{0.915, 2.743},{1.219, 2.743},{1.829, 1.219},
{1.829, 1.524},{ 1.829, 1.829}, {1.829, 2.134},{2.743, 0.305},
{2.743, 0.61},{2.743, 0.915},{2.743, 2.743},{3.048, 2.743},
{3.353, 2.743},
{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
{-1,-1},{-1,-1},{-1,-1}};
double start[2] = {0.305, 1.219}; /* start location */
double goal[2] = {3.658, 1.829}; /* goal location */


struct cell {
    double x_axis;
    double y_axis;
};


int main (){


//Create unit length for one cell in our workspace.
double unit_x = MAX_LENGTH/X;
double unit_y = MAX_WIDTH/Y;

printf("Unit_X: %f\n",unit_x);
printf("Unit_Y: %f\n\n",unit_y);

struct cell workspace [X][Y];

//Fill out 2-d array representing our workspace.
for(int i = 0; i <= Y; i++)
{
    for (int j = 0; j <= X; j++)
    {
        workspace[i][j].x_axis = j*unit_x;
        workspace[i][j].y_axis = i*unit_y;
           
    }
}

//print 2-d workspace

for(int i = 0; i <= Y; i++)
{
    for (int j = 0; j <= X; j++)
    {
        printf("X: %f Y: %f ",workspace[i][j].x_axis,workspace[i][j].y_axis);
    }
     printf("\n\n");
}


return 0;

}