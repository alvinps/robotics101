// CPP Program to find sum of Manhattan distance 
// between all the pairs of given points 
#include <stdio.h> 
#include <math.h>
  
// Return the sum of distance between all 
// the pair of points of a n array
int distancesum(int x[], int y[], int n) 
{ 
    int sum = 0; 
  
    // for each point, finding distance to 
    // rest of the point 
    for (int i = 0; i < n; i++) 
        for (int j = i + 1; j < n; j++) 
            sum += (abs(x[i] - x[j]) + 
                    abs(y[i] - y[j])); 
    return sum; 
} 
  
// Driven Program 
int main() 
{ 
    int x[] = {2, 2,}; 
    int y[] = {2,3}; 
    int n = sizeof(x) / sizeof(x[0]);

   // printf("%d",distancesum(x,y,n)); 

    int max_x = 12;
    int max_y = 10;

    int goal_x = 5;
    int goal_y = 6;

    int weight[max_x][max_y];
    for(int i=0;i<max_y;i++)
    {
        for (int j=0;j<max_x;j++)
        {
            weight[i][j] = -1;
        }
      }

      for(int i=0;i<max_y;i++)
    {
        for (int j=0;j<max_x;j++)
        {
            //at point 0,0 of the array need to make it max y but min x and then iterte throughout
            //the 2-d array updating the weights accordingly
            int x [] = {goal_x,j};
            int y [] = {goal_y,max_y-i};
            n = sizeof(x) / sizeof(x[0]);
            weight[i][j] = distancesum(x,y,n);
        }
      }
      for(int i=0;i<max_y;i++)
    {
        for (int j=0;j<max_x;j++)
        {
            //at point 0,0 of the array need to make it max y but min x and then iterte throughout
            //the 2-d array updating the weights accordingly
        
            printf("%d\t",weight[i][j]);
        }
        printf("\n");
      }

    

    return 0; 
} 