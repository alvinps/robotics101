#include <ev3.h>



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

	      for (j = 0; j < n; ++j) {
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

