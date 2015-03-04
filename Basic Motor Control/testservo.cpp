#include <stdio.h>
#include <stdlib.h>
#include "wiringPi.h"
int main(void)
{
 FILE *fp;
 fp = fopen("/dev/servoblaster", "w");
 if (fp == NULL) {
 printf("Error opening file\n");
 exit(0);
 }
 
 int motor = -1;
 printf("choose motor to diagnose:");
 scanf("%d", &motor);

 int val = -1;
 int max = -1;
 int min = -1;
 int direction = 0;

 printf("enter min max:");
 scanf("%d %d", &min, &max);

 printf("Now running motor%d with max val = %d\n", motor, max);
fprintf(fp, "%d=%d\n", motor, max); //Servo#0, Counter Clockwise //RIGHT WHEEL
       delay(1000);
       fflush(fp);

 printf("Is it forward(1) or backward(-1)?\n");
 scanf("%d", &direction);
 int error = -1;

 while (1) {

       val = (min+max)/2;
       printf("-------\nRunning with val %d\n", val);

       fprintf(fp, "%d=%d\n", motor, val); //Servo#0, Counter Clockwise //RIGHT WHEEL
       delay(1000);
       fflush(fp);

       printf("Is it forward(1) or backward(-1)?\n");
       scanf("%d", &error);

       if (error > 0 && direction >0) max = val;
       else if (error < 0 && direction >0) min = val;
       else if (error > 0 && direction <0) min = val;
       else if (error < 0 && direction <0) max = val;
       else if (error == 0) {
              printf("The stopping val for motor %d is %d\n", motor, val);
              fclose(fp);
              return 0;
       }
 }
 fclose(fp);
 return 0;
}



