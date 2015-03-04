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

 int num = -1;
 
 int direction = 0;

 printf("enter max or min:");
 scanf("%d", &num);

 printf("Now running motor%d with val = %d\n", motor, num);
fprintf(fp, "%d=%d\n", motor, num); //Servo#0, Counter Clockwise //RIGHT WHEEL
       delay(1000);
       fflush(fp);

 while (1) {

       printf("-------\nRunning with val %d\n", num);

       fprintf(fp, "%d=%d\n", motor, num); //Servo#0, Counter Clockwise //RIGHT WHEEL
       delay(1000);
       fflush(fp);

       printf("Enter new value");
       scanf("%d", &num);
 }
 fclose(fp);
 return 0;
}



