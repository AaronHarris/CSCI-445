
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
 while (1) {
       
       printf("Min Max: ");
       scanf("%d %d", &min, &max);
       val = min+max/2;
       printf("-------\nStarting with val %d", val);

       fprintf(fp, "0=182\n"); //Servo#0, Counter Clockwise //LEFT WHEEL
       fprintf(fp, "1=100\n"); //Servo#1, Counter Clockwise
       delay(1000);
       fflush(fp);
       fprintf(fp, "0=182\n"); //Servo#0, Counter Clockwise //LEFT WHEEL
       fprintf(fp, "1=100\n"); //Servo#1, Counter Clockwise
       delay(1000);
       fflush(fp);
       fprintf(fp, "0=123\n"); //Stop //LEFT WHEEL
       fprintf(fp, "1=129\n"); //Stop
       delay(1000);
       fflush(fp);
       fprintf(fp, "0=100\n"); //Clockwise //LEFT WHEEL
       fprintf(fp, "1=182\n"); //Clockwise
       delay(1000);
       fflush(fp);
       fprintf(fp, "0=123\n"); //Stop //LEFT WHEEL
       fprintf(fp, "1=129\n"); //Stop
       delay(1000);
       fflush(fp);
 }
 fclose(fp);
 return 0;
}

void stop(FILE *fp, int delay) {
       fprintf(fp, "0=123\n"); //Stop //LEFT WHEEL
       fprintf(fp, "1=129\n"); //Stop
       delay(delay);
       fflush(fp);
} //stop()


