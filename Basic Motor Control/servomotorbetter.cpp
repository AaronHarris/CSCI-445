include <stdio.h>
#include <stdlib.h>
#include "wiringPi.h"

void stop(FILE *fp, int delay);
void forward(FILE *fp, int delay);
void backward(FILE *fp, int delay);
void left(FILE *fp, int delay);
void right(FILE *fp, int delay);


int main(void)
{
 FILE *fp;
 fp = fopen("/dev/servoblaster", "w");
 if (fp == NULL) {
 printf("Error opening file\n");
 exit(0);
 }
 

 while (1) {
       
//       forward(fp, 1000);
       //stop(fp, 100);
//       backward(fp, 1000);
       //stop(fp, 100);
       forward(fp, 2000);
       //left(fp, 1000);
       //stop(fp, 100);
       //right(fp, 1000);
       //stop(fp, 100);
       stop(fp, 1000);

 }
 fclose(fp);
 return 0;
}

void stop(FILE *fp, int delay_num) {
       fprintf(fp, "0=123\n"); //Stop //LEFT WHEEL
       fprintf(fp, "1=151\n"); //Stop
       delay(delay_num);
       fflush(fp);
} //stop()

void forward(FILE *fp, int delay_num) {
       fprintf(fp, "0=133\n"); //Servo#0, Counter Clockwise //LEFT WHEEL
       fprintf(fp, "1=142\n"); //Servo#1, Counter Clockwise
       delay(1000);
       fflush(fp);
}

void backward(FILE *fp, int delay_num) {
       fprintf(fp, "0=115\n"); //Clockwise //LEFT WHEEL
       fprintf(fp, "1=161\n"); //Clockwise
       delay(1000);
       fflush(fp);
}

void right(FILE *fp, int delay_num) {
       fprintf(fp, "0=133\n"); //Servo#0, Counter Clockwise //LEFT WHEEL
       fprintf(fp, "1=161\n"); //Servo#1, Counter Clockwise
       delay(1000);
       fflush(fp);
}

void left(FILE *fp, int delay_num) {
       fprintf(fp, "0=115\n"); //Clockwise //LEFT WHEEL
       fprintf(fp, "1=142\n"); //Clockwise
       delay(1000);
       fflush(fp);
}
