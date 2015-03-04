#include <stdio.h>
#include <stdlib.h>
#include "wiringPi.h"
#include <algorithm>

using namespace std;

#define TRIG 5

void stop(FILE *fp, int delay);
void forward(FILE *fp, int delay);
void backward(FILE *fp, int delay);
void left(FILE *fp, int delay);
void right(FILE *fp, int delay);
void drive(FILE *fp, int delay_num, int left, int right);
//------------------------
void setup();
int getCM();


int main(void)       
{
 FILE *fp;
 fp = fopen("/dev/servoblaster", "w");
 if (fp == NULL) {
   printf("Error opening file\n");
   exit(0);
 }

int left = 128; 
int right = 146;
int left_num = 0;
int right_num = 0;

 setup();

double err, derr, ierr, lasterr, sumerr, output;
double kp = .1;
double kd = .1;
double ki = 0;

double commandedPos = 30;
int count;
double driveleft;
double driveright;

printf("Distance\terr\tderr\tierr\touput\tleft\tright\n");

 while (1) {
       
       printf("%dcm\t", getCM());
       count++;
      

       err = commandedPos - getCM();
       derr = lasterr - err;
       lasterr = err;
       sumerr += err;
       ierr = sumerr / count;


       output = ki*ierr + kp*err - kd * derr;
       driveleft = min(max(128+output,100.0), 200.0);
       driveright = min(max(146+output,100.0), 200.0);

       printf("%f\t%f\t%f\t%f\t%f\t%f\t\n", err, derr, ierr,output,driveleft,driveright);
       drive(fp, 200, driveleft, driveright);
       //drive(fp, 100, 128, 146);

       // if(getCM() < 25) 
       // {
       //        //increase speed on left wheel (veer right)
       //        printf("greater than 30, veer right by %d\n", getCM(), right_num);
       //        right_num=3;
       //        left_num = 0;
       //        drive(fp, 100, 128, 149);//left(fp, 100);
       // } //if
       // else if(getCM() > 30) 
       // {      
       //        //increase speed on right wheel (veer left)
       //        printf("less than 25, veer left by %d\n", getCM(), left_num);
       //        left_num=3;
       //        right_num = 0;
       //        drive(fp, 100, 125, 146);//right(fp, 100);
       // } //else if
       // else 
       // {
       //        printf("in range, left = %d, right = %d\n", getCM(),left, right);
       //        left_num = 0;
       //        right_num = 0;
       //        drive(fp, 100, 128, 146);//right(fp, 100);
       // }

       //forward(fp, 100);

       //stop(fp, 3000);
       //delay(1000);
 }
 fclose(fp);
 return 0;
}

void stop(FILE *fp, int delay_num) {
       fprintf(fp, "0=123\n"); //Stop //LEFT WHEEL
       fprintf(fp, "1=151\n"); //Stop
       fflush(fp);
       delay(delay_num);
} //stop()

void forward(FILE *fp, int delay_num) {
       fprintf(fp, "0=128\n"); //Servo#0, Counter Clockwise //LEFT WHEEL
       fprintf(fp, "1=147\n"); //Servo#1, Counter Clockwise
       fflush(fp);
       delay(delay_num);
}

void backward(FILE *fp, int delay_num) {
       fprintf(fp, "0=120\n"); //Clockwise //LEFT WHEEL
       fprintf(fp, "1=156\n"); //Clockwise
       fflush(fp);
       delay(delay_num);
}

void right(FILE *fp, int delay_num) {
       fprintf(fp, "0=128\n"); //Servo#0, Counter Clockwise //LEFT WHEEL
       fprintf(fp, "1=156\n"); //Servo#1, Counter Clockwise
       fflush(fp);
       delay(delay_num);
}

void left(FILE *fp, int delay_num) {
       fprintf(fp, "0=120\n"); //Clockwise //LEFT WHEEL
       fprintf(fp, "1=147\n"); //Clockwise
       fflush(fp);
       delay(delay_num);
}

void drive(FILE *fp, int delay_num, int left, int right) {
       fprintf(fp, "0=%d\n", left); //Clockwise //LEFT WHEEL
       fprintf(fp, "1=%d\n", right); //Clockwise
       fflush(fp);
       delay(delay_num);
}

//---------------------------------

void setup() {
        wiringPiSetup();
}

int getCM() {
        pinMode(TRIG, OUTPUT);
        digitalWrite(TRIG, LOW);
        delay(30);
        digitalWrite(TRIG, HIGH);
        delay(50);
        digitalWrite(TRIG, LOW);
        pinMode(TRIG, INPUT);

        //Wait for echo start
        while(digitalRead(TRIG) == LOW);

        //Wait for echo end
        long startTime = micros(); //this stands for microseconds
        while(digitalRead(TRIG) == HIGH);
        long travelTime = micros() - startTime;

        //Get distance in cm
        int distance = travelTime / 58;

        return distance;
}

