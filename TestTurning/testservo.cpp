#include <stdio.h>
#include <stdlib.h>
#include "wiringPi.h"

void turnLeft(FILE *fp, int time)
{
    //printf("turn left"); 
    puts("Update simulated position to turn left");
    fprintf(fp,"0=141\n");//turn right
    fprintf(fp,"1=142\n");
    fflush(fp);
    delay(time); //600
    fprintf(fp, "0=151\n");//stopped - reach middle
    fprintf(fp, "1=152\n");
    fflush(fp);

}

void turnRight(FILE *fp, int time)
{
    puts("Update simulated position to turn right");

    fprintf(fp,"0=163\n");//turn right
    fprintf(fp,"1=162\n");
    fflush(fp);
    delay(time); //600
    fprintf(fp, "0=151\n");//stopped - reach middle
    fprintf(fp, "1=152\n");
    fflush(fp);

}

void moveForward(FILE *fp, int time)
{
    puts("Update simulated position to move forward");

    printf("move forward"); 
    fprintf(fp,"0=163\n");//moves forward
    fprintf(fp,"1=142\n");
    fflush(fp);
    delay(time);
    fprintf(fp, "0=151\n");//stopped - reach middle
    fprintf(fp, "1=152\n");
    fflush(fp);
}

void turnBackward(FILE *fp, int time)
{

    puts("Update simulated position to move backward");
    fprintf(fp,"0=163\n");//turn right
    fprintf(fp,"1=162\n");
    fflush(fp);
    delay(time); //1400
    fprintf(fp, "0=151\n");//stopped
    fprintf(fp, "1=152\n");
    fflush(fp);
}



int main(void)
{
    FILE *fp;
    fp = fopen("/dev/servoblaster", "w");
    if (fp == NULL) {
        printf("Error opening file\n");
        exit(0);
    }

    int turn = -1;
    printf("choose turn to diagnose (1-4): Left, Right, Forward, Backward:");
    const char* direction[] = {"Left", "Right", "Forward","Backward"};
    scanf("%d", &turn);

    int val = -1;
    int max = -1;
    int min = -1;

    printf("enter min max time:");
    scanf("%d %d", &min, &max);

    int error = -1;

    while (1) {

        val = (min+max)/2;
        printf("-------\nRunning %s with val %d\n", direction[turn], val);

        switch(turn){
            case 1: turnLeft(fp,val); break;
            case 2: turnRight(fp,val); break;
            case 3: moveForward(fp,val); break;
            case 4: turnBackward(fp,val); break;
        }

        printf("Is it too much(1) or too little(-1)?\n");
        scanf("%d", &error);

        if (error > 0) max = val;
        else if (error < 0) min = val;
        else if (error == 0) {
            printf("The ideal time for going %s is %d\n", direction[turn], val);
            fclose(fp);
            return 0;
        }
    }
    fclose(fp);
    return 0;
}
void turnLeft(FILE *fp, int time)
{
    //printf("turn left"); 
    puts("Update simulated position to turn left");
    fprintf(fp,"0=141\n");//turn right
    fprintf(fp,"1=142\n");
    fflush(fp);
    delay(time); //600
    fprintf(fp, "0=151\n");//stopped - reach middle
    fprintf(fp, "1=152\n");
    fflush(fp);

}

void turnRight(FILE *fp, int time)
{
    puts("Update simulated position to turn right");

    fprintf(fp,"0=163\n");//turn right
    fprintf(fp,"1=162\n");
    fflush(fp);
    delay(time); //600
    fprintf(fp, "0=151\n");//stopped - reach middle
    fprintf(fp, "1=152\n");
    fflush(fp);

}

void moveForward(FILE *fp, int time)
{
    puts("Update simulated position to move forward");

    printf("move forward"); 
    fprintf(fp,"0=163\n");//moves forward
    fprintf(fp,"1=142\n");
    fflush(fp);
    delay(time);
    fprintf(fp, "0=151\n");//stopped - reach middle
    fprintf(fp, "1=152\n");
    fflush(fp);
}

void turnBackward(FILE *fp, int time)
{

    puts("Update simulated position to move backward");
    fprintf(fp,"0=163\n");//turn right
    fprintf(fp,"1=162\n");
    fflush(fp);
    delay(time); //1400
    fprintf(fp, "0=151\n");//stopped
    fprintf(fp, "1=152\n");
    fflush(fp);
}
