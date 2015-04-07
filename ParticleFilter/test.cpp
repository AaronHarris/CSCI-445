//#define SIMULATION



#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/legacy/legacy.hpp>

#ifndef SIMULATION
#include <wiringPi.h>
#endif

#include <stdio.h> 
#include <stdlib.h> 
#include <math.h> 
#include <iostream>
#include <vector>

#include "Particle2.h"
#include "Util2.h"
#include "Util_Map.h"
#define TRIG 5

#define NORTH 0
#define WEST 1
#define SOUTH 2
#define EAST 3

cv::Point findEdgeCoord(cv::Mat& image3, int angle, cv::Point roboCoord);

void moveParticles(std::vector<Particle>& particles,double angle, double magnitude, double variance);
void turnLeft(FILE *fp);
void turnRight(FILE *fp);
void moveForward(FILE *fp);
void turnBackward(FILE *fp);


void printDirection(int direction) {
	printf("\nYou are now faceing: ");
	if(direction == NORTH) printf("NORTH\n\n");
	else if(direction == WEST) printf("WEST\n\n");
	else if(direction == SOUTH) printf("SOUTH\n\n");
	else if(direction == EAST) printf("EAST\n\n");
	else printf("NO WHERE\n\n");
}

void setup(){
	puts("WiringPiSetup");
#ifndef SIMULATION
	wiringPiSetup();
#endif
}


int getCM() {
#ifdef SIMULATION
	puts("get CM reading based on simulator real position and some noise and distance to wall");
	puts("Simulator, returns 25");
	return 25;
#else
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
	long startTime = micros();
	while(digitalRead(TRIG) == HIGH);
	long travelTime = micros() - startTime;

	//Get distance in cm
	int distance = travelTime / 58;
	return distance;

#endif
}


int main( )
{

	setup();
	//system("/home/pi/enableMotor.sh");
	int fd; 
	unsigned char buf[16]; 
	int left=0;
	int right=0;
	int middle=0;

	int direction = EAST;

	FILE *fp;
#ifndef SIMULATION
	fp = fopen("/dev/servoblaster", "w");
	if(fp==NULL){
		printf("Error opening file\n");
		exit(0);
	}
#endif

	cv::Mat image3;
	image3 = cv::Mat::zeros(300,300,CV_8UC3);

	std::vector<cv::Point> edge;
	std::vector<cv::Point> rocket;
	std::vector<cv::Point> dock;
	cv::Point arcLoad;

	cv::namedWindow("Display", cv::WINDOW_AUTOSIZE); // create a window
	drawMap(image3, edge, rocket, dock, arcLoad); // draw map into window

	cv::Point robot(150,150);
	cv::circle(image3, robot, 5, cv::Scalar(145,255,0),10, 8, 0); // create an object circle at the robot location

	std::vector<Particle> particles;
	cv::Point center(150,150);
	for(int i = 0; i < 15; i++){
		Particle dot(center,1/15.0);
		particles.push_back(dot);
	}
	cv::Point temp;
	std::vector<cv::Point> particleSho(15);

	/*edgeCoord will hold the coordinates of the edge that the robot is facing*/
	cv::Point edgeCoord;
	float roboAngle=90;
	double dist;// = distToEdge(robot, cv::Point(297,robot.y));
	bool exit = false;
	int update_angle=0;
	int update_magnitude=5;
	int update_variance=4;

	while (!exit){ //
		printf("Current robot position:  X-%d, Y-%d\n", robot.x,robot.y);

		cv::imshow("window",image3); // display image
		char k = cv::waitKey(0);
		if(robot.x < 284){
			cv::line(image3, cv::Point(robot.x+12,robot.y), cv::Point(294,robot.y), cv::Scalar(0,0,0), 2, 8, 0);
		}
		update_magnitude=18;
		update_variance=4;


		switch(k)
		{

			case 'w': // up key
				{
					puts("move forward");
					if(direction != NORTH) {
						if(direction == WEST) {
							turnRight(fp);                       
						}
						else if(direction == SOUTH) {
							turnBackward(fp);
						}
						else if(direction == EAST) {
							turnLeft(fp);
						}
						direction = NORTH;
					}
					moveForward(fp);

					robot.y -= update_magnitude;
					update_angle=-90;
					break;
				}
			case 's': // down key
				{
					puts("move backward");
					if(direction != SOUTH) {
						if(direction == WEST) {
							turnLeft(fp);
						}
						else if(direction == NORTH) {
							turnBackward(fp);
						}
						else if(direction == EAST) {
							turnRight(fp);
						}

						direction = SOUTH;
					}
					moveForward(fp);


					robot.y += update_magnitude;
					update_angle=90;           
					break;
				}
			case 'a': // left key
				{   puts("turn left");
					if(direction != WEST) {
						if(direction == NORTH) {
							turnLeft(fp);
						}
						else if(direction == SOUTH) {
							turnRight(fp);
						}
						else if(direction == EAST) {
							turnBackward(fp);
						}
						direction = WEST;
					}

					moveForward(fp);
					robot.x -= update_magnitude;
					update_angle=-180;
					break;
				}
			case 'd': // right key
				{
					puts("turn right");
					if(direction != EAST) {
						if(direction == WEST) {
							turnBackward(fp);
						}
						else if(direction == SOUTH) {
							turnLeft(fp);
						}
						else if(direction == NORTH) {
							turnRight(fp);
						}
						direction = EAST;
					}

					moveForward(fp);
					robot.x += update_magnitude;
					update_angle=0;
					break;
				}
			case 'q': // q for sensing
				{

					int x = getCM();
					printf("Sonar reads: %d cm \n",x);
					updateProbability(particles, particleSho, x);


					drawMap(image3, edge, rocket, dock, arcLoad); // draw map into window
					cv::circle(image3, robot, 5, cv::Scalar(145,255,0),10, 8, 0);  //draw robot location 
					edgeCoord = findEdgeCoord(image3,update_angle,robot); 
					cv::line(image3, cv::Point(robot.x,robot.y), edgeCoord, cv::Scalar(255,255,0), 2, 8, 0);

					for(int i=0; i<15; i++)
					{
						particleSho[i]=findEdgeCoord(image3,update_angle,particles[i].getPosition());
					}

					double weight=0.0;
					cv::Point highProb;
					for(vector<Particle>::iterator i = particles.begin(); i!= particles.end(); i++){

						printf("x: %d y: %d ",i->getPosition().x, i->getPosition().y);
						if (i->getWeight() > weight){
							weight = i->getWeight();
							highProb.x = i->getPosition().x;
							highProb.y = i->getPosition().y;
						}
						cv::circle(image3, i->getPosition(), 2, cv::Scalar(0,255,255),i->getWeight(), 8, 0);
					}
					cv::circle(image3, highProb, 4, cv::Scalar(255,0,255), 5, 8, 0);

					particles = resampleParticles(particles);

					break;
				}
			case 'e': // exit window with ESC key
				{
					exit = true;
					break;
					// default:
				}
		} 
		printDirection(direction);
		switch(k){
			case 'a':
			case 's':
			case 'd':
			case 'w':

				moveParticles(particles,update_angle,update_magnitude,update_variance);
				for(vector<Particle>::iterator i = particles.begin(); i!= particles.end(); i++){

					printf("x: %d y: %d ",i->getPosition().x, i->getPosition().y);
				}
				drawMap(image3, edge, rocket, dock, arcLoad); // draw map into window
				cv::circle(image3, robot, 5, cv::Scalar(145,255,0),10, 8, 0);     //draw robot location                
				drawParticles(image3, particles, particleSho); 

		} 
	}
	return 0;
}

cv::Point findEdgeCoord(cv::Mat& image3, int angle, cv::Point roboCoord)
{
	cv::Point edgeCoord;
	edgeCoord.x = roboCoord.x;
	edgeCoord.y = roboCoord.y;
	//if up key is pressed, loop through pixels above the robot until you detect an edge
	if(angle == -90)
	{
		// as long as we are not hitting the top of our image, keep looping upwards until we find an edge
		for(int j=roboCoord.y; j>0; j--)
		{
			int r = image3.at<cv::Vec3b>(roboCoord.x,j)[0];
			int g = image3.at<cv::Vec3b>(roboCoord.x,j)[1];
			int b = image3.at<cv::Vec3b>(roboCoord.x,j)[2];//check color at pixel (x,y)
			if(r==255 & g == 0 & b == 0){ //If we find an edge,
				edgeCoord.y = j;
				break;
			}
		}
	}
	//if down key is pressed, loop through pixels below the robot until you detect an edge
	else if(angle == 90)
	{
		// as long as we are not hitting the bottom of our image, keep looping down until we find an edge
		for(int j=roboCoord.y; j<300; j++)
		{
			int r = image3.at<cv::Vec3b>(roboCoord.x,j)[0];
			int g = image3.at<cv::Vec3b>(roboCoord.x,j)[1];
			int b = image3.at<cv::Vec3b>(roboCoord.x,j)[2];//check color at pixel (x,y)
			if(r==255 & g == 0 & b == 0){ //If we find an edge,
				edgeCoord.y = j;
				break;
			}
		}
	}
	//if left key is pressed, loop through pixels to the left of the robot until you detect an edge
	else if(angle == -180)
	{
		// as long as we are not hitting the left of our image, keep looping left until we find an edge
		for(int j=roboCoord.x; j>0; j--)
		{
			int r = image3.at<cv::Vec3b>(j,roboCoord.y)[0];
			int g = image3.at<cv::Vec3b>(j,roboCoord.y)[1];
			int b = image3.at<cv::Vec3b>(j,roboCoord.y)[2];//check color at pixel (x,y)
			if(r==255 & g == 0 & b == 0){ //If we find an edge,
				edgeCoord.x = j;
				break;
			}
		}
	}
	//if right key is pressed, loop through pixels to the right of the robot until you detect an edge
	else if(angle == 0)
	{
		// as long as we are not hitting the right of our image, keep looping right until we find an edge
		for(int j=roboCoord.x; j<300; j++)
		{
			int r = image3.at<cv::Vec3b>(j,roboCoord.y)[0];
			int g = image3.at<cv::Vec3b>(j,roboCoord.y)[1];
			int b = image3.at<cv::Vec3b>(j,roboCoord.y)[2];//check color at pixel (x,y)
			if(r==255 & g == 0 & b == 0){ //If we find an edge,
				edgeCoord.x = j;
				break;
			}
		}
	}
	return edgeCoord;
}

void moveParticles(std::vector<Particle>& particles, double angle, double magnitude, double variance){
	for(vector<Particle>::iterator i = particles.begin(); i!= particles.end(); i++){
		i->moveParticle(angle, magnitude, variance);
	}
}

void turnLeft(FILE *fp)
{
	//printf("turn left"); 
	puts("Update simulated position to turn left");
	fprintf(fp,"0=141\n");//turn right
	fprintf(fp,"1=142\n");
	fflush(fp);
	delay(600);
	fprintf(fp, "0=151\n");//stopped - reach middle
	fprintf(fp, "1=152\n");
	fflush(fp);

}

void turnRight(FILE *fp)
{
	puts("Update simulated position to turn right");

	fprintf(fp,"0=163\n");//turn right
	fprintf(fp,"1=162\n");
	fflush(fp);
	delay(600);
	fprintf(fp, "0=151\n");//stopped - reach middle
	fprintf(fp, "1=152\n");
	fflush(fp);

}

void moveForward(FILE *fp)
{
	puts("Update simulated position to move forward");

	printf("move forward"); 
	fprintf(fp,"0=163\n");//moves forward
	fprintf(fp,"1=142\n");
	fflush(fp);
	delay(350);
	fprintf(fp, "0=151\n");//stopped - reach middle
	fprintf(fp, "1=152\n");
	fflush(fp);
	delay(100);

}

void turnBackward(FILE *fp)
{

	puts("Update simulated position to move backward");
	fprintf(fp,"0=163\n");//turn right
	fprintf(fp,"1=162\n");
	fflush(fp);
	delay(730);
	fprintf(fp,"0=163\n");//turn right
	fprintf(fp,"1=162\n");
	fflush(fp);
	delay(730);
	fprintf(fp, "0=151\n");//stopped
	fprintf(fp, "1=152\n");
	fflush(fp);
	delay(100); 
}
