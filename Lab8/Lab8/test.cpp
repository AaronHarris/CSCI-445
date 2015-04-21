//*#define SIMULATION */
#ifdef SIMULATION
    const bool simulation=true; //const defaults to static
#else
    const bool simulation=false;
#endif

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
//#include <random>
#define TRIG 5

#define NORTH 0
#define WEST 1
#define SOUTH 2
#define EAST 3

const int N_Particles= 30;


cv::Point findEdgeCoord(cv::Mat& image3, int orientation, cv::Point roboCoord);

void rotate_Sonar(FILE *fp, int sonar_relative_orientation);
void SonarSensing(std::vector<Particle>& particles, double orientation, double magnitude, double variance,cv::Mat& image3,cv::Point& robot);
void moveParticles(std::vector<Particle>& particles,double orientation, double magnitude, double variance);
void moveLeft(FILE *fp,int& orientation);
void moveRight(FILE *fp,int& orientation);
void moveUp(FILE *fp,int& orientation);
void moveDown(FILE *fp,int& orientation);

void moveForward(FILE *fp);
void turnLeft(FILE *fp);
void turnRight(FILE *fp);
void turnBackward(FILE *fp);

int getCM();

void setFalse(bool direction[]) {
	for(int i = 0; i < 4; ++i) {
		direction[4] = false;
	}
}

void localization(int x, int y, FILE *fp, std::vector<Particle>& particles, double orientation, double magnitude, double variance,cv::Mat& image3,cv::Point& robot, int sonar_relative_orientation, /**/ std::vector<cv::Point> edge, std::vector<cv::Point> rocket, std::vector<cv::Point> dock, cv::Point arcLoad);


void turnToTarget(cv::Point& robot, int x, int y);

void drive(FILE *fp, int delay_num, int left, int right) {
	fprintf(fp, "0=%d\n", left); //Clockwise //LEFT WHEEL
	fprintf(fp, "1=%d\n", right); //Clockwise
	fflush(fp);
	delay(delay_num);
}

int wallFollow(int x, int y, FILE *fp, std::vector<Particle>& particles, double orientation, double magnitude, double variance,cv::Mat& image3,cv::Point& robot, int sonar_relative_orientation, /**/ std::vector<cv::Point> edge, std::vector<cv::Point> rocket, std::vector<cv::Point> dock, cv::Point arcLoad, int targetDist, bool isLeft) {

	int left = 151;
	int right = 152;

	double err, lasterr, sumerr, output, driveleft, driveright;
	double kp = .1;

	double commandedPos = targetDist;
		
	//Rotates Sonar towards closest wall //uses info from Localize()
	int rlfactor = 1;
	if (isLeft) {
		// do nothing
	} else { //robot following wall on the right
		rlfactor = -1;
	}
	//move Robot With P Control
	int dist = getCM();
		if (dist < 50) {
			if(robot.x == x && robot.y == y) {
				printf("*****************  You made it!\n");
			} //if
			else {
				err = commandedPos - dist;
				output = kp*err;
				driveleft = min(max(160+ output*rlfactor,left-20.0), left+20.0);
				driveright = min(max(145+ output*rlfactor,right-20.0), right+20.0);

				printf("%f\t%f\t%f\t%f\t\n", err, output,driveleft,driveright);
				drive(fp, 200, driveleft, driveright);

				// if(distance > 50 after 3 readings) {
				// 	if(Sonar is facing left) {
				// 		rotateSonar(-90)
				// 	} //if
				// 	else if(Sonar is facing right) {
				// 		rotateSonar(90)
				// 	} //else if
				// } //if
				// else if(distance < 30) {
				// 	rotate Sonar towards target
				// 	while(robot.x != x && robot.y != y) {
				// 		moveForward(fp);
				// 	} //while
				// } //else if
			} //else
		} // if dist < 50	
	return dist;


} //WallFollow()

bool checkforWall(cv::Mat image3,cv::Point xy);

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

int getCM_simulator(cv::Mat& image3,int orientation,cv::Point point ){

	cv::Point edgepoint=findEdgeCoord(image3,orientation,point);
	float dist=distToEdge(point,edgepoint);
	return (int)dist;

}

int getCM() {
#ifdef SIMULATION
	puts("simulation mode, no access to sonar, use getCM_simulator instead");
	return 0;
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

	cv::Point robot(50,150);
	cv::circle(image3, robot, 5, cv::Scalar(145,255,0),10, 8, 0); // create an object circle at the robot location

	std::vector<Particle> particles;
	for(int i = 0; i < N_Particles; i++){
		cv::Point center(300.0*rand()/RAND_MAX,300.0*rand()/RAND_MAX);

		Particle dot(center,1.0/N_Particles);
		particles.push_back(dot);
	}
	bool exit = false;
	int sonar_relative_orientation=0;
	int orientation=0;
	int update_magnitude=5;
	int update_variance=3;
	int update_angle;

	int userX = -1;
	int userY = -1;
	printf("Give me some points (x y): ");
	scanf("%d %d", &userX, &userY);

	localization(userX, userY, fp, particles, sonar_relative_orientation,  update_magnitude, update_variance, image3, robot, sonar_relative_orientation,/**/ edge, rocket, dock, arcLoad);


	//* Follow wall on left
	rotate_Sonar(fp, -75);
	int counts = 0;
	int dist;
	while (counts < 1) {
		dist = wallFollow(userX, userY, fp, particles, sonar_relative_orientation,  update_magnitude, update_variance, image3, robot, sonar_relative_orientation,/**/ edge, rocket, dock, arcLoad, 25, true);		
		if (dist > 50) counts++;
	}

	//* Follow wall on the right
	rotate_Sonar(fp, 75);
	counts = 0;
	while (counts < 1) {
		dist = wallFollow(userX, userY, fp, particles, sonar_relative_orientation,  update_magnitude, update_variance, image3, robot, sonar_relative_orientation,/**/ edge, rocket, dock, arcLoad, 25, false);		
		if (dist > 50) counts++;
	}
	//* Follow wall on the left
	rotate_Sonar(fp, -75);
	counts = 0;
	while (counts < 2) {
		dist = wallFollow(userX, userY, fp, particles, sonar_relative_orientation,  update_magnitude, update_variance, image3, robot, sonar_relative_orientation,/**/ edge, rocket, dock, arcLoad, 25, true);		
		counts++;
	}
	// aproach
	rotate_Sonar(fp, 0);
	while (getCM() > 20){
		drive(fp, 200, 160, 145);
	}
	drive(fp, 500, 151, 152);


	
	/*while(1) {
		if(robot.x == userX && robot.y == userY) {
			printf("*****************  You made it!\n");
		} //if
		else {
			
		} //else
		
	}*/

	//moveToTarget(fp, robot, userX, userY);

	while (!exit){ //
		printf("Current robot position:  X-%d, Y-%d\n", robot.x,robot.y);
		cv::imshow("Display",image3); // display image
		char k = cv::waitKey(0);

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
					puts("sensing");
					drawMap(image3, edge, rocket, dock, arcLoad); // draw map into window
					int sonar_orientation=orientation+sonar_relative_orientation; 
					//     printf("Sonar:  orientation %d, rel_orientation %d, sonar_orientation %d\n",orientation,sonar_relative_orientation,sonar_orientation);
					if(sonar_orientation==-180)
						sonar_orientation=180;
					else if(sonar_orientation==270)
						sonar_orientation=-90;
					SonarSensing(particles, sonar_orientation,  update_magnitude, update_variance, image3, robot); 

					break;
				}
			case 'z': //rotate Sonar left
				{
					sonar_relative_orientation=-90;
					rotate_Sonar(fp, sonar_relative_orientation);
					break;
				}
			case 'x': //rotate Sonar center
				{
					sonar_relative_orientation=0;
					rotate_Sonar(fp, sonar_relative_orientation);
					break;
				}
			case 'c': //rotate Sonar right
				{
					sonar_relative_orientation=90;
					rotate_Sonar(fp, sonar_relative_orientation);
					break;
				}

			case 'e': // exit window with ESC key
				{
					exit = true;
					break;
					// default:
				}
		} 
		switch(k){
			case 'a':
			case 's':
			case 'd':
			case 'w':

				moveParticles(particles,orientation,update_magnitude,update_variance);
			case 'z':
			case 'x':
			case 'c':
				//           for(vector<Particle>::iterator i = particles.begin(); i!= particles.end(); i++)
				//             printf("x: %d y: %d weight: %f \n",i->getPosition().x, i->getPosition().y, i->getWeight());
				drawMap(image3, edge, rocket, dock, arcLoad); // draw map into window
				cv::circle(image3, robot, 5, cv::Scalar(145,255,0),10, 8, 0);     //draw robot location                
				drawParticles(image3, particles); 
				drawOrientation(image3,orientation,sonar_relative_orientation,robot);
				break;

		} 

	}
	return 0;
}



bool checkforWall(cv::Mat image3,cv::Point xy){
	int b = image3.at<cv::Vec3b>(xy)[0];
	int g = image3.at<cv::Vec3b>(xy)[1];
	int r = image3.at<cv::Vec3b>(xy)[2];//check color at pixel (x,y)
	if(b==255 & g == 0 & r == 0)
		return 1;
	else
		return 0;
}

cv::Point findEdgeCoord(cv::Mat& image3, int orientation, cv::Point roboCoord)
{
	cv::Point edgeCoord;
	edgeCoord.x = roboCoord.x;
	edgeCoord.y = roboCoord.y;

	switch(orientation){
		//if up key is pressed, loop through pixels above the robot until you detect an edge
		case 270:
		case -90:
			for(int j=roboCoord.y; j>0; j--)
				if(checkforWall(image3,cv::Point(roboCoord.x,j))){
					edgeCoord.y = j;
					break;
				}
			break;
			//if down key is pressed, loop through pixels below the robot until you detect an edge
		case 90:
			// as long as we are not hitting the bottom of our image, keep looping down until we find an edge
			for(int j=roboCoord.y; j<300; j++)
				if(checkforWall(image3,cv::Point(roboCoord.x,j))){
					edgeCoord.y = j;
					break;        
				}
			break;
			//if left key is pressed, loop through pixels to the left of the robot until you detect an edge
		case 180:
		case -180:
			// as long as we are not hitting the left of our image, keep looping left until we find an edge
			for(int j=roboCoord.x; j>0; j--)
				if(checkforWall(image3,cv::Point(j,roboCoord.y))){
					edgeCoord.x = j;
					break;
				}
			break;
			//if right key is pressed, loop through pixels to the right of the robot until you detect an edge
		case 0:
			// as long as we are not hitting the right of our image, keep looping right until we find an edge        
			for(int j=roboCoord.x; j<300; j++)
				if(checkforWall(image3,cv::Point(j,roboCoord.y))){
					edgeCoord.x = j;
					break;
				}         
			break;
	}

	return edgeCoord;
}

void moveParticles(std::vector<Particle>& particles, double orientation, double magnitude, double variance){
	for(vector<Particle>::iterator i = particles.begin(); i!= particles.end(); i++){
		i->moveParticle(orientation, magnitude, variance);
	}
}



void SonarSensing(std::vector<Particle>& particles, double sonar_orientation, double magnitude, double variance,cv::Mat& image3,cv::Point& robot){
	std::vector<cv::Point> particle_closest_edge_point(N_Particles);
	/*edgeCoord will hold the coordinates of the edge that the robot is facing*/
	cv::Point edgeCoord;
	double dist;// = distToEdge(robot, cv::Point(297,robot.y));
	int x;
	if(simulation){
		//  int choice= rand()%particles.size();
		x= getCM_simulator(image3,sonar_orientation,robot);//particles[choice].getPosition() );
	}else{
		x = getCM();
	}
	printf("sonar orientation %f\n",sonar_orientation);
	printf("Sonar reads: %d cm \n",x);
	for(int i=0; i<N_Particles; i++){
		particle_closest_edge_point[i]=findEdgeCoord(image3,sonar_orientation,particles[i].getPosition());
	}

	updateProbability(particles, particle_closest_edge_point, x);
	cv::circle(image3, robot, 5, cv::Scalar(145,255,0),10, 8, 0);  //draw robot location 
	edgeCoord = findEdgeCoord(image3,sonar_orientation,robot); 
	printf("edgecoord: %d %d\n", edgeCoord.x, edgeCoord.y);
	cv::line(image3, cv::Point(robot.x,robot.y), edgeCoord, cv::Scalar(255,255,0), 2, 8, 0);

	double weight=0.0;
	cv::Point highProb;

	for(vector<Particle>::iterator i = particles.begin(); i!= particles.end(); i++){
		printf("x: %d y: %d ",i->getPosition().x, i->getPosition().y);
		if (i->getWeight() > weight){
			weight = i->getWeight();
			highProb.x = i->getPosition().x;
			highProb.y = i->getPosition().y;
		}

		cv::circle(image3, i->getPosition(), 2, cv::Scalar(0,255,255),5*i->getWeight(), 8, 0);
	}
	cv::circle(image3, highProb, 4, cv::Scalar(255,0,255), 5, 8, 0);
	robot=highProb;
	particles = resampleParticles(particles);
}

void moveLeft(FILE *fp,int& orientation)
{
	switch(orientation){
		case 0:
			turnRight(fp);
			turnRight(fp);
			break;
		case 90:
			turnRight(fp);
			break;
		case -90:
			turnLeft(fp);
			break;
	}
	orientation = 180;
	moveForward(fp);
}

void moveRight(FILE *fp,int& orientation)
{
	switch(orientation){
		case 180:
			turnRight(fp);
			turnRight(fp);
			break;
		case -90:
			turnRight(fp);
			break;
		case 90:
			turnLeft(fp);
			break;
	}
	orientation = 0;
	moveForward(fp);

}


void moveUp(FILE *fp,int& orientation)
{
	switch(orientation){
		case 0:
			turnLeft(fp);
			break;
		case 90:
			turnLeft(fp);
			turnLeft(fp);
			break;
		case 180:
			turnRight(fp);
			break;
	}
	orientation = -90;
	moveForward(fp);

}

void moveDown(FILE *fp,int& orientation)
{
	switch(orientation){
		case -90:
			turnRight(fp);
			turnRight(fp);
			break;
		case 0:
			turnRight(fp);
			break;
		case 180:
			turnLeft(fp);
			break;
	}
	orientation = 90;
	moveForward(fp);

}

void rotate_Sonar(FILE *fp, int x){
#ifndef SIMULATION
	int cmdpos = (int) ((1.0/1080)*x*x - (11.0/12)*x +125);
	printf("ROTATE SONAR\n     2=%d\n",cmdpos);
	fprintf(fp,"2=%d\n",cmdpos); 
	fflush(fp);
	delay(1000);
#endif
}

void turnLeft(FILE *fp)
{
	//printf("turn left"); 
	puts("Update simulated position to turn left");
	fprintf(fp,"0=141\n");//turn right
	fprintf(fp,"1=142\n");
	fflush(fp);
	delay(920);
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
	delay(850);
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
	delay(1460);
	fprintf(fp, "0=151\n");//stopped
	fprintf(fp, "1=152\n");
	fflush(fp);
	delay(300); 
}

void localization(int x, int y, FILE *fp, std::vector<Particle>& particles, double orientation, double magnitude, double variance,cv::Mat& image3,cv::Point& robot, int sonar_relative_orientation,/**/ std::vector<cv::Point> edge, std::vector<cv::Point> rocket, std::vector<cv::Point> dock, cv::Point arcLoad) {
		rotate_Sonar(fp, -90);
		drawMap(image3, edge, rocket, dock, arcLoad);
		SonarSensing(particles, orientation,  magnitude, variance, image3, robot); 
		cv::imshow("Display",image3);
		cv::waitKey(30);
		delay(1000);

		rotate_Sonar(fp, 0);
		drawMap(image3, edge, rocket, dock, arcLoad);
		SonarSensing(particles, orientation,  magnitude, variance, image3, robot); 
		cv::imshow("Display",image3);
		cv::waitKey(30);
		delay(1000);

		rotate_Sonar(fp, 90);
		drawMap(image3, edge, rocket, dock, arcLoad);
		SonarSensing(particles, orientation,  magnitude, variance, image3, robot); 
		cv::imshow("Display",image3);
		cv::waitKey(30);
		delay(1000);

		turnRight(fp);
		/*drawMap(image3, edge, rocket, dock, arcLoad);
		SonarSensing(particles, orientation,  magnitude, variance, image3, robot); 
		cv::imshow("Display",image3);*/
		cv::waitKey(30);
		delay(1000);

		rotate_Sonar(fp, 0);
		drawMap(image3, edge, rocket, dock, arcLoad);
		SonarSensing(particles, orientation,  magnitude, variance, image3, robot); 
		cv::imshow("Display",image3);
		cv::waitKey(30);
		delay(1000);

		//orientation
		turnLeft(fp);
		rotate_Sonar(fp, 0);
		delay(1000);

		drawMap(image3, edge, rocket, dock, arcLoad); // draw map into window
		cv::circle(image3, robot, 5, cv::Scalar(145,255,0),10, 8, 0);     //draw robot location                
		drawParticles(image3, particles); 
		drawOrientation(image3,orientation,sonar_relative_orientation,robot);


	//turnToTarget(robot, x, y);

		cv::imshow("Display",image3);
		cv::waitKey(30);
	

} //localization()

/*
void turnToTarget(cv::Point& robot, int x, int y) {
	if(x == 0) {
		if(y > 0) {
			rotateToNorth();
		}
		else if(y < 0) {
			rotateToSouth();
		}
		else {}
			//do nothing
	}
	else if(y == 0) {
		if(x > 0) {
			rotateToEast();
		}
		else if(x < 0) {
			rotateToWest();
		}
		else {}
			//do nothing
	}
} //turnToTarget()
*/