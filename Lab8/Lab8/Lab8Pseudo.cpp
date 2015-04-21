Localize() {
	for(int i = 0; i < 4; ++i) {
		turnLeft();
		read();
	} //for

	turnToTarget(position of robot, x, y);
} //localize()


turnToTarget(position of robot, x, y) {
	if(x == 0) {
		if(y > 0) {
			rotateToNorth()
		}
		else if(y < 0) {
			rotateToSouth()		
		}
		else 
			//do nothing
	}
	else if(y == 0) {
		if(x > 0) {
			rotateToEast()
		}
		else if(x < 0) {
			rotateToWest()		
		}
		else 
			//do nothing
	}
} //TurnToTarget

MoveToTarget(int x, int y) {

	if(robot is at target position) {
		//print "You're here!"
	}
	else {
		wallFollow();
	} //else

} //MoveToTarget()


wallFollow() {

	Rotates Sonar towards closest wall //uses info from Localize()

	//move Robot With P Control
	if(distance > 50 after 3 readings) {
		if(Sonar is facing left) {
			rotateSonar(-90)
		}
		else if(Sonar is facing right) {
			rotateSonar(-90)
		}
	} //if
	else if(distance < 30) {
		rotate Sonar towards target
		while(robot is not at target) {
			moveForward()
		} //while
	} //if


} //WallFollow()