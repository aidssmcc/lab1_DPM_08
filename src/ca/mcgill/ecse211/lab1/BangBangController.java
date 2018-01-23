package ca.mcgill.ecse211.lab1;

import lejos.hardware.motor.*;

public class BangBangController implements UltrasonicController {

  private final int bandCenter;
  private final int bandwidth;
  private final int motorLow;
  private final int motorHigh;
  private int distance;
  private int thresh = 60;
  private int cooldown = 0;
  private int moveAway = 90;

  public BangBangController(int bandCenter, int bandwidth, int motorLow, int motorHigh) {
    // Default Constructor
    this.bandCenter = bandCenter;
    this.bandwidth = bandwidth;
    this.motorLow = motorLow;
    this.motorHigh = motorHigh;
    WallFollowingLab.leftMotor.setSpeed(motorHigh); // Start robot moving forward
    WallFollowingLab.rightMotor.setSpeed(motorHigh);
    WallFollowingLab.leftMotor.backward();
    WallFollowingLab.rightMotor.backward();
  }

  @Override
  public void processUSData(int distance) {
    this.distance = distance;
    if (Math.abs(distance - this.bandCenter) < this.bandwidth) {
    	//goldilocks point (just right)
    	WallFollowingLab.leftMotor.setSpeed(motorHigh);
    	WallFollowingLab.rightMotor.setSpeed(motorHigh);
    	WallFollowingLab.leftMotor.backward();
    	WallFollowingLab.rightMotor.backward();
    } else if (distance  < (this.bandCenter)) {
    	//too close to wall
   	 	WallFollowingLab.leftMotor.setSpeed(motorHigh);
   	 	WallFollowingLab.rightMotor.setSpeed(motorHigh);
   	 	WallFollowingLab.leftMotor.forward();
   	 	WallFollowingLab.rightMotor.backward();
    } else if (distance > this.thresh) {
    	//wall not in sight
    	if (this.cooldown == 0) {
    		//Go into a corner mode. 
    		this.cooldown = this.moveAway;
    	} else if (this.cooldown > 20) {
    		//stage 1 of corner mode
        	//drive forward
    		WallFollowingLab.leftMotor.setSpeed(motorHigh);
    		WallFollowingLab.rightMotor.setSpeed(motorHigh);
    		WallFollowingLab.leftMotor.backward();
    		WallFollowingLab.rightMotor.backward();
    	} else {
    		//stage 2 of corner mode
    		//turn sharply
   	 		WallFollowingLab.leftMotor.setSpeed(motorHigh);
   	 		WallFollowingLab.rightMotor.setSpeed(motorHigh);
   	 		WallFollowingLab.leftMotor.backward();
   	 		WallFollowingLab.rightMotor.forward();
    	}
    	//reduce timer on corner mode
    	cooldown--;
    }
    else {
    	//too far from wall, wall still in sight
   	 	WallFollowingLab.leftMotor.setSpeed(motorHigh);
   	 	WallFollowingLab.rightMotor.setSpeed(motorLow);
   	 	WallFollowingLab.leftMotor.backward();
   	 	WallFollowingLab.rightMotor.backward();
    }
  }

  @Override
  public int readUSDistance() {
    return this.distance;
  }
}
