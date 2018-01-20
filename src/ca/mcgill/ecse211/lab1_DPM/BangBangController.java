package ca.mcgill.ecse211.lab1_DPM;

import lejos.hardware.motor.*;

public class BangBangController implements UltrasonicController {

  private final int bandCenter;
  private final int bandwidth;
  private final int motorLow;
  private final int motorMed;
  private final int motorHigh;
  private int distance;
  private int thresh = 60;
  private int cooldown = 0;
//  private int tooClose = 5;
  private int moveAway = 100;

  public BangBangController(int bandCenter, int bandwidth, int motorLow, int motorHigh, int motorMed) {
    // Default Constructor
    this.bandCenter = bandCenter;
    this.bandwidth = bandwidth;
    this.motorLow = motorLow;
    this.motorHigh = motorHigh;
    this.motorMed = motorMed;
    WallFollowingLab.leftMotor.setSpeed(motorHigh); // Start robot moving forward
    WallFollowingLab.rightMotor.setSpeed(motorHigh);
    WallFollowingLab.leftMotor.backward();
    WallFollowingLab.rightMotor.backward();
  }

  @Override
  public void processUSData(int distance) {
    this.distance = distance;
//    if ( cooldown == 0 && distance < this.tooClose ) {
//    	this.cooldown = moveAway;
//    }
//    if (cooldown > 0) {
//    	//if too close to the wall, turn away quickly
//    	WallFollowingLab.leftMotor.setSpeed(motorLow);
//    	WallFollowingLab.rightMotor.setSpeed(motorHigh);
//    	WallFollowingLab.leftMotor.backward();
//    	WallFollowingLab.rightMotor.backward();
//    	cooldown--;
//    } else
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
    	if (this.cooldown == 0) {
    		this.cooldown = this.moveAway;
    	} else if (this.cooldown > 20) {
        	//drive forward
    		WallFollowingLab.leftMotor.setSpeed(motorHigh);
    		WallFollowingLab.rightMotor.setSpeed(motorHigh);
    		WallFollowingLab.leftMotor.backward();
    		WallFollowingLab.rightMotor.backward();
    	} else {
    		//then turn sharply
   	 		WallFollowingLab.leftMotor.setSpeed(motorHigh);
   	 		WallFollowingLab.rightMotor.setSpeed(motorHigh);
   	 		WallFollowingLab.leftMotor.backward();
   	 		WallFollowingLab.rightMotor.forward();
    	}
    	cooldown--;
    }
    else {
    	//too far from wall
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
