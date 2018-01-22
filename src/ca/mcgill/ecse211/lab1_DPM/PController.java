package ca.mcgill.ecse211.lab1_DPM;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class PController implements UltrasonicController {

  /* Constants */
  private static final int MOTOR_SPEED = 200;
  private static final int FILTER_OUT = 20;
  private static final int MIN_SPEED = 75;
  private static final int OFFSET = 10;

  private int thresh = 255;
  private int cooldown = 0;
  private int moveAway = 100;
  private float cornerOffset = 1 / 3;	//less than 1. 1/3 means turning for 1/3 of time in corner mode
  
  private final int bandCenter;
  private final int bandWidth;
  private int distance;
  private int filterControl;

  public PController(int bandCenter, int bandwidth) {
    this.bandCenter = bandCenter;
    this.bandWidth = bandwidth;
    this.filterControl = 0;

    WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED); // Initalize motor rolling forward
    WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED);
    WallFollowingLab.leftMotor.backward();
    WallFollowingLab.rightMotor.backward();
  }

  @Override
  public void processUSData(int distance) {

    // rudimentary filter - toss out invalid samples corresponding to null
    // signal.
    // (n.b. this was not included in the Bang-bang controller, but easily
    // could have).
    //
    if (distance >= 255 && filterControl < FILTER_OUT) {
      // bad value, do not set the distance var, however do increment the
      // filter value
      filterControl++;
    } else if (distance >= 255) {
      // We have repeated large values, so there must actually be nothing
      // there: leave the distance alone
      this.distance = distance;
    } else {
      // distance went below 255: reset filter and leave
      // distance alone.
      filterControl = 0;
      this.distance = distance;
    }

    if (Math.abs(distance - this.bandCenter) < this.bandWidth) {
    	//if in the right zone
 		WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED);
 		WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED);
 		WallFollowingLab.leftMotor.backward();
 		WallFollowingLab.rightMotor.backward();
    } else if (distance > this.thresh) {
    	if (this.cooldown == 0) {
    		this.cooldown = this.moveAway;
    	} else if (this.cooldown > (this.moveAway * cornerOffset)) {
        	//drive forward
    		WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED);
    		WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED);
    		WallFollowingLab.leftMotor.backward();
    		WallFollowingLab.rightMotor.backward();
    		
    	} else {
    		//then turn sharply
     		WallFollowingLab.leftMotor.setSpeed(propCalc(distance, true));
     		WallFollowingLab.rightMotor.setSpeed(propCalc(distance, false));
     		WallFollowingLab.leftMotor.backward();
     		WallFollowingLab.rightMotor.backward();
    	}
    	cooldown--;
    	
  	}else {
 		WallFollowingLab.leftMotor.setSpeed(propCalc(distance, true));
 		WallFollowingLab.rightMotor.setSpeed(propCalc(distance, false));
 		WallFollowingLab.leftMotor.backward();
 		WallFollowingLab.rightMotor.backward();
    }
  }

  /**
   * Helper function calculates the speed a motor should be running at
   * @param dist Current distance from the wall
   * @param lOrR Select calculation for left (true) or right (false) motor
   * @return speed New speed of the motor
   */
  private int propCalc(int dist, boolean lOrR) {
	  int ratio = Math.abs(dist - this.bandCenter);
	  int speed;
	  if (ratio == 0) {
		  //if at right distance, go full speed for either motor
		  return MOTOR_SPEED;
	  }
	  
	  if (ratio > this.bandCenter) {
		  //cap ratio size to that of bandCenter
		  ratio = this.bandCenter;
	  }
	  	  
	  int adjust = MOTOR_SPEED * (ratio / this.bandCenter) + OFFSET;	
	  if (adjust > MOTOR_SPEED) {
		  adjust = MOTOR_SPEED;
	  }

	  if(dist  < (this.bandCenter)) {
		  //too close to wall
		  //left to go fast
		  //right go slow
		  if (lOrR) {
			  if (adjust < MIN_SPEED) {
				  return MIN_SPEED;
			  }
			  return adjust;
		  } else {
			  speed = MOTOR_SPEED - adjust;
			  if (speed < MIN_SPEED) {
				  return MIN_SPEED;
			  }
			  return speed;
		  }
	  } else {
		  //too far from wall
		  //right to go fast
		  //left go slow

		  if (lOrR) {
			  if (adjust < MIN_SPEED) {
				  return MIN_SPEED;
			  }
			  return adjust;
		  } else {
			  speed = MOTOR_SPEED - adjust;
			  if (speed < MIN_SPEED) {
				  return MIN_SPEED;
			  }
			  return speed;
		  }	  
	  }
  }

  @Override
  public int readUSDistance() {
    return this.distance;
  }

}
