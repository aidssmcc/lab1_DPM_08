package ca.mcgill.ecse211.lab1_DPM;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class PController implements UltrasonicController {

  /* Constants */
  private static final int MOTOR_SPEED = 200;
  private static final int FILTER_OUT = 40;	//30
  private static final int MIN_SPEED = 140;
  private static final int P_OFFSET = 10;
  private static final int OUTSIDE_RAT_NUM = 2;	//2 previous values that worked
  private static final int OUTSIDE_RAT_DEN = 3;	//3
  private static final int RATIO_MULTIPLIER = 2;
  private static final int THRESH = 60;
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
    
    int lSpeed = (propCalc(distance, false));
    int rSpeed = (propCalc(distance, true));
    
    if (lSpeed < 0) {
    	WallFollowingLab.leftMotor.setSpeed(-1 * lSpeed);
    	WallFollowingLab.leftMotor.forward();
    } else {
    	WallFollowingLab.leftMotor.setSpeed(lSpeed);
    	WallFollowingLab.leftMotor.backward();
    }
    if (rSpeed < 0) {
    	WallFollowingLab.rightMotor.setSpeed(-1 * rSpeed);
    	WallFollowingLab.rightMotor.forward();
    } else {
    	WallFollowingLab.rightMotor.setSpeed(rSpeed);
    	WallFollowingLab.rightMotor.backward();
    }
  }
  
  /**
   * Helper function to cap a value
   * @param value
   * @param cap
   * @return capped value if value > cap, returns the cap. Else the value
   */
  private int capVal(int value, int cap) {
	  if (value > cap) {
		  return cap;
	  }
	  return value;
  }

  /**
   * Helper function to cap a value to a minimum
   * @param value
   * @param cap
   * @return capped value if value < cap, returns the cap. Else the value
   */
  private int minCapVal(int value, int cap) {
	  if (value < cap) {
		  return cap;
	  }
	  return value;
  }
  
  /**
   * Helper function calculates the speed a motor should be running at
   * @param dist Current distance from the wall
   * @param lOrR Select calculation for left (true) or right (false) motor
   * @return speed New speed of the motor. Returning a negative value would
   * mean that the motor should run in the reverse direction
   */
  private int propCalc(int dist, boolean lOrR) {
	  int ratio = dist - bandCenter;	//distance from center
	  int speed;
	  
	  if (Math.abs(ratio) < bandWidth) {
		  //goldilocks zone
		  return MOTOR_SPEED;
	  }
	  if (ratio < (-1 * P_OFFSET)) {
		  //too close to wall
		  if(lOrR) {
			  //left motor
			  return MOTOR_SPEED;
		  } else {
			  //right motor
			  //reverse
			  speed = MOTOR_SPEED * ratio / bandCenter ;
			  speed = minCapVal(speed, -1 * MIN_SPEED);
			  return speed;
		  }
		  
	  } else if (ratio < 0) {
		  //Near zone but still not close
		  if(lOrR) {
			  //left
			  return MOTOR_SPEED;
		  } else {
			  //right
			  speed = MOTOR_SPEED / ratio * -1;
			  speed = minCapVal(speed, MIN_SPEED);
			  return speed;
		  }
		  
	  } else if(ratio > THRESH) {
		  //robot cannot see wall
//		  if (this.cooldown <= 0) {
//			  this.cooldown = this.moveAway;
//		  }
//		  if (this.cooldown > (this.moveAway * cornerOffset)) {
//	        	//drive forward for some time
//			  return MOTOR_SPEED;
//		  } else {
		  
		  
		  if(lOrR) {
			  //left
			  return MOTOR_SPEED * OUTSIDE_RAT_NUM / OUTSIDE_RAT_DEN;
		  } else {
			  //right
			  return MOTOR_SPEED;  
		  }
		  
		  
//		  }
//		  cooldown--;
		  
	  } else if(ratio > P_OFFSET) {
		  //far away from wall, but still sees it
		  if(lOrR) {
			  //left motor
			  ratio = capVal(ratio, bandWidth);
			  
//			  speed = MOTOR_SPEED * ratio / bandWidth * -1;
//			  speed = capVal(speed, -1 * MIN_SPEED);
			  
			  speed = MOTOR_SPEED * RATIO_MULTIPLIER / ratio;
			  speed = minCapVal(speed, MIN_SPEED);
			  return speed;
		  } else {
			  //right motor
			  return MOTOR_SPEED;
		  }
		  
	  } else {
		  //close to center, ratio positive
		  if(lOrR) {
			  //left
			  speed = MOTOR_SPEED * RATIO_MULTIPLIER / ratio;
			  speed = minCapVal(speed, MIN_SPEED);
			  return speed;
		  } else {
			  //right
			  return MOTOR_SPEED;
		  }
	  }
  }

  @Override
  public int readUSDistance() {
    return this.distance;
  }

}
