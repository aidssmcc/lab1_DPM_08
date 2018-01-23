package ca.mcgill.ecse211.lab1;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class PController implements UltrasonicController {

  /* Constants */
  private static final int MOTOR_SPEED = 300;
  private static final int FILTER_OUT = 20;
  private static final int MIN_SPEED = 140;
  private static final int P_OFFSET = 10;
  private static final int RATIO_MULTIPLIER = 2;
  private static final int THRESH = 60;
  private static final int CORNER_TURN = 47;
  private static final int CORNER_MAX = 170;

  private int cornerMode = 0;
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
	  int speed;	//speed used later to hold proportion from MOTOR_SPEED
	  
	  if (Math.abs(ratio) < bandWidth) {
		  //goldilocks zone
		  return MOTOR_SPEED;
	  }
	  
	  //various cases the robot can be in
	  if (ratio < (-1 * P_OFFSET)) {
		  //too close to wall
		  
		  if(lOrR) {
			  //left motor
			  return MOTOR_SPEED;
		  } else {
			  //right motor
			  //reverse direction
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
		  
	  } 
	  else if(ratio > THRESH) {
		  //robot cannot see wall
		  //special case causes robot to enter a corner mode
		  
		  if(cornerMode > CORNER_TURN) {
			  //stage 1 of corner mode
			  //go straight
			  cornerMode--;
			  return MOTOR_SPEED;
		  } else if (cornerMode > 0) {
			  //stage 2 of corner mode
			  //turn for a while, determined by CORNER_TURN
			  cornerMode--;
			  if(lOrR) {
				  return (-1 * MOTOR_SPEED);
			  } else {
				  return MOTOR_SPEED;
			  }
		  } else {
			  //at a corner and cornerMode inactive, activate it
			  cornerMode = CORNER_MAX;
			  return MOTOR_SPEED;
		  }
		  
	  } else if(ratio > P_OFFSET) {
		  //far away from wall, but still sees it
		  if(lOrR) {
			  //left motor
			  ratio = capVal(ratio, bandWidth);
			  
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
