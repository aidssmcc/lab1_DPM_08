package ca.mcgill.ecse211.lab1_DPM;

public interface UltrasonicController {

  public void processUSData(int distance);

  public int readUSDistance();
}
