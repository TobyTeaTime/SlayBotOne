// https://docs.wpilib.org/en/stable/index.html for documentation
// https://store.ctr-electronics.com/content/api/java/html/index.html

/*
Running the program:
press ctrl + shift + p
look up "WPILib: Simulate Robot Code on Desktop"
press "halsim_gui.dll"
assign desired system joystick to virtual joystick
click "Map Gamepad"
click "Teleoperated"
*/

package frc.robot;

import static frc.robot.common.RobotMap.*;

// Imports neccessary methods from wpi or ctre

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.networktables.NetworkTableEntry;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

// Start of robot code, declare variables to be used in robot code here
@SuppressWarnings("FieldCanBeLocal")
public class Robot extends TimedRobot {
  private static boolean press = false;
  public static boolean overide = false;
  double fineControlSpeedDouble = .6;
  private final Timer m_timer = new Timer();
  private static double dist = 0;
  

  @Override
  public void disabledInit() {

  }

  @Override
  public void disabledPeriodic() {
  }

  // robotInit = code the roboRIO runs on startup, only runs once
  // Good for things like resets, calibration, turning things on etc
  @Override
  public void robotInit() {
    UsbCamera camera1 = CameraServer.startAutomaticCapture(0);
    UsbCamera camera2 = CameraServer.startAutomaticCapture(1);
    // Format all motor controllers
    intakeWheel.configFactoryDefault();
    storageLeft.configFactoryDefault();
    storageRight.configFactoryDefault();
    flywheel.configFactoryDefault();

    leftAft.configFactoryDefault();
    leftFront.configFactoryDefault();
    rightAft.configFactoryDefault();
    rightFront.configFactoryDefault();

    innerClimbLeft.configFactoryDefault();
    innerClimbRight.configFactoryDefault();
    outerClimbLeft.configFactoryDefault();
    outerClimbRight.configFactoryDefault();

    //innerClimbLeft.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);

/*
   innerClimbLeft.setSelectedSensorPosition(0,0,10);
   innerClimbRight.setSelectedSensorPosition(0,0,10);
   outerClimbLeft.setSelectedSensorPosition(0,0,10);
   outerClimbRight.setSelectedSensorPosition(0,0,10);
   */
    // Invert certain motors depending on which direction they need to go
    // Clock wise is default
    intakeWheel.setInverted(true);
    rightAft.setInverted(true);
    rightFront.setInverted(true);
    storageLeft.setInverted(true);
    storageRight.setInverted(true);
    outerClimbRight.setInverted(true);
    outerClimbLeft.setInverted(false);
    

    // Makes Master Follower pair, RIO sends same signal to two motors
    innerClimbRight.follow(innerClimbLeft);
    outerClimbRight.follow(outerClimbLeft);

    // Sets deadband for the drive train
    // Any imput detected from the controller under 5% will not be used
    m_drive.setDeadband(.05);

    
  }

  // robotPeriodic = code that will be run as long as the robot is on
  // Good for monitoring encoders, sensors, etc (Bot does not need to be enabled)
  @Override
  public void robotPeriodic() {
    // Puts numbers from Encoders on the SmartDashboard network table
    // Open Shuffleboard to see numbers, can be visualized as graphs or other
    // graphic
   // int absolutePosition = innerClimbLeft.getSensorCollection().getPulseWidthPosition();
  //

    ultrasonic.setEnabled(true);

    double distance = ultrasonic.getRangeInches();

    SmartDashboard.putNumber("Ultrasonic",
        distance);

    // Uses the Pressure Switxh to turn off the compressor
    // PSI depends on Pressure Switch, 2022 bot uses a Pressure Switch that switches
    // at 125PSI
    if (compressor.getPressureSwitchValue() == true) {
      compressor.disable();
    }

    SmartDashboard.putNumber("limelight target", limelightTable.getEntry("tv").getDouble(0));
    SmartDashboard.putNumber("limelight angle", limelightTable.getEntry("ty").getDouble(0));
    SmartDashboard.putNumber("limelight distance", getDistance());
    SmartDashboard.putNumber("Target distance", 160);
    SmartDashboard.putBoolean("Flywheel Ready", flywheel.getSelectedSensorVelocity() > 13300);
   // innerClimbLeft.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    //SmartDashboard.putNumber("InnerClimbLeft Encoder", innerClimbLeft.getSelectedSensorPosition());
    // System.out.println(SmartDashboard.getNumber("Target distance",160));

  }
  

  // teleopInit = code that is run on teleop (teleoperation) startup
  // Good for initializing certain manips or pneumatic systems
  @Override
  public void teleopInit() {
    solenoid1.set(Value.kForward);
    solenoid2.set(Value.kForward);

    // Sets solenoids to Forward Position
    // Check with Pneumatic System to verify which position is forward
    

  }

  // teleopPeriodic = code that is run as long as teleop is enabled (runs every
  // 20ms)
  // Going to be the majority of where your code is, troubleshoot and debug
  // thoroug//
  @Override
  public void teleopPeriodic() {

   // double position = innerClimbLeft.getSelectedSensorPosition();
   SmartDashboard.putNumber("RPM", flywheel.getSelectedSensorVelocity());

    compressor.enableDigital();
    
    //System.out.println("compressor on");
    // starts a timer object created previously
    m_timer.start();

    // Asks if the A Button of the Logitech controller was Pressed last time the
    // code was ran
    // If it was, set the solenoids to the Reverse position
    // If it was not, set the solenoids to the Forward position
    if (m_xbox.getXButton()) {
      solenoid1.set(Value.kForward);
      solenoid2.set(Value.kForward);
    } 
    else if (m_xbox.getStartButton()) {
      solenoid1.set(Value.kReverse);
      solenoid2.set(Value.kReverse);
    }

    if (m_xbox.getAButton() == true) {
      intakeWheel.set(.6);
    } else {
      intakeWheel.set(0);
    }
    
    
    if (m_xbox.getBButton() == true) {
      storageGroup.set(.60);
    } else {
      storageGroup.set(0);
    }

    overide = false;
    if (m_xbox.getRightBumper() == true){
        alignDist(90);

     
     overide = true;
    }

    if (m_xbox.getLeftBumper() == true)
    {
        alignX2();
      overide = true;
    }
    if (m_xbox.getLeftTriggerAxis() != 0){
      m_drive.curvatureDrive(m_xbox.getLeftY()*0.5*m_xbox.getLeftTriggerAxis() ,m_xbox.getLeftX()*-0.3*m_xbox.getLeftTriggerAxis() , true);
      overide = true;
    }
    
     
      //alignDist(100);
  
    // Asks if the X Button is currently being pressed
    // If it is, set the intake motor controller to 60% power
    // If the ultrasonic detects less that 4 inches AND the timer has elapsed 2
    // seconds
    // Set storage motor controller at 65% power
    // Set intake motor controller at 40% power
    // If the X Button is not being held, set both motor controllers to 0% power
    
    /*
    if (m_xbox.getXButton()) {
      intakeWheel.set(.6);
      while (distance <= 4) {
        intakeWheel.set(.4);
        storageGroup.set(.65);
      }
    } else {
      intakeWheel.set(0);
      storageGroup.set(0);
    }
    */

    
     // flywheel speed adjust
     /*
      if (m_xbox.getBButtonPressed()){
      //val += .05;
      //System.out.println(val);
      innerClimbLeft.set(ControlMode.PercentOutput, 0.1);
      } else if (m_xbox.getYButtonPressed()) {
      //val -= .05;
      //System.out.println(val);
      innerClimbLeft.set(ControlMode.PercentOutput, -0.1);
      }
     */

    // Asks what the current state of the Right Trigger is on the Logitech
    // Controller
    // This axis in range from 0 to 1
    // If the state is greater that 0, set Flywheel motor controller at 70% power
    // If the state is 0 or lower, set Flywheel motor controller at 0% power
    flywheel.set(ControlMode.PercentOutput,0);
    if (m_xbox.getRightTriggerAxis() > 0.2){
      flywheel.set(ControlMode.PercentOutput, 0.7 * m_xbox.getRightTriggerAxis());
    }
    //flywheel.set(m_xbox.getRightTriggerAxis()*0.65); 
    if (m_joy.getRawButton(8)){
      flywheel.set(ControlMode.PercentOutput,-0.2);
    }
    if (m_joy.getRawButtonReleased(8)){
      flywheel.set(ControlMode.PercentOutput,0);
    }
    if (m_xbox.getYButton()){
      if (flywheel.getSelectedSensorVelocity() > 15000){
        storageGroup.set(0.6);
      }
    }

  

    // Asks which position the Directional Pad of the Logitech controller is in
    // Sets the Climber motor controllers to different values depending on position
    // Motor controllers using position mode, recquires encoder data, as well as
    // Usable Unit equation
    innerClimbLeft.set(ControlMode.PercentOutput, 0); 
    innerClimbLeft.set(ControlMode.PercentOutput, 0);
    outerClimbLeft.set(ControlMode.PercentOutput, 0);
    outerClimbLeft.set(ControlMode.PercentOutput, 0);
    
    if (m_xbox.getPOV() == 0) {
      innerClimbLeft.set(ControlMode.PercentOutput, -.55);
    } else if (m_xbox.getPOV() == 180) {
      innerClimbLeft.set(ControlMode.PercentOutput, .7);
    }
     outerClimbLeft.set(ControlMode.PercentOutput, m_xbox.getRightY() *0.5);

    // Asks which position the Directional Input of the Logitech joystick is in
    // Sets DriveTrain to specified speed declared previously
    // Might need to modify negatives depending on robot or drivetrain type
    if (m_joy.getPOV() == 0) { // Forward
      m_drive.arcadeDrive(fineControlSpeedDouble, 0);
    } else if (m_joy.getPOV() == 90) { // Right
      m_drive.arcadeDrive(0, fineControlSpeedDouble);
    } else if (m_joy.getPOV() == 180) { // Backward
      m_drive.arcadeDrive(-fineControlSpeedDouble, 0);
    } else if (m_joy.getPOV() == 270) { // Left
      m_drive.arcadeDrive(0, -fineControlSpeedDouble);
    } else {
    }

    // Asks what the currnet position of the Y and X axes are
    // Uses axes position from -1 to 1 as motor controller percentage power
    // Might need to modify negatices depending on robot or drivetrain type
    if (!overide){
      m_drive.curvatureDrive(m_joy.getY(), -m_joy.getX()*0.6,true);
    }

    if (m_joy.getRawButton(12)) {
      limelightTable.getEntry("ledMode").setNumber(1);
    } else if (m_joy.getRawButton(11)) {
      limelightTable.getEntry("ledMode").setNumber(3);
    }
  }

  // autonomousInit = code that is run upon the start of autonomous
  // Good for initializing certain manipulators such as pneumatics
  @Override
  public void autonomousInit() {
    solenoid1.set(Value.kReverse);
    solenoid2.set(Value.kReverse);
    m_timer.start();
    m_timer.reset();
  }

  //@Override
  public void autonomousPeriodic2() {
    m_drive.curvatureDrive(0, 0, false);
    double T1 = 2;
    double T2 = T1+0.0;
    double T3 = T2 + 3;
    double T4 = T3 + 3;
    double T5 = T3 + 2;
    if (m_timer.get() < T1){
      m_drive.curvatureDrive(0.3, 0, false);
    }
    //else if (m_timer.get() < T2){
    //  alignX2();
    //}
    else if (m_timer.get() < T3){
      alignDist(85);
    }
    else if (m_timer.get() < T4){
     flywheel.set(ControlMode.PercentOutput, 0.70);
      if (m_timer.get() > T5){
        storageGroup.set(0.6);
      }
    }
    else{
      flywheel.set(ControlMode.PercentOutput,0);
      storageGroup.set(0);
    }
  }

  @Override
  public void autonomousPeriodic() {
    m_drive.curvatureDrive(0, 0, false);
    double T1 = 1;//1
    double T2 = T1+3;//2
    double T3 = T1 + 2;//2
    double T4 = T2 + 0.7; // 0.7 - this is moving back after fire
    double T5 = T4 +2; //2 turning left
    double T6 = T5 + 1;// 1
    double T7 = T6 + 2;//2

    double T8 = T7 + 0.9;// 0.9moving forward to range
    double T9 = T8 + 2; //2
    double T10 = T8 +2; //2
    if (m_timer.get() < T1){
    m_drive.curvatureDrive(0.24, 0, false);   
     //alignDist(105);
    }
    else if (m_timer.get() < T2){
      flywheel.set(ControlMode.PercentOutput, 0.65);
      if (m_timer.get() > T3){
        storageGroup.set(0.6);
      }
    }
    else if (m_timer.get() < T4){ //move back
      m_drive.curvatureDrive(0.32, 0,false);

    }
    else if (m_timer.get() < T5){ //turn left
      //System.out.println("moving lef");
      flywheel.set(ControlMode.PercentOutput, 0.0);
      m_drive.curvatureDrive(0, 0.36, true);
      storageGroup.set(0);
     
    }
    else if (m_timer.get() < T6){ //intake ball 2
      intakeWheel.set(ControlMode.PercentOutput,0.65);
      m_drive.curvatureDrive(-0.33,0,false);
    }
    else if (m_timer.get() < T7){ // turn right
      m_drive.curvatureDrive(0,-0.34,true);
    }
    else if (m_timer.get() < T8){ // move forward
      m_drive.curvatureDrive(-0.3,0, false);
      storageGroup.set(0.35);
    }
    else if (m_timer.get() < T9){
      flywheel.set(ControlMode.PercentOutput, 0.65);
      storageGroup.set(0);
      if (m_timer.get() > T10){
        System.out.println("storage moving");
        storageGroup.set(0.6);
       // intakeWheel.set(ControlMode.PercentOutput, 0.6);
      }    
    }   
  }

  public static double getDistance() { // returns distance from upper hub horizontally

    double ang_lime = 29;
    double H_tape = 2.642;
    double H_lime = 0.66;
    //System.out.println("distancing");
    //System.out.println( limelightTable.getEntry("ty").getDouble(0.0));
   //String k =  limelightTable.getPath();
   //System.out.println(k);
		try{

			double Theta_t = limelightTable.getEntry("ty").getDouble(0); // angle that limelight sees
			
			if (Theta_t == 0) {
                return 0; // this returns if limelight cant detect the tape
      }

			Math.tan(Math.toRadians(Theta_t+ang_lime)); // math
			
			double dist = (H_tape - H_lime)/Math.tan(Math.toRadians(Theta_t+ang_lime)); // more math
			
			dist = dist *39.37; // conversion from meters to inches
                                    //dist *= 1.11; // error correction
                                    //dist -= 2; // error correction

		//	System.out.println(dist + " - distance    -----    " + Theta_t + " - ang lime");

			return dist;

		} catch (Exception e) {
			System.out.println("dist error"+ e);
			return 0;
		}

  }
  
  public static void alignDist(double Target_distance) { // moves to align distance of robot
		if (getDistance() != 0){
      dist = getDistance();
    }
		double error = Target_distance - dist;
    System.out.println(error);
   
		if (error > 5) {
			m_drive.arcadeDrive(0.5, 0);
		}
    else if (error < -5){
      m_drive.arcadeDrive(-0.5, 0);
    }
     else if (error < -10) {
			m_drive.arcadeDrive(-0.6, 0 );
		} else if (error > 10.0) {
			m_drive.arcadeDrive(0.6, 0);
		} 
  }
  public static void alignX() {
    double error = limelightTable.getEntry("tx").getDouble(0);
    System.out.println(error);
    if (error > 3){
      m_drive.arcadeDrive(0, -0.4);
    }
    else if (error > 10){
      m_drive.arcadeDrive(0, -0.6);
    }
    else if (error < -3){
      m_drive.arcadeDrive(0, 0.4);
    }
    else if (error < -10){
      m_drive.arcadeDrive(0, 0.6);
    }
  }

  public static void alignX2(){
    double error = limelightTable.getEntry("tx").getDouble(0);
    double k = -0.05;
    double speed = error*k;
    m_drive.arcadeDrive(0, speed);
  }
  public static void alignDist2(double Target_distance) { // moves to align distance of robot
      double dist = getDistance();
      double error = Target_distance - dist;
      double speed = error * 0.01;
      m_drive.arcadeDrive(speed, 0);
    }

}