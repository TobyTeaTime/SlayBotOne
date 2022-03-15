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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.motorcontrol.ControlMode;

// Start of robot code, declare variables to be used in robot code here
@SuppressWarnings("FieldCanBeLocal")
public class Robot extends TimedRobot {

  double distance = ultrasonic.getRangeInches();
  double fineControlSpeedDouble = .45;
  private final Timer m_timer = new Timer();

  // robotInit = code the roboRIO runs on startup, only runs once
  // Good for things like resets, calibration, turning things on etc
  @Override
  public void robotInit() {
    // File file = new
    // File(Robot.class.getProtectionDomain().getCodeSource().getLocation().getPath());
    // Shuffleboard.getTab("DEBUG").add("cimMotor
    // Firm",cimMotor.getFirmwareVersion());
    // Shuffleboard.getTab("DEBUG").add("miniCimMotor1
    // Firm",miniCimMotor1.getFirmwareVersion());
    // Shuffleboard.getTab("DEBUG").add("miniCimMotor2
    // Firm",miniCimMotor2.getFirmwareVersion());
    // Shuffleboard.getTab("DEBUG").add("Right Front Drivetrain
    // Firm",m_rightFront.getFirmwareVersion());
    // Shuffleboard.getTab("DEBUG").add("Last code
    // deploy",sdf.format(file.lastModified()));

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

    // Invert certain motors depending on which direction they need to go
    // Clock wise is default
    intakeWheel.setInverted(true);
    rightAft.setInverted(true);
    rightFront.setInverted(true);
    storageLeft.setInverted(true);
    storageRight.setInverted(true);

    // Makes Master Follower pair, RIO sends same signal to two motors
    innerClimbRight.follow(innerClimbLeft);
    outerClimbRight.follow(outerClimbRight);

    // Enables the Compressor, Make sure to turn the compressor off later
    compressor.enableDigital();

    // Sets deadband for the drive train
    // Any imput detected from the controller under 5% will not be used
    m_drive.setDeadband(.05);

  }

  // robotPeriodic = code that will be run as long as the robot is on
  // Good for monitoring encoders, sensors, etc (Bot does not need to be enabled)
  @Override
  public void robotPeriodic() {
    // Puts numbers from Encoders on the SmartDashboard network table
    // Open Shuffleboard to see numbers, can be visualized as graphs or ot other
    // graphic
    SmartDashboard.putNumber("InnerClimbLeft Encoder Value",
        innerClimbLeft.getSelectedSensorPosition() * kInnerClimbTick2Inches);
    SmartDashboard.putNumber("InnerClimbRight Encoder Value",
        innerClimbRight.getSelectedSensorPosition() * kInnerClimbTick2Inches);
    SmartDashboard.putNumber("OuterClimbLeft Encoder Value",
        outerClimbLeft.getSelectedSensorPosition() * kOuterClimbTick2Deg);
    SmartDashboard.putNumber("OuterClimbRight Encoder Value",
        outerClimbRight.getSelectedSensorPosition() * kOuterClimbTick2Deg);

    // Uses the Pressure Switxh to turn off the compressor
    // PSI depends on Pressure Switch, 2022 bot uses a Pressure Switch that switches
    // at 125PSI
    if (compressor.getPressureSwitchValue() == false) {
      compressor.disable();

    }
  }

  // teleopInit = code that is run on teleop (teleoperation) startup
  // Good for initializing certain manips or pneumatic systems
  @Override
  public void teleopInit() {

    // Sets solenoids to Forward Position
    // Check with Pneumatic System to verify which position is forward
    solenoid1.set(Value.kForward);
    solenoid2.set(Value.kForward);

  }

  // teleopPeriodic = code that is run as long as teleop is enabled (runs every
  // 20ms)
  // Going to be the majority of where your code is, troubleshoot and debug
  // thouroughly
  @Override
  public void teleopPeriodic() {

    // starts a timer object created previously
    m_timer.start();

    // Asks if the A Button of the Logitech controller was Pressed last time the
    // code was ran
    // If it was, set the solenoids to the Reverse position
    // If it was not, set the solenoids to the Forward position
    if (m_xbox.getAButtonPressed()) {
      solenoid1.set(Value.kReverse);
      solenoid2.set(Value.kReverse);
    } else {
      solenoid1.set(Value.kForward);
      solenoid2.set(Value.kForward);
    }

    // Asks if the X Button is currently being pressed
    // If it is, set the intake motor controller to 60% power
    // If the ultrasonic detects less that 4 inches AND the timer has elapsed 2
    // seconds
    // Set storage motor controller at 65% power
    // Set intake motor controller at 40% power
    // If the X Button is not being held, set both motor controllers to 0% power
    if (m_xbox.getXButton()) {
      intakeWheel.set(.6);
      while (distance <= 4 /* || m_timer.hasElapsed(2) */) {
        intakeWheel.set(.4);
        storageGroup.set(.65);
      }
    } else {
      intakeWheel.set(0);
      storageGroup.set(0);
    }

    /*
     * flywheel speed adjust
     * if (m_xbox.getBButtonPressed()){
     * val += .05;
     * System.out.println(val);
     * } else if (m_xbox.getYButtonPressed()) {
     * val -= .05;
     * System.out.println(val);
     * }
     */

    // Asks what the current state of the Right Trigger is on the Logitech
    // Controller
    // This axis in range from 0 to 1
    // If the state is greater that 0, set Flywheel motor controller at 70% power
    // If the state is 0 or lower, set Flywheel motor controller at 0% power
    if (m_xbox.getRightTriggerAxis() > 0) {
      flywheel.set(.7);
    } else {
      flywheel.set(0);
    }

    // Asks which position the Directional Pad of the Logitech controller is in
    // Sets the Climber motor controllers to different values depending on position
    // Motor controllers using position mode, recquires encoder data, as well as
    // Usable Unit equation
    if (m_xbox.getPOV() == 0) {
      innerClimbLeft.set(ControlMode.Position, 6 / kInnerClimbTick2Inches);
    } else if (m_xbox.getPOV() == 180) {
      innerClimbLeft.set(ControlMode.Position, -6 / kInnerClimbTick2Inches);
    } else if (m_xbox.getPOV() == 90) {
      outerClimbLeft.set(ControlMode.Position, 45 / kOuterClimbTick2Deg);
    } else if (m_xbox.getPOV() == 270) {
      outerClimbLeft.set(ControlMode.Position, -45 / kOuterClimbTick2Deg);
    }

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
    m_drive.arcadeDrive(m_joy.getY(), -m_joy.getX());
  }

  // autonomousInit = code that is run upon the start of autonomous
  // Good for initializing certain manipulators such as pneumatics
  @Override
  public void autonomousInit() {

  }

}