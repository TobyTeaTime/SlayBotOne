//https://docs.wpilib.org/en/stable/index.html for documentation

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

//import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
//import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static frc.robot.common.RobotMap.*;

import com.ctre.phoenix.motorcontrol.ControlMode;

@SuppressWarnings("FieldCanBeLocal")
public class Robot extends TimedRobot {

  double distance = ultrasonic.getRangeInches();
  double fineControlSpeedDouble = .45;
  private final Timer m_timer = new Timer();

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

    intakeWheel.setInverted(true);
    rightAft.setInverted(true);
    rightFront.setInverted(true);
    storageLeft.setInverted(true);
    storageRight.setInverted(true);

    innerClimbRight.follow(innerClimbLeft);
    outerClimbRight.follow(outerClimbRight);

    compressor.enableDigital();

    m_drive.setDeadband(.05);

  }

  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("InnerClimbLeft Encoder Value",
        innerClimbLeft.getSelectedSensorPosition() * kInnerClimbTick2Inches);
    SmartDashboard.putNumber("InnerClimbRight Encoder Value",
        innerClimbRight.getSelectedSensorPosition() * kInnerClimbTick2Inches);
    SmartDashboard.putNumber("OuterClimbLeft Encoder Value",
        outerClimbLeft.getSelectedSensorPosition() * kOuterClimbTick2Deg);
    SmartDashboard.putNumber("OuterClimbRight Encoder Value",
        outerClimbRight.getSelectedSensorPosition() * kOuterClimbTick2Deg);

    if (compressor.getPressureSwitchValue() == false) {
      compressor.disable();

    }
  }

  @Override
  public void teleopInit() {

    solenoid1.set(Value.kForward);
    solenoid2.set(Value.kForward);

  }

  @Override
  public void teleopPeriodic() {

    m_timer.start();

    if (m_xbox.getAButton()) {
      solenoid1.set(Value.kReverse);
      solenoid2.set(Value.kReverse);
    } else {
      solenoid1.set(Value.kForward);
      solenoid2.set(Value.kForward);
    }

    if (m_xbox.getXButton()) {
      intakeWheel.set(.6);
      while (distance <= 4 || m_timer.hasElapsed(2)) {
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

    if (m_xbox.getRightTriggerAxis() > 0) {
      flywheel.set(.7);
    } else {
      flywheel.set(0);
    }

    if (m_xbox.getPOV() == 0) {
      innerClimbLeft.set(ControlMode.Position, 6 / kInnerClimbTick2Inches);
    } else if (m_xbox.getPOV() == 180) {
      innerClimbLeft.set(ControlMode.Position, -6 / kInnerClimbTick2Inches);
    } else if (m_xbox.getPOV() == 90) {
      outerClimbLeft.set(ControlMode.Position, 45 / kOuterClimbTick2Deg);
    } else if (m_xbox.getPOV() == 270) {
      outerClimbLeft.set(ControlMode.Position, -45 / kOuterClimbTick2Deg);
    }

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

    m_drive.arcadeDrive(m_joy.getY(), -m_joy.getX());
  }

  @Override
  public void autonomousInit() {

  }

}