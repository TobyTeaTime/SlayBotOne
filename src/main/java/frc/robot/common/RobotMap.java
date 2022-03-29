package frc.robot.common;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
/*------------------------Imports------------------------------------*/
import com.ctre.phoenix.motorcontrol.SensorCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableEntry;


public class RobotMap {

	// Initialize devices

	// Motor controllers use canIDS for parameters
	public static WPI_TalonFX leftAft = new WPI_TalonFX(0);
	public static WPI_TalonFX leftFront = new WPI_TalonFX(1);
	public static WPI_TalonFX rightAft = new WPI_TalonFX(2);
	public static WPI_TalonFX rightFront = new WPI_TalonFX(3);

	public static WPI_VictorSPX intakeWheel = new WPI_VictorSPX(4);
	public static WPI_VictorSPX storageLeft = new WPI_VictorSPX(5);
	public static WPI_VictorSPX storageRight = new WPI_VictorSPX(6);
	public static WPI_TalonFX flywheel = new WPI_TalonFX(7);

	public static WPI_TalonSRX innerClimbLeft = new WPI_TalonSRX(10);
	public static WPI_TalonSRX innerClimbRight = new WPI_TalonSRX(9);
	public static WPI_VictorSPX outerClimbLeft = new WPI_VictorSPX(8);  //TODO changed from talon to victor (might break things)
	public static WPI_TalonSRX outerClimbRight = new WPI_TalonSRX(11);

	// Solenoids use pcm ports as parameters
	// Starting in 2022, you must also specify what type of PCM you are using
	public static DoubleSolenoid solenoid1 = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 7, 5);
	public static DoubleSolenoid solenoid2 = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 6, 4);
	public static Compressor compressor = new Compressor(PneumaticsModuleType.CTREPCM);

	// Order of controllers is selected in driver station
	public static Joystick m_joy = new Joystick(0);
	public static XboxController m_xbox = new XboxController(1);

	// Motor Controller Groups are useful for drive train, and motors that don't
	// have encoders
	// You cannot use different cntrol methods for motor controller groups other
	// than percentage
	public static MotorControllerGroup leftGroup = new MotorControllerGroup(leftAft, leftFront);
	public static MotorControllerGroup rightGroup = new MotorControllerGroup(rightAft, rightFront);
	public static MotorControllerGroup storageGroup = new MotorControllerGroup(storageLeft, storageRight);

	// Differential drivetrain uses a left motor and right motor as parameters
	public static DifferentialDrive m_drive = new DifferentialDrive(leftGroup, rightGroup);

	// Sensors use DIO ports as parameters
	// Ultrasonic uses two ports, one for Vcc, Ping, Gnd, one for Echo
	
	public static Ultrasonic ultrasonic = new Ultrasonic(0, 1);
	//limelight for hoop tracking and shooting

	
	public static NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");

	
	
	// Getting encoder data from TalonSRX Sensor ports
	

	// Calculating Encoder Ticks to Usable Units
	// (will be different for every mechanism, dependant on gear ratio and encoder)

	// 1/1024 = 1 rotation is equal to 1024 ticks
	// 1/21 = gear ratio 21 on the input gear equals 1 turn on the output
	// 1.79pi/1 rotation = circumference of thing turning per rotation
	// (optional) 1/12 = 1 foot in 12 inches to convert to feet
	public static double kInnerClimbTick2Inches = 1 / 1024 * 1 / 21 * 1.79 * Math.PI;
	public static double kOuterClimbTick2Deg = 1 / 1024 * 1 / 100;

	
}
