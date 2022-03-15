package frc.robot.common;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
/*------------------------Imports------------------------------------*/

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.PneumaticsModuleType;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;


public class RobotMap {
     
	//Initialize

	public static WPI_TalonFX leftAft = new WPI_TalonFX(0);
	public static WPI_TalonFX leftFront = new WPI_TalonFX(1);
	public static WPI_TalonFX rightAft = new WPI_TalonFX(2);
	public static WPI_TalonFX rightFront = new WPI_TalonFX(3);

	public static WPI_VictorSPX intakeWheel = new WPI_VictorSPX(4);
	public static WPI_VictorSPX storageLeft = new WPI_VictorSPX(5);
	public static WPI_VictorSPX storageRight = new WPI_VictorSPX(6);
	public static WPI_TalonFX flywheel = new WPI_TalonFX(7);


	
	public static DoubleSolenoid solenoid1 = new DoubleSolenoid(PneumaticsModuleType.CTREPCM,7,5);
    public static DoubleSolenoid solenoid2 = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 6, 4);
	public static Compressor compressor = new Compressor(PneumaticsModuleType.CTREPCM);
	
	public static Joystick m_joy = new Joystick(0);
	public static XboxController m_xbox = new XboxController(1);



	public static Encoder encoder1 = new Encoder(0, 1, false, EncodingType.k4X);
	public static Encoder encoder2 = new Encoder(0, 1, false, EncodingType.k4X);
	public static Encoder encoder3 = new Encoder(0, 1, false, EncodingType.k4X);
	public static Encoder encoder4 = new Encoder(0, 1, false, EncodingType.k4X);



	public static MotorControllerGroup leftGroup = new MotorControllerGroup(leftAft, leftFront);
	public static MotorControllerGroup rightGroup = new MotorControllerGroup(rightAft, rightFront);
	public static MotorControllerGroup storageGroup = new MotorControllerGroup(storageLeft, storageRight);

	public static DifferentialDrive m_drive = new DifferentialDrive(leftGroup, rightGroup);


}
// x - 2.62
// y - 3.85

