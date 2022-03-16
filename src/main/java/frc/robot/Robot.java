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
//import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static frc.robot.common.RobotMap.*;

@SuppressWarnings("FieldCanBeLocal")
public class Robot extends TimedRobot {
    
    private SendableChooser<Double> fineControlSpeed = new SendableChooser<>();
    private SendableChooser<Double> deadBandOptions = new SendableChooser<>();
    private SendableChooser<Double> Encoder1 = new SendableChooser<>();
    private SendableChooser<Double> Encoder2 = new SendableChooser<>();
    private SendableChooser<Double> Encoder3 = new SendableChooser<>();
    private SendableChooser<Double> Encoder4 = new SendableChooser<>();
    double val = 0.65;
   // private SimpleDateFormat sdf = new SimpleDateFormat("MM/dd/yy HH:mm:ss");
    //private final RamseteController m_ramsete = new RamseteController();
    //private final Timer m_timer = new Timer();

  
    
    @Override
    public void robotInit() {
        //File file = new File(Robot.class.getProtectionDomain().getCodeSource().getLocation().getPath());
        //Shuffleboard.getTab("DEBUG").add("cimMotor Firm",cimMotor.getFirmwareVersion());
        //Shuffleboard.getTab("DEBUG").add("miniCimMotor1 Firm",miniCimMotor1.getFirmwareVersion());
        //Shuffleboard.getTab("DEBUG").add("miniCimMotor2 Firm",miniCimMotor2.getFirmwareVersion());
        //Shuffleboard.getTab("DEBUG").add("Right Front Drivetrain Firm",m_rightFront.getFirmwareVersion());
        //Shuffleboard.getTab("DEBUG").add("Last code deploy",sdf.format(file.lastModified()));
        
        //Format all motor controllers
        intakeWheel.configFactoryDefault();
        storageLeft.configFactoryDefault();
        storageRight.configFactoryDefault();
        flywheel.configFactoryDefault();
        leftAft.configFactoryDefault();
        leftFront.configFactoryDefault();
        rightAft.configFactoryDefault();
        rightFront.configFactoryDefault();

        intakeWheel.setInverted(true);
        rightAft.setInverted(true);
        rightFront.setInverted(true);
        storageLeft.setInverted(true);
        storageRight.setInverted(true);

        //Fine Control Speed chooser
        fineControlSpeed.addOption("35% Speed", 0.35);
        fineControlSpeed.addOption("40% Speed", 0.40);
        fineControlSpeed.setDefaultOption("45% Speed", 0.45);
        fineControlSpeed.addOption("50% Speed", 0.50);
        fineControlSpeed.addOption("55% Speed", 0.55);
        fineControlSpeed.addOption("60% Speed", 0.60);
        Shuffleboard.getTab("Vision").add("Fine Control Speed", fineControlSpeed);
        
        //Deadband chooser
        deadBandOptions.setDefaultOption("5%", 0.05);
        deadBandOptions.addOption("10%", 0.10);
        deadBandOptions.addOption("15%", 0.15);
        Shuffleboard.getTab("Vision").add("Dead Band", deadBandOptions);

        
        
        

        //double dist = sonic.getRangeInches();

    }
    

    
    @Override
    public void teleopInit() {
      
/*
      solenoid1.set(Value.kForward);
      solenoid2.set(Value.kForward);
*/
     

    }
    @Override
    public void teleopPeriodic() {
       
      if (m_xbox.getAButton() == true) {
        solenoid1.set(Value.kReverse);
        solenoid2.set(Value.kReverse);
      } else if (m_xbox.getAButton() == false) {
        solenoid1.set(Value.kForward);
        solenoid2.set(Value.kForward);
      }

      if (m_xbox.getXButton() == true) {
        intakeWheel.set(.6);
      } else {
        intakeWheel.set(0);
      }
      
      if (m_xbox.getLeftBumper() == true) {
        storageGroup.set(.65);
      } else {
        storageGroup.set(0);
      }

      /*
      flywheel speed adjust
      if (m_xbox.getBButtonPressed()){
        val += .05;
        System.out.println(val);
      } else if (m_xbox.getYButtonPressed()) {
        val -= .05;
        System.out.println(val);
      }
      */

        if (m_xbox.getRightTriggerAxis() > 0) {
        flywheel.set(.7);
      } else {
        flywheel.set(0);
      }
    
      m_drive.arcadeDrive(m_joy.getY(), -m_joy.getX());
  }
  @Override
  public void robotPeriodic() {

    compressor.enableDigital();


    if (compressor.getPressureSwitchValue() == true) {
      compressor.disable();
    }
      
    }
    
  
  @Override
  public void autonomousInit() {

  }

}