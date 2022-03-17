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
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import static frc.robot.common.RobotMap.*;

@SuppressWarnings("FieldCanBeLocal")
public class Robot extends TimedRobot {
    
    
    double val = 0.65;
   // private SimpleDateFormat sdf = new SimpleDateFormat("MM/dd/yy HH:mm:ss");
    //private final RamseteController m_ramsete = new RamseteController();
    //private final Timer m_timer = new Timer();

  
    
    @Override
    public void robotInit() {
       
        
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


        
        
        

        //double dist = sonic.getRangeInches();

    }
    

    
    @Override
    public void teleopInit() {
      compressor.enableDigital();
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

   


    if (compressor.getPressureSwitchValue() == true) {
      compressor.disable();
    }
      
    }
    
  
  @Override
  public void autonomousInit() {

  }

}