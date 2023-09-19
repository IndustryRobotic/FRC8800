package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType; 
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;

public class Robot extends TimedRobot {
  // Motors:

  VictorSP sag1 = new VictorSP(0);
  VictorSP sag2 = new VictorSP(1);
  VictorSP sol1 = new VictorSP(2);
  PWMVictorSPX sol2 = new PWMVictorSPX(3);
  PWMVictorSPX kol1 = new PWMVictorSPX(4);
  PWMVictorSPX kol2 = new PWMVictorSPX(5);
  PWMSparkMax gripper = new PWMSparkMax(9);

  // Limelight, Vision Processing:
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  
  NetworkTableEntry pipeline = table.getEntry("pipeline"); // pipeline adjustments
  NetworkTableEntry tid = table.getEntry("tid"); // Apriltag ID

  NetworkTableEntry ty = table.getEntry("ty"); // Y axis distance
  NetworkTableEntry tx = table.getEntry("tx"); // X axis distance
  NetworkTableEntry ta = table.getEntry("ta"); // percentage on the screen

  //
  
  MotorControllerGroup sol = new MotorControllerGroup(sol1, sol2);
  MotorControllerGroup sag = new MotorControllerGroup(sag1, sag2);
  MotorControllerGroup lifter = new MotorControllerGroup(kol1, kol2);
  DifferentialDrive m_robotDrive = new DifferentialDrive(sol, sag);
  
  // Pneumatics:

  DoubleSolenoid exampleDoublePCM = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 1, 0);

  // Other:
  
  Joystick kol = new Joystick(0);
  private final Timer m_timer = new Timer();
  
  // Variables:

  public static double position = 0;
  public static double preciseDriveSpeed = 0.1;
  public static boolean emergency;
  public static boolean tapeDebounce;
  public static double xAxisPeriodic;
  public static boolean camDebounce;


  /** This function is run once when the robot activates. */
  @Override
  public void robotInit() {
    sol1.setInverted(true);
    sol2.setInverted(true);
  }

  public void robotPeriodic() {
  }

  /** This function is run once each time the robot enters autonomous mode. */
  @Override
  public void autonomousInit() {
    emergency = false;
    pipeline.setNumber(0); // Tag
    m_timer.reset();
    m_timer.start();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic()  
  {
    double targetID = tid.getInteger(0);
    
    if(m_timer.get() < 0.3)
    {
      if(targetID <= 3 && targetID != -1)
      {
        position = targetID;
      }
      else
      {
        if(targetID == -1)
          emergency = true;
        if(targetID == 8)
          position = 1;
        if(targetID == 7)
          position = 2;
        if(targetID == 6)
          position = 3;
        
      }
      System.out.println(position);
    }
    if(m_timer.get() < 14.7)
    {
      if(position == 1) // Position 1
      {
        if(m_timer.get() < 1)
        {
          m_robotDrive.arcadeDrive(0.2, 0, false); // Go straight
        }
        else
        {
          m_robotDrive.stopMotor();
        }
      }
      if(position == 2) // Position 2
      {
        if(m_timer.get() < 1)
        {
          m_robotDrive.arcadeDrive(0, 0.1, false); // Turn to the right
        }
        else
        {
          m_robotDrive.stopMotor();
        }
      }
      if(position == 3) // Position 3
      {
        if(m_timer.get() < 1)
        {
          m_robotDrive.arcadeDrive(0, -0.1, false); // Turn to the left
        }
        else
        {
          m_robotDrive.stopMotor();
        }
      }
      if(emergency == true) // Position emergency
      {
        if(m_timer.get() < 1)
        {
          m_robotDrive.arcadeDrive(0.2, 0.0335, false); // Go backwards
        }
        else
        {
          m_robotDrive.stopMotor();
        }
      }
    }
    else
    {
      m_robotDrive.stopMotor();
    }
  }

  /** This function is called once each time the robot enters teleoperated mode. */
  @Override
  public void teleopInit() {
    pipeline.setNumber(1); // Reflective Tape
    preciseDriveSpeed = 0.1;
    tapeDebounce = false;
    camDebounce = false;
        
  }

  /** This function is called periodically during teleoperated mode. */
  @Override
  public void teleopPeriodic() {

    if(kol.getRawButtonPressed(7)){ // Changing the pipeline to the Apriltag Pipeline
      if(tapeDebounce == true)
      {
        tapeDebounce = false;
      }
      else // if false
      {
        tapeDebounce = true;
      }
    }

    if(tapeDebounce == false)
    {
      m_robotDrive.arcadeDrive(kol.getRawAxis(1), kol.getRawAxis(0));

      // Precise Movement Controls (Slow)
  
      if(kol.getRawButton(9) && kol.getRawButton(10))
      {
        m_robotDrive.arcadeDrive(-(preciseDriveSpeed), 0, false);    
      }
      else if (kol.getRawButton(9) && !kol.getRawButton(10))
      {      
        m_robotDrive.arcadeDrive(0,preciseDriveSpeed - (preciseDriveSpeed*2),false);
      }
      else if(kol.getRawButton(10) && !kol.getRawButton(9))
      {
        m_robotDrive.arcadeDrive(0,preciseDriveSpeed,false);
      }
  
      // Lifter
  
      if(kol.getRawButton(2)){
        lifter.set(-0.5);
      }
  
      if(kol.getRawButtonReleased(2)){
        lifter.set(0);
      }
      
      if(kol.getRawButton(1)){
        lifter.set(0.5);
      }
  
      if(kol.getRawButtonReleased(1)){
        lifter.set(0);
      }
      
      // Lifter (Slow)
    
      if(kol.getRawButton(11)){
        lifter.set(0.3);
      }
  
      if(kol.getRawButtonReleased(11)){
        lifter.set(0);
      }
      
      if(kol.getRawButton(12)){
        lifter.set(-0.2);
      }
  
      if(kol.getRawButtonReleased(12)){
        lifter.set(0);
      }

      // Driver Cam:

      if(kol.getRawButtonPressed(8)){ // pipeline changing
        if(camDebounce == true)
        {
          camDebounce = false;
          pipeline.setNumber(1);
        }
        else // if false
        {
          camDebounce = true;
          pipeline.setNumber(2);
        }
      }
  
      //// Gripper:
  
      // Pneumatics:
  
      if(kol.getRawButtonPressed(4)){
        exampleDoublePCM.set(Value.kReverse);
      }   
      if(kol.getRawButtonPressed(6)){
        exampleDoublePCM.set(Value.kForward);
      }
  
      // Motors:
  
      if(kol.getRawButtonPressed(5)){
        gripper.set(-0.15);
      }  
      
      if(kol.getRawButtonReleased(5))
      {
        gripper.set(0);
      }

      if(kol.getRawButtonPressed(3)){
        gripper.set(0.4);
      }  

      if(kol.getRawButtonReleased(3))
      {
        gripper.set(0);
      }
    }
    else
    {
      double xAxis = tx.getDouble(0);
      double range = 3;
      System.out.println(range);

      if(xAxis > range) // if +x
      {
        m_robotDrive.arcadeDrive(0,0.1,false);
      }
      else if(xAxis < -(range)) // if -x
      {
        m_robotDrive.arcadeDrive(0,-0.1,false);
      }
      else if(xAxis > -(range) && xAxis < range) // if its in the range
      {
        m_robotDrive.arcadeDrive(-0.2,-0.0335,false);    
      }
      else
      {
        System.out.println("No tape detected!");
      }
    }   
  }
    
  /** This function is called once each time the robot enters test mode. */
    @Override
    public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
