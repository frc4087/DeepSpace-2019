package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.OI;
import frc.robot.subsystems.*;
import frc.robot.commands.*;


public class Robot extends TimedRobot {
  public static Drivebase m_drivebase = new Drivebase();
  public OI m_oi;
  public static VisionTracking m_visiontracking = new VisionTracking();

  // Pneumatics
  Compressor c = new Compressor(0);
  public static ToggledSolenoid winchPiston = new ToggledSolenoid(2, 5);
  public static ToggledSolenoid shifters = new ToggledSolenoid(3, 4);
  public static DoubleSolenoid hatchIntake = new DoubleSolenoid(6, 7);
  public static ToggledSolenoid intakeActuator = new ToggledSolenoid(0, 1);

  // Winch
  public static Spark leftWinch = new Spark(4);
  public static Spark rightWinch = new Spark(5);
  public static Winch winch = new Winch(leftWinch, rightWinch, winchPiston);
 

  // Drivebase
  /*public static VictorSP vaLeftDrive = new VictorSP(RobotMap.VA_LEFT_DRIVE.value);
  public static VictorSP vbLeftDrive = new VictorSP(RobotMap.VB_LEFT_DRIVE.value);
  public static VictorSP vaRightDrive = new VictorSP(RobotMap.VA_RIGHT_DRIVE.value);
  public static VictorSP vbRightDrive = new VictorSP(RobotMap.VB_RIGHT_DRIVE.value);
  public static WPI_TalonSRX LeftDrive = new WPI_TalonSRX(1);
  public static WPI_TalonSRX RightDrive = new WPI_TalonSRX(2);
 
  public SpeedControllerGroup m_left = new SpeedControllerGroup(LeftDrive, vaLeftDrive, vbLeftDrive);
  public SpeedControllerGroup m_right = new SpeedControllerGroup(RightDrive, vaRightDrive, vbRightDrive);
  DifferentialDrive m_drive = new DifferentialDrive(m_left, m_right);*/

  // Constants
  public boolean shiftState = false;

  @Override
  public void robotInit() {
    m_oi = new OI();
    //CameraServer.getInstance().startAutomaticCapture();
  }

  
  @Override
  public void robotPeriodic() {
  }

  @Override
  public void autonomousInit() {
  }

  @Override
  public void autonomousPeriodic() {
    matchPeriodic();
  }

  @Override
  public void teleopPeriodic() {
    matchPeriodic();
  }

  @Override
  public void testPeriodic() {
  }

/*------------------------------------------------------------------------------------*/

  public void matchPeriodic(){
    c.setClosedLoopControl(true);

    // Drivebase
    if (m_oi.driveJoy.getXButton()) {
      /*m_visiontracking.setTracking(true);
      double steeringAdjust = .038 * m_visiontracking.pidX();
      m_drive.arcadeDrive(-m_oi.getDriveJoyYL() * m_visiontracking.get("tv"), steeringAdjust);*/
      double steeringAdjust = m_visiontracking.pidX();
      m_drivebase.arcadeDrive(m_visiontracking.zoomForward(), steeringAdjust * m_visiontracking.getTarget());
      shifters.set(false);
      shiftState = false;
      if (m_visiontracking.get("ta") > 5) {
        intakeActuator.set(true);
      }

    } else {
      m_visiontracking.setTracking(false);
      m_drivebase.curvatureDrive(-m_oi.getDriveJoyYL(), m_oi.getDriveJoyXR(), m_oi.isQuickTurn());
    }
    
    //Shifters
    if (m_oi.getDriveJoyBLPressed()) {
      shifters.togglePiston();
      shiftState = !shiftState;
    }
    SmartDashboard.putString("Gear", shiftState ? "High" : "Low");

    // hatchIntake
    if (Math.abs(m_oi.driveJoy.getTriggerAxis(Hand.kLeft)) > 0.1) {
      hatchIntake.set(Value.kForward);
    } else if (Math.abs(m_oi.driveJoy.getTriggerAxis(Hand.kRight)) > .1) {
      hatchIntake.set(Value.kReverse); 
    } else {
      hatchIntake.set(Value.kOff);
    }

    // intakeActuator
    if (m_oi.getOpJoyBLPressed()) {
      intakeActuator.togglePiston();
    }

    // winch
    winch.setWinch(-m_oi.getOpJoyYL());

    // winchPiston
    if (m_oi.opJoy.getBButtonPressed()) {
      winchPiston.togglePiston();
    }


  }
}
