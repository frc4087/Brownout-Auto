// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import java.io.IOException;
import java.nio.file.Path;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  //SUBSYSTEMS---------------------------------------------------------------------------------------------------
  public final DriveBase m_DriveBase = new DriveBase();
  public SendableChooser<String> autoChooser = new SendableChooser<String>();
  public Command m_autonomousCommand;
  public Trajectory trajectory;

  
  

  //JOYSTICKS ---------------------------------------------------------------------------------------------------
  public final XboxController driveJoy = new XboxController(0);
  public final XboxController opJoy = new XboxController(1);
  public final JoystickButton aButton = new JoystickButton(opJoy,1);
  public final JoystickButton bButton = new JoystickButton(opJoy,2);
  public final JoystickButton startButton = new JoystickButton(opJoy,8);

  
  public double getDriveJoy(int axis) {
    double raw = driveJoy.getRawAxis(axis);
    return Math.abs(raw) < 0.1 ? 0.0 : raw;
  }
  public double getDriveJoyXR() {
    double raw = getDriveJoy(4);
    return raw;
    //return Math.abs(raw) < 0.1 ? 0.0 : raw > 0 ? (raw * raw) / 1.5 : (-raw * raw) / 1.5;
  }

  public double getDriveJoyYL() {
    double raw = getDriveJoy(1);
    return raw;
    //return Math.abs(raw) < 0.1 ? 0.0 : raw > 0 ? (raw * raw) / 1.5 : (-raw * raw) / 1.5;
  }
    /**
     * Converts the speed readable by the talons to the RPM
     *
     * @param axis The axis of the joystick (i.e. YL is y-axis on left joystick)
     * 
     * @return the joystick values from -1 to 1, implements a deadzone around zero to prevent drift
     */


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  private void configureButtonBindings() {}

  //ROBOT INIT -------------------------------------------------------------------------------------------------

  public void roboInit(){
    //ballCount = 0;
    
    //autonomous paths 
    autoChooser.addOption("Mobility1","Mobility1");
    autoChooser.addOption("Mobility2","Mobility2");
    autoChooser.addOption("Mobility2Dock","Mobility2Dock");
    autoChooser.addOption("Mobility3","Mobility3");

    SmartDashboard.putData("Auto Routine", autoChooser);

    m_DriveBase.resetEncoders();
  

  }

  public void roboPeriodic(){
    

    }

  

  //AUTO INIT --------------------------------------------------------------------------------------------------

    public void autoInit(){
      
      m_DriveBase.resetEncoders();
      m_DriveBase.m_gyro.reset();
      // if(RobotController.getBatteryVoltage() < 7){
      //   blinkPattern = Constants.red;
      // }
      // else{
      //   blinkPattern = Constants.autoIdle;
      // }
      // m_BlinkinBase.set(blinkPattern); 
      // m_IntakeBase.intakeSol1.set(Value.kReverse);

      if (autoChooser.getSelected() != null){
        m_autonomousCommand = getAutonomousCommand(autoChooser.getSelected());
        m_autonomousCommand.schedule();
      }
    }

    public void autoPeriodic(){
      SmartDashboard.putNumber("Pose X", m_DriveBase.m_odometry.getPoseMeters().getX());
      SmartDashboard.putNumber("Pose Y", m_DriveBase.m_odometry.getPoseMeters().getY());
      SmartDashboard.putNumber("Pose Angle", m_DriveBase.m_odometry.getPoseMeters().getRotation().getDegrees());
    }

    public void teleopInit(){
    }

  //TELEOP PERIODIC --------------------------------------------------------------------------------------------

  public void telePeroidic(){
    m_DriveBase.m_drive.arcadeDrive(getDriveJoyYL(), getDriveJoyXR());
    // SmartDashboard.putNumber("LimelightX", m_limelightBase.x);
    // SmartDashboard.putNumber("LimelightY", m_limelightBase.y);
    // SmartDashboard.putNumber("LimelightArea", m_limelightBase.a);
    // SmartDashboard.putNumber("LimelightValid", m_limelightBase.v);


     }

  // //COMMANDS METHODS --------------------------------------------------------------------------------------------

  public Command getAutonomousCommand(String path) {
    switch(path){

    case "Mobility1":
      return pathFollow("output/Mobility1.wpilib.json", false);
    case "Mobility2":
      return pathFollow("output/Mobility2.wpilib.json", false);
    case "Mobility2Dock":
      return pathFollow("output/Mobility2.wpilib.json", false)
              .andThen(pathFollow("output/Mobility2Rev.wpilib.json", true));
    case "Mobility3":
      return pathFollow("output/Mobility3.wpilib.json", false);
      
    // case "Taxi":
    // //  tracking = false;
    //   return pathFollow("output/Taxi.wpilib.json", false);

    // case "Taxi 1 Ball Low":
    //   //tracking = false;
    //   return //timedLaunchCommand(true, 3)
    //         //.andThen(
    //           pathFollow("output/Taxi.wpilib.json", false);

    // case "Taxi 2 Ball Low":
  
    //   return //new IntakeActivate()
    //         //.alongWith(
    //           //new ParallelRaceGroup(
    //           //  new BottomFeederActivate(true), 
    //            // new BeamBreakTriggered(1)))
    //        // .alongWith(
    //           pathFollow("output/Taxi.wpilib.json", false)
    //           .andThen(
    //             pathFollow("output/TaxiRev.wpilib.json", true));
    //          // .andThen(
   
    }

    return null;
  }

  /** 
   * @TODO Auto-generated catch block
  */
  public Command pathFollow(String trajectoryJSON, boolean multiPath){
    try {
      Path testTrajectory = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
      trajectory = TrajectoryUtil.fromPathweaverJson(testTrajectory);
    } catch (final IOException ex) {


      DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
    }
    //m_drivebase.m_gyro.reset();
    
    RamseteCommand ramseteCommand = new RamseteCommand(trajectory,
                                                    m_DriveBase::getPose,
                                                    new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
                                                    new SimpleMotorFeedforward(Constants.ksVolts, 
                                                                               Constants.kvVoltSecondsPerMeter,
                                                                               Constants.kaVoltSecondsSquaredPerMeter),
                                                    Constants.m_driveKinematics,
                                                    m_DriveBase::getWheelSpeeds,
                                                    new PIDController(Constants.kP, 0, 0),
                                                    new PIDController(Constants.kP, 0, 0),
                                                    m_DriveBase::voltageControl,
                                                    m_DriveBase);
    
    // Run path following command, then stop at the end.
    // Robot.m_robotContainer.m_driveAuto.m_drive.feed();
    //m_drivebase.resetOdometry(trajectory.getInitialPose());
    
    if (!multiPath){
      m_DriveBase.resetOdometry(trajectory.getInitialPose());
    } 
    return ramseteCommand;
  }
}
