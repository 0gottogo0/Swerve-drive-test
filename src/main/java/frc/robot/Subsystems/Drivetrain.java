// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import static frc.robot.Constants.BACK_LEFT_MODULE_DRIVE_MOTOR;
import static frc.robot.Constants.BACK_LEFT_MODULE_STEER_ENCODER;
import static frc.robot.Constants.BACK_LEFT_MODULE_STEER_MOTOR;
import static frc.robot.Constants.BACK_LEFT_MODULE_STEER_OFFSET;
import static frc.robot.Constants.BACK_RIGHT_MODULE_DRIVE_MOTOR;
import static frc.robot.Constants.BACK_RIGHT_MODULE_STEER_ENCODER;
import static frc.robot.Constants.BACK_RIGHT_MODULE_STEER_MOTOR;
import static frc.robot.Constants.BACK_RIGHT_MODULE_STEER_OFFSET;
import static frc.robot.Constants.FRONT_LEFT_MODULE_DRIVE_MOTOR;
import static frc.robot.Constants.FRONT_LEFT_MODULE_STEER_ENCODER;
import static frc.robot.Constants.FRONT_LEFT_MODULE_STEER_MOTOR;
import static frc.robot.Constants.FRONT_LEFT_MODULE_STEER_OFFSET;
import static frc.robot.Constants.FRONT_RIGHT_MODULE_DRIVE_MOTOR;
import static frc.robot.Constants.FRONT_RIGHT_MODULE_STEER_ENCODER;
import static frc.robot.Constants.FRONT_RIGHT_MODULE_STEER_MOTOR;
import static frc.robot.Constants.FRONT_RIGHT_MODULE_STEER_OFFSET;

import com.kauailabs.navx.frc.AHRS;
import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {

  private final AHRS navx = new AHRS(Port.kMXP, (byte) 200);
  
  private SwerveDriveOdometry odometry;

  private Pose2d pose = new Pose2d();
  
  private SwerveModuleState[] states = Constants.kinematics.toSwerveModuleStates(new ChassisSpeeds(0, 0, 0));

// Create a new SimpleMotorFeedforward with gains kS, kV, and kA
  private SimpleMotorFeedforward feedforwardRight = new SimpleMotorFeedforward(Constants.kSRight, Constants.kVRight, Constants.kARight);
  private SimpleMotorFeedforward feedforwardLeft = new SimpleMotorFeedforward(Constants.kSLeft, Constants.kVLeft, Constants.kALeft);
  
  private ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

  private SwerveModule[] modules;

  private SwerveModule frontLeftModule;
  private SwerveModule frontRightModule;
  private SwerveModule backLeftModule;
  private SwerveModule backRightModule;

  /** Creates a new Drivetrain. */
  public Drivetrain() {
    new Thread(() -> {
                try {
                    Thread.sleep(1000);
                    
                    zeroGyroscope();
                } catch (Exception e) {
                }
            }).start();
    // By default we will use Falcon 500s in standard configuration. But if you use a different configuration or motors
    // you MUST change it. If you do not, your code will crash on startup.
    // Setup motor configuration
    frontLeftModule = Mk4SwerveModuleHelper.createFalcon500(
            // This parameter is optional, but will allow you to see the current state of the module on the dashboard.
            tab.getLayout("Front Left Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(0, 0),
            
            Mk4SwerveModuleHelper.GearRatio.L1,
            // This is the ID of the drive motor
            FRONT_LEFT_MODULE_DRIVE_MOTOR,
            // This is the ID of the steer motor
            FRONT_LEFT_MODULE_STEER_MOTOR,
            // This is the ID of the steer encoder
            FRONT_LEFT_MODULE_STEER_ENCODER,
            // This is how much the steer encoder is offset from true zero (In our case, zero is facing straight forward)
            FRONT_LEFT_MODULE_STEER_OFFSET
    );

    // We will do the same for the other modules
    frontRightModule = Mk4SwerveModuleHelper.createFalcon500(
            tab.getLayout("Front Right Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(2, 0),
            Mk4SwerveModuleHelper.GearRatio.L1,
            FRONT_RIGHT_MODULE_DRIVE_MOTOR,
            FRONT_RIGHT_MODULE_STEER_MOTOR,
            FRONT_RIGHT_MODULE_STEER_ENCODER,
            FRONT_RIGHT_MODULE_STEER_OFFSET
    );

    backLeftModule = Mk4SwerveModuleHelper.createFalcon500(
            tab.getLayout("Back Left Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(4, 0),
            Mk4SwerveModuleHelper.GearRatio.L1,
            BACK_LEFT_MODULE_DRIVE_MOTOR,
            BACK_LEFT_MODULE_STEER_MOTOR,
            BACK_LEFT_MODULE_STEER_ENCODER,
            BACK_LEFT_MODULE_STEER_OFFSET
    );

    backRightModule = Mk4SwerveModuleHelper.createFalcon500(
            tab.getLayout("Back Right Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(6, 0),
            Mk4SwerveModuleHelper.GearRatio.L1,
            BACK_RIGHT_MODULE_DRIVE_MOTOR,
            BACK_RIGHT_MODULE_STEER_MOTOR,
            BACK_RIGHT_MODULE_STEER_ENCODER,
            BACK_RIGHT_MODULE_STEER_OFFSET
    );

    modules = new SwerveModule[] {frontLeftModule, frontRightModule, backLeftModule, backRightModule};
     
    odometry =  
        new SwerveDriveOdometry(Constants.kinematics, getGyroscopeRotation(), getSwerveModulePositions());
  }

  public Rotation2d getGyroscopeRotation() {
    //  Remove if you are using a Pigeon
    //return Rotation2d.fromDegrees(m_pigeon.getFusedHeading());

    //  Uncomment if you are using a NavX
    //if (navx.isMagnetometerCalibrated()) {
      // We will only get valid fused headings if the magnetometer is calibrated
      //return Rotation2d.fromDegrees(navx.getFusedHeading());
    //}

    // We have to invert the angle of the NavX so that rotating the robot counter-clockwise makes the angle increase.
    return navx.getRotation2d();//Rotation2d.fromDegrees(Math.IEEEremainder(navx.getAngle(), 360));//Rotation2d.fromDegrees(360.0 - navx.getYaw());
  }

  public SwerveModulePosition[] getSwerveModulePositions(){
        SwerveModulePosition[] swerveModulePositions = {
                modules[0].getSwerveModulePosition(),
                modules[1].getSwerveModulePosition(),
                modules[2].getSwerveModulePosition(),
                modules[3].getSwerveModulePosition()
            };
        return swerveModulePositions;
  } 

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  /**
   * Sets the gyroscope angle to zero. This can be used to set the direction the robot is currently facing to the
   * 'forwards' direction.
   */
  public void zeroGyroscope() {
    //  Remove if you are using a Pigeon
   // m_pigeon.setFusedHeading(0.0);

    //  Uncomment if you are using a NavX
    navx.reset();
    
  }

  private void updateOdometry() {
        //states = moduleStates;
        pose = odometry.update(getGyroscopeRotation(), getSwerveModulePositions());
        
    }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param inputStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] inputStates) {
        states = inputStates;
        
        
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    updateOdometry();

    SwerveDriveKinematics.desaturateWheelSpeeds(
                states, Constants.MAX_VELOCITY_METERS_PER_SECOND);

    frontLeftModule.set(feedforwardLeft.calculate(states[0].speedMetersPerSecond), states[0].angle.getRadians());
        frontRightModule.set(feedforwardRight.calculate(states[1].speedMetersPerSecond), states[1].angle.getRadians());
        backLeftModule.set(feedforwardLeft.calculate(states[2].speedMetersPerSecond), states[2].angle.getRadians());
        backRightModule.set(feedforwardRight.calculate(states[3].speedMetersPerSecond), states[3].angle.getRadians());
    
  }

  

}
