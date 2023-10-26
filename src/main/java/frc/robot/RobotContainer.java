// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;

import com.pathplanner.lib.util.PIDConstants;
import com.swervedrivespecialties.swervelib.SwerveModuleFactoryBuilder;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Commands.DriveCommand;
import frc.robot.Subsystems.Drivetrain;

public class RobotContainer {
  private final Drivetrain drivetrain;
  
  private final DriveCommand driveCommand;

  private final XboxController controllerDriver = new XboxController(0);
  public RobotContainer() {
    drivetrain = new Drivetrain();
    // Set up the default command for the drivetrain.
    // The controls are for field-oriented driving:
    // Left stick Y axis -> forward and backwards movement
    // Left stick X axis -> left and right movement
    // Right stick X axis -> rotation
    driveCommand = new DriveCommand(
      drivetrain,
      () -> -modifyAxis(controllerDriver.getLeftY()) * Constants.MAX_VELOCITY_METERS_PER_SECOND,
      () -> -modifyAxis(controllerDriver.getLeftX()) * Constants.MAX_VELOCITY_METERS_PER_SECOND,
      () -> -modifyAxis(controllerDriver.getRightX()) * Constants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
      );

    drivetrain.setDefaultCommand(driveCommand);
    configureBindings();
  }

  private void configureBindings() {

    //  button zeros the gyroscope
    new Trigger(controllerDriver::getBackButton)
      .onTrue( new InstantCommand(() -> drivetrain.zeroGyroscope()));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    // This will load the file "Example Path.path" and generate it with a max velocity of 8 m/s and a max acceleration of 5 m/s^2
    

    HashMap<String, Command> eventMap = new HashMap<>();
    
    SwerveModuleFactoryBuilder autoBuilder = new SwerveAutoBuilder(
      drivetrain::getPose, 
      drivetrain::resetOdometry, 
      Constants.kinematics, 
      new PIDConstants(Constants.kPXYController, 0, 0), 
      new PIDConstants(Constants.kPThetaController, 0, Constants.kDThetaController), 
      drivetrain::setModuleStates, 
      eventMap, 
      true,
      drivetrain
      );
      
    Command auto;
    
    auto = autoBuilder.fullAuto(autoChooser.getSelected());
    
      
      

    // Reset odometry to the starting pose of the trajectory.
    //drivetrain.resetOdometry(examplePath.getInitialPose());

    // Run path following command, then stop at the end.
    return auto
    .andThen(() -> drivetrain.stopDrive());

  }

  private static double deadband(double value, double deadband) { 
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }

  private static double modifyAxis(double value) {
    // Deadband
    value = deadband(value, 0.2);

    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
  }

}
