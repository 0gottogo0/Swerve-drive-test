// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Subsystems.Drivetrain;

public class DriveCommand extends CommandBase {
  private Drivetrain drivetrain;

  private DoubleSupplier translationXSupplier;
  private DoubleSupplier translationYSupplier;
  private DoubleSupplier rotationSupplier;
  private SlewRateLimiter xLimiter, yLimiter, turningLimiter;
  /** Creates a new DriveCommand. */
  public DriveCommand(Drivetrain drivetrainSubsystem,
                               DoubleSupplier translationXSupplier,
                               DoubleSupplier translationYSupplier,
                               DoubleSupplier rotationSupplier
                               ) {
    this.drivetrain = drivetrainSubsystem;
    this.translationXSupplier = translationXSupplier;
    this.translationYSupplier = translationYSupplier;
    this.rotationSupplier = rotationSupplier;

    this.xLimiter = new SlewRateLimiter(Constants.MAX_VELOCITY_METERS_PER_SECOND/.5);
    this.yLimiter = new SlewRateLimiter(Constants.MAX_VELOCITY_METERS_PER_SECOND/.5);
    this.turningLimiter = new SlewRateLimiter(Constants.MAX_VELOCITY_METERS_PER_SECOND/.125);

    addRequirements(drivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double rotationRate = rotationSupplier.getAsDouble();
        double xAxisRate            = translationXSupplier.getAsDouble();
        double yAxisRate            = translationYSupplier.getAsDouble();

        xAxisRate = xLimiter.calculate(xAxisRate);
        yAxisRate = yLimiter.calculate(yAxisRate);
        rotationRate = turningLimiter.calculate(rotationRate);
            
        
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            xAxisRate,
            yAxisRate,
            rotationRate,
            drivetrain.getGyroscopeRotation());
        
        // You can use `new ChassisSpeeds(...)` for robot-oriented movement instead of field-oriented movement
        drivetrain.setModuleStates(Constants.kinematics.toSwerveModuleStates(speeds));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
