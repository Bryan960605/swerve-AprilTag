// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class ManualDrive extends Command {
  /** Creates a new ManualDrive. */
  private final SwerveSubsystem m_SwerveSubsystem;
  private final LimeLightSubsystem m_LimelightSubsystem;
  // Inputs
  private DoubleSupplier xSpeedFunc;
  private DoubleSupplier ySpeedFunc;
  private DoubleSupplier zSpeedFunc;
  private BooleanSupplier isAimModeFunc;
  private double xSpeed;
  private double ySpeed;
  private double zSpeed;
  private boolean isAimMode;
  // Slew rate limiter
  private final SlewRateLimiter xLimiter;
  private final SlewRateLimiter yLimiter;
  private final SlewRateLimiter zLimiter;
  public ManualDrive(SwerveSubsystem swerveSubsystem, LimeLightSubsystem limelightSubsystem, DoubleSupplier xSpeed, DoubleSupplier ySpeed, DoubleSupplier zSpeed, BooleanSupplier isSAimMode) {
    this.m_SwerveSubsystem = swerveSubsystem;
    this.m_LimelightSubsystem = limelightSubsystem;
    this.xSpeedFunc = xSpeed;
    this.ySpeedFunc = ySpeed;
    this.zSpeedFunc = zSpeed;
    xLimiter = new SlewRateLimiter(4);
    yLimiter = new SlewRateLimiter(4);
    zLimiter = new SlewRateLimiter(4);
    addRequirements(m_SwerveSubsystem, m_LimelightSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Get value
    // 負號加在這
    xSpeed = -xSpeedFunc.getAsDouble();
    ySpeed = -ySpeedFunc.getAsDouble();  
    zSpeed = -zSpeedFunc.getAsDouble();
    isAimMode = isAimModeFunc.getAsBoolean();
    if(isAimMode){
      xSpeed = m_LimelightSubsystem.xMove();
      ySpeed = m_LimelightSubsystem.yMove();
      zSpeed = m_LimelightSubsystem.turn();
      m_SwerveSubsystem.drive(xSpeed, ySpeed, zSpeed, false);
    }
    else{
      xSpeed = xSpeed*0.4;
      ySpeed = ySpeed*0.4;
      zSpeed = zSpeed*0.4;
      m_SwerveSubsystem.drive(xSpeed, ySpeed, zSpeed, true);
    }
    // SlewRate
    xSpeed = xLimiter.calculate(xSpeed);
    ySpeed = yLimiter.calculate(ySpeed);
    zSpeed = zLimiter.calculate(zSpeed);

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
