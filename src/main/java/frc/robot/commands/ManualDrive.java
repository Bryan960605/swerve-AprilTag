// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import static frc.robot.RobotContainer.*;

import frc.robot.Constants;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class ManualDrive extends Command {
  /** Creates a new ManualDrive. */
  private final SwerveSubsystem swerveSubsystem;
  private final LimeLightSubsystem limelightSubsystem;
  private double xSpeed;
  private double ySpeed;
  private double zSpeed;
  public ManualDrive(SwerveSubsystem _swerveSubsystem, LimeLightSubsystem _limelightSubsystem) {
    this.swerveSubsystem = _swerveSubsystem;
    this.limelightSubsystem = _limelightSubsystem;
    addRequirements(swerveSubsystem, limelightSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(joystick.getRawButton(1) == true ){
      xSpeed = limelightSubsystem.xMove();
      ySpeed = limelightSubsystem.yMove();
      zSpeed = limelightSubsystem.turn();
      swerveSubsystem.drive(xSpeed, ySpeed, zSpeed, false);
    }
    else{
      xSpeed = Constants.SwerveConstants.joysickValue(-joystick.getRawAxis(1), 0.08)*0.6;
      ySpeed = Constants.SwerveConstants.joysickValue(joystick.getRawAxis(0), 0.08)*0.4;
      zSpeed = Constants.SwerveConstants.joysickValue(joystick.getRawAxis(2), 0.08)*0.4;
      swerveSubsystem.drive(xSpeed, ySpeed, zSpeed, true);
    }

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
