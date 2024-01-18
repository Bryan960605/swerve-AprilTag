// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

public class AutoCommand extends Command {
  /** Creates a new Autocoand. */
  private SwerveSubsystem swervesubsystem;
  private Timer time = new Timer();
  private double now_time;
  private double xSpeed;
  private double ySpeed;
  private double zSpeed;
  private boolean fieldOriented; 
  public AutoCommand(SwerveSubsystem _swervesubsystem) {
    this.swervesubsystem = _swervesubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    time.reset();
    time.start();
    fieldOriented = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    now_time = time.get();
    if(now_time <= 15){
      xSpeed = 0.3;
      ySpeed = 0.0;
      zSpeed = 0.3;
    }
    else{
      xSpeed = 0.0;
      ySpeed = 0.0;
      zSpeed = 0.0;
    }
    swervesubsystem.drive(xSpeed, ySpeed, zSpeed, fieldOriented);
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
