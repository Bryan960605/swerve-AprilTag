// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import frc.robot.commands.AutoCommand;
import frc.robot.commands.ManualDrive;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem m_SwerveSubsystem = new SwerveSubsystem();
  private final LimeLightSubsystem m_LimelightSubsystem = new LimeLightSubsystem();
  public AutoCommand autocommand = new AutoCommand(m_SwerveSubsystem);
  private final CommandXboxController DriverJoystick = new CommandXboxController(0);
  private SendableChooser<Command> m_Chooser = new SendableChooser<>();

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    m_Chooser.setDefaultOption("autoCommand", autocommand);
    SmartDashboard.putData(m_Chooser);

    /* ============
     *    Driver 
     * ============
    */
    /* Manual Drive */
    DoubleSupplier xSpeedFunc = () -> DriverJoystick.getRawAxis(1);
    DoubleSupplier ySpeedFunc = () -> DriverJoystick.getRawAxis(0);
    DoubleSupplier zSppedFunc = () -> DriverJoystick.getRawAxis(4);
    BooleanSupplier isAimModeFunc = () -> DriverJoystick.getHID().getRightTriggerAxis()>0.4;
    m_SwerveSubsystem.setDefaultCommand(new ManualDrive(m_SwerveSubsystem, m_LimelightSubsystem, xSpeedFunc, ySpeedFunc, zSppedFunc, isAimModeFunc));
    /* Reset Gyro */
    DriverJoystick.b().whileTrue(
      Commands.runOnce(()->{
        m_SwerveSubsystem.resetGyro();
    }));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return m_Chooser.getSelected();
  }
}
