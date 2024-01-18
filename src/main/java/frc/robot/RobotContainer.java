// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import frc.robot.commands.AutoCommand;
import frc.robot.commands.ManualDrive;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem _swervesubsystem = new SwerveSubsystem();
  private final LimeLightSubsystem _limelightSubsystem = new LimeLightSubsystem();
  public ManualDrive manualDrive = new ManualDrive(_swervesubsystem, _limelightSubsystem);
  public AutoCommand autocommand = new AutoCommand(_swervesubsystem);
  public static final XboxController baseJoystick = new XboxController(0);
  private SendableChooser<Command> m_Chooser = new SendableChooser<>();
  public static final Joystick joystick = new Joystick(0);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    m_Chooser.setDefaultOption("autoCommand", autocommand);
    SmartDashboard.putData(m_Chooser);
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
