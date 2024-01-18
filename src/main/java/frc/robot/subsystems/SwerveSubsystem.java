package frc.robot.subsystems;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.RobotContainer.*;

import static frc.robot.Constants.SwerveConstants.*;


public class SwerveSubsystem extends SubsystemBase{
    private final SwerveModule leftFrontModule, rightFrontModule, leftRearModule, rightRearModule;
    private final Pigeon2 gyro = new Pigeon2(gyroID);
    private SwerveDriveOdometry mOdometry;
    public SwerveSubsystem(){
        gyro.reset();
        gyro.setYaw(45);
        leftFrontModule = new SwerveModule(
            leftFrontDriveID, 
            leftFrontTurningID, 
            leftFrontdriveMotorReversed, 
            leftFrontTurningMotorReversed, 
            leftFrontCANCoderID, 
            leftFrontOffset);

        rightFrontModule = new SwerveModule(
            rightFrontDriveID,
            rightFrontTurningID,
            rightFrontDriveMotorReversed, 
            rightfrontTurningMotorReversed, 
            rightFrontCANCoderID, 
            rightFrontOffset);

        leftRearModule = new SwerveModule(
            leftRearDriveID, 
            leftRearTurningID, 
            leftRearDriveMotorreversed, 
            leftRearTurningMotorReversed, 
            leftRearCANCoderID, 
            leftRearOffset);

        rightRearModule = new SwerveModule(
            rightRearDriveID, 
            rightRearTurningID, 
            rightRearDriveMotorReversed, 
            rightRearTurningMotorReversed, 
            rightRearCANCoderID, 
            rightRearOffset);
        mOdometry = new SwerveDriveOdometry(
            swerveKinematics, 
            gyro.getRotation2d(), 
            getModulePosition());
    }
    public SwerveModulePosition[] getModulePosition(){
        return new SwerveModulePosition[]{
            leftFrontModule.getPosition(),
            rightFrontModule.getPosition(),
            leftRearModule.getPosition(),
            rightRearModule.getPosition()
        };
    }
    public SwerveModuleState[] getModuleStates(){
        return new SwerveModuleState[]{
            leftFrontModule.getState(),
            rightFrontModule.getState(),
            leftRearModule.getState(),
            rightRearModule.getState()
        };
    }
    public void setModuleStates(SwerveModuleState[] desiredStates){
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, 1);
        leftFrontModule.setDesiredState(desiredStates[0]);
        rightFrontModule.setDesiredState(desiredStates[1]);
        leftRearModule.setDesiredState(desiredStates[2]);
        rightRearModule.setDesiredState(desiredStates[3]);
    }
    public void drive(double xSpeed, double ySpeed, double zSpeed, boolean fieldOriented){
        SwerveModuleState[] states = null;
        if(fieldOriented){
            states = swerveKinematics.toSwerveModuleStates(
                ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, zSpeed, gyro.getRotation2d()));
        }else{
            states = swerveKinematics.toSwerveModuleStates(new ChassisSpeeds(xSpeed, ySpeed, zSpeed));
        }
        setModuleStates(states);
    }
    public Pose2d getPose(){
        return mOdometry.getPoseMeters();
    }
    public void setPose(Pose2d pose){
        mOdometry.resetPosition(gyro.getRotation2d(), getModulePosition(), pose);
    }
    @Override
    public void periodic(){
        mOdometry.update(gyro.getRotation2d(), getModulePosition());
        SmartDashboard.putNumber("xSpeed", baseJoystick.getRawAxis(1));
        SmartDashboard.putNumber("ySpeed", baseJoystick.getRawAxis(0));
        SmartDashboard.putNumber("zSpeed", baseJoystick.getRawAxis(4));
    }
  
    
}