// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LimeLightSubsystem extends SubsystemBase {
  /** Creates a new LimeLightSubsystem. */
  private final PhotonCamera photonLimelight = new PhotonCamera( "OV5647");

  private final PIDController yMovePID = new PIDController(0.005, 0, 0);
  private final PIDController xMovePID = new PIDController(0.0030, 0, 0);
  private final PIDController turnPID = new PIDController(0.005, 0, 0);

  private NetworkTable table = NetworkTableInstance.getDefault().getTable("photonvision/OV5647");
  private DoubleArraySubscriber botPose3DGet = table.getDoubleArrayTopic("targetPose").subscribe(new double[] {0, 0, 0});

  private double yMovePIDOutput, xMovePIDOutput, turnPIDOutput;

  private final double maxXMovepPIDOutput = 0.3; 
  private final double maxYMovePIDOutput = 0.3;
  private final double maxTurnPIDOutput = 0.5;

  private double[] botPoseValue;
  private double botX;
  private double botY;
  private double botZ;

  public LimeLightSubsystem() {
    
  }

  public double xMove(){
    return xMovePIDOutput;
  }

  public double yMove(){
    return yMovePIDOutput;
  }

  public double turn(){
    return turnPIDOutput;
  }

  @Override
  public void periodic() {
    PhotonPipelineResult result = photonLimelight.getLatestResult();
    PhotonTrackedTarget target = result.getBestTarget();
    boolean hasTarget = result.hasTargets();
    botPoseValue = botPose3DGet.get();
    if(hasTarget){
      botX = botPoseValue[0]*100;
      botY = botPoseValue[1];
      botZ = Math.toDegrees(target.getBestCameraToTarget().getRotation().getAngle()); //photonLimelight.getLatestResult().getBestTarget().getYaw();
    }
    else{
      botX = 50;
      botY = 0;
      botZ = 180;
    }
    yMovePIDOutput = yMovePID.calculate(botY, 0);
    xMovePIDOutput = xMovePID.calculate(botX, 50);
    turnPIDOutput = -turnPID.calculate(botZ, 180);

    xMovePIDOutput = Constants.setMaxOutput(xMovePIDOutput, maxXMovepPIDOutput);
    yMovePIDOutput = Constants.setMaxOutput(yMovePIDOutput, maxYMovePIDOutput);
    turnPIDOutput = Constants.setMaxOutput(turnPIDOutput, maxTurnPIDOutput);
   
    SmartDashboard.putNumber("Yaw", botZ);
    SmartDashboard.putNumber("photonY", botY);
    SmartDashboard.putNumber("photonX", botX);

    SmartDashboard.putNumber("xMovePIDOutput", xMovePIDOutput);
    SmartDashboard.putNumber("yMovePIDOutput", yMovePIDOutput);
    SmartDashboard.putNumber("turn", turnPIDOutput);
  }
}