package frc.robot.subsystems;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveModuleConstants;

public class SwerveModule extends SubsystemBase{
    
    private final CANSparkMax driveMotor;
    private final CANSparkMax turningMotor;

    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder turningEncoder;

    private final PIDController turningPIDController;

    private final CANcoder absoluteEncoder;
    private final CANcoderConfiguration cancoderConfig;
    private final double absoluteEncoderOffsetDegree;

    private double setpointangle;
    private double angle;

    public SwerveModule(int driveMotorID, int turningMotorID, boolean driveMotorReversed, boolean turningMotorReversed, 
                        int absoluteEncoderID, double absoluteEncoderOffsetDegree){
        this.absoluteEncoderOffsetDegree = absoluteEncoderOffsetDegree;
        absoluteEncoder = new CANcoder(absoluteEncoderID);
        cancoderConfig = new CANcoderConfiguration();

        driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
        turningMotor = new CANSparkMax(turningMotorID, MotorType.kBrushless);

        driveMotor.restoreFactoryDefaults();
        turningMotor.restoreFactoryDefaults();
        driveMotor.setInverted(driveMotorReversed);
        turningMotor.setInverted(turningMotorReversed);
        driveMotor.setIdleMode(IdleMode.kBrake);
        turningMotor.setIdleMode(IdleMode.kBrake);
        driveMotor.burnFlash();
        turningMotor.burnFlash();

        cancoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
        cancoderConfig.MagnetSensor.MagnetOffset = absoluteEncoderOffsetDegree;
        cancoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;

        driveEncoder = driveMotor.getEncoder();
        turningEncoder = turningMotor.getEncoder();
        
        driveEncoder.setPositionConversionFactor(SwerveModuleConstants.driveEncoderRot2Meter);
        driveEncoder.setVelocityConversionFactor(SwerveModuleConstants.driveEncoderRPM2MeterPerSec);
        turningEncoder.setPositionConversionFactor(SwerveModuleConstants.turningEncoderRot2Rad);
        turningEncoder.setVelocityConversionFactor(SwerveModuleConstants.turningEncoderRPM2RadPerSec);

        turningPIDController = new PIDController(SwerveModuleConstants.turningMotorkP, 0, 0);
        turningPIDController.enableContinuousInput(-180, 180);
        
        resetEncoders();
    }


   
    public double getDrivePosition(){
        return driveEncoder.getPosition();
    }
    public double getTurningPosition(){
        return absoluteEncoder.getAbsolutePosition().getValueAsDouble();
    }
    public double getTurnintEncoderPosition(){
        return turningEncoder.getPosition();
    }
    public double getDriveVelocity(){
        return driveEncoder.getVelocity();
    }
    public double getTurningVelocity(){
        return turningEncoder.getVelocity();
    }
    public void resetEncoders(){
        driveEncoder.setPosition(0);
        turningEncoder.setPosition(0);
        absoluteEncoder.getConfigurator().apply(cancoderConfig);

    }
    public SwerveModuleState getState(){
        return new SwerveModuleState(getDriveVelocity(), Rotation2d.fromDegrees(getTurningPosition()));
    }
    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(getDrivePosition(), Rotation2d.fromDegrees(getTurningPosition()));
    }
     
    public void setDesiredState(SwerveModuleState state){
        state = SwerveModuleState.optimize(state, getState().angle);
        driveMotor.set(state.speedMetersPerSecond);
        turningMotor.set(turningPIDController.calculate(getState().angle.getDegrees(),state.angle.getDegrees()));
        setpointangle = state.angle.getDegrees();
        angle = getState().angle.getDegrees();
    }


    @Override
    public void periodic(){
        SmartDashboard.putNumber("angle", angle);
        SmartDashboard.putNumber("setpoint", setpointangle);
    }
}