package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.ExponentialProfile.State;
import edu.wpi.first.wpilibj.AnalogEncoder;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule {

    private final CANSparkMax m_drivingMotor;

    private final CANSparkMax m_turningMotor;

    private RelativeEncoder m_drivingEncoder;
    private RelativeEncoder m_turningEncoder;

    private AnalogEncoder m_analogEncoder;

    private PIDController m_turningPID;
    private SparkPIDController m_drivingPID;

    private double m_offset = 0;

    public SwerveModule(int drivingCANID, int turningCANID, double offset, int rioPort){

        m_drivingMotor = new CANSparkMax(drivingCANID, MotorType.kBrushless);
        m_turningMotor = new CANSparkMax(turningCANID, MotorType.kBrushless);

        m_analogEncoder = new AnalogEncoder(rioPort);

        m_drivingMotor.restoreFactoryDefaults();
        m_turningMotor.restoreFactoryDefaults();

        m_drivingMotor.setSmartCurrentLimit(40);
        m_turningMotor.setSmartCurrentLimit(30);


        m_drivingEncoder = m_drivingMotor.getEncoder();
        m_turningEncoder = m_turningMotor.getEncoder();

        m_drivingEncoder.setPositionConversionFactor(ModuleConstants.kDrivingPoseConversionFactor);
        m_drivingEncoder.setVelocityConversionFactor(ModuleConstants.kDrivingVeloConversionFactor);

        m_drivingPID = m_drivingMotor.getPIDController();
        m_drivingPID.setP(0.0001);
        m_drivingPID.setFF(1.0/5676.0);







    }


    public double getAnalog(){
        return m_analogEncoder.getAbsolutePosition()*2*Math.PI-m_offset;
    }

    public SwerveModuleState getModuleState(){
        return new SwerveModuleState(m_drivingEncoder.getVelocity(), new Rotation2d(getAnalog()));
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(m_drivingEncoder.getPosition(), new Rotation2d(getAnalog()));
    }

    public void setDesiredState(SwerveModuleState desiredState){

        SwerveModuleState state = SwerveModuleState.optimize(desiredState, new Rotation2d(getAnalog()));
        m_drivingPID.setReference(state.speedMetersPerSecond, ControlType.kVelocity);
        final double turnOutput = m_turningPID.calculate(getAnalog(), state.angle.getRadians());
        m_turningMotor.set(turnOutput);

    }

}