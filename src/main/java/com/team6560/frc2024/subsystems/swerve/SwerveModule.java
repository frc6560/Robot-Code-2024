/*
 * Copyright (C) Cross The Road Electronics.  All rights reserved.
 * License information can be found in CTRE_LICENSE.txt
 * For support and suggestions contact support@ctr-electronics.com or file
 * an issue tracker at https://github.com/CrossTheRoadElec/Phoenix-Releases
 */
package com.team6560.frc2024.subsystems.swerve;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
// import com.ctre.phoenix6.hardware.DeviceIdentifier;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
// import com.ctre.phoenix6.spns.SpnValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;

/**
 * Swerve Module class that encapsulates a swerve module powered by CTR
 * Electronics devices.
 * <p>
 * This class handles the hardware devices and configures them for
 * swerve module operation using the Phoenix 6 API.
 * <p>
 * This class constructs hardware devices internally, so the user
 * only specifies the constants (IDs, PID gains, gear ratios, etc).
 * Getters for these hardware devices are available.
 */
public class SwerveModule {
    /**
     * Supported closed-loop output types.
     */
    public enum ClosedLoopOutputType {
        Voltage,
        /** Requires Pro */
        TorqueCurrentFOC,
    }

    /**
     * All possible control requests for the module steer motor.
     */
    public enum SteerRequestType {
        /**
         * Control the drive motor using a Motion Magic® request.
         * The control output type is determined by {@link SwerveModuleConstants#SteerMotorClosedLoopOutput}
         */
        MotionMagic,
        /**
         * Control the drive motor using a Motion Magic® Expo request.
         * The control output type is determined by {@link SwerveModuleConstants#SteerMotorClosedLoopOutput}
         */
        MotionMagicExpo,
    }

    /**
     * All possible control requests for the module drive motor.
     */
    public enum DriveRequestType {
        /**
         * Control the drive motor using an open-loop voltage request.
         */
        OpenLoopVoltage,
        /**
         * Control the drive motor using a velocity closed-loop request.
         * The control output type is determined by {@link SwerveModuleConstants#DriveMotorClosedLoopOutput}
         */
        Velocity,
    }

    private final TalonFX m_driveMotor;
    private final CANSparkMax m_steerMotor;
    private final CANcoder m_cancoder;

    private final StatusSignal<Double> m_drivePosition;
    private final StatusSignal<Double> m_driveVelocity;
    private final double m_steerPosition;
    private final double m_steerVelocity;
    private final BaseStatusSignal[] m_signals_drive;
    private final Double[] m_signals_steer;
    private final double m_driveRotationsPerMeter;
    private final double m_couplingRatioDriveRotorToCANcoder;

    private final double m_speedAt12VoltsMps;

    /* steer motor controls */
    private final MotionMagicVoltage m_angleVoltageSetter = new MotionMagicVoltage(0);
    private final MotionMagicTorqueCurrentFOC m_angleTorqueSetter = new MotionMagicTorqueCurrentFOC(0);
    private final MotionMagicExpoVoltage m_angleVoltageExpoSetter = new MotionMagicExpoVoltage(0);
    private final MotionMagicExpoTorqueCurrentFOC m_angleTorqueExpoSetter = new MotionMagicExpoTorqueCurrentFOC(0);
    /* drive motor controls */
    private final VoltageOut m_voltageOpenLoopSetter = new VoltageOut(0);
    private final VelocityVoltage m_velocityVoltageSetter = new VelocityVoltage(0);
    private final VelocityTorqueCurrentFOC m_velocityTorqueSetter = new VelocityTorqueCurrentFOC(0);

    private final ClosedLoopOutputType m_steerClosedLoopOutput;
    private final ClosedLoopOutputType m_driveClosedLoopOutput;

    private final SwerveModulePosition m_internalState = new SwerveModulePosition();
    private SwerveModuleState m_targetState = new SwerveModuleState();

    /**
     * Construct a SwerveModule with the specified constants.
     *
     * @param constants   Constants used to construct the module
     * @param canbusName  The name of the CAN bus this module is on
     */
    public SwerveModule(SwerveModuleConstants constants, String canbusName) {
        m_driveMotor = new TalonFX(constants.DriveMotorId, canbusName);
        m_steerMotor = new CANSparkMax(constants.SteerMotorId, MotorType.kBrushless);
        m_cancoder = new CANcoder(constants.CANcoderId, canbusName);

        TalonFXConfiguration talonConfigs = new TalonFXConfiguration();

        talonConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        talonConfigs.Slot0 = constants.DriveMotorGains;
        talonConfigs.TorqueCurrent.PeakForwardTorqueCurrent = constants.SlipCurrent;
        talonConfigs.TorqueCurrent.PeakReverseTorqueCurrent = -constants.SlipCurrent;
        talonConfigs.CurrentLimits.StatorCurrentLimit = constants.SlipCurrent;
        talonConfigs.CurrentLimits.StatorCurrentLimitEnable = true;

        talonConfigs.MotorOutput.Inverted = constants.DriveMotorInverted ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;
        StatusCode response = m_driveMotor.getConfigurator().apply(talonConfigs);
        if (!response.isOK()) {
            System.out
                    .println("Talon ID " + constants.DriveMotorId + " failed config with error " + response.toString());
        }

        /* Undo changes for torqueCurrent */
        talonConfigs.TorqueCurrent = new TorqueCurrentConfigs();
        /* And to current limits */
        talonConfigs.CurrentLimits = new CurrentLimitsConfigs();

        talonConfigs.Slot0 = constants.SteerMotorGains;
        // Modify configuration to use remote CANcoder fused
        talonConfigs.Feedback.FeedbackRemoteSensorID = constants.CANcoderId;
        switch (constants.FeedbackSource) {
            case RemoteCANcoder:
                talonConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
                break;
            case FusedCANcoder:
                talonConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
                break;
            case SyncCANcoder:
                talonConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.SyncCANcoder;
                break;
        }
        talonConfigs.Feedback.RotorToSensorRatio = constants.SteerMotorGearRatio;

        talonConfigs.MotionMagic.MotionMagicCruiseVelocity = 100.0 / constants.SteerMotorGearRatio;
        talonConfigs.MotionMagic.MotionMagicAcceleration = talonConfigs.MotionMagic.MotionMagicCruiseVelocity / 0.100;
        talonConfigs.MotionMagic.MotionMagicExpo_kV = 0.12 * constants.SteerMotorGearRatio;
        talonConfigs.MotionMagic.MotionMagicExpo_kA = 0.1;

        // talonConfigs.ClosedLoopGeneral.ContinuousWrap = true; // Enable continuous wrap for swerve modules

        m_steerMotor.getPIDController().setPositionPIDWrappingEnabled(true);
        m_steerMotor.getPIDController().setSmartMotionMaxVelocity(100.0 / constants.SteerMotorGearRatio, 0);
        m_steerMotor.getPIDController().setSmartMotionMaxAccel(100.0 / constants.SteerMotorGearRatio / 0.100, 0);
        m_steerMotor.setInverted(constants.SteerMotorInverted);
        // m_steerMotor.
        // if (!response.isOK()) {
        //     System.out
        //             .println("CANSparkMax ID " + constants.DriveMotorId + " failed config with error " + response.toString());
        // }

        CANcoderConfiguration cancoderConfigs = new CANcoderConfiguration();
        cancoderConfigs.MagnetSensor.MagnetOffset = constants.CANcoderOffset;
        response = m_cancoder.getConfigurator().apply(cancoderConfigs);
        if (!response.isOK()) {
            System.out.println(
                    "CANcoder ID " + constants.DriveMotorId + " failed config with error " + response.toString());
        }

        m_drivePosition = m_driveMotor.getPosition().clone();
        m_driveVelocity = m_driveMotor.getVelocity().clone();
        m_steerPosition = m_steerMotor.getEncoder().getPosition();
        m_steerVelocity = m_steerMotor.getEncoder().getVelocity();

        m_signals_drive = new BaseStatusSignal[2];
        m_signals_steer = new Double[2];
        m_signals_drive[0] = m_drivePosition;
        m_signals_drive[1] = m_driveVelocity;
        m_signals_steer[2] = m_steerPosition;
        m_signals_steer[3] = m_steerVelocity;

        /* Calculate the ratio of drive motor rotation to meter on ground */
        double rotationsPerWheelRotation = constants.DriveMotorGearRatio;
        double metersPerWheelRotation = 2 * Math.PI * Units.inchesToMeters(constants.WheelRadius);
        m_driveRotationsPerMeter = rotationsPerWheelRotation / metersPerWheelRotation;
        m_couplingRatioDriveRotorToCANcoder = constants.CouplingGearRatio;

        /* Make control requests synchronous */
        m_angleVoltageSetter.UpdateFreqHz = 0;
        m_angleTorqueSetter.UpdateFreqHz = 0;
        m_angleVoltageExpoSetter.UpdateFreqHz = 0;
        m_angleTorqueExpoSetter.UpdateFreqHz = 0;

        m_velocityTorqueSetter.UpdateFreqHz = 0;
        m_velocityVoltageSetter.UpdateFreqHz = 0;
        m_voltageOpenLoopSetter.UpdateFreqHz = 0;

        /* Set the drive motor closed-loop output type */
        m_steerClosedLoopOutput = constants.SteerMotorClosedLoopOutput;
        m_driveClosedLoopOutput = constants.DriveMotorClosedLoopOutput;

        /* Get the expected speed when applying 12 volts */
        m_speedAt12VoltsMps = constants.SpeedAt12VoltsMps;
    }

    /**
     * Gets the state of this module and passes it back as a
     * SwerveModulePosition object with latency compensated values.
     *
     * @param refresh True if the signals should be refreshed
     * @return SwerveModulePosition containing this module's state.
     */
    public SwerveModulePosition getPosition(boolean refresh) {
        if (refresh) {
            /* Refresh all signals */
            m_drivePosition.refresh();
            m_driveVelocity.refresh();
        }

        /* Now latency-compensate our signals */
        double drive_rot = BaseStatusSignal.getLatencyCompensatedValue(m_drivePosition, m_driveVelocity);
        double angle_rot = m_steerPosition; //BaseStatusSignal.getLatencyCompensatedValue(m_steerPosition, m_steerVelocity);

        /*
         * Back out the drive rotations based on angle rotations due to coupling between
         * azimuth and steer
         */
        drive_rot -= angle_rot * m_couplingRatioDriveRotorToCANcoder;

        /* And push them into a SwerveModulePosition object to return */
        m_internalState.distanceMeters = drive_rot / m_driveRotationsPerMeter;
        /* Angle is already in terms of steer rotations */
        m_internalState.angle = Rotation2d.fromRotations(angle_rot);

        return m_internalState;
    }

    /**
     * Applies the desired SwerveModuleState to this module.
     *
     * @param state            Speed and direction the module should target
     * @param driveRequestType The {@link DriveRequestType} to apply
     */
    public void apply(SwerveModuleState state, DriveRequestType driveRequestType) {
        apply(state, driveRequestType, SteerRequestType.MotionMagic);
    }

    /**
     * Applies the desired SwerveModuleState to this module.
     *
     * @param state            Speed and direction the module should target
     * @param driveRequestType The {@link DriveRequestType} to apply
     * @param steerRequestType The {@link SteerRequestType} to apply; defaults to {@link SteerRequestType#MotionMagic}
     */
    public void apply(SwerveModuleState state, DriveRequestType driveRequestType, SteerRequestType steerRequestType) {
        var optimized = SwerveModuleState.optimize(state, m_internalState.angle);
        m_targetState = optimized;

        double angleToSetDeg = optimized.angle.getRotations();
        switch (steerRequestType) {
            case MotionMagic:
                switch (m_steerClosedLoopOutput) {
                    case Voltage:
                        m_steerMotor.getPIDController().setReference(angleToSetDeg, ControlType.kSmartMotion);
                        // m_steerMotor.setControl(m_angleVoltageSetter.withPosition(angleToSetDeg));
                        break;

                    case TorqueCurrentFOC:
                        m_steerMotor.getPIDController().setReference(angleToSetDeg, ControlType.kSmartMotion);
                        // m_steerMotor.setControl(m_angleTorqueSetter.withPosition(angleToSetDeg));
                        break;
                }
                break;

            case MotionMagicExpo:
                switch (m_steerClosedLoopOutput) {
                    case Voltage:
                        // m_steerMotor.setControl(m_angleVoltageExpoSetter.withPosition(angleToSetDeg));
                        m_steerMotor.getPIDController().setReference(angleToSetDeg, ControlType.kSmartMotion);
                        break;

                    case TorqueCurrentFOC:
                        m_steerMotor.getPIDController().setReference(angleToSetDeg, ControlType.kSmartMotion);
                        // m_steerMotor.setControl(m_angleTorqueExpoSetter.withPosition(angleToSetDeg));
                        break;
                }
                break;
        }

        double velocityToSet = optimized.speedMetersPerSecond * m_driveRotationsPerMeter;

        /* From FRC 900's whitepaper, we add a cosine compensator to the applied drive velocity */
        /* To reduce the "skew" that occurs when changing direction */
        double steerMotorError = angleToSetDeg - m_steerPosition;
        /* If error is close to 0 rotations, we're already there, so apply full power */
        /* If the error is close to 0.25 rotations, then we're 90 degrees, so movement doesn't help us at all */
        double cosineScalar = Math.cos(Units.rotationsToRadians(steerMotorError));
        /* Make sure we don't invert our drive, even though we shouldn't ever target over 90 degrees anyway */
        if (cosineScalar < 0.0) {
            cosineScalar = 0.0;
        }
        velocityToSet *= cosineScalar;

        /* Back out the expected shimmy the drive motor will see */
        /* Find the angular rate to determine what to back out */
        double azimuthTurnRps = m_steerVelocity;
        /* Azimuth turn rate multiplied by coupling ratio provides back-out rps */
        double driveRateBackOut = azimuthTurnRps * m_couplingRatioDriveRotorToCANcoder;
        velocityToSet -= driveRateBackOut;

        switch (driveRequestType) {
            case OpenLoopVoltage:
                /* Open loop ignores the driveRotationsPerMeter since it only cares about the open loop at the mechanism */
                /* But we do care about the backout due to coupling, so we keep it in */
                velocityToSet /= m_driveRotationsPerMeter;
                m_driveMotor.setControl(m_voltageOpenLoopSetter.withOutput(velocityToSet / m_speedAt12VoltsMps * 12.0));
                break;

            case Velocity:
                switch (m_driveClosedLoopOutput) {
                    case Voltage:
                        m_driveMotor.setControl(m_velocityVoltageSetter.withVelocity(velocityToSet));
                        break;

                    case TorqueCurrentFOC:
                        m_driveMotor.setControl(m_velocityTorqueSetter.withVelocity(velocityToSet));
                        break;
                }
                break;
        }
    }

    /**
     * Gets the last cached swerve module position.
     * This differs from {@link getPosition} in that it will not
     * perform any latency compensation or refresh the signals.
     *
     * @return Last cached SwerveModulePosition
     */
    public SwerveModulePosition getCachedPosition() {
        return m_internalState;
    }

    /**
     * Get the current state of the module.
     * <p>
     * This is typically used for telemetry, as the SwerveModulePosition
     * is used for odometry.
     *
     * @return Current state of the module
     */
    public SwerveModuleState getCurrentState() {
        return new SwerveModuleState(m_driveVelocity.getValue() / m_driveRotationsPerMeter, Rotation2d.fromRotations(m_steerPosition));
    }

    /**
     * Get the target state of the module.
     * <p>
     * This is typically used for telemetry.
     *
     * @return Target state of the module
     */
    public SwerveModuleState getTargetState() {
        return m_targetState;
    }


    /**
     * Resets this module's drive motor position to 0 rotations.
     */
    public void resetPosition() {
        /* Only touch drive pos, not steer */
        m_driveMotor.setPosition(0);
    }

    /**
     * Gets this module's Drive Motor TalonFX reference.
     * <p>
     * This should be used only to access signals and change configurations that the
     * swerve drivetrain does not configure itself.
     *
     * @return This module's Drive Motor reference
     */
    public TalonFX getDriveMotor() {
        return m_driveMotor;
    }

    /**
     * Gets this module's Steer Motor TalonFX reference.
     * <p>
     * This should be used only to access signals and change configurations that the
     * swerve drivetrain does not configure itself.
     *
     * @return This module's Steer Motor reference
     */
    public CANSparkMax getSteerMotor() {
        return m_steerMotor;
    }

    /**
     * Gets this module's CANcoder reference.
     * <p>
     * This should be used only to access signals and change configurations that the
     * swerve drivetrain does not configure itself.
     *
     * @return This module's CANcoder reference
     */
    public CANcoder getCANcoder() {
        return m_cancoder;
    }

    public BaseStatusSignal[] getSignals() {
        return m_signals_drive;
    }
}
