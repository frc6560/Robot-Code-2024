// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6560.frc2024.subsystems;

// WPI & REV & SYSTEM:
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import edu.wpi.first.wpilibj.DriverStation;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
// import com.revrobotics.CANSparkMax.IdleMode;
import com.swervedrivespecialties.swervelib.MotorType;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

// UTIL:
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.Pair;
// import java.util.function.Supplier;
import static com.team6560.frc2024.Constants.*;

// SWERVE:
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import com.swervedrivespecialties.swervelib.MkModuleConfiguration;
import com.swervedrivespecialties.swervelib.MkSwerveModuleBuilder;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;
import com.team6560.frc2024.Constants;

public class Drivetrain extends SubsystemBase {
        /**
         * The maximum voltage that will be delivered to the drive motors.
         * This can be reduced to cap the robot's maximum speed. Typically, this is
         * useful during initial testing of the robot.
         */
        public static final double MAX_VOLTAGE = 12.0;

        private final Pigeon2 pigeon = new Pigeon2(GYRO_ID);

        private final SwerveModule m_frontLeftModule;
        private final SwerveModule m_frontRightModule;
        private final SwerveModule m_backLeftModule;
        private final SwerveModule m_backRightModule;

        // private SwerveDriveKinematics kinematics;

        /**
         * The default states for each module, corresponding to an X shape.
         */
        public static final SwerveModuleState[] DEFAULT_MODULE_STATES = new SwerveModuleState[] {
                        new SwerveModuleState(0.0, Rotation2d.fromDegrees(45.0)),
                        new SwerveModuleState(0.0, Rotation2d.fromDegrees(-45.0)),
                        new SwerveModuleState(0.0, Rotation2d.fromDegrees(-45.0)),
                        new SwerveModuleState(0.0, Rotation2d.fromDegrees(45.0))
        };

        // SETUP
        public SwerveModule[] modules;

        // ODOMETRY
        private final SwerveDriveOdometry odometry;

        public Drivetrain() {
                m_frontLeftModule = new MkSwerveModuleBuilder(MkModuleConfiguration.getDefaultSteerNEO())
                                // .withLayout(tab.getLayout("Back Right Module", BuiltInLayouts.kList)
                                // .withSize(2, 4)
                                // .withPosition(6, 0))
                                .withGearRatio(SdsModuleConfigurations.MK4I_L2)
                                .withDriveMotor(MotorType.FALCON, FRONT_LEFT_MODULE_DRIVE_MOTOR)
                                .withSteerMotor(MotorType.NEO, FRONT_LEFT_MODULE_STEER_MOTOR)
                                .withSteerEncoderPort(FRONT_LEFT_MODULE_STEER_ENCODER)
                                .withSteerOffset(FRONT_LEFT_MODULE_STEER_OFFSET)
                                .build();

                m_frontRightModule = new MkSwerveModuleBuilder(MkModuleConfiguration.getDefaultSteerNEO())
                                .withGearRatio(SdsModuleConfigurations.MK4I_L2)
                                .withDriveMotor(MotorType.FALCON, FRONT_RIGHT_MODULE_DRIVE_MOTOR)
                                .withSteerMotor(MotorType.NEO, FRONT_RIGHT_MODULE_STEER_MOTOR)
                                .withSteerEncoderPort(FRONT_RIGHT_MODULE_STEER_ENCODER)
                                .withSteerOffset(FRONT_RIGHT_MODULE_STEER_OFFSET)
                                .build();

                m_backLeftModule = new MkSwerveModuleBuilder(MkModuleConfiguration.getDefaultSteerNEO())
                                .withGearRatio(SdsModuleConfigurations.MK4I_L2)
                                .withDriveMotor(MotorType.FALCON, BACK_LEFT_MODULE_DRIVE_MOTOR)
                                .withSteerMotor(MotorType.NEO, BACK_LEFT_MODULE_STEER_MOTOR)
                                .withSteerEncoderPort(BACK_LEFT_MODULE_STEER_ENCODER)
                                .withSteerOffset(BACK_LEFT_MODULE_STEER_OFFSET)
                                .build();

                m_backRightModule = new MkSwerveModuleBuilder(MkModuleConfiguration.getDefaultSteerNEO())
                                .withGearRatio(SdsModuleConfigurations.MK4I_L2)
                                .withDriveMotor(MotorType.FALCON, BACK_RIGHT_MODULE_DRIVE_MOTOR)
                                .withSteerMotor(MotorType.NEO, BACK_RIGHT_MODULE_STEER_MOTOR)
                                .withSteerEncoderPort(BACK_RIGHT_MODULE_STEER_ENCODER)
                                .withSteerOffset(BACK_RIGHT_MODULE_STEER_OFFSET)
                                .build();

                modules = new SwerveModule[] { m_frontLeftModule, m_frontRightModule, m_backLeftModule,
                                m_backRightModule };

                // kinematics = new SwerveDriveKinematics();

                odometry = new SwerveDriveOdometry(m_kinematics, getRawGyroRotation(), getModulePositions());
                resetOdometry(new Pose2d());
        }

        public SwerveModule[] getModules() {
                return this.modules;
        }

        @Override
        public void periodic() {
                updateOdometry();
        }

        // Updates the field-relative position.
        private void updateOdometry() {
                odometry.update(getRawGyroRotation(), getModulePositions());
        }

        // This method is used to control the movement of the chassis.
        public void drive(ChassisSpeeds chassisSpeeds) {
                SwerveModuleState[] speeds = m_kinematics.toSwerveModuleStates(chassisSpeeds);
                SwerveDriveKinematics.desaturateWheelSpeeds(speeds, MAX_VELOCITY_METERS_PER_SECOND);
                setChassisState(speeds);
        }

        // Sets the speeds and orientations of each swerve module.
        // array order: front left, front right, back left, back right
        public void setChassisState(SwerveModuleState[] states) {
                m_frontLeftModule.set(states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                                states[0].angle.getRadians());
                m_frontRightModule.set(states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                                states[1].angle.getRadians());
                m_backLeftModule.set(states[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                                states[2].angle.getRadians());
                m_backRightModule.set(states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                                states[3].angle.getRadians());

        }

        public void setChassisState(double fLdeg, double fRdeg, double bLdeg, double bRdeg) {
                setChassisState(
                                new SwerveModuleState[] {
                                                new SwerveModuleState(0.0, Rotation2d.fromDegrees(fLdeg)),
                                                new SwerveModuleState(0.0, Rotation2d.fromDegrees(fRdeg)),
                                                new SwerveModuleState(0.0, Rotation2d.fromDegrees(bLdeg)),
                                                new SwerveModuleState(0.0, Rotation2d.fromDegrees(bRdeg))
                                });
        }

        // Sets drive motor idle mode to be either brake mode or coast mode.
        public void setDriveMotorBrakeMode(boolean brake) {
                IdleMode sparkMaxMode = brake ? IdleMode.kBrake : IdleMode.kCoast;
                NeutralModeValue phoenixMode = brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;

                for (SwerveModule i : modules) {
                        if (i.getSteerMotor() instanceof CANSparkMax)
                                ((CANSparkMax) i.getSteerMotor()).setIdleMode(IdleMode.kCoast);
                        else
                                ((TalonFX) i.getSteerMotor()).setNeutralMode(NeutralModeValue.Coast);

                        if (i.getDriveMotor() instanceof CANSparkMax)
                                ((CANSparkMax) i.getDriveMotor()).setIdleMode(sparkMaxMode);
                        else
                                ((TalonFX) i.getDriveMotor()).setNeutralMode(phoenixMode);
                }
        }

        // This method is used to stop all of the swerve drive modules.
        public void stopModules() {
                for (SwerveModule i : modules) {
                        i.set(0.0, i.getSteerAngle());
                }
        }

        public Rotation2d getRawGyroRotation() {
                return Rotation2d.fromDegrees(pigeon.getYaw().getValueAsDouble());
        }

        public Rotation2d getGyroscopeRotationNoApriltags() {
                return getOdometryPose2dNoApriltags().getRotation();
        }

        public Pose2d getOdometryPose2dNoApriltags() {
                return odometry.getPoseMeters();
        }

        public SwerveModulePosition[] getModulePositions() {
                return new SwerveModulePosition[] { m_frontLeftModule.getPosition(), m_frontRightModule.getPosition(),
                                m_backLeftModule.getPosition(), m_backRightModule.getPosition() };
        }

        // Gets the current pose of the robot according to the odometer/estimator
        public Pose2d getPose() {
                return odometry.getPoseMeters();
        }

        public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
                ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);

                SwerveModuleState[] targetStates = m_kinematics.toSwerveModuleStates(targetSpeeds);
                setChassisState(targetStates);
        }

        // This method is used to reset the position of the robot's pose estimator.
        public void resetOdometry(Pose2d pose) {
                odometry.resetPosition(getRawGyroRotation(), getModulePositions(), pose);
        }

        // Sets the gyroscope angle to zero. This can be used to set the direction the
        // robot is currently facing to the 'forwards' direction.
        public void zeroGyroscope() {
                resetOdometry(new Pose2d(getPose().getTranslation(), new Rotation2d(0.0)));
        }

        public ChassisSpeeds getChassisSpeeds() {
                return m_kinematics.toChassisSpeeds(getStates());
        }

        public SwerveModuleState[] getStates() {
                return new SwerveModuleState[] { m_frontLeftModule.getState(), m_frontRightModule.getState(),
                                m_backLeftModule.getState(), m_backRightModule.getState() };
        }
}