// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.utils.SwerveModuleConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
/*
 * 
 * Swerve Drive configuration:
 * 6/7 --F-- 8/1
 * | |
 * L R
 * | |
 * 4/5 --R-- 2/3
 * 
 */
public final class Constants {
  public static final class DriveConstants {
    /*
     * public static final int kFrontLeftDriveMotorPort = 0;
     * public static final int kRearLeftDriveMotorPort = 2;
     * public static final int kFrontRightDriveMotorPort = 4;
     * public static final int kRearRightDriveMotorPort = 6;
     * 
     * public static final int kFrontLeftTurningMotorPort = 1;
     * public static final int kRearLeftTurningMotorPort = 3;
     * public static final int kFrontRightTurningMotorPort = 5;
     * public static final int kRearRightTurningMotorPort = 7;
     */

    public static final boolean kFrontLeftTurningMotorReversed = false;
    public static final boolean kRearLeftTurningMotorReversed = true;
    public static final boolean kFrontRightTurningMotorReversed = false;
    public static final boolean kRearRightTurningMotorReversed = true;

    public static final boolean kFrontLeftDriveMotorReversed = false;
    public static final boolean kRearLeftDriveMotorReversed = true;
    public static final boolean kFrontRightDriveMotorReversed = false;
    public static final boolean kRearRightDriveMotorReversed = true;

    public static final double kTrackWidth = 0.5;
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = 0.7;
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    public static final boolean kGyroReversed = false;

    // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
    // These characterization values MUST be determined either experimentally or
    // theoretically
    // for *your* robot's drive.
    // The SysId tool provides a convenient method for obtaining these values for
    // your robot.
    public static final double ksVolts = 1;
    public static final double kvVoltSecondsPerMeter = 0.8;
    public static final double kaVoltSecondsSquaredPerMeter = 0.15;

    public static final double kMaxSpeedMetersPerSecond = 3;
  }

  public static final class ModuleConstants {
    public static final double kMaxModuleAngularSpeedRadiansPerSecond = 2 * Math.PI;
    public static final double kMaxModuleAngularAccelerationRadiansPerSecondSquared = 2 * Math.PI;

    public static final int kEncoderCPR = 1024;
    public static final double kWheelDiameterMeters = 0.15;
    public static final double kDriveEncoderDistancePerPulse =
        // Assumes the encoders are directly mounted on the wheel shafts
        (kWheelDiameterMeters * Math.PI) / (double) kEncoderCPR;

    public static final double kTurningEncoderDistancePerPulse =
        // Assumes the encoders are on a 1:1 reduction with the module shaft.
        (2 * Math.PI) / (double) kEncoderCPR;

    public static final double kAngleEncodeAnglePerRev = 2 * Math.PI;

    public static final double kPModuleTurningController = 1;

    public static final double kPModuleDriveController = 1;

    /** Idle modes. */
    public static final IdleMode DRIVE_IDLE_MODE = IdleMode.kBrake;
    public static final IdleMode ANGLE_IDLE_MODE = IdleMode.kCoast;

    /** Current limiting. */
    public static final int DRIVE_CURRENT_LIMIT = 35;
    public static final int ANGLE_CURRENT_LIMIT = 25;

    /**
     * Module specific constants.
     * CanCoder offset is in DEGREES, not radians like the rest of the repo.
     * This is to make offset slightly more accurate and easier to measure.
     */
    // Front Left Module
    public static final SwerveModuleConstants kMOD_1_Constants = new SwerveModuleConstants(
        1,
        6,
        7,
        false,
        false,
        254.5 // 360 * 0.7069
    );

    // Front Right
    public static final SwerveModuleConstants kMOD_2_Constants = new SwerveModuleConstants(
        2,
        4,
        5,
        true,
        true,
        152.0 // 360*0.4221
    );

    // Back Left
    public static final SwerveModuleConstants kMOD_3_Constants = new SwerveModuleConstants(
        3,
        2,
        3,
        false,
        false,
        0.0);

    // Back Right
    public static final SwerveModuleConstants kMOD_4_Constants = new SwerveModuleConstants(
        4,
        1,
        8,
        true,
        true,
        0.0);
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }
}
