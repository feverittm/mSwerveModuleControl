// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.utils.NavXSwerve;
import edu.wpi.first.wpilibj.SerialPort;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  // The gyro sensor
  public static NavXSwerve m_gyro;
  double joy_angle = 0.0;

  private RobotContainer m_robotContainer;

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    m_gyro = new NavXSwerve(SerialPort.Port.kMXP);
  }

  @Override
  public void autonomousPeriodic() {
    m_robotContainer.module.setDesiredState(new SwerveModuleState(0.0, new Rotation2d(0.0)));
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    // swerve module state
    SwerveModuleState state = m_robotContainer.module.getState();
    SmartDashboard.putNumber("Module State - Velocity: ", state.speedMetersPerSecond);
    SmartDashboard.putNumber("Module State - Angle: ", state.angle.getDegrees());
    SmartDashboard.putNumber("Module Position - Distance: ", m_robotContainer.module.getPosition().distanceMeters);
    SmartDashboard.putNumber("Module Position - Angle: ", m_robotContainer.module.getPosition().angle.getDegrees());
    // raw hardware
    SmartDashboard.putNumber("Raw Turning Motor Angle", m_robotContainer.module.getRawAngle());
    SmartDashboard.putNumber("Module Angle", m_robotContainer.module.getAngle().getDegrees());
    SmartDashboard.putBoolean("Module Zeroed", (m_robotContainer.module.getAngle().getRadians() == 0.0));
    //
    SmartDashboard.putNumber("Gyro YAW", m_gyro.getYaw());
    SmartDashboard.putNumber("Gyro Rotation", m_gyro.getRotation3d().getAngle());
  }
}
