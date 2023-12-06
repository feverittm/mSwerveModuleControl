// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.SwerveModule;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
	// The robot's subsystems
	public SwerveModule module = new SwerveModule(ModuleConstants.kMOD_3_Constants);

	// The driver's controller
	CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);

	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {
		// Configure the button bindings
		configureButtonBindings();
	}

	/**
	 * Use this method to define bindings between conditions and commands. These
	 * are useful for
	 * automating robot behaviors based on button and sensor input.
	 *
	 * <p>
	 * Should be called during {@link Robot#robotInit()}.
	 *
	 * <p>
	 * Event binding methods are available on the {@link Trigger} class.
	 */

	private void configureButtonBindings() {
		m_driverController.a().onTrue(module.ZeroModule());
		// m_driverController.b().onTrue(m_robotDrive.ResetEncoders());
		// m_driverController.x().onTrue(moveTurnCommand(Math.PI/2.0));
	}
}