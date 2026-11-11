// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.subsystems.*;;

public class RobotContainer {
  // Subsystems do robo
  private final ElevatorSubsystem m_elevator = new ElevatorSubsystem();

  // Controle Xbox do piloto
  private final CommandXboxController m_driverController = new CommandXboxController(0);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {

  
    
m_driverController.a().onTrue(m_elevator.goToHeightCommand(35.0));
m_driverController.b().onTrue(m_elevator.goToHeightCommand(10.0));
m_driverController.x().onTrue(m_elevator.goToTestTargetCommand());


   // m_driverController.y().onTrue(m_elevator.zeroEncoder());



    // --- Bumpers (manual) ---
    m_driverController.rightBumper().whileTrue(m_elevator.upCommand());
    m_driverController.leftBumper().whileTrue(m_elevator.downCommand());
  }
}