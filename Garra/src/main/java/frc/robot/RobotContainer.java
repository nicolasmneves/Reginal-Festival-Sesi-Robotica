// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Subsystems.GarraIntakeSubsystem;

public class RobotContainer {

  private GarraIntakeSubsystem intake = new GarraIntakeSubsystem();
  public CommandXboxController xboxController = new CommandXboxController(0);
 
  public RobotContainer() {

    configureBindings();
  }

  private void configureBindings() {

    xboxController.x().onTrue(intake.setDutyCycle(0.8));
    xboxController.y().onTrue(intake.setDutyCycle(-0.8));
    xboxController.a().onTrue(intake.in());
    xboxController.b().onTrue(intake.out());

  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
