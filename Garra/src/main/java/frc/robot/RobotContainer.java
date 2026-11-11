// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Subsystems.DriveTrain;
import frc.robot.Subsystems.GarraIntakeSubsystem;

public class RobotContainer {

  private final DriveTrain m_DriveTrain = new DriveTrain();
  private final GarraIntakeSubsystem intake = new GarraIntakeSubsystem();

  public final CommandXboxController xboxController = new CommandXboxController(0);
 
  public RobotContainer() {
    configureBindings();


    new RunCommand(() -> m_DriveTrain.periodic(xboxController.getHID()))
        .ignoringDisable(false)
        .schedule();
  }

  private void configureBindings() {

    xboxController.y().onTrue(intake.setRPM(800));
    xboxController.b().onTrue(intake.setRPM(600));
    xboxController.x().onTrue(intake.setRPM(400));
    
    xboxController.a().onTrue(intake.setRPM(-1500)); // suga

    xboxController.leftBumper().onTrue(intake.setRPM(0));
}
}
