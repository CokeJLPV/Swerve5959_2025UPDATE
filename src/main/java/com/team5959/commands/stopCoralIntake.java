// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team5959.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import com.team5959.subsystems.intakeCoralSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class stopCoralIntake extends InstantCommand {

  intakeCoralSubsystem intakeCoralSubsystem;

  public stopCoralIntake(intakeCoralSubsystem intakeCoralSubsystem) {
    this.intakeCoralSubsystem = intakeCoralSubsystem;
    addRequirements(intakeCoralSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intakeCoralSubsystem.stopCoralIntake();
  }
}
