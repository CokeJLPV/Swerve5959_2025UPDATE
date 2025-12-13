// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team5959;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.team5959.Constants.ControllerConstants;
import com.team5959.subsystems.SwerveChassis;
import com.team5959.commands.SwerveDrive;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.team5959.subsystems.intakeCoralSubsystem;

import com.team5959.commands.runInCoralIntake;
import com.team5959.commands.runOutCoralIntake;
import com.team5959.commands.stopCoralIntake;


public class RobotContainer {
  //SUBSYSTEMS 
  private final SwerveChassis swerveChassis = new SwerveChassis(); 
  private final intakeCoralSubsystem intakeCoralSubsystem = new intakeCoralSubsystem();

  //COMMANDS
  private final runInCoralIntake runInCoralIntake = new runInCoralIntake(intakeCoralSubsystem);
  private final runOutCoralIntake runOutCoralIntake = new runOutCoralIntake(intakeCoralSubsystem);
  private final stopCoralIntake stopCoralIntake = new stopCoralIntake(intakeCoralSubsystem);

  //CONTROLLERS  
  private final PS4Controller control = new PS4Controller(ControllerConstants.kDriverControllerPort);
  private final CommandPS4Controller CommandPS4Controller = new CommandPS4Controller(ControllerConstants.kDriverControllerPort);

  //DRIVE BUTTONS 
  private final JoystickButton resetNavxButton = new JoystickButton(control, 10); 

  private final JoystickButton resetPosButton = new JoystickButton(control, 9);



  //AXIS 
  @SuppressWarnings("unused")
  private final int joystickAxis = PS4Controller.Axis.kRightY.value;

  //Pathplanner
  public final SendableChooser<String> autoChooser;
  private PathPlannerPath path;
  public String autoChoose;

  //AUTONOMOUS
  public static final String kForward = "Forward";
  public static final String kRight = "Right";
  public static final String kLeft = "Left";

  public RobotContainer() {

    //swerveSubs.setDefaultCommand(new S_DriveCommand(swerveSubs, () -> -.getLeftY(), () -> -xbox.getLeftX(), () -> -xbox.getRightX(), true));
    swerveChassis.setDefaultCommand(new SwerveDrive(swerveChassis, () -> control.getLeftY(), () -> control.getLeftX(), () -> control.getRightX(), true));
    intakeCoralSubsystem.setDefaultCommand(stopCoralIntake);
   
    // shooter.setDefaultCommand(new Sh_JoystickControlCommand(shooter, () -> xbox.getRawAxis(joystickAxis) * 0.9));

    //AUTONOMOUS
    autoChooser = new SendableChooser<>();
    autoChooser.setDefaultOption("Forward", kForward);
    autoChooser.addOption("Right", kRight);
    autoChooser.addOption("Left", kLeft);
    SmartDashboard.putData("Auto Selector", autoChooser);

    // Configure the trigger bindings
    configureBindings();
  }

  private void configureBindings() {
    resetNavxButton.onTrue(new InstantCommand(() -> {
        swerveChassis.resetNavx();
        swerveChassis.resetDriveEncoders();
    }));

    resetPosButton.onTrue(new InstantCommand(() -> swerveChassis.resetOdometry(new Pose2d(0, 0, new Rotation2d(0)))));
  //  limelightStrafeAlign.onTrue(new LimelightRotationAlignCommand(swerveSubs, () -> -xbox.getLeftY(), () -> -xbox.getLeftX(), () -> -xbox.getRightX()));

    CommandPS4Controller.R1().whileTrue(runInCoralIntake);
    CommandPS4Controller.L1().whileTrue(runOutCoralIntake);
    
  }
  
  public void periodic(){
    
  }
  
  public Command getAutonomousCommand() {

    autoChoose = autoChooser.getSelected();

    try{
      switch (autoChoose){
        case kForward:
          path = PathPlannerPath.fromPathFile(autoChoose);
          break;
        case kRight:
          path = PathPlannerPath.fromPathFile(autoChoose);
          break;
        case kLeft:
          path = PathPlannerPath.fromPathFile(autoChoose);
          break;
        default:
          break;
      }
      return AutoBuilder.followPath(path);
    } catch (Exception e){
      DriverStation.reportError("Error loading path: " + e.getMessage(), e.getStackTrace());
      return Commands.none();
    }
  }
}
