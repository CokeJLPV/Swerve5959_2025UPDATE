// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team5959;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;
import com.team5959.Constants.ControllerConstants;
import com.team5959.subsystems.SwerveChassis;
import com.team5959.commands.SwerveDrive;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.team5959.subsystems.intakeCoralSubsystem;
import com.team5959.subsystems.armIntakeAlgaeSubsystem;
import com.team5959.subsystems.elevatorSubsystem;
import com.team5959.subsystems.miniArmSubsystem;

import com.team5959.commands.runInCoralIntake;
import com.team5959.commands.runOutCoralIntake;
import com.team5959.commands.stopCoralIntake;


public class RobotContainer {
  //SUBSYSTEMS 
  private final SwerveChassis swerveChassis; 
  private final intakeCoralSubsystem intakeCoralSubsystem = new intakeCoralSubsystem();
  private final armIntakeAlgaeSubsystem armIntakeAlgaeSubsystem = new armIntakeAlgaeSubsystem();
  private final elevatorSubsystem elevatorSubsystem = new elevatorSubsystem();
  private final miniArmSubsystem miniArmSubsystem = new miniArmSubsystem();

  //COMMANDS
  private final runInCoralIntake runInCoralIntake = new runInCoralIntake(intakeCoralSubsystem);
  private final runOutCoralIntake runOutCoralIntake = new runOutCoralIntake(intakeCoralSubsystem);
  private final stopCoralIntake stopCoralIntake = new stopCoralIntake(intakeCoralSubsystem);

  //COMMANDS without files
  //Algae Arm Intake Commands
  private Command getRunInAlgaeCommand() {
    return armIntakeAlgaeSubsystem.startEnd(() -> armIntakeAlgaeSubsystem.setAlgaeIntakeSpeed(0.7), armIntakeAlgaeSubsystem::stopAlgaeIntake);
  }

  private Command getRunOutAlgaeCommand() {
    return armIntakeAlgaeSubsystem.startEnd(() -> armIntakeAlgaeSubsystem.setAlgaeIntakeSpeed(-0.5), armIntakeAlgaeSubsystem::stopAlgaeIntake);
  }

  //Algae Arm Position Commands
  private Command getArmPIDMovement() {
    return armIntakeAlgaeSubsystem.run(armIntakeAlgaeSubsystem::runPIDArmTarget).until(armIntakeAlgaeSubsystem::isAtTargetPosition);
  }

  private Command getInOrOutPositionCommand() {
    return Commands.sequence(armIntakeAlgaeSubsystem.runOnce(armIntakeAlgaeSubsystem::inorOutPositionSwitch), getArmPIDMovement());
  }

  private Command getInPerimeterPositionCommand() {
    return Commands.sequence(armIntakeAlgaeSubsystem.runOnce(armIntakeAlgaeSubsystem::moveToInPerimeterPosition), getArmPIDMovement());
  }

  private Command getHoldAlgaeArmPositionCommand() {
    return Commands.sequence(armIntakeAlgaeSubsystem.runOnce(armIntakeAlgaeSubsystem::currentToTargetPosition), armIntakeAlgaeSubsystem.run(armIntakeAlgaeSubsystem::runPIDArmTarget));
  }

  //Elevator Commands
  private Command getElevatorPIDMovement() {
    return elevatorSubsystem.run(elevatorSubsystem::runPIDElevatorTarget).until(elevatorSubsystem::atTargetPosition);
  }

  private Command getHoldElevatorPositionCommand() {
    return Commands.sequence(elevatorSubsystem.runOnce(elevatorSubsystem::CurrentToTargetPosition), elevatorSubsystem.run(elevatorSubsystem::runPIDElevatorTarget));
  }

  private Command getElevatorMoveCommand(Runnable positionSetter) {
    return Commands.sequence(elevatorSubsystem.runOnce(positionSetter), getElevatorPIDMovement());
  }

  //Mini Arm Commands
  private Command getMiniArmPIDMovement() {
    return miniArmSubsystem.run(miniArmSubsystem::runPIDMiniArmTarget).until(miniArmSubsystem::atTargetPosition);
  }

  private Command getHoldMiniArmPositionCommand() {
    return Commands.sequence(miniArmSubsystem.runOnce(miniArmSubsystem::currentToTargetPosition), miniArmSubsystem.run(miniArmSubsystem::runPIDMiniArmTarget));
  }

  private Command scoreReffCommandSequence(){
    return Commands.sequence(getInOrOutPositionCommand(),getRunOutAlgaeCommand(),getInOrOutPositionCommand());
  }
 
  //CONTROLLERS  
  private final PS4Controller control = new PS4Controller(ControllerConstants.kDriverControllerPort);
  private final CommandPS4Controller CommandPS4Controller = new CommandPS4Controller(ControllerConstants.kDriverControllerPort);
  private final CommandGenericHID CommandGenericController = new CommandGenericHID(ControllerConstants.kOperatorControllerPort);

  //DRIVE BUTTONS 
  private final JoystickButton resetNavxButton = new JoystickButton(control, 10); 

  private final JoystickButton resetPosButton = new JoystickButton(control, 9);

  //AXIS 
  @SuppressWarnings("unused")
  private final int joystickAxis = PS4Controller.Axis.kRightY.value;

  //Pathplanner
  public static SendableChooser<Command> autoCommandChooser; //Create sendable chooser for Autos
  List<PathPlannerPath> pathGroup; //Call a group of pathplanner paths
  public String autoChoose;

  public RobotContainer() {

    //REGISTER NAME AUTONOMOUS COMMANDS
    NamedCommands.registerCommand("outCoral", runOutCoralIntake.withTimeout(2));
    NamedCommands.registerCommand("algaeDown", getInOrOutPositionCommand());
    NamedCommands.registerCommand("getAlgae", getRunInAlgaeCommand().withTimeout(2));
    NamedCommands.registerCommand("algaeUp", getInOrOutPositionCommand());
    NamedCommands.registerCommand("score",getInOrOutPositionCommand());
    

    swerveChassis = new SwerveChassis();

    //swerveSubs.setDefaultCommand(new S_DriveCommand(swerveSubs, () -> -.getLeftY(), () -> -xbox.getLeftX(), () -> -xbox.getRightX(), true));
    swerveChassis.setDefaultCommand(new SwerveDrive(swerveChassis, () -> control.getLeftY(), () -> control.getLeftX(), () -> control.getRightX(), true));
    intakeCoralSubsystem.setDefaultCommand(stopCoralIntake);
    armIntakeAlgaeSubsystem.setDefaultCommand(getHoldAlgaeArmPositionCommand());
    elevatorSubsystem.setDefaultCommand(getHoldElevatorPositionCommand());
    miniArmSubsystem.setDefaultCommand(getHoldMiniArmPositionCommand());
   
    // shooter.setDefaultCommand(new Sh_JoystickControlCommand(shooter, () -> xbox.getRawAxis(joystickAxis) * 0.9));

    autoCommandChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Command Selector", autoCommandChooser);

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

    CommandPS4Controller.R2().whileTrue(getRunInAlgaeCommand());
    CommandPS4Controller.L2().whileTrue(getRunOutAlgaeCommand());

    CommandPS4Controller.triangle().onTrue(Commands.sequence(miniArmSubsystem.runOnce(miniArmSubsystem::downOrStartingPositionSwitch), getMiniArmPIDMovement())); //Triangle
    CommandPS4Controller.circle().onTrue(Commands.sequence(miniArmSubsystem.runOnce(miniArmSubsystem::moveToDropAlgaePosition), getMiniArmPIDMovement())); //Circle

    CommandPS4Controller.square().onTrue(getInOrOutPositionCommand()); //Square
    CommandPS4Controller.cross().onTrue(getInPerimeterPositionCommand()); //Cross

    CommandGenericController.button(5).whileTrue(elevatorSubsystem.run(elevatorSubsystem::elevatorUpManualMode)); //LB
    CommandGenericController.button(6).whileTrue(elevatorSubsystem.run(elevatorSubsystem::elevatorDownManualMode)); //RB

    CommandGenericController.button(8).whileTrue(runOutCoralIntake);//LT
    CommandGenericController.button(7).whileTrue(runInCoralIntake);//RT

    Trigger coralButtonsPressed = CommandGenericController.button(8).or(CommandGenericController.button(7));
    coralButtonsPressed.negate().onTrue(stopCoralIntake);

    CommandGenericController.button(3).onTrue(getElevatorMoveCommand(elevatorSubsystem::moveToL1Position)); //X
    CommandGenericController.button(4).onTrue(getElevatorMoveCommand(elevatorSubsystem::moveToL2Position)); //Y
    CommandGenericController.button(2).onTrue(getElevatorMoveCommand(elevatorSubsystem::moveToL3Position)); //B
    CommandGenericController.button(1).onTrue(getElevatorMoveCommand(elevatorSubsystem::moveToStartingPosition)); //A
  }
  
  public void periodic(){
    
  }
  
  public Command getAutonomousCommand() {
    
    try{
      PathPlannerPath choreoTraj = PathPlannerPath.fromChoreoTrajectory(autoChoose);
      return AutoBuilder.followPath(choreoTraj);
    } catch (Exception e){
      DriverStation.reportError("Error loading path: " + e.getMessage(), e.getStackTrace());
      return Commands.none();
    }
    return autoCommandChooser.getSelected();
  }
}
