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

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.auto.AutoBuilder;
import com.team5959.Constants.ControllerConstants;
import com.team5959.subsystems.SwerveChassis;
import com.team5959.commands.SwerveDrive;

import edu.wpi.first.wpilibj.DriverStation;
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
  private final SwerveChassis swerveChassis = new SwerveChassis(); 
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
  Command runInAlgaeCommand = armIntakeAlgaeSubsystem.startEnd(() -> armIntakeAlgaeSubsystem.setAlgaeIntakeSpeed(0.7), () -> armIntakeAlgaeSubsystem.stopAlgaeIntake());
  Command runOutAlgaeCommand = armIntakeAlgaeSubsystem.startEnd(() -> armIntakeAlgaeSubsystem.setAlgaeIntakeSpeed(-0.5), () -> armIntakeAlgaeSubsystem.stopAlgaeIntake());

  //Algae Arm Position Commands
  Command moveToAlgaeArmPostionPIDCommand = armIntakeAlgaeSubsystem.run(() -> armIntakeAlgaeSubsystem.runPIDArmTarget()).until(() -> armIntakeAlgaeSubsystem.isAtTargetPosition());
  Command holdAlgaeArmPositionPIDCommand = armIntakeAlgaeSubsystem.run(() -> armIntakeAlgaeSubsystem.runPIDArmTarget());

  Command inOrOutPositionCommand = armIntakeAlgaeSubsystem.runOnce(() -> armIntakeAlgaeSubsystem.inorOutPositionSwitch()).andThen(moveToAlgaeArmPostionPIDCommand);
  Command inPerimeterPositionCommand = armIntakeAlgaeSubsystem.runOnce(() -> armIntakeAlgaeSubsystem.moveToInPerimeterPosition()).andThen(moveToAlgaeArmPostionPIDCommand);
  Command holdAlgaeArmPositionCommand = armIntakeAlgaeSubsystem.runOnce(() -> armIntakeAlgaeSubsystem.currentToTargetPosition()).andThen(holdAlgaeArmPositionPIDCommand);

  //Elevator Commands
  Command moveToElevatorPositionPIDCommand = elevatorSubsystem.run(() -> elevatorSubsystem.runPIDElevatorTarget()).until(() -> elevatorSubsystem.atTargetPosition());
  Command holdElevatorPositionPIDCommand = elevatorSubsystem.run(() -> elevatorSubsystem.runPIDElevatorTarget());

  Command holdElevatorPositionCommand = elevatorSubsystem.runOnce(() -> elevatorSubsystem.CurrentToTargetPosition()).andThen(holdElevatorPositionPIDCommand);
  Command startingElevatorPositionCommand = elevatorSubsystem.runOnce(()-> elevatorSubsystem.moveToStartingPosition()).andThen(moveToElevatorPositionPIDCommand);
  Command l1ElevatorPositionCommand = elevatorSubsystem.runOnce(()-> elevatorSubsystem.moveToL1Position()).andThen(moveToElevatorPositionPIDCommand);
  Command l2ElevatorPositionCommand = elevatorSubsystem.runOnce(()-> elevatorSubsystem.moveToL2Position()).andThen(moveToElevatorPositionPIDCommand);
  Command l3ElevatorPositionCommand = elevatorSubsystem.runOnce(()-> elevatorSubsystem.moveToL3Position()).andThen(moveToElevatorPositionPIDCommand);
  Command upElevatorCommand = elevatorSubsystem.run(() -> elevatorSubsystem.elevatorUpManualMode());
  Command downElevatorCommand = elevatorSubsystem.run(() -> elevatorSubsystem.elevatorDownManualMode());

  //Mini Arm Commands
  Command moveToMiniArmPositionPIDCommand = miniArmSubsystem.run(() -> miniArmSubsystem.runPIDMiniArmTarget()).until(() -> miniArmSubsystem.atTargetPosition());
  Command holdMiniArmPositionPIDCommand = miniArmSubsystem.run(() -> miniArmSubsystem.runPIDMiniArmTarget());

  Command holdMiniArmPositionCommand = miniArmSubsystem.runOnce(() -> miniArmSubsystem.currentToTargetPosition()).andThen(holdMiniArmPositionPIDCommand);
  Command downOrStartingPositionSwitch = miniArmSubsystem.runOnce(() -> miniArmSubsystem.downOrStartingPositionSwitch()).andThen(moveToMiniArmPositionPIDCommand);
  Command miniArmDropPositionCommand = miniArmSubsystem.runOnce(() -> miniArmSubsystem.moveToDropAlgaePosition()).andThen(moveToMiniArmPositionPIDCommand);
 
  //A command is created without creating the file, since a small action is being performed.
  //Command inCoral = intakeCoralSubsystem.startEnd(() -> intakeCoralSubsystem.runInCoralIntake(), () -> intakeCoralSubsystem.stopCoralIntake());

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
  public final SendableChooser<String> autoChooser; //Create sendable chooser for paths
  public static SendableChooser<Command> autoCommandChooser; //Create sendable chooser for Autos
  private PathPlannerPath path; //Call a pathplanner path
  List<PathPlannerPath> pathGroup; //Call a group of pathplanner paths
  public String autoChoose;

  //AUTONOMOUS
  public static final String kForward = "Forward";
  public static final String kRight = "Right";
  public static final String kLeft = "Left";

  public RobotContainer() {

    //swerveSubs.setDefaultCommand(new S_DriveCommand(swerveSubs, () -> -.getLeftY(), () -> -xbox.getLeftX(), () -> -xbox.getRightX(), true));
    swerveChassis.setDefaultCommand(new SwerveDrive(swerveChassis, () -> control.getLeftY(), () -> control.getLeftX(), () -> control.getRightX(), true));
    intakeCoralSubsystem.setDefaultCommand(stopCoralIntake);
    armIntakeAlgaeSubsystem.setDefaultCommand(holdAlgaeArmPositionCommand);
    elevatorSubsystem.setDefaultCommand(holdElevatorPositionCommand);
    miniArmSubsystem.setDefaultCommand(holdMiniArmPositionCommand);
   
    // shooter.setDefaultCommand(new Sh_JoystickControlCommand(shooter, () -> xbox.getRawAxis(joystickAxis) * 0.9));

    //AUTONOMOUS CHOOSER
    autoChooser = new SendableChooser<>();
    autoChooser.setDefaultOption("Forward", kForward);
    autoChooser.addOption("Right", kRight);
    autoChooser.addOption("Left", kLeft);
    SmartDashboard.putData("Auto Selector", autoChooser);

    autoCommandChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Command Selector", autoCommandChooser);

    //REGISTER NAME AUTONOMOUS COMMANDS
    NamedCommands.registerCommand("outCoral", runOutCoralIntake);

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

    CommandPS4Controller.R2().whileTrue(runInAlgaeCommand);
    CommandPS4Controller.L2().whileTrue(runOutAlgaeCommand);

    CommandPS4Controller.triangle().onTrue(downOrStartingPositionSwitch); //Triangle
    CommandPS4Controller.circle().onTrue(miniArmDropPositionCommand); //Circle

    CommandPS4Controller.square().onTrue(inOrOutPositionCommand); //Square
    CommandPS4Controller.cross().onTrue(inPerimeterPositionCommand); //Cross

    CommandGenericController.button(5).whileTrue(upElevatorCommand); //LB
    CommandGenericController.button(6).whileTrue(downElevatorCommand); //RB

    CommandGenericController.button(8).whileTrue(runOutCoralIntake);//LT
    CommandGenericController.button(7).whileTrue(runInCoralIntake);//RT

    /*(boton 8 o boton 7) estan sueltos y es false negate lo vuelve true, entonces onTrue se activa y se para la intake
    R2	L2	R2 OR L2	      NOT (R2 OR L2)	  ¿Se ejecuta stopCoralIntake?
    P	  UP	TRUE	          FALSE	            NO (El motor está corriendo)
    UP	P	  TRUE	          FALSE            	NO (El motor está corriendo)
    P	  P	  TRUE	          FALSE	            NO (El motor está corriendo)
    UP	UP	FALSE	          TRUE	            SÍ (Parada)
    */
    //No estoy seguro si esto funciona bien, probarlo
    //CommandGenericController.button(8).or(CommandGenericController.button(7)).negate().onTrue(stopCoralIntake);

    CommandGenericController.button(3).onTrue(l1ElevatorPositionCommand); //X
    CommandGenericController.button(4).onTrue(l2ElevatorPositionCommand); //Y
    CommandGenericController.button(2).onTrue(l3ElevatorPositionCommand); //B
    CommandGenericController.button(1).onTrue(startingElevatorPositionCommand); //A

    //idea fugas, cuando se selecione el autonomo, llamar un comando que reinicie la posicion en la 
    //que esta el robot, new InstantCommand(()->swerveChassis.resetOdometry(new Pose2d(X de patplanner, Y de pathplanner,navx.getRotation2d() ))


    
  }
  
  public void periodic(){
    
  }
  
  public Command getAutonomousCommand() {

    //Descomentar esta linea si se quiere probar un auto y no un path
    //return autoCommandChooser.getSelected();

    //Comentar estas lineas si se quiere probar un auto y no un path
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
