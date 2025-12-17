package com.team5959.subsystems;

import com.studica.frc.AHRS;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.team5959.Constants.SwerveConstants;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//PathPlanner
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.trajectory.Trajectory;

public class SwerveChassis extends SubsystemBase {

  /* * * INITIALIZATION * * */

  // initialize SwerveModules
  private SwerveModule[] swerveModules;

  // odometer
  private SwerveDriveOdometry odometer;
  private AHRS navx;

  // Pathplanner
  RobotConfig config;
  private SwerveDrivePoseEstimator m_PoseEstimator;

  Field2d field = new Field2d();

  public SwerveChassis() {

    swerveModules = new SwerveModule[] {
        new SwerveModule(0, SwerveConstants.FrontLeft.constants),
        new SwerveModule(1, SwerveConstants.BackLeft.constants),
        new SwerveModule(2, SwerveConstants.FrontRight.constants),
        new SwerveModule(3, SwerveConstants.BackRight.constants)
    };

    // instantiate navx
    navx = new AHRS(AHRS.NavXComType.kMXP_SPI);
    navx.setAngleAdjustment(0); // FIXME //adjustment may be needed depending on robot orientation

    // instantiate odometer
    odometer = new SwerveDriveOdometry(
        SwerveConstants.DRIVE_KINEMATICS,
        getRotation2d(),
        getModulePositions());

    m_PoseEstimator = new SwerveDrivePoseEstimator(
        SwerveConstants.DRIVE_KINEMATICS, // SwerveDriveKinematics object
        getRotation2d(), // Initial gyro angle (Rotation2d)
        getModulePositions(), // Initial module positions (SwerveModulePosition array)
        new Pose2d(0, 0, getRotation2d()), // Initial pose (X, Y, Theta)
        VecBuilder.fill(0.1, 0.1, 0.05), // SDs of the Modela's state (Odometry)
        VecBuilder.fill(0.8, 0.8, 0.8) // Vision measurement SDs (AprilTag)
    );

    try {
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      DriverStation.reportError("Error loading path: " + e.getMessage(), e.getStackTrace());
    }

    AutoBuilder.configure(
        this::getPoseEstimator,
        this::resetPose,
        this::getRobotRelativeSpeeds,
        (speeds, feedforwards) -> driveRobotRelative(speeds),
        new PPHolonomicDriveController(
            new PIDConstants(2.9, 0, 0), // Translation PID X/Y kp:0.5 ki:0.000001 kd:0.001
            new PIDConstants(1.1, 0, 0)), // Rotation PID Theta ki:0.0001 kd:0.1
        config,
        this::getAlliance,
        this);
/* 
    PathPlannerLogging.setLogActivePathCallback((poses) -> {field.getObject("path").setPoses(poses);});

    PathPlannerLogging.setLogCurrentPoseCallback((pose) -> {field.setRobotPose(pose);});

    PathPlannerLogging.setLogTargetPoseCallback((pose) -> {field.getObject("target").setPose(pose);});
*/
    SmartDashboard.putData("fieldTestPosition", field);

  }

  private boolean getAlliance() {
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      return alliance.get() == DriverStation.Alliance.Red;
    } else {
      return false;
    }
  }

  public Pose2d getPoseEstimator() {
    return m_PoseEstimator.getEstimatedPosition();
  }

  public void resetPose(Pose2d pose) {
    m_PoseEstimator.resetPosition(
        getRotation2d(),
        getModulePositions(),
        pose);
  }

  /* * * ODOMETRY * * */

  // returns the Rotation2d object
  // a 2d coordinate represented by a point on the unit circle (the rotation of
  // the robot)
  public Rotation2d getRotation2d() {
    return navx.getRotation2d();
  }

  public void resetNavx() {
    navx.reset();
  }

  public Pose2d getPose() {
    return odometer.getPoseMeters();
  }

  // i dont think this works as intended,, resetPosition should reset everything
  // to 0
  public void setPose(Pose2d pose) {
    odometer.resetPosition(getRotation2d(), getModulePositions(), pose);
  }

  public void resetOdometry(Pose2d pose) {
    odometer.resetPosition(getRotation2d(), getModulePositions(), pose);
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return new ChassisSpeeds(SwerveConstants.DRIVE_KINEMATICS.toChassisSpeeds(getModuleStates()).vxMetersPerSecond,
        SwerveConstants.DRIVE_KINEMATICS.toChassisSpeeds(getModuleStates()).vyMetersPerSecond,
        SwerveConstants.DRIVE_KINEMATICS.toChassisSpeeds(getModuleStates()).omegaRadiansPerSecond);
  }

  public void driveRobotRelative(ChassisSpeeds chassis) {
    SwerveModuleState[] state = SwerveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(chassis);

    setModuleStates(state);
  }

  /* * * STATES * * */

  // SET STATES
  // gets a SwerveModuleStates array from driver control and sets each module to
  // the corresponding SwerveModuleState
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveConstants.MAX_SPEED);

    for (SwerveModule swerveMod : swerveModules) {
      swerveMod.setState(desiredStates[swerveMod.moduleID]);
    }
  }

  // GET STATES
  // returns the states of the swerve modules in an array
  // getState uses drive velocity and module rotation
  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];

    for (SwerveModule swerveMod : swerveModules) {
      states[swerveMod.moduleID] = swerveMod.getState();
    }

    return states;
  }

  // GET POSITIONS
  // returns the positions of the swerve modules in an array
  // getPosition uses drive enc and module rotation
  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];

    for (SwerveModule swerveMod : swerveModules) {
      positions[swerveMod.moduleID] = swerveMod.getPosition();
    }

    return positions;
  }

  // LOCK
  public void lock() {
    SwerveModuleState[] states = new SwerveModuleState[4];

    states[0] = new SwerveModuleState(0, new Rotation2d(Math.toRadians(45)));
    states[1] = new SwerveModuleState(0, new Rotation2d(Math.toRadians(-45)));
    states[2] = new SwerveModuleState(0, new Rotation2d(Math.toRadians(45)));
    states[3] = new SwerveModuleState(0, new Rotation2d(Math.toRadians(-45)));

    for (SwerveModule swerveMod : swerveModules) {
      swerveMod.setAngle(states[swerveMod.moduleID]);
    }
  }

  // STRAIGHTEN THE WHEELS
  public void straightenWheels() { // set all wheels to 0 degrees
    SwerveModuleState[] states = new SwerveModuleState[4];

    states[0] = new SwerveModuleState(0, new Rotation2d(Math.toRadians(0)));
    states[1] = new SwerveModuleState(0, new Rotation2d(Math.toRadians(0)));
    states[2] = new SwerveModuleState(0, new Rotation2d(Math.toRadians(0)));
    states[3] = new SwerveModuleState(0, new Rotation2d(Math.toRadians(0)));

    for (SwerveModule swerveMod : swerveModules) {
      swerveMod.setState(states[swerveMod.moduleID]);
    }
  }

  // DRIVE
  public void drive(double xSpeed, double ySpeed, double zSpeed, boolean fieldOriented) {
    SwerveModuleState[] states;
    if (fieldOriented) {
      states = SwerveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(
          ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, zSpeed, getRotation2d())

      );
    } else {
      states = SwerveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(
          new ChassisSpeeds(xSpeed, ySpeed, zSpeed));
    }

    setModuleStates(states);

  }

  // STOP
  public void stopModules() {
    for (SwerveModule swerveMod : swerveModules) {
      swerveMod.stop();
    }
  }

  public void resetDriveEncoders() {
    for (SwerveModule swerveMod : swerveModules) {
      swerveMod.resetDriveEncoder();
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    odometer.update(getRotation2d(), getModulePositions());

    m_PoseEstimator.update(
        getRotation2d(),
        getModulePositions());

    field.setRobotPose(getPoseEstimator());

    for (SwerveModule swerveMod : swerveModules) {
      swerveMod.print();
    }

    SmartDashboard.putNumber("NAVX", -navx.getAngle());
    SmartDashboard.putNumber("NAVXYAW", navx.getYaw());
    SmartDashboard.putData("NAVX2D", navx);
    SmartDashboard.putString("POSE INFO", odometer.getPoseMeters().toString());
    SmartDashboard.putNumber("rot 2d", ((getRotation2d().getDegrees() % 360) + 360) % 360);

    SmartDashboard.putNumber("Distancia FL", swerveModules[0].getPosition().distanceMeters);
    SmartDashboard.putNumber("Distancia RL", swerveModules[1].getPosition().distanceMeters);
    SmartDashboard.putNumber("Distancia FR", swerveModules[2].getPosition().distanceMeters);
    SmartDashboard.putNumber("Distancia RR", swerveModules[3].getPosition().distanceMeters);
  }

  /* * * ADDED METHODS * * */
  public double deadzone(double num) {
    return Math.abs(num) > 0.1 ? num : 0;
  }

  @SuppressWarnings("unused")
  private static double modifyAxis(double num) {
    // Square the axis
    num = Math.copySign(num * num, num);

    return num;
  }

}
