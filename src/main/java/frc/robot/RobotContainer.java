// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
// Base Imports
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.cameraserver.CameraServer;
// Math And Mapping Imports
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
// Smartdashboard Imports
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// Contoller Imports
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
// Constants Imports
import frc.robot.Constants.OperatorConstants;
// Swerve Imports
import frc.robot.commands.swervedrive.drivebase.AbsoluteDriveAdv;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
// Climber Imports
import frc.robot.subsystems.climber.Climber;
import frc.robot.commands.Climber.ClimberSpin;
// Intake Imports
import frc.robot.subsystems.intake.Intake;
import frc.robot.commands.Intake.IntakeSpin;
// Shooter Imports
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.commands.Shooter.ShooterSpin;
//import frc.robot.commands.Auto.AutoShoot;
//import frc.robot.commands.Auto.AutoShootM;
import frc.robot.commands.Auto.Wait;
// Limelight Imports
import frc.robot.subsystems.limelight.Limelight;
// Files
import edu.wpi.first.wpilibj.Filesystem;
import java.io.File;
// Auto
//import com.pathplanner.lib.auto.AutoBuilder;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{

  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve/neo"));
  private final Shooter m_shooter = new Shooter(); 
  private final Intake m_intake = new Intake(); 
  private final Climber m_climber = new Climber();
  private final Limelight m_limelight = new Limelight();

  // CommandJoystick rotationController = new CommandJoystick(1);
  // Replace with CommandPS4Controller or CommandJoystick if needed
  CommandJoystick driverController = new CommandJoystick(1);

  // CommandJoystick driverController   = new CommandJoystick(3);//(OperatorConstants.DRIVER_CONTROLLER_PORT);
  XboxController driverXbox = new XboxController(0);
  XboxController stuffController = new XboxController(1);
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  
  // Sendable Chooser Auto Setup
  SendableChooser<String> autoChooser = new SendableChooser<>();
  final String shooterAuto = "Shooter";

  public RobotContainer()
  {
    // Initialize Camera
    CameraServer.startAutomaticCapture();
    // Configure the trigger bindings
    configureBindings();

    m_shooter.setDefaultCommand(new ShooterSpin(m_shooter,
      //stuffController::getRightBumperPressed,
      stuffController::getRightTriggerAxis,
      stuffController::getBButtonPressed
      ));

    m_intake.setDefaultCommand(new IntakeSpin(m_intake,
      //stuffController::getLeftTriggerAxis,
      stuffController::getLeftBumper,
      stuffController::getRightBumper,
      stuffController::getAButtonPressed
      ));  

    m_climber.setDefaultCommand(new ClimberSpin(m_climber,
      stuffController::getLeftY, 
      stuffController::getRightY
    )); 

    AbsoluteDriveAdv closedAbsoluteDriveAdv = new AbsoluteDriveAdv(drivebase,
      () -> MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
      () -> MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
      () -> MathUtil.applyDeadband(driverXbox.getRightX(), OperatorConstants.RIGHT_X_DEADBAND),
      driverXbox::getYButtonPressed,
      driverXbox::getAButtonPressed,
      driverXbox::getXButtonPressed,
      driverXbox::getBButtonPressed);

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the desired angle NOT angular rotation
    Command driveFieldOrientedDirectAngle = drivebase.driveCommand(
        () -> MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> driverXbox.getRightX(),
        () -> driverXbox.getRightY());

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the angular velocity of the robot
    Command driveFieldOrientedAnglularVelocity = drivebase.driveCommand(
        () -> MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> driverXbox.getRawAxis(2));

    Command driveFieldOrientedDirectAngleSim = drivebase.simDriveCommand(
        () -> MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> driverXbox.getRawAxis(2));

    drivebase.setDefaultCommand(
        !RobotBase.isSimulation() ? driveFieldOrientedDirectAngle : driveFieldOrientedDirectAngleSim);

    // Build an auto chooser. This will use Commands.none() as the default option.
    //autoChooser = AutoBuilder.buildAutoChooser();
    // Another option that allows you to specify the default auto by its name
    // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");

    // Auto commands
    
    autoChooser.setDefaultOption("Shoot", shooterAuto);
    /* 
    drivebase.setupPathPlanner();

    autoChooser.addOption(
      "LD4A", 
      drivebase.getAutonomousCommand("Left Amp 3 Note", true)
    );
    autoChooser.addOption(
      "LD3", 
      drivebase.getAutonomousCommand("Left Diagonal 3 Note", true)
    );
    autoChooser.addOption(
      "LD4", 
      drivebase.getAutonomousCommand("Left Diagonal 4 Note", true)
    );
    autoChooser.addOption(
      "RD3", 
      drivebase.getAutonomousCommand("Right Diagonal 3 Note", true)
    );
    autoChooser.addOption(
      "RD4", 
      drivebase.getAutonomousCommand("Right Diagonal 4 Note", true)
    );
    autoChooser.addOption(
      "M3", 
      drivebase.getAutonomousCommand("Middle 3 Note", true)
    );
    autoChooser.addOption(
      "Troll", 
      drivebase.getAutonomousCommand("Troll", true)
    );
    autoChooser.setDefaultOption(
      "Shoot", 
      AutoShoot
    );
    */
    /*autoChooser.setDefaultOption(
      null, 
      new AutoShoot(m_shooter, m_intake)
    );*/

    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings()
  {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    //new JoystickButton(stuffController, OperatorConstants.XBOX_R_BUMPER).whileTrue(new AutoShootM(m_shooter, 0.1));
    new JoystickButton(driverXbox, OperatorConstants.XBOX_A_BUTTON).onTrue((new InstantCommand(drivebase::zeroGyro)));
    new JoystickButton(driverXbox, OperatorConstants.XBOX_X_BUTTON).onTrue(new InstantCommand(drivebase::addFakeVisionReading));
    new JoystickButton(driverXbox, OperatorConstants.XBOX_B_BUTTON).whileTrue(
      Commands.deferredProxy(
        () -> drivebase.driveToPose(
          new Pose2d(
            new Translation2d(4, 4),
            Rotation2d.fromDegrees(0)
          )
        )
      )
    );
//    new JoystickButton(driverXbox, OperatorConstants.XBOX_Y_BUTTON).whileTrue(new RepeatCommand(new InstantCommand(drivebase::lock, drivebase)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
    // return autoChooser.getSelected();
    
    SmartDashboard.putBoolean("Auto Running", true);
    return new SequentialCommandGroup(
      new InstantCommand(() -> {
      m_shooter.setShooterSpeed(1);
      }),
      //new WaitCommand(2),
      new Wait(m_intake, 1),
      new InstantCommand(() -> {
        m_intake.setIntakeSpeed(0.4);
      }),
      new Wait(m_shooter, 2),
      new Wait(m_intake, 2),
      //new WaitCommand(3)
      new InstantCommand(() -> {
      m_shooter.setShooterSpeed(0);
      }),
      new InstantCommand(() -> {
        m_intake.setIntakeSpeed(0);
      }),
      new InstantCommand(() -> {
        drivebase.drive(new Translation2d(-3.0, 0.0), 0, true);
      }));

      /*
       * return new SequentialCommandGroup(
      new InstantCommand(() -> {
      m_shooter.setShooterSpeed(1);
      }),
      //new WaitCommand(2),
      new Wait(m_intake, 1),
      new InstantCommand(() -> {
        m_intake.setIntakeSpeed(0.4);
      }),
      new Wait(m_shooter, 2),
      new Wait(m_intake, 2),
      //new WaitCommand(3)
      new InstantCommand(() -> {
      m_shooter.setShooterSpeed(0);
      }),
      new InstantCommand(() -> {
        m_intake.setIntakeSpeed(0);
      }),
      new InstantCommand(() -> {
        drivebase.drive(new Translation2d(-3.0, 0.0), 0, true);
      })),
      new InstantCommand(() -> {
        m_intake.setIntakeSpeed(0.4);
      }),
      new Wait(m_intake, 1),
      new InstantCommand(() -> {
        drivebase.drive(new Translation2d(3.0, 0.0), 0, true);
      })),
      new InstantCommand(() -> {
        m_intake.setIntakeSpeed(0);
      }),
      new InstantCommand(() -> {
      m_shooter.setShooterSpeed(1);
      }),
      //new WaitCommand(2),
      new Wait(m_intake, 1),
      new InstantCommand(() -> {
        m_intake.setIntakeSpeed(0.4);
      }),
      new Wait(m_shooter, 2),
      new Wait(m_intake, 2),
      //new WaitCommand(3)
      new InstantCommand(() -> {
      m_shooter.setShooterSpeed(0);
      }),
      new InstantCommand(() -> {
        m_intake.setIntakeSpeed(0);
      });
       */
  }

  public void setDriveMode()
  {
    //drivebase.setDefaultCommand();
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
}
