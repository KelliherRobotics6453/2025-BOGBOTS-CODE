// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.commands.ElevatorCommand;
import frc.robot.commands.ElevatorPIDCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public final PivotSubsystem m_PivotSubsystem = new PivotSubsystem();

    public final ElevatorSubsystem m_ElevatorSubsystem = new ElevatorSubsystem();
    

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

    /*    joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric())); */

        // bindings added by Kendrick and Don
        // final JoystickButton ElevatorStartNew = new JoystickButton(m_XboxController, XboxController.Button.kStart.value);
        //ElevatorStartNew.onTrue(new ElevatorPIDCommand(m_ElevatorSubsystem, 0));
        joystick.start().onTrue(new ElevatorPIDCommand(m_ElevatorSubsystem, 0));

        //final JoystickButton ElevatorL1 = new JoystickButton(m_XboxController, XboxController.Button.kA.value);
        // ElevatorL1.onTrue(new ElevatorPIDCommand(m_ElevatorSubsystem, ElevatorConstants.kL1));
        joystick.a().onTrue(new ElevatorPIDCommand(m_ElevatorSubsystem, ElevatorConstants.kL1));

        //final JoystickButton ElevatorL2 = new JoystickButton(m_XboxController, XboxController.Button.kB.value);
        //ElevatorL2.onTrue(new ElevatorPIDCommand(m_ElevatorSubsystem, ElevatorConstants.kL2));
        joystick.b().onTrue(new ElevatorPIDCommand(m_ElevatorSubsystem, ElevatorConstants.kL2));

        //final JoystickButton ElevatorL3 = new JoystickButton(m_XboxController, XboxController.Button.kX.value);
        //ElevatorL3.onTrue(new ElevatorPIDCommand(m_ElevatorSubsystem, ElevatorConstants.kL3));
        joystick.x().onTrue(new ElevatorPIDCommand(m_ElevatorSubsystem, ElevatorConstants.kL3));

        //final JoystickButton ElevatorL4 = new JoystickButton(m_XboxController, XboxController.Button.kY.value);
        //ElevatorL4.onTrue(new ElevatorPIDCommand(m_ElevatorSubsystem, ElevatorConstants.kL4));
        joystick.y().onTrue(new ElevatorPIDCommand(m_ElevatorSubsystem, ElevatorConstants.kL4));

        //final JoystickButton ResetElevatorEncoder = new JoystickButton(m_XboxController, XboxController.Button.kBack.value);
        //ResetElevatorEncoder.onTrue(m_ElevatorSubsystem.ResetEncoder());
        joystick.back().onTrue(m_ElevatorSubsystem.ResetEncoder());

        //final JoystickButton JogElevatorUp = new JoystickButton(m_XboxController, XboxController.Button.kRightStick.value);
        //ResetElevatorEncoder.whileTrue(new ElevatorCommand(ElevatorConstants.ElevatorSpeed, m_ElevatorSubsystem)).onFalse(new ElevatorCommand(0, m_ElevatorSubsystem));
        joystick.pov(0).whileTrue(new ElevatorCommand(ElevatorConstants.ElevatorSpeed, m_ElevatorSubsystem)).onFalse(new ElevatorCommand(0, m_ElevatorSubsystem));

        //final JoystickButton JogElevatorDown = new JoystickButton(m_XboxController, XboxController.Button.kRightBumper.value);
        //ResetElevatorEncoder.whileTrue(new ElevatorCommand((ElevatorConstants.ElevatorSpeed * -1), m_ElevatorSubsystem)).onFalse(new ElevatorCommand(0, m_ElevatorSubsystem));
        joystick.pov(180).whileTrue(new ElevatorCommand((ElevatorConstants.ElevatorSpeed * -1), m_ElevatorSubsystem)).onFalse(new ElevatorCommand(0, m_ElevatorSubsystem));
        
        joystick.rightTrigger().whileTrue(m_PivotSubsystem.pivotCommand(joystick.getRightTriggerAxis())).onFalse(m_PivotSubsystem.pivotCommand(0));
        joystick.leftTrigger().whileTrue(m_PivotSubsystem.pivotCommand(joystick.getLeftTriggerAxis()*-1)).onFalse(m_PivotSubsystem.pivotCommand(0));
        
        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
