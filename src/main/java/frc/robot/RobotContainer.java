// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.Joystick;
//import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
//import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.ElevatorCommand;
import frc.robot.commands.ElevatorPIDCommand;
import frc.robot.commands.PivotPIDCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.*;;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second
                                                                                      // max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    public final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);
    
    private final CommandXboxController joystick = new CommandXboxController(0);
     Joystick m_Joystick1 = new Joystick(1);
    

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public final PivotSubsystem m_PivotSubsystem = new PivotSubsystem();

    public final ShooterSubsystem m_ShooterSubsystem = new ShooterSubsystem();

    public final ElevatorSubsystem m_ElevatorSubsystem = new ElevatorSubsystem();

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically
                drivetrain.applyRequest(() -> drive.withVelocityX(-(DriveConstants.adjustSpeed(joystick.getLeftY()))) // Drive
                                                                                                                      // forward
                                                                                                                      // with
                                                                                                                      // negative
                                                                                                                      // Y
                                                                                                                      // (forward)//
                                                                                                                      // speed
                                                                                                                      // was
                                                                                                                      // getting
                                                                                                                      // multipled
                                                                                                                      // by
                                                                                                                      // MaxSpeed.
                        .withVelocityY(-(DriveConstants.adjustSpeed(joystick.getLeftX()))) // Drive left with negative X
                                                                                           // (left)
                        .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with
                                                                                    // negative X (left)
                ));

        joystick.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        // bindings added by Kedrick and Don

        joystick.start().onTrue(new ElevatorPIDCommand(m_ElevatorSubsystem, 0));

        joystick.a().onTrue(new ElevatorPIDCommand(m_ElevatorSubsystem, ElevatorConstants.kL1));

        joystick.b().onTrue(new ElevatorPIDCommand(m_ElevatorSubsystem, ElevatorConstants.kL2));

        joystick.x().onTrue(new ElevatorPIDCommand(m_ElevatorSubsystem, ElevatorConstants.kL3));

        joystick.y().onTrue(new ElevatorPIDCommand(m_ElevatorSubsystem, ElevatorConstants.kL4));
        

        joystick.back().onTrue(m_ElevatorSubsystem.ResetEncoder());

        joystick.pov(0).whileTrue(new ElevatorCommand(ElevatorConstants.ElevatorSpeed, m_ElevatorSubsystem))
                .onFalse(new ElevatorCommand(0, m_ElevatorSubsystem));

        joystick.pov(180).whileTrue(new ElevatorCommand((ElevatorConstants.ElevatorSpeed * -1), m_ElevatorSubsystem))
                .onFalse(new ElevatorCommand(0, m_ElevatorSubsystem));

        joystick.rightBumper().whileTrue(m_PivotSubsystem.pivotCommand(PivotConstants.PivotSpeed))
                .onFalse(m_PivotSubsystem.pivotCommand(0));
        joystick.rightTrigger(.5).whileTrue(m_PivotSubsystem.pivotCommand(-PivotConstants.PivotSpeed))
                .onFalse(m_PivotSubsystem.pivotCommand(0));

        joystick.leftBumper().whileTrue(m_ShooterSubsystem.runShooter(ShooterConstants.ShooterSpeed))
                .onFalse(m_ShooterSubsystem.runShooter(0));
        joystick.leftTrigger(.5).whileTrue(m_ShooterSubsystem.runShooter(-ShooterConstants.ShooterSpeed))
                .onFalse(m_ShooterSubsystem.runShooter(0));

        drivetrain.registerTelemetry(logger::telemeterize);



        //oporator joystick 
        final JoystickButton PivotLoad = new JoystickButton(m_Joystick1, 7);
        PivotLoad.onTrue(new PivotPIDCommand(m_PivotSubsystem, PivotConstants.kIntake));
    }
    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
