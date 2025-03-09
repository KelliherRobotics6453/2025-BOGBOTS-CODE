package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {

    //private static PivotShooterSubsystem pivotShooter = new PivotShooterSubsystem();

    /*public static PivotShooterSubsystem getInstance() {
        return instance;
    }*/

    private static SparkMax shooterMotor = new SparkMax(ShooterConstants.ShooterCanID, MotorType.kBrushed);
    private static SparkMaxConfig shooterMotorConfig = new SparkMaxConfig();

    public static void configureShooterMotor() {
        shooterMotorConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(MotorConstants.AmpLimitNeo);
        shooterMotor.configure(shooterMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }

    public ShooterSubsystem() {
        configureShooterMotor();
    }

    public Command intake() {
        return Commands.runOnce(() -> shooterMotor.set(ShooterConstants.ShooterSpeed),this);
    }
    
    public Command outtake() {
        return Commands.runOnce(() -> shooterMotor.set(-ShooterConstants.ShooterSpeed),this);
    }
}
