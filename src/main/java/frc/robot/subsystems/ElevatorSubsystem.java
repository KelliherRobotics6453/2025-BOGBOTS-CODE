package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.*;

public class ElevatorSubsystem extends SubsystemBase {
  private static SparkMax LeftElavatorSpark = new SparkMax(ElevatorConstants.ElevatorCanID, MotorType.kBrushless);
  private static SparkMax RightElavatorSpark = new SparkMax(ElevatorConstants.ElevatorCanID2, MotorType.kBrushless);
  private static SparkMaxConfig elevatorConfig = new SparkMaxConfig();
  private static RelativeEncoder encoder;
  private static PIDController PIDElevator = new PIDController(ElevatorConstants.kp, ElevatorConstants.ki,ElevatorConstants.kd);

  public ElevatorSubsystem() {
    elevatorConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(MotorConstants.AmpLimitNeo);
    elevatorConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    elevatorConfig.encoder.positionConversionFactor((Math.PI * (ElevatorConstants.PitchDia)) / (ElevatorConstants.GearRedution));
    LeftElavatorSpark.configure(elevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    PIDElevator.setTolerance(ElevatorConstants.kErrorTol);
    RightElavatorSpark.configure(elevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }
  
  public static double ElavatorReverse() {
    if (ElevatorConstants.ElevatorReverse == true) {
      return -1.0;
    } else {
      return 1.0;
    }
  }

  public void goToSetpoint(double setpoint) {
    double speed = PIDElevator.calculate(getEncoderPosition(), setpoint);
    LeftElavatorSpark.set(speed * ElavatorReverse());
    RightElavatorSpark.set(-speed* ElavatorReverse());
    SmartDashboard.putNumber("Elevator Speed", speed);
  
  }
 
  
  public boolean atSetpoint() {
    return PIDElevator.atSetpoint();
  }

  public void stopMotor() {
    LeftElavatorSpark.set(0);
    RightElavatorSpark.set(0);
  }

  public RelativeEncoder getRelativeEncoder() {
    return encoder;
  }

  public double getEncoderPosition() {
    return encoder.getPosition();
  }

  public void resetEncoder() {
    encoder.setPosition(0);
  }

  public static  void Extend(double speed) {
    LeftElavatorSpark.set(speed * ElavatorReverse());
    RightElavatorSpark.set(-speed * ElavatorReverse());
  }

  public Command ResetEncoder() {
    return runOnce(() -> resetEncoder());
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Pivot Height", getEncoderPosition() + 0); // change 0 to the height of the pivot
    
}
}