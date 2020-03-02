package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import frc.robot.Constants.ShooterConstants;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShooterPID extends SubsystemBase {
  private final CANSparkMax motorOne = new CANSparkMax(ShooterConstants.SHOOTER_MOTOR_ONE,
      MotorType.kBrushless);
  private final CANSparkMax motorTwo = new CANSparkMax(ShooterConstants.SHOOTER_MOTOR_TWO,
      MotorType.kBrushless);
  // private final CANPIDController sparkPIDControllerOne = motorOne.getPIDController();
  // private final CANPIDController sparkPIDControllerTwo = motorTwo.getPIDController();

  private final CANEncoder sparkEncoderOne = motorOne.getEncoder();
  private final CANEncoder sparkEncoderTwo = motorTwo.getEncoder();

  private final ShooterFeedbackController motorOneController = new ShooterFeedbackController(this::getMotorOneVelocity);
  private final ShooterFeedbackController motorTwoController = new ShooterFeedbackController(this::getMotorTwoVelocity);

  private double setpointVelocity = 0;
  private boolean enabled = true;

  public ShooterPID() {

    configureSpark();
  }

  private void configureSpark() {
    motorOne.restoreFactoryDefaults();
    motorTwo.restoreFactoryDefaults();

    motorOne.setInverted(true);
    motorTwo.setInverted(true);

    motorOne.setIdleMode(IdleMode.kCoast);
    motorTwo.setIdleMode(IdleMode.kCoast);

    motorOne.setSmartCurrentLimit(40, 35);

    motorTwo.setSmartCurrentLimit(40, 35);
  }

  public double getShooterSpeed() {
    return (sparkEncoderOne.getVelocity() + sparkEncoderTwo.getVelocity()) / 2;
  }

  protected void useOutput(double motorOneVoltage, double motorTwoVoltage) {
    motorOne.setVoltage(motorOneVoltage);
    motorTwo.setVoltage(motorTwoVoltage);
  }

  public double getMotorOneVelocity() {
    return sparkEncoderOne.getVelocity();
  }

  public double getMotorTwoVelocity() {
    return sparkEncoderTwo.getVelocity();
  }

  /**
   * Calculates the speed that the shooter must spin at to get the powercell in the power port
   * based on the distance from the power port and the angle of the arm
   * 
   * @param distanceFromTarget the distance from the target - METERS
   * @param armAngle           the angle of the arm         - RADIANS
   * @return                   the speed of the shooter     - RPM
   */
  public static double calculateShooterSpeed(double distanceFromTarget, double armAngle) {
    return 600 + (((1 / (Math.sqrt((Limelight.getWorkingHeight(armAngle)
        - (distanceFromTarget * Math.tan(armAngle)) / (-.5 * ShooterConstants.G)))))
        * (distanceFromTarget / Math.cos(armAngle))
        * (ShooterConstants.CONVERSION_FACTOR / ShooterConstants.FLYWHEEL_RADIUS)));
  }

  public void log() {
    SmartDashboard.putNumber("velocity one", sparkEncoderOne.getVelocity());
    SmartDashboard.putNumber("velocity two", sparkEncoderTwo.getVelocity());
  }

  @Override
  public void periodic() {
    log();

    if (enabled) {
      motorOneController.setSetpoint(setpointVelocity);
      motorTwoController.setSetpoint(setpointVelocity);

      motorOne.setVoltage(motorOneController.calculate());
      motorTwo.setVoltage(motorTwoController.calculate());
    }
  }

  public void enable() {
    enabled = true;
  }

  public void disable() {
    enabled = false;
  }

  public void setSetpoint(double setpointVelocity) {
    this.setpointVelocity = setpointVelocity;
  }
}