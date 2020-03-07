package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import frc.robot.Constants.ShooterConstants;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShooterJRAD extends SubsystemBase {
  private final CANSparkMax motorOne = new CANSparkMax(ShooterConstants.SHOOTER_MOTOR_ONE, MotorType.kBrushless);
  private final CANSparkMax motorTwo = new CANSparkMax(ShooterConstants.SHOOTER_MOTOR_TWO, MotorType.kBrushless);

  private final CANEncoder sparkEncoderOne = motorOne.getEncoder();
  private final CANEncoder sparkEncoderTwo = motorTwo.getEncoder();

  private final ShooterFeedbackController motorOneController = new ShooterFeedbackController(ShooterConstants.MOTOR_ONE_JRAD, this::getMotorOneVelocity);
  private final ShooterFeedbackController motorTwoController = new ShooterFeedbackController(ShooterConstants.MOTOR_TWO_JRAD, this::getMotorTwoVelocity);

  private double setpointVelocity = 0;
  private boolean enabled = true;
  private double lastVelocity = 0;
  private double nowVelocity = 0;

  public ShooterJRAD() {
    configureSpark();
  }

  /**
   * Configures the sparks with the following - sets factory defaults - inverts
   * the direction - sets to coast mode - set current limits
   */
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

  /**
   * Returns that average motor speed between the two shooter motors
   * 
   * @return average velocity - ROTATIONS PER SECOND
   */
  public double getShooterSpeed() {
    return (sparkEncoderOne.getVelocity() + sparkEncoderTwo.getVelocity()) / 2;
  }

  /**
   * Uses the output from the JRAD controller
   * 
   * @param motorOneVoltage
   * @param motorTwoVoltage
   */
  public void useOutput(double motorOneVoltage, double motorTwoVoltage) {
    motorOne.setVoltage(motorOneVoltage);
    motorTwo.setVoltage(motorTwoVoltage);
  }

  /**
   * Gets the velocity of the first motor
   * 
   * @return velocity - ROTATIONS PER SECOND
   */
  public double getMotorOneVelocity() {
    return sparkEncoderOne.getVelocity();
  }

  /**
   * Gets the velocity of the second motor
   * 
   * @return velocity - ROTATIONS PER SECOND
   */
  public double getMotorTwoVelocity() {
    return sparkEncoderTwo.getVelocity();
  }

  /**
   * Calculates the speed that the shooter must spin at to get the powercell in
   * the power port based on the distance from the power port and the angle of the
   * arm
   * 
   * @param distanceFromTarget the distance from the target - METERS
   * @param armAngle           the angle of the arm - RADIANS
   * @return the speed of the shooter - RPM
   */
  public static double calculateShooterSpeed(double distanceFromTarget, double armAngle) {
    return 60 * Math.sqrt(2 * ShooterConstants.G * Limelight.getWorkingHeight(armAngle))
            / (ShooterConstants.FLYWHEEL_CIRCUMFERENCE * Math.sin(armAngle) * ShooterConstants.GEARING);

    // return (((1 / (Math.sqrt((Limelight.getWorkingHeight(armAngle)
    //     - (distanceFromTarget * Math.tan(armAngle)) / (-.5 * ShooterConstants.G)))))
    //     * (distanceFromTarget / Math.cos(armAngle))
    //     * (ShooterConstants.CONVERSION_FACTOR / (ShooterConstants.FLYWHEEL_RADIUS * ShooterConstants.GEARING))));
  }

  /**
   * Logs velocity data
   */
  public void log() {
    SmartDashboard.putNumber("Velocity difference", nowVelocity - lastVelocity);
    SmartDashboard.putNumber("Velocity Setpoint Slice", Math.max(0,  getShooterSpeed() - setpointVelocity));
    SmartDashboard.putNumber("velocity one", sparkEncoderOne.getVelocity());
    SmartDashboard.putNumber("velocity two", sparkEncoderTwo.getVelocity());
  }

  /**
   * If the JRAD control in enabled, it will update the motors based on the
   * setpoint
   */
  @Override
  public void periodic() {
    lastVelocity = nowVelocity;
    nowVelocity = sparkEncoderOne.getVelocity();
    log();
    motorOneController.sendDebugInfo();
    // motorTwoController.sendDebugInfo();

    if (enabled) {
      useOutput(motorOneController.setSetpointAndCalculate(setpointVelocity),
          motorTwoController.setSetpointAndCalculate(setpointVelocity));
    }
  }

  /**
   * Enables JRAD control
   */
  public void enable() {
    motorOneController.reset();
    motorTwoController.reset();
    enabled = true;
  }

  /**
   * Disables JRAD control
   */
  public void disable() {
    enabled = false;
  }

  /**
   * Sets a new velocity setpoint for the shooter
   * 
   * @param setpointVelocity new setpoint - ROTATIONS PER SECOND
   */
  public void setVelocity(double setpointVelocity) {
    this.setpointVelocity = setpointVelocity;
  }

  /**
   * If JRAD is disable, this controls the percent output of the shooter
   * 
   * @param throttle percent output - [-1, 1]
   */
  public void setThrottle(double throttle) {
    if (!enabled) {
      motorOne.set(throttle);
      motorTwo.set(throttle);
    }
  }

  public boolean atSetpoint() {
    return motorOneController.atSetpoint() && motorTwoController.atSetpoint();
  }
}