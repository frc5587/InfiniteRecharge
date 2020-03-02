package frc.robot.subsystems;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ShooterConstants;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Timer;

public class ShooterFeedbackController {
  private double kF;
  private double kJ;
  private double kLoadRatio;
  private double lastOutput;
  private double setpointVelocityRPS;
  private double lastTime;

  private double errorThreshold = 0.06;

  private Timer timer = new Timer();
  private DoubleSupplier motorVelocitySupplier;

  /**
   * This build a controller based on
   * https://www.team254.com/frc-day-12-13-build-blog/ Its controls the shooter to
   * anticipate the drop in speed from shooting the ball.
   * 
   * I really made a mess of this controller, many of the inherited methods will
   * not work, so you should really only use the methods defines in this class
   * 
   * @param kJ         arbitrary constant
   * @param kLoadRatio arbitrary constant
   * @param kF         voltage constant
   */
  public ShooterFeedbackController(double kJ, double kLoadRatio, double kF, DoubleSupplier motorVelocitySupplier) {
    this.kF = kF;
    this.kJ = kJ;
    this.kLoadRatio = kLoadRatio;
    this.motorVelocitySupplier = motorVelocitySupplier;
    timer.start();
    lastTime = timer.get();
  }

  /**
   * Builds a controller that uses 254's suggest voltage constant of 8V
   * 
   * @param kJ         arbitrary constant
   * @param kLoadRatio arbitrary constant
   */
  public ShooterFeedbackController(double kJ, double kLoadRatio, DoubleSupplier motorVelocitySupplier) {
    this(kJ, kLoadRatio, 8, motorVelocitySupplier);
  }

  /**
   * Build a controller that uses the shooter constants
   */
  public ShooterFeedbackController(DoubleSupplier motorVelocitySupplier) {
    this(ShooterConstants.kJ, ShooterConstants.kLoadRatio, ShooterConstants.kF, motorVelocitySupplier);
  }

  /**
   * @param currentVelocityRPS the current velocity - ROTATION PER SECOND
   * @return the voltage to set the motor to
   */
  public double calculate(double currentVelocityRPS) {
    this.lastOutput = (kF * setpointVelocityRPS) + lastOutput
        + (kJ * elapsedTime() * ((kLoadRatio * setpointVelocityRPS) - currentVelocityRPS));
    return this.lastOutput;
  }

  public double calculate() {
    return calculate(motorVelocitySupplier.getAsDouble());
  }

  /**
   * Calculates the voltage to set the motor to
   * 
   * @param currentVelocityRPS  current speed of the shooter - ROTATIONS PER SECOND
   * @param setpointVelocityRPS setpoint speed - ROTATIONS PER SECOND
   * @return voltage - VOLTS
   */
  public double calculate(double currentVelocityRPS, double setpointVelocityRPS) {
    setSetpoint(setpointVelocityRPS);
    return calculate(currentVelocityRPS);
  }

  /**
   * Get (and set) the time since this method was called last
   * 
   * @return elapsed time - SECONDS
   */
  public double elapsedTime() {
    double timeDiff = timer.get() - this.lastTime;
    this.lastTime = timer.get();
    return timeDiff;
  }

  /**
   * Sets the setpoint for the PID
   * 
   * @param setpointVelocityRPS setpoint - ROTATIONS PER SECOND
   */
  public void setSetpoint(double setpointVelocityRPS) {
    this.setpointVelocityRPS = setpointVelocityRPS;
  }

  /**
   * Sets the setpoints, the calculates the PID based on the current speed of the
   * arm, which is retrieved via the DoubleSupplier
   * 
   * @param setpointVelocityRPS setpoint - ROTATIONS PER SECOND
   * @return voltage to set the shooter - VOLTS
   */
  public double setSetpointAndCalculate(double setpointVelocityRPS) {
    setSetpoint(setpointVelocityRPS);
    return calculate();
  }

  /*
   * Sends debug info to Smart Dashboard
   */
  public void sendDebugInfo() {
    SmartDashboard.putNumber("Shooter Setpoint", setpointVelocityRPS);
    SmartDashboard.putNumber("Shooter Last Output", lastOutput);
    SmartDashboard.putNumber("Shooter Current Velocity", motorVelocitySupplier.getAsDouble());
  }

  /**
   * Get the current setpoint of the shooter
   * 
   * @return setpoint - ROTATIONS PER SECOND
   */
  public double getSetpoint() {
    return setpointVelocityRPS;
  }

  /**
   * True if the shooter speed is within [+/-] `errorThreshold` percent
   * 
   * @return true if "close enough"
   */
  public boolean atSetpoint() {
    return ((1 - errorThreshold) < motorVelocitySupplier.getAsDouble() / setpointVelocityRPS)
        && (motorVelocitySupplier.getAsDouble() / setpointVelocityRPS < (1 + errorThreshold));
  }
}
