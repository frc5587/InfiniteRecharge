package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.frc5587.lib.pid.JRAD;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Timer;

public class ShooterFeedbackController {
  private JRAD JRADConstants;
  private double lastOutput = 0;
  private double setpointVelocityRPM;
  private double lastTime;
  private double diff;

  // private double errorThreshold = 0.98;

  private Timer timer = new Timer();
  private DoubleSupplier motorVelocitySupplier;

  /**
   * This build a controller based on
   * https://www.team254.com/frc-day-12-13-build-blog/ Its controls the shooter to
   * anticipate the drop in speed from shooting the ball.
   * 
   * @param kJ         arbitrary constant
   * @param kLoadRatio arbitrary constant
   * @param kF         voltage constant
   */
  public ShooterFeedbackController(JRAD JRADConstants, DoubleSupplier motorVelocitySupplier) {
    this.JRADConstants = JRADConstants;
    this.motorVelocitySupplier = motorVelocitySupplier;
    timer.start();
    lastTime = timer.get();
  }

  /**
   * @param currentVelocityRPM the current velocity - ROTATIONS PER MINUTE
   * @return the voltage to set the motor to
   */
  public double calculate(double currentVelocityRPM) {
    this.lastOutput = (JRADConstants.kF * setpointVelocityRPM) + lastOutput
        + (JRADConstants.kJ * 0.02 * ((JRADConstants.kLoadRatio * setpointVelocityRPM) - currentVelocityRPM));
    return this.lastOutput;
  }

  public double calculate() {
    return calculate(motorVelocitySupplier.getAsDouble());
  }

  /**
   * Calculates the voltage to set the motor to
   * 
   * @param currentVelocityRPM  current speed of the shooter - ROTATIONS PER
   *                            MINUTE
   * @param setpointVelocityRPM setpoint speed - ROTATIONS PER MINUTE
   * @return voltage - VOLTS
   */
  public double calculate(double currentVelocityRPM, double setpointVelocityRPM) {
    setSetpoint(setpointVelocityRPM);
    return calculate(currentVelocityRPM);
  }

  /**
   * Get (and set) the time since this method was called last
   * 
   * @return elapsed time - MINUTES
   */
  public double elapsedTime() {
    this.diff = timer.get() - this.lastTime;
    this.lastTime = timer.get();
    return diff;
  }

  /**
   * Sets the setpoint for the PID
   * 
   * @param setpointVelocityRPM setpoint - ROTATIONS PER MINUTE
   */
  public void setSetpoint(double setpointVelocityRPM) {
    this.setpointVelocityRPM = setpointVelocityRPM;
  }

  /**
   * Sets the setpoints, the calculates the PID based on the current speed of the
   * arm, which is retrieved via the DoubleSupplier
   * 
   * @param setpointVelocityRPM setpoint - ROTATIONS PER MINUTE
   * @return voltage to set the shooter - VOLTS
   */
  public double setSetpointAndCalculate(double setpointVelocityRPM) {
    setSetpoint(setpointVelocityRPM);
    return calculate();
  }

  /*
   * Sends debug info to Smart Dashboard
   */
  public void sendDebugInfo() {
    SmartDashboard.putNumber("Shooter Last Output (V)", lastOutput);
    SmartDashboard.putNumber("Shooter Current Velocity", motorVelocitySupplier.getAsDouble());
  }

  /**
   * Get the current setpoint of the shooter
   * 
   * @return setpoint - ROTATIONS PER MINUTE
   */
  public double getSetpoint() {
    return setpointVelocityRPM;
  }

  /**
   * Because this overshoots the setpoint (as designed), we should only check if
   * it is above the setpoint. Once a ball shoots the speed will drop, but the
   * speed should always stay above the setpoint
   * 
   * @return true if the speed is above the setpoint
   */
  public boolean atSetpoint() {
    return (motorVelocitySupplier.getAsDouble() >= setpointVelocityRPM * JRADConstants.kLoadRatio);
  }

  public void reset() {
    lastOutput = 0;
  }
}
