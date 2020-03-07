package frc.robot.subsystems;

import com.revrobotics.AlternateEncoderType;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ControlType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase {
    private final CANSparkMax armSpark = new CANSparkMax(ArmConstants.ARM_MOTOR, MotorType.kBrushless);
    private final CANPIDController armPIDController = armSpark.getPIDController();
    private final CANEncoder armEncoder = armSpark.getAlternateEncoder(AlternateEncoderType.kQuadrature, 8192);
    private final DigitalInput armLimitSwitch = new DigitalInput(ArmConstants.ARM_LIMIT_SWITCH);

    private double lastSetpointTicks;

    private double lastSetpoint;

    public Arm() {
        configSpark();
        resetEncoder();
        // startPID();

        lastSetpointTicks = getPositionTicks();
    }

    /**
     * Configure the SparkMax, encoder, and PIDController used in the arm
     */
    public void configSpark() {
        armSpark.restoreFactoryDefaults();

        armSpark.setInverted(true);
        armEncoder.setInverted(true);
        armSpark.setIdleMode(IdleMode.kBrake);

        armPIDController.setFeedbackDevice(armEncoder);

        armPIDController.setP(ArmConstants.ARM_PID.kP);
        armPIDController.setI(ArmConstants.ARM_PID.kI);
        armPIDController.setD(ArmConstants.ARM_PID.kD);
        armPIDController.setFF(calcFeedForward(), 0);
    }

    /**
     * Set the arm through a desired percent output
     * 
     * @param speed percent output used to set motor to a certain speed
     */
    public void setArm(double speed) {
        // armSpark.set(speed);
        speed = speed * 10 + getAngleDegrees();
        setArmAngleDegrees(speed);
    }

    /**
     * Set the arm to a specific angle, clamps the value on the upper limit, and let periodic()
     * stop the arm once it gets to its lower limit.
     * 
     * @param angleDegrees angle wanted to set the arm - DEGREES
     */
    // public void setArmAngleDegrees(double angle) {
    //     angle = Math.min(angle, Constants.ArmConstants.UPPER_LIMIT_DEGREES);
    //     if (angle > lastSetpoint || !getLimitSwitchVal()) {
    //         armPIDController.setReference(degreesToTicks(angle), ControlType.kPosition);
    //         lastSetpoint = angle;
    //     }
    // }
        
    public void setArmAngleDegrees(double angleDegrees) {
        setArmAngleTicks(degreesToTicks(angleDegrees));
    }

    /**
     * Set the arm to a specific angle in radians
     * 
     * @param angleRadians angle to set the arm to - RADIANS
     */
    public void setArmAngleRadians(double angleRadians) {
        setArmAngleTicks(radiansToTicks(angleRadians));
    }

    /**
     * Set the arm to a specific angle in ticks.
     * 
     * <p>
     * Unlike {@link #setArmAngleDegrees(double)} and
     * {@link #setArmAngleRadians(double)}, this meethod also clamps the output to
     * ensure that it is within the valid range of motion of the shooter.
     * 
     * @param angleTicks angle to set the arm to - TICKS
     */
    public void setArmAngleTicks(double angleTicks) {
        var clamped = Math.min(angleTicks, ArmConstants.UPPER_BOUND_TICKS);
        if(lastSetpointTicks < angleTicks || !getLimitSwitchVal()) {
            lastSetpointTicks = clamped;
            armPIDController.setReference(clamped, ControlType.kPosition);
        }
    }

    /**
     * Reset arm encoder to zero
     */
    public void resetEncoder() {
        armEncoder.setPosition(degreesToTicks(16));
    }

    /**
     * Get current position of the encoder relative to the starting position
     * 
     * @return the position of the arm encoder - ENCODER TICKS
     */
    public double getPositionTicks() {
        return armEncoder.getPosition();
    }

    /**
     * Get the current velocity of the encoder
     * 
     * @return output velocity of the encoder - DEGREES / SECOND
     */
    public double getVelocityDegreesPerSecond() {
        return ticksToDegrees(armEncoder.getVelocity());
    }

    /**
     * Get the current angle of the arm relative to the drivetrain in degrees
     * 
     * @return current position of the arm - DEGREES
     */
    public double getAngleDegrees() {
        // return Math.toRadians(armEncoder.getPosition() * 180 + 15);
        return ticksToDegrees(getPositionTicks());
    }

    /**
     * Get the current angle of the arm relative to the down position
     * Get the current angle of the arm in radians relative to the drivetrain
     * 
     * @return current position of the arm - RADIANS
     */
    public double getAngleRadians() {
        // return Math.toRadians(armEncoder.getPosition() * 180 + 15);
        return ticksToRadians(getPositionTicks());
    }

    /**
     * Calculate the FeedForward for the arm necessary in PID based on the value
     * given in {@link ArmConstants#FF}
     * 
     * @return calculated FeedForward value
     */
    public double calcFeedForward() {
        return ArmConstants.FF.calculate(getAngleRadians(), 0) / 12.0;
    }

    public void startPID() {
        SmartDashboard.putNumber("Goto Position", 14);
    }

    /**
     * Refresh the in SmartDashboard , as well as gets input from SmartDashboard to
     * update FeedForward
     */
    public void refreshPID() {
    }

    // @Override
    // public void periodic() {
    //     SmartDashboard.putNumber("Actual Arm Angle", this.getAngleDegrees());
    //     refreshPID();
    //     armPIDController.setFF(calcFeedForward());
    // }

    /**
     * Converts degrees of a circle to encoder ticks
     * 
     * @param degrees angle to convert to ticks
     * @return angle in ticks
     */
    public static double degreesToTicks(double degrees) {
        return degrees / 180;
    }

    /**
     * Converts radians of a circle to encoder ticks
     * 
     * @param radians angle to convert to ticks in radians
     * @return angle in encoder ticks
     */
    public static double radiansToTicks(double radians) {
        return radians / Math.PI;
    }

    /**
     * Converts encoder ticks to degrees of a circle
     * 
     * @param ticks encoder ticks to convert to degrees
     * @return angle in degrees
     */
    public static double ticksToDegrees(double ticks) {
        return ticks * 180;
    }

    /**
     * Converts encoder ticks to radians of a circle
     * 
     * @param ticks encoder ticks to convert to radians
     * @return angle in radians
     */
    public static double ticksToRadians(double ticks) {
        return ticks * Math.PI;
    }

    /**
     * Returns the limit switch for the arm
     * 
     * @return arm limit switch
     */
    public DigitalInput getArmLimitSwitch() {
        return armLimitSwitch;
    }

    /**
     * gets the value for the limit switch and switches it
     * 
     * @return limit switch value
     */
    public boolean getLimitSwitchVal() {
        return !(armLimitSwitch.get());
    }

    /**
     * Gets the last setpoint that the arm was set to, in ticks. Note that this not
     * necessarily the current position of the arm.
     * 
     * @return the last setpoint of the arm in ticks
     */
    public double getLastSetpoint() {
        return lastSetpointTicks;
    }

    @Override
    public void periodic() {
        refreshPID();
        SmartDashboard.putNumber("Actual Arm Angle", this.getAngleDegrees());
        armPIDController.setFF(calcFeedForward());
    }
}