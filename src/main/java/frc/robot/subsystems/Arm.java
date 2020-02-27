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

public class Arm extends SubsystemBase {
    private final CANSparkMax armSpark = new CANSparkMax(Constants.ArmConstants.ARM_MOTOR, MotorType.kBrushless);
    private final CANPIDController armPIDController = armSpark.getPIDController();
    private final CANEncoder armEncoder = armSpark.getAlternateEncoder(AlternateEncoderType.kQuadrature, 8192);
    private final DigitalInput armLimitSwitch = new DigitalInput(Constants.ArmConstants.ARM_LIMIT_SWITCH);

    public Arm() {
        configSpark();
        resetEncoder();
        // startPID();
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

        armPIDController.setP(Constants.ArmConstants.ARM_PID.kP);
        armPIDController.setI(Constants.ArmConstants.ARM_PID.kI);
        armPIDController.setD(Constants.ArmConstants.ARM_PID.kD);
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
        armPIDController.setReference(degreesToTicks(speed), ControlType.kPosition);
    }

    /**
     * Set the arm to a specific angle in degrees
     * 
     * @param angleDegrees angle wanted to set the arm - DEGREES
     */
    public void setArmAngleDegrees(double angleDegrees) {
        armPIDController.setReference(degreesToTicks(angleDegrees), ControlType.kPosition);
    }

    /**
     * Set the arm to a specific angle in radians
     * 
     * @param angleRadians angle to set the arm to - RADIANS
     */
    public void setArmAngleRadians(double angleRadians) {
        armPIDController.setReference(radiansToTicks(angleRadians), ControlType.kPosition);
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
     * Get the current angle of the arm in radians relative to the drivetrain
     * 
     * @return current position of the arm - RADIANS
     */
    public double getAngleRadians() {
        return ticksToRadians(getPositionTicks());
    }

    /**
     * Calculate the FeedForward for the arm necessary in PID based on the value
     * given in {@link Constants.ArmConstants#FF}
     * 
     * @return calculated FeedForward value
     */
    public double calcFeedForward() {
        return Constants.ArmConstants.FF.calculate(getAngleRadians(), 0) / 12.0;
    }

    public void startPID() {
        SmartDashboard.putNumber("Goto Position", 14);
    }

    /**
     * Refresh the constants in SmartDashboard , as well as gets input from
     * SmartDashboard to update FeedForward
     */
    public void refreshPID() {
        SmartDashboard.putNumber("Angle", getAngleDegrees());
        SmartDashboard.putNumber("Encoder Val", getPositionTicks());
        SmartDashboard.putNumber("FF", calcFeedForward());
        SmartDashboard.putNumber("Vel", getVelocityDegreesPerSecond());
        // setArmAngleDegrees(SmartDashboard.getNumber("Goto Position", 14));
    }

    /**
     * Converts degrees of a circle to encoder ticks
     * 
     * @param degrees angle to convert to ticks
     * @return angle in ticks
     */
    public double degreesToTicks(double degrees) {
        return degrees / 180;
    }

    /**
     * Converts radians of a circle to encoder ticks
     * 
     * @param radians angle to convert to ticks in radians
     * @return angle in encoder ticks
     */
    public double radiansToTicks(double radians) {
        return radians / Math.PI;
    }

    /**
     * Converts encoder ticks to degrees of a circle
     * 
     * @param ticks encoder ticks to convert to degrees
     * @return angle in degrees
     */
    public double ticksToDegrees(double ticks) {
        return ticks * 180;
    }

    /**
     * Converts encoder ticks to radians of a circle
     * 
     * @param ticks encoder ticks to convert to radians
     * @return angle in radians
     */
    public double ticksToRadians(double ticks) {
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

    @Override
    public void periodic() {
        refreshPID();
        var ff = calcFeedForward();
        armPIDController.setFF(ff);
    }
}