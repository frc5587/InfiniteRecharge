package frc.robot.subsystems;

import com.revrobotics.AlternateEncoderType;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
    private final CANSparkMax armSpark = new CANSparkMax(Constants.ArmConstants.ARM_MOTOR, MotorType.kBrushless);
    private final CANPIDController armPIDController = armSpark.getPIDController();
    private final CANEncoder armEncoder = armSpark.getAlternateEncoder(AlternateEncoderType.kQuadrature, 8192);

    public Arm() {
        configSpark();
        resetEncoder();
        startPID();
        refreshPID();
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

        // armPIDController.setP(Constants.ArmConstants.ARM_PID.kP);
        // armPIDController.setI(Constants.ArmConstants.ARM_PID.kI);
        // armPIDController.setD(Constants.ArmConstants.ARM_PID.kD);
        armPIDController.setFF(calcFeedForward(), 0);
    }

    /**
     * Set the arm through a desired percent output
     * 
     * @param speed percent output used to set motor to a certain speed
     */
    public void setArm(double speed) {
        armSpark.set(speed);
    }

    /**
     * Set the arm to a specific angle
     * 
     * @param angle angle wanted to set the arm - DEGREES
     */
    public void setArmAngle(double angle) {
        armPIDController.setReference(angle / 180., ControlType.kPosition);
    }

    /**
     * Reset arm encoder to zero
     */
    public void resetEncoder() {
        armEncoder.setPosition(15. / 180);
    }

    /**
     * Get current position of the encoder relative to the starting position
     * 
     * @return the position of the arm encoder - ENCODER TICKS
     */
    public double getPosition_ticks() {
        return armEncoder.getPosition();
    }

    /**
     * Get the current velocity of the encoder
     * 
     * @return output velocity of the encoder - DEGREES / SECOND
     */
    public double getVelocity_degreesPerSecond() {
        return armEncoder.getVelocity() * 180;
    }

    /**
     * Get the current angle of the arm relative to the down position
     * 
     * @return current position of the arm - DEGREES
     */
    public double getAngle_degrees() {
        // return Math.toRadians(armEncoder.getPosition() * 180 + 15);
        return (getPosition_ticks() * 180);
    }

    /**
     * Calculate the FeedForward for the arm necessary in PID based on the value
     * given in {@link Constants.ArmConstants#FF}
     * 
     * @return calculated FeedForward value
     */
    public double calcFeedForward() {
        return Constants.ArmConstants.FF.calculate(Math.toRadians(getAngle_degrees()), Math.toRadians(getVelocity_degreesPerSecond()));
    }

    public void startPID() {
        SmartDashboard.putNumber("Goto Position", 15);
    }

    /**
     * Refresh the constants in SmartDashboard , as well as gets input from
     * SmartDashboard to update FeedForward
     */
    public void refreshPID() {
        SmartDashboard.putNumber("Angle", getAngle_degrees());
        SmartDashboard.putNumber("Encoder Val", getPosition_ticks());
        SmartDashboard.putNumber("FF", calcFeedForward());
        SmartDashboard.putNumber("Vel", getVelocity_degreesPerSecond());
        setArmAngle(SmartDashboard.getNumber("Goto Position", 15));
    }

    @Override
    public void periodic() {
        refreshPID();
        armPIDController.setFF(calcFeedForward(), 0);
    }
}