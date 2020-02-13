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
import frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase {
    private final CANSparkMax armSpark = new CANSparkMax(ArmConstants.ARM_MOTOR, MotorType.kBrushless);
    private final CANPIDController armPIDController = armSpark.getPIDController();
    private final CANEncoder armEncoder = armSpark.getAlternateEncoder(AlternateEncoderType.kQuadrature, 8192);

    public Arm() {
        configSpark();
        resetEncoder();
    }

    /**
     * Configure the SparkMax, encoder, and PIDController used in the arm
     */
    public void configSpark() {
        armSpark.restoreFactoryDefaults();

        // TODO: Determine if brake mode makes feedforward ineffective b/c
        // characterization doesn't use it
        // armSpark.setIdleMode(IdleMode.kBrake);
        armSpark.setIdleMode(IdleMode.kCoast);

        // Invert so that counter-clockwise is positive
        armSpark.setInverted(true);
        armEncoder.setInverted(true);

        // Conversion factor for rotations of axle encoder -> angle of arm in radians
        // Changes both the value when retrieved and the units the onboard PID loop uses
        // (so make sure setReference uses the correct units too!)
        armEncoder.setPositionConversionFactor(Math.PI);
        armEncoder.setVelocityConversionFactor(Math.PI);

        // Set the PID loop to use the axle encoder (which uses radians as its units now)
        armPIDController.setFeedbackDevice(armEncoder);

        armPIDController.setP(ArmConstants.ARM_PID.kP);
        armPIDController.setI(ArmConstants.ARM_PID.kI);
        armPIDController.setD(ArmConstants.ARM_PID.kD);
        armPIDController.setFF(calcFeedForward());
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
     * Set the arm to a specific angle in degrees
     * 
     * @param angleDegrees angle to set the arm to in degrees
     */
    public void setArmAngleDegrees(double angleDegrees) {
        armPIDController.setReference(Math.toRadians(angleDegrees), ControlType.kPosition);
    }

    /**
     * Reset arm encoder to zero
     */
    public void resetEncoder() {
        armEncoder.setPosition(Math.toRadians(ArmConstants.ARM_OFFSET_DEG));
    }

    /**
     * Get the current velocity of the encoder in radians per second
     * 
     * @return output velocity of the encoder in radians per second
     */
    public double getVelocityRadiansPerSecond() {
        return armEncoder.getVelocity();
    }

    /**
     * Get the current angle of the arm in radians
     * 
     * @return current position of the arm in radians
     */
    public double getAngleRadians() {
        return armEncoder.getPosition();
    }

    /**
     * Calculate the FeedForward for the arm necessary in PID based on the value
     * given in {@link Constants.ArmConstants#FF}
     * 
     * @return calculated FeedForward value
     */
    public double calcFeedForward() {
        // Uses current angle and velocity to effectively hold the current value
        return ArmConstants.FF.calculate(getAngleRadians(), getVelocityRadiansPerSecond());
    }

    public void startPID() {
        SmartDashboard.putNumber("Goto Position", ArmConstants.ARM_OFFSET_DEG);
    }

    /**
     * Refresh the constants in SmartDashboard , as well as gets input from
     * SmartDashboard to update FeedForward
     */
    public void refreshPID() {
        var angle = getAngleRadians();
        SmartDashboard.putNumber("Angle Rad", angle);
        SmartDashboard.putNumber("Angle Deg", Math.toDegrees(angle));
        SmartDashboard.putNumber("FF", calcFeedForward());
        SmartDashboard.putNumber("Vel Rad/s", getVelocityRadiansPerSecond());
        setArmAngleDegrees(SmartDashboard.getNumber("Goto Position", ArmConstants.ARM_OFFSET_DEG));
    }

    @Override
    public void periodic() {
        armPIDController.setFF(calcFeedForward());
    }
}