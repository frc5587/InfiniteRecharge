package frc.robot.subsystems;

import com.revrobotics.AlternateEncoderType;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
    private final CANSparkMax armSpark = new CANSparkMax(Constants.ArmConstants.ARM_MOTOR, MotorType.kBrushless);
    private final CANPIDController armPIDController = armSpark.getPIDController();
    private final CANEncoder armEncoder = armSpark.getAlternateEncoder(AlternateEncoderType.kQuadrature, 8192);

    public Arm() {

    }

    /**
     * Configure the SparkMax, encoder, and PIDController used in the arm
     */
    public void configSpark() {
        armSpark.restoreFactoryDefaults();

        armSpark.setInverted(true);
        armEncoder.setInverted(true);

        armPIDController.setFeedbackDevice(armEncoder);

        armPIDController.setP(Constants.ArmConstants.ARM_PID.kP);
        armPIDController.setI(Constants.ArmConstants.ARM_PID.kI);
        armPIDController.setD(Constants.ArmConstants.ARM_PID.kD);
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
     * Set the arm to a specific angle
     * 
     * @param angle angle wanted to set the arm, divided by 360 in order to get
     *              encoder ticks
     */
    public void setArmAngle(double angle) {
        armPIDController.setReference(angle, ControlType.kPosition);
    }

    /**
     * Reset arm encoder to zero
     */
    public void resetEncoder() {
        armEncoder.setPosition(0);
    }

    /**
     * Get current position of the encoder relative to the starting position
     * 
     * @return the position of the arm encoder
     */
    public double getPosition() {
        return armEncoder.getPosition();
    }

    /**
     * Get the current velocity of the encoder
     * 
     * @return output velocity of the encoder
     */
    public double getVelocity() {
        return armEncoder.getVelocity();
    }

    /**
     * Get the current angle of the arm relative to the down position
     * 
     * @return current position of the arm
     */
    public double getAngle() {
        return Math.toRadians(armEncoder.getPosition() * 180 + 7);
    }

    /**
     * Calculate the FeedForward for the arm necessary in PID based on the value
     * given in {@link Constants.ArmConstants#FF}
     * 
     * @return calculated FeedForward value
     */
    public double calcFeedForward() {
        return Constants.ArmConstants.FF.calculate(getAngle(), getVelocity() * Math.PI / 60);
    }

    /**
     * Refresh the constants in SmartDashboard , as well as gets input from
     * SmartDashboard to update FeedForward
     */
    public void refreshPID() {
        SmartDashboard.putNumber("Angle", getAngle());
        SmartDashboard.putNumber("Position", getPosition());
        SmartDashboard.putNumber("Goto Position", 0.0);
        SmartDashboard.putNumber("FF", calcFeedForward());
        SmartDashboard.putNumber("Vel", armEncoder.getVelocity() * Math.PI);
        setArmAngle(SmartDashboard.getNumber("Goto Position", 0.0));
    }

    @Override
    public void periodic() {
        armPIDController.setFF(calcFeedForward());
        refreshPID();
    }
}