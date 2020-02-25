package frc.robot.subsystems;

import com.revrobotics.AlternateEncoderType;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
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
        // refreshPID();
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
        armSpark.set(speed);
    }

    /**
     * Set the arm to a specific angle
     * 
     * @param angle angle wanted to set the arm - DEGREES
     */
    public void setArmAngleDegrees(double angle) {
        armPIDController.setReference(degreesToTicks(angle), ControlType.kPosition);

    }

    /**
     * Reset arm encoder to zero
     */
    public void resetEncoder() {
        armEncoder.setPosition(degreesToTicks(14));
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
     * Get the current angle of the arm relative to the down position
     * 
     * @return current position of the arm - DEGREES
     */
    public double getAngleDegrees() {
        // return Math.toRadians(armEncoder.getPosition() * 180 + 15);
        return ticksToDegrees(getPositionTicks());
    }

    /**
     * Calculate the FeedForward for the arm necessary in PID based on the value
     * given in {@link Constants.ArmConstants#FF}
     * 
     * @return calculated FeedForward value
     */
    public double calcFeedForward() {
        // var ff = Constants.ArmConstants.FF.calculate(Math.toRadians(getAngleDegrees()), 0) / 12;
        // // System.out.println("FF: " + ff);
        // return ff;
        return Constants.ArmConstants.FF.calculate(Math.toRadians(getAngleDegrees()), 0) / 12.0;
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

    // @Override
    // public void periodic() {
    //     System.out.println(getAngleDegrees());
    //     refreshPID();
    //     armPIDController.setFF(calcFeedForward());
    // }

    /**
     * Converts degrees of a circle to encoder ticks 1 tick == 180 degrees
     * 
     * @param degrees angle to convert to ticks
     * @return angle in ticks
     */
    public double degreesToTicks(double degrees) {
        return degrees / 180;
    }

    /**
     * Converts encoder ticks to degrees of a circle 1 tick == 180 degrees
     * 
     * @param ticks angle to convert to degrees
     * @return angle in degrees
     */
    public double ticksToDegrees(double ticks) {
        return ticks * 180;
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