package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.support.HardwareUtility.motorInit;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Settings;

enum State {
    MOVING,
    STOPPED
}

enum Mode {
    POSITION,
    MANUAL
}

/**
 * LinearSlide subsystem for controlling the robot's linear slide mechanism.
 */
public class LinearSlide implements Subsystem {
    private final LinearOpMode opMode;
    public DcMotorEx linearSlide;
    public int positionIndex = 0;


    public LinearSlide(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    /**
     * Initializes the linear slide motor.
     */
    @Override
    public void init() {
        linearSlide = motorInit(opMode.hardwareMap, "linearSlide", DcMotor.Direction.REVERSE);
    }

    @Override
    public void start() {
        return;
    }

    /**
     * Resets the motor encoder and moves to the starting position.
     */
    public void reset() {
        linearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Stops the linear slide motor.
     */
    // TODO: This causes a lot of current loss when the linear slide is at the bottom. FIX IT (drops 2-2.5 volts)
    public void stop() {
        switch (linearSlide.getMode()) {
            case RUN_TO_POSITION:
                linearSlide.setPower(0.1);
                break;
            case RUN_USING_ENCODER:
                linearSlide.setPower(0);
                break;
            default:
                break;
        }
    }

    /**
     * Moves the slide to a specific position with a given power.
     *
     * @param position Target position in encoder ticks.
     * @param power    Power level for the motor.
     */
    // TODO: add PIDF to linear slide movement
    public void moveToPosition(int position, double power) {
        linearSlide.setTargetPosition(position);
        linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linearSlide.setPower(power);
    }

    /**
     * Increases the slide position to the next preset position.
     *
     * @param power Power level for the motor.
     */
    public void increasePosition(double power) {
        if (positionIndex < Settings.LinearSlide.POSITIONS.size() - 1) {
            positionIndex++;
            moveToPosition(Settings.LinearSlide.POSITIONS.get(positionIndex), power);
        }
    }

    /**
     * Decreases the slide position to the previous preset position.
     *
     * @param power Power level for the motor.
     */
    public void decreasePosition(double power) {
        if (positionIndex > 0) {
            positionIndex--;
            moveToPosition(Settings.LinearSlide.POSITIONS.get(positionIndex), power);
        }
    }

    /**
     * Waits for the slide motor to finish moving to its target position.
     */
    public void waitForCompletion() {
        while (linearSlide.isBusy() && opMode.opModeIsActive()) {
            opMode.idle();
        }
    }

    /**
     * Sets a minimal feed-forward power to hold the slide position.
     */
    public void holdPosition() {
        linearSlide.setPower(0.05);
    }

    @Override
    public void telemetry() {
        opMode.telemetry.addData("Linear Slide Position Index", positionIndex);
        opMode.telemetry.addData("Linear Slide Encoder", linearSlide.getCurrentPosition());
    }
}
