package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.HardwareUtility.motorInit;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantFunction;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Settings;

/**
 * LinearSlide subsystem for controlling the robot's linear slide mechanism.
 */
public class LinearSlide {
    private final LinearOpMode opMode;
    public DcMotor linearSlide;
    public int positionIndex = 0;
    public boolean inManualMode = false;

    public LinearSlide(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    /**
     * Initializes the linear slide motor.
     */
    public void init() {
        linearSlide = motorInit(opMode.hardwareMap, "linearSlide", DcMotor.Direction.REVERSE);
    }

    /**
     * Resets the motor encoder and moves to the starting position.
     */
    public void start() {
    }

    /**
     * Stops the linear slide motor.
     */
    public void stop() {
        linearSlide.setPower(0);
        linearSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /**
     * Moves the slide to a specific position with a given power.
     *
     * @param position Target position in encoder ticks.
     * @param power    Power level for the motor.
     */
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

    public void telemetry() {
        opMode.telemetry.addData("Linear Slide Position Index", positionIndex);
        opMode.telemetry.addData("Linear Slide Encoder", linearSlide.getCurrentPosition());
    }

    public class RaiseLinearSlideToHangSpecimenAction implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            positionIndex = Settings.LinearSlide.POSITIONS.indexOf(Settings.LinearSlide.SPECIMEN_APPROACH_POSITION);
            moveToPosition(Settings.LinearSlide.POSITIONS.get(positionIndex), Settings.LinearSlide.POWER);
            return false;
        }
    }

    public Action raiseLinearSlideToHangSpecimen() {
        return new RaiseLinearSlideToHangSpecimenAction();
    }

    public class LowerLinearSlideToStartAction implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            positionIndex = Settings.LinearSlide.POSITIONS.indexOf(Settings.LinearSlide.STARTING_POSITION);
            moveToPosition(Settings.LinearSlide.POSITIONS.get(positionIndex), Settings.LinearSlide.POWER);
            return false;
        }
    }

    public Action lowerLinearSlideToStart() {
        return new LowerLinearSlideToStartAction();
    }
}
