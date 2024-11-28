package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.HardwareUtility.motorInit;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Settings;

public class LinearSlide {
    private LinearOpMode opMode;

    public DcMotor linearSlide;

    public int positionIndex = 0;

    public LinearSlide(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    public void init() {
        linearSlide = motorInit(opMode.hardwareMap, "linearSlide", DcMotor.Direction.FORWARD);
        linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void start() {
        move(Settings.LinearSlide.STARTING_POSITION, Settings.LinearSlide.POWER);
        stop();
        positionIndex = 0;
    }

    public void stop() {
        linearSlide.setPower(0);
    }

    public void stopFeedForward() {
        linearSlide.setPower(0.05);
    }

    // MOVE
    public void move(int position, double power, boolean async) {
        linearSlide.setTargetPosition(position);
        linearSlide.setPower(power);

        if (!async) {
            waitForCompletion();

            stop();
        }
    }

    public void move(int position, double power) {
        move(position, power, false);
    }

    public void move(int position, boolean async) {
        move(position, Settings.LinearSlide.POWER, async);
    }

    public void move(int position) {
        move(position, Settings.LinearSlide.POWER);
    }


    public void increasePosition(double power) {
        if (positionIndex < Settings.LinearSlide.POSITIONS.length() - 1) {
            positionIndex++;
            move(Settings.LinearSlide.POSITIONS.get(positionIndex), power);
        }

        if (positionIndex == 1) {
            linearSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    public void decreasePosition(double power) {
        if (positionIndex > 0) {
            positionIndex--;
            move(Settings.LinearSlide.POSITIONS.get(positionIndex), power);
        }

        if (positionIndex == 0) {
            linearSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
    }

    public void waitForCompletion() {
        while (linearSlide.isBusy() && opMode.opModeIsActive()) {
            opMode.idle();
        }
    }
}
