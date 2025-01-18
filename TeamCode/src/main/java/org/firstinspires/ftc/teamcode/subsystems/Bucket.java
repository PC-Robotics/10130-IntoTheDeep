package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.support.HardwareUtility.servoInit;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Settings;

public class Bucket implements Subsystem {
    private LinearOpMode opMode;

    public Servo bucket;

    public int positionIndex = 0;

    public Bucket(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    @Override
    public void init() {
        bucket = servoInit(opMode.hardwareMap, "bucket", Servo.Direction.REVERSE);
    }

    @Override
    public void start() {
        pickup();
    }

    public void pickup() {
        bucket.setPosition(Settings.Bucket.PICKUP_POSITION);
        positionIndex = 0;
    }

    public void release() {
        bucket.setPosition(Settings.Bucket.RELEASE_POSITION);
        positionIndex = 1;
    }

    @Override
    public void telemetry() {
        opMode.telemetry.addData("Bucket Position Index", positionIndex);
    }
}
