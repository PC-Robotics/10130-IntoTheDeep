package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.HardwareUtility.servoInit;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Settings;

public class Bucket {
    private LinearOpMode opMode;

    public Servo bucket;

    public int positionIndex = 0;

    public Bucket(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    public void init() {
        bucket = servoInit(opMode.hardwareMap, "bucket", Servo.Direction.FORWARD);
    }

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

    public void telemetry() {
        opMode.telemetry.addData("Bucket Position Index", positionIndex);
    }
}
