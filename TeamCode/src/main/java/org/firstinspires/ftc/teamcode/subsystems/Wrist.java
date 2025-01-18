package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.support.HardwareUtility.servoInit;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Settings;

public class Wrist {
    public LinearOpMode opMode;

    public Servo wrist;

    public int positionIndex = 0;

    public Wrist(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    public void init() {
        wrist = servoInit(opMode.hardwareMap, "wrist", Servo.Direction.REVERSE);
    }

    public void start() {
        moveToPositionIndex(1); // start at drive position
    }

    public void moveToPositionIndex(int index) {
        wrist.setPosition(Settings.Wrist.POSITIONS.get(index));
        positionIndex = index;
    }

    public void moveTowardsRelease() {
        if (positionIndex < 2) {
            moveToPositionIndex(positionIndex + 1);
        }
    }

    public void moveTowardsPickup() {
        if (positionIndex > 0) {
            moveToPositionIndex(positionIndex - 1);
        }
    }

    public void telemetry() {
        opMode.telemetry.addData("Wrist Position Index", positionIndex);
    }
}
