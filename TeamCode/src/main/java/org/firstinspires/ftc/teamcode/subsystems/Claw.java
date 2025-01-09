package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.HardwareUtility.servoInit;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Settings;

public class Claw {
    private LinearOpMode opMode;

    public Servo claw;

    public int positionIndex = 0;

    public Claw(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    public void init() {
        claw = servoInit(opMode.hardwareMap, "claw", Servo.Direction.FORWARD);
    }

    public void start() {
        close();
    }

    public void open() {
        claw.setPosition(Settings.Claw.OPEN_POSITION);
        positionIndex = 1;
    }

    public void close() {
        claw.setPosition(Settings.Claw.CLOSED_POSITION);
        positionIndex = 0;
    }

    public void telemetry() {
        opMode.telemetry.addData("Claw Position Index", positionIndex);
    }
}
