package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.HardwareUtility.servoInit;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Settings;

public class Trolley {
    private LinearOpMode opMode;

    public Servo leftTrolley;
    public Servo rightTrolley;

    public Trolley(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    public void init() {
        leftTrolley = servoInit(opMode.hardwareMap, "leftTrolley", Servo.Direction.FORWARD);
        rightTrolley = servoInit(opMode.hardwareMap, "rightTrolley", Servo.Direction.FORWARD);
    }

    public void start() {
        moveToInnerPosition();
    }

    public void setPosition(double position) {
        leftTrolley.setPosition(position);
        rightTrolley.setPosition(position);
    }

    public double getPosition() {
        return (leftTrolley.getPosition() + rightTrolley.getPosition()) / 2;
    }

    public void moveToInnerPosition() {
        setPosition(Settings.Trolley.IN_POSITION);
    }

    public void moveToOuterPosition() {
        setPosition(Settings.Trolley.OUT_POSITION);
    }
}