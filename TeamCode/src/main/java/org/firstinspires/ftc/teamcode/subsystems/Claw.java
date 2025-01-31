package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.HardwareUtility.servoInit;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantFunction;
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

    public void setPosition(double position) {
        claw.setPosition(position);
    }

    public void telemetry() {
        opMode.telemetry.addData("Claw Position Index", positionIndex);
    }

    public class OpenClawAction implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            open();
            return false;
        }
    }

    public Action openClaw() {
        return new OpenClawAction();
    }

    public class CloseClawAction implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            close();
            return false;
        }
    }

    public Action closeClaw() {
        return new CloseClawAction();
    }
}
