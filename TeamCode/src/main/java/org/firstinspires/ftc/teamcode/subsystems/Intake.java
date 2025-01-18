package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.support.HardwareUtility.CRServoInit;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.Settings;

public class Intake implements Subsystem {
    private LinearOpMode opMode;

    public CRServo intake;

    public Intake(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    @Override
    public void init() {
        intake = CRServoInit(opMode.hardwareMap, "intake", CRServo.Direction.FORWARD);
    }

    @Override
    public void start() {
    }

    public void intake() {
        intake.setPower(Settings.Intake.MAX_POWER);
    }

    public void intake(double power) {
        intake.setPower(power);
    }

    public void outtake() {
        intake.setPower(-Settings.Intake.MAX_POWER);
    }

    public void outtake(double power) {
        intake.setPower(-power);
    }

    public void stop() {
        intake.setPower(0);
    }

    @Override
    public void telemetry() {
        opMode.telemetry.addData("Intake Moving", intake.getPower() != 0);
    }
}
