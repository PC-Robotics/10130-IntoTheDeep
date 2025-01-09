package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.adafruit.AdafruitBNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class OurIMU {
    private LinearOpMode opMode;

    public IMU imu;

    public OurIMU(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    public void init() {
        imu = opMode.hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.RIGHT // its backwards
                )
        );
    }

    // get an object containing euler angles
    public YawPitchRollAngles getEulerAngles() {
        return imu.getRobotYawPitchRollAngles();
    }

    public double getHeading() {
        return getHeading(AngleUnit.RADIANS);
    }

    public double getHeading(AngleUnit unit) {
        return imu.getRobotYawPitchRollAngles().getYaw(unit);
    }

    public void resetYaw() {
        imu.resetYaw();
    }

    public void telemetry() {
        opMode.telemetry.addData("Heading", getHeading());
    }
}
