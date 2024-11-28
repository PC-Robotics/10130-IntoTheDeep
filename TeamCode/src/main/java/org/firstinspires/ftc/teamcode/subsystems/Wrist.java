package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.HardwareUtility.servoInit;

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
        wrist = servoInit(opMode.hardwareMap, "wrist", Servo.Direction.FORWARD);
    }

    public void start() {
        wrist.setPosition(Settings.Wrist.DRIVING_POSITION);
        positionIndex = Settings.Wrist.POSITIONS.indexOf(Settings.Wrist.DRIVING_POSITION);
    }

    public void moveTowardsRelease() {
        if (positionIndex < 2) {
            wrist.setPosition(Settings.Wrist.POSITIONS.get(positionIndex));
            positionIndex++;
        }
    }

    public void moveTowardsPickup() {
        if (positionIndex > 0) {
            wrist.setPosition(Settings.Wrist.POSITIONS.get(positionIndex));
            positionIndex--;
        }
    }
}
