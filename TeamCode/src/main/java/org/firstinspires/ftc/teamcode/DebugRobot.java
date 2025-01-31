package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.lang.reflect.InvocationTargetException;

public class DebugRobot extends Robot {
    public DebugRobot(LinearOpMode opMode) {
        super(opMode);
    }

    public void init() {
        super.init();
    }

    public void init(@NonNull String[] subsystems) throws ClassNotFoundException, NoSuchMethodException, InvocationTargetException, IllegalAccessException {
        for (String subsystem : subsystems) {
            Class.forName(subsystem).getDeclaredMethod("init").invoke(this);
        }
    }

    public void start(@NonNull String[] subsystems) throws ClassNotFoundException, NoSuchMethodException, InvocationTargetException, IllegalAccessException {
        for (String subsystem : subsystems) {
            Class.forName(subsystem).getDeclaredMethod("start").invoke(this);
        }
    }
}
