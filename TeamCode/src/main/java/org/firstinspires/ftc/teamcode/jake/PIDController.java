package org.firstinspires.ftc.teamcode.jake;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class PIDController {
    double lastOutput;
    double Kp;
    double Ki;
    double Kd;

    double integralSum;
    double lastError;

    double accelLimit;
    double defaultOutputLimit;
    double liveOutputLimit;
    double setPoint;
    double tolerance;
    double deadband;
    boolean circular;
    boolean inPosition;
    ElapsedTime cycleTime = new ElapsedTime();

    public PIDController(double Kp, double Ki, double Kd, double accelLimit, double outputLimit, double tolerance, double deadband, boolean circular)
    {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
        this.accelLimit = accelLimit;
        this.defaultOutputLimit = outputLimit;
        this.liveOutputLimit = outputLimit;
        this.tolerance = tolerance;
        this.deadband = deadband;
        this.circular = circular;
        reset(0.0);
    }

    public double getOutput(double input)
    {

        double error = setPoint - input;
        double dV = cycleTime.seconds() * accelLimit;
        double output;

        if(circular)
        {
            while (error > Math.PI) error -= 2*Math.PI;
            while (error <= -Math.PI) error += 2*Math.PI;
        }

        inPosition = (Math.abs(error) < tolerance);


        if(Math.abs(error) <= deadband)
        {
            output = 0;
        }

        else
        {
            // Only begin using integral component when the robot gets close
            // This helps dampen overshooting
            if(Math.abs(error) < 0.5) integralSum += error * cycleTime.seconds();
            double derivative = (error - lastError)/cycleTime.seconds();
            lastError = error;

            output = (error*Kp) + (derivative*Kd) + (integralSum*Ki);
            output = Range.clip(output, -liveOutputLimit,liveOutputLimit);

            if((output-lastOutput) > dV) output = lastOutput + dV;
            else if((output-lastOutput) < dV) output = lastOutput - dV;

        }
        lastOutput = output;
        cycleTime.reset();
        return output;

    }

    public boolean inPosition()
    {
        return inPosition;
    }

    public double getSetpoint()
    {
        return setPoint;
    }

    public void reset(double setPoint, double powerLimit)
    {
        liveOutputLimit = Math.abs(powerLimit);
        this.setPoint = setPoint;
        reset();
    }
    public void reset(double setPoint)
    {
        liveOutputLimit = defaultOutputLimit;
        this.setPoint = setPoint;
        reset();
    }
    public void reset()
    {
        cycleTime.reset();
        inPosition = false;
        lastOutput = 0.0;
        lastError = 0.0;
        integralSum = 0.0;
    }

}
