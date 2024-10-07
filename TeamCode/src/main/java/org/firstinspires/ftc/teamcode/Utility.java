package org.firstinspires.ftc.teamcode;

public final class Utility {
    private Utility() {
        throw new java.lang.UnsupportedOperationException(
                "This is a utility class and cannot be instantiated"
        );
    }

    /**
     * Normalize each value in the powers array if any of the values are greater than 1.
     * This makes sure that the motors won't receive a |value| > 1.0
     * @param powers the array powers for all motors
     * @return the powers array but normalized to a range of [-1.0, 1.0]
     */
    public static double[] normalizePowers(double[] powers) {
        // no need for check for 0 length array since length is given
        // mind max
        double max = Math.abs(powers[0]);
        for (int i = 1; i < powers.length; i++) {
            max = Math.max(max, Math.abs(powers[i]));
        }

        // normalize values to range [0, 1]
        if (max > 1.0) {
            for (int i = 0; i < powers.length; i++) {
                powers[i] /= max;
            }
        }
        return powers;
    }

    /**
     * apply a deadzone to an input.
     * If the input is between [-DEADZONE_THRESHOLD, THRESHOLD], return 0.
     * @param padValue the value of the joystick input
     * @param DEADZONE_THRESHOLD the deadzone that the pad value will be filtered by
     * @return the joystick value with the deadzone filter applied
     */
    public static double applyDeadzone(double padValue, double DEADZONE_THRESHOLD) {
        if (Math.abs(padValue) > DEADZONE_THRESHOLD) {
            return padValue;
        } else {
            return 0.0;
        }
    }
}