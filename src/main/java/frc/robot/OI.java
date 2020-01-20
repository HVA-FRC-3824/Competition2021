package frc.robot;

import edu.wpi.first.wpilibj.Joystick;

public class OI {

    /**
     * Declare all joysticks and buttons here.
     */
    public static Joystick driverJoystick;

    public OI() {
        /**
         * Instantiate declared joysticks and buttons here.
         */
        driverJoystick = new Joystick(0);
    }

    /**
     * Allows use of driverJoystick object outside of OI class.
     * @return access to driverJoystick values/attributes.
     */
    public static Joystick getDriverJoystick() {
        return driverJoystick;
    }

}