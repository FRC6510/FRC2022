package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Button;

public class xboxTrigger extends Button{
    XboxController joystick;
    int axis;

    public xboxTrigger(XboxController joystick, int axis) {
        this.joystick = joystick;
        this.axis = axis;
    }

    public double getTriggerValue() {
        return joystick.getRawAxis(axis);
    }

    @Override
    public boolean get() {
        return joystick.getRawAxis(axis) > 0.5;
    }
}