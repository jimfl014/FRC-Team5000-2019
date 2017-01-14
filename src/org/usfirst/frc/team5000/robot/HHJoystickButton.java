
package org.usfirst.frc.team5000.robot;

import edu.wpi.first.wpilibj.Joystick;

public class HHJoystickButton {

    Joystick joystick;
    int button;
    HHJoystickButtonState state;
    boolean isReleased;
    boolean isPressed;
    
    public HHJoystickButton( Joystick joystick, int button ) {

        this.joystick = joystick;
        this.button = button;
        this.state = HHJoystickButtonState.Released;
        this.isReleased = true;
        this.isPressed = false;
    }

    public void updateState() {

        HHJoystickButtonState oldState = state;

        boolean buttonIsPressed = joystick.getRawButton( button );

        state = buttonIsPressed ? HHJoystickButtonState.Pressed : HHJoystickButtonState.Released;

        if( oldState == HHJoystickButtonState.Released && state == HHJoystickButtonState.Pressed ) {
            isPressed = true;
        }
        else if( oldState == HHJoystickButtonState.Pressed && state == HHJoystickButtonState.Released ) {
            isReleased = true;
        }

    }
    
    public HHJoystickButtonState getState() {

        return state;
    }

    public boolean isPressed() {
        boolean currentIsPressed = isPressed;
        isPressed = false;
        return currentIsPressed;
    }

    public boolean isReleased() {
        boolean currentIsReleased = isReleased;
        isReleased = false;
        return currentIsReleased;
    }
}
