package org.firstinspires.ftc.teamcode.helpers;

public class Toggle {
    private boolean state = false;
    private boolean prevInput = false;

    public Toggle() {}

    public Toggle(boolean initialState) {
        this.state = initialState;
    }

    public boolean update(boolean pressed) {
        if (pressed && !prevInput) {
            state = !state;
        }
        prevInput = pressed;
        return state;
    }
    //for triggers
    public boolean updateTrigger(double triggerValue, double threshold) {
        return update(triggerValue > threshold);
    }

    public boolean momentary(boolean pressed) {
        boolean fired = pressed && !prevInput;
        prevInput = pressed;
        return fired;
    }

    public void set(boolean newState) {
        state = newState;
    }

    public void reset() {
        state = false;
    }

    public boolean get() {
        return state;
    }
}