package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;

/**
 * Created by petr-konstantin on 6/24/19.
 */

public class Toggle {
    private boolean button; // this variable also
    private boolean state = false; // starting state of every controller


    /* Constructor */
    public void Toggle(){}

    public boolean toggle(boolean button){
        // this will only work in a while loop or as it is run asynchronously
        if (button && !state){
            // double conditions
            state = true;
        }else if(!button && state){
            state = false;
        }
        return state;
    }
}
