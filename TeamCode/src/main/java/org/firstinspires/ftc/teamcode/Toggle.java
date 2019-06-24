package org.firstinspires.ftc.teamcode;

/**
 * Created by petr-konstantin on 6/24/19.
 */

public class Toggle {
    private boolean button; // this variable also
    private boolean pressed = false; // starting state of every controller
    private String str = "afasfd";

    /* Constructor */
    public void Toggle( boolean button){
        this.button = button;
    }

    public void toggle(){
        // this will only work in a while loop or as it is run asynchronously
        if (button && !pressed){
            // double conditions

            pressed = true;
        }else if(!button && pressed){
            pressed = false;
        }
    }
}
