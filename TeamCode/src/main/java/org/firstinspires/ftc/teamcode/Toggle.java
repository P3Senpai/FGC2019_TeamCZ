package org.firstinspires.ftc.teamcode;

import java.util.ArrayList;

/**
 * Created by petr-konstantin on 6/24/19.
 */

class Toggle{
    ArrayList<Boolean> previousState = new ArrayList<Boolean>();

    /* Constructor */
    public void Toggle(){}

    /* Main Method
    *       doesn't allow for holding down a button to count as multiple presses
    */

    // arrayPosition parameter should be entered in incrementing order from 0 as used in your code
    public boolean toggle(int arrayPosition, boolean button){
        boolean previousPressed;
        boolean action = false;

        try {
            previousPressed = getState(arrayPosition); // Checks if button with this array position already exists
        }catch (IndexOutOfBoundsException e){
            setState(arrayPosition,button);     // If it doesn't then it is created
            previousPressed = getState(arrayPosition);
        }

        // Main Logic
        if (button && !previousPressed){
            setState(arrayPosition,true);
            action = true;
        }else if(!button && previousPressed){
            setState(arrayPosition,false);
        }
        return action;
    }

    private boolean getState(int arrayPosition){
        return previousState.get(arrayPosition);
    }
    private void setState(int arrayPosition, boolean state){
        previousState.add(arrayPosition,state);
    }
}
/* EXAMPLE
*
* if(obj.toggle(0, gamepad1.a)
* {
* // your code
* }
*
*/
