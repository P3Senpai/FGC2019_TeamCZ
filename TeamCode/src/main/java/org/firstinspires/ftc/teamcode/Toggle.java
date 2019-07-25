package org.firstinspires.ftc.teamcode;

import java.util.ArrayList;

/**
 * Created by petr-konstantin on 6/24/19.
 */

class Toggle{
    ArrayList<Boolean> previousState = new ArrayList<Boolean>();
    ArrayList<Boolean> automaticTogglePast = new ArrayList<Boolean>();
    static int counter;

    /* Constructor */
    public void Toggle(){
        counter = 0;
    }

    /* Main Method
    *       doesn't allow for holding down a button to count as multiple presses
    */

    // arrayPosition parameter should be entered in incrementing order from 0 as used in your code
    public boolean toggle(boolean button, int arrayPosition){
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

    // description of the functionality of the automatic toggle
    /* This toggle method works by automatically assigning a index to the method call inside your code.
    * For this toggle method to work it needs to be used in a loop which is always reset in the end,
    * otherwise you will not be able to read/write to previously stored items. If you want to do a toggle for
    * changing out put such as moving a servo from max to a min nest these conditional statements inside of a if
    * statement calling the toggle method.
    *
    * e.g.
    * loop
    *
    *   if method call 1
    *       // your code
    *   end if
    *
    *   if method call 2
    *       if condition1
    *           // your code
    *       end if
    *       else if condition 2
    *           // your code
    *       end else if
    *   end if
    *
    *   reset
    * end loop
    *
    * endregion
    */
    public boolean toggle(boolean button){
        boolean lastPress;
        boolean result = false;

        try{
            lastPress = automaticTogglePast.get(counter);
        }catch(IndexOutOfBoundsException e){
            automaticTogglePast.add(counter, button);
            lastPress = automaticTogglePast.get(counter);
        }

        if(button && !lastPress){
            automaticTogglePast.remove(counter);
            automaticTogglePast.add(counter, button);
            result = true;
        }else if(!button && lastPress){
            // does not need to change result since default is false
            automaticTogglePast.remove(counter);
            automaticTogglePast.add(counter, button);
        }
        counter++; // consider moving this under each if statement
        return result;
    }
    // this is used with the automatic toggle method
    public void reset(){
        counter = 0;
        System.out.println("Counter has been reset");
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
