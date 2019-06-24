package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by petr-konstantin on 6/24/19.
 */

public class Bot {
    /* Protected Op Mode variables */
    protected DcMotor leftDrive   = null;
    protected DcMotor rightDrive  = null;
    protected DcMotor lift        = null;
    protected Servo   openLeft    = null;
    protected Servo   openRight   = null;
    protected Servo   wingtipLeft = null;
    protected Servo   wingtipRight = null;
    protected double  speedLimit   = 0.8;



    /* Local Op Mode variables */
    // something

    /* Constructor */
    public void Bot(){}

    /* Initialize hardware */
    public void init(){
        /* Local Initialization of HardwareMap */
        HardwareMap hwmap = null;

        /*  Initialization of Motors */
        leftDrive = hwmap.get(DcMotor.class,"left_drive");
        rightDrive = hwmap.get(DcMotor.class, "right_drive");
        lift       = hwmap.get(DcMotor.class, "lift");

        leftDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        rightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        lift.setDirection(DcMotorSimple.Direction.REVERSE);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // works partially (so is a fail safe)

        /*  Initialization of Servos */
        openLeft    = hwmap.get(Servo.class, "open_left");
        openRight   = hwmap.get(Servo.class, "open_right");
        wingtipLeft = hwmap.get(Servo.class, "wing_tip_left");
        wingtipRight = hwmap.get(Servo.class, "wing_tip_right");

        /*  Setting up robot for match */
        openLeft.setPosition(0.7);  // open position is 0.3
        openRight.setPosition(0.3); // open position is 0.7

    }
}

