package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

/**
 * Created by petr-konstantin on 6/24/19.
 */

public class Bot {
    /* Protected Op Mode variables */
    protected DcMotorEx leftDrive   = null; // todo test extended version of motor on this one
    protected DcMotor rightDrive  = null;
    protected DcMotor lift        = null;
    protected Servo   wingtipLeft = null;
    protected Servo   wingtipRight = null;
    protected DigitalChannel liftLimitSwitch = null;
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
        leftDrive = hwmap.get(DcMotorEx.class,"left_drive");
        rightDrive = hwmap.get(DcMotor.class, "right_drive");
        lift       = hwmap.get(DcMotor.class, "lift");

        leftDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        rightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        lift.setDirection(DcMotorSimple.Direction.REVERSE);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // works partially (so is a fail safe)

        /*  Initialization of Servos */
        wingtipLeft = hwmap.get(Servo.class, "wing_tip_left");
        wingtipRight = hwmap.get(Servo.class, "wing_tip_right");

        /* Initialization of Sensors*/
        liftLimitSwitch = hwmap.get(DigitalChannel.class, "limit_switch");
        liftLimitSwitch.setMode(DigitalChannel.Mode.INPUT);

    }
    public double shooterPID(double currentPower, double targetPower){
        double output;
        double P,I,D = 1;
        double error = targetPower - currentPower;

        return 0.0;
    }
}

