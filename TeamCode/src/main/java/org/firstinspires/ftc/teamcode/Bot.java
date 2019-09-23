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
    // Motors
    protected DcMotorEx leftDrive   = null; // todo test extended version of motor on this one
    protected DcMotor rightDrive  = null;
    protected DcMotor lift        = null;
    protected DcMotor shooter     = null;
    protected DcMotor ziptieIntake = null;
    protected DcMotor beltIntake   = null;
    // Servos
    protected Servo   wingtipLeft = null;
    protected Servo   wingtipRight = null;
    protected Servo   liftBrake    = null;
    protected Servo   tightenSide  = null;
    protected Servo   pushBall     = null;
    protected Servo   shooterTrigger = null;
    // Sensors
    protected DigitalChannel maxHeight = null;
    // Constants
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
        shooter    = hwmap.get(DcMotor.class, "shooter");
        ziptieIntake = hwmap.get(DcMotor.class, "intake");
        beltIntake   = hwmap.get(DcMotor.class, "belt_intake");

        leftDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        rightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        lift.setDirection(DcMotorSimple.Direction.REVERSE);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // works partially (so is a fail safe)

        leftDrive.setPower(0);
        rightDrive.setPower(0);
        lift.setPower(0);
        shooter.setPower(0);
        ziptieIntake.setPower(0);
        beltIntake.setPower(0);

        /*  Initialization of Servos */
        wingtipLeft = hwmap.get(Servo.class, "wing_tip_left");      // find and set starting pos
        wingtipRight = hwmap.get(Servo.class, "wing_tip_right");    // find and set starting pos
        liftBrake   = hwmap.get(Servo.class, "lift_brake");         // find and set starting pos
        tightenSide = hwmap.get(Servo.class, "tighten_side");       // find and set starting pos
        pushBall    = hwmap.get(Servo.class, "push_ball");          // find and set starting pos
        shooterTrigger    = hwmap.get(Servo.class, "shooter");          // find and set starting pos  //todo get config name

        /* Initialization of Sensors*/
        maxHeight = hwmap.get(DigitalChannel.class, "limit_switch");
        maxHeight.setMode(DigitalChannel.Mode.INPUT);

    }
    public double shooterPID(double currentPower, double targetPower){
        double output;
        double P,I,D = 1;
        double error = targetPower - currentPower;

        return 0.0;
    }
}

