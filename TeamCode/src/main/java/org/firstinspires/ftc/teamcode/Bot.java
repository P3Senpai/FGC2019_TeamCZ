package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Created by petr-konstantin on 6/24/19.
 */

public class Bot {
    HardwareMap hwmap = null;
    // Motors
    protected DcMotorEx shooter = null; // todo test extended version of motor on this one
    protected DcMotor leftDrive, rightDrive, lift, ziptieIntake, beltIntake, leftWing, rightWing = null;
    // Servos
    protected Servo  liftBrake, tightenSide, pushBall, shooterTrigger = null;
    // Sensors
    protected DigitalChannel maxHeight, minHeight = null;
    protected Rev2mDistanceSensor distanceSensor = null;
    // todo add camera

    // Key Variables
    protected double shooterSpeedLimit = 0.7;
    protected int ballHopperQuantity = 0; //todo
    // Constants
    protected final double SPEED_LIMIT = 0.6;
    protected final double BOOST_SPEED_LIMIT = 1;
    protected final double DISTANCE_TO_TOP_CM = 2.5;
    protected final double DISTANCE_TO_GROUND_CM = 34;
    protected final double INTAKE_SPEED = 0.5;
    // Servo positions
    protected final double LOAD_TRIGGER_SERVO =  0.4078;
    protected final double FIRE_TRIGGER_SERVO =  0.1139;
    protected final double OPEN_PUSH_BALL     =  0.09 ;
    protected final double PUSHED_PUSH_BALL   =  0.345;

    /* Constructor */
    public void Bot(){}

    /* Initialize hardware */
    protected void init(HardwareMap ahwmap){
        /* Local Initialization of HardwareMap */
        hwmap = ahwmap;

    /*  Initialization of Motors */
        leftDrive = hwmap.get(DcMotor.class,"left_drive");
        rightDrive = hwmap.get(DcMotor.class, "right_drive");
        lift       = hwmap.get(DcMotor.class, "lift");
        shooter    = hwmap.get(DcMotorEx.class, "shooter");
        ziptieIntake = hwmap.get(DcMotor.class, "intake");
        beltIntake   = hwmap.get(DcMotor.class, "belt_intake");
        leftWing    = hwmap.get(DcMotor.class, "left_wing");
        rightWing   = hwmap.get(DcMotor.class, "right_wing");

    // Left and Right wings
//        leftWing.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        rightWing.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        leftWing.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rightWing.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftWing.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightWing.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    // Shooter motor
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    // Left Drive
        leftDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);    // release brake after end of program
    // Right Drive
        rightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);   // release brake after end of program
    // Lift Motor
        lift.setDirection(DcMotorSimple.Direction.FORWARD);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);         // redundancy for lift brake system

    // Both Intake
        beltIntake.setDirection(DcMotorSimple.Direction.REVERSE);   // this changes positive numbers to intake and negative to release
        ziptieIntake.setDirection(DcMotorSimple.Direction.REVERSE);

    // Zero Power
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        lift.setPower(0);
        shooter.setPower(0);
        ziptieIntake.setPower(0);
        beltIntake.setPower(0);
        leftWing.setPower(0);
        rightWing.setPower(0);

    /*  Initialization of Servos */
        liftBrake   = hwmap.get(Servo.class, "lift_brake");
        tightenSide = hwmap.get(Servo.class, "tighten_side");
        pushBall    = hwmap.get(Servo.class, "push_ball");
        shooterTrigger    = hwmap.get(Servo.class, "shooter");

        pushBall.setPosition(OPEN_PUSH_BALL);
        shooterTrigger.setPosition(FIRE_TRIGGER_SERVO); // close the shooter for the balls before we are ready to shoot

    /* Initialization of Sensors*/
        maxHeight = hwmap.get(DigitalChannel.class, "max_height ");
        maxHeight.setMode(DigitalChannel.Mode.INPUT);
//        minHeight = hwmap.get(DigitalChannel.class, "min_height");
//        minHeight.setMode(DigitalChannel.Mode.INPUT); //todo mount the min height to prevent chain from loosing
        distanceSensor = hwmap.get(Rev2mDistanceSensor.class, "distance");

    }
    protected boolean percentTolerance(double value, double threshold, double percent){
        double maxLimit = threshold + (threshold*percent);
        double minLimit = threshold - (threshold*percent);
        return value <= maxLimit && value >= minLimit;
    }
}

