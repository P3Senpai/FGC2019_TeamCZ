/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.ftccommon.FtcEventLoop;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.HashMap;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Testing: Linear OpMode", group="Linear Opmode")
//@Disabled
public class BasicOpMode_Linear extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private Bot robot = new Bot();
    private Toggle tgg = new Toggle();
    private static double speedLimit = 0.8;
    static int controllerId = 0;



    // servo set up
//    HashMap <Servo, Double> servoPosMap = new HashMap<>();
    Servo[] allServos = {robot.wingtipLeft, robot.wingtipRight, robot.pushBall, robot.tightenSide, robot.liftBrake, robot.shooterTrigger};
    static int rotation = 0;


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initializing");

        // initialisation of components found in Bot class
        robot.init(hardwareMap);

        // setup servo var
//        servoPosMap.putAll(intiServoMap(allServos));
        telemetry.addLine("To test the motors use Gamepad 1");
        telemetry.addLine("To test the servos use Gamepad 2");
        
        // Wait for the game to start (driver presses PLAY)
        telemetry.update();
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double sorterCurrentDistance = robot.distanceSensor.getDistance(DistanceUnit.CM);

            motorsTests(gamepad1);
            servoControllers(gamepad2);

            // if you hold the b button the intake will automatically work
            if(gamepad1.b){
//                autoPushBall(sorterCurrentDistance, );
                //todo find the target distance for the ball at the top and enter the parameter
            }


        // Show the elapsed game time and wheel power.
            telemetry.addLine();
            telemetry.addData("Status", "Run Time: %.1f" + runtime.toString());
            telemetry.addData("Distance sensor: %.2f", sorterCurrentDistance);
            telemetry.update();

        // counter needs to go to zero in order for the loop to work!!!
            tgg.reset();
        }

        /* Code the happens after the stop button */
        robot.leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        telemetry.addLine("Brake is activate");
        telemetry.update();
    }



    private void motorsTests(Gamepad gp) {

    // drive motors
        double leftPower = Range.clip(-gp.left_stick_y, -1.0, 1.0);
        double rightPower = Range.clip(-gp.right_stick_y, -1.0, 1.0);
        robot.leftDrive.setPower(leftPower * speedLimit);
        robot.rightDrive.setPower(rightPower * speedLimit);

    //shooter motors
        if (tgg.toggle(gp.dpad_up)) {
            speedLimit += (speedLimit < 1.0) ? 0.1 : 0; // increments by 0.1 if limit is under 1.0
        }
        if (tgg.toggle(gp.dpad_down)) {
            speedLimit -= (speedLimit > 0.1) ? 0.1 : 0; // increments by -0.1 if limit is above 0.1
        }

        // setting speed to shooter motor
        if (gp.x)
            robot.shooter.setPower(-speedLimit);
        else
            robot.shooter.setPower(0);
    // intake motors

        if (gp.right_bumper){
            robot.ziptieIntake.setPower(0.5);
            robot.beltIntake.setPower(0.5);
        }else if (gp.left_bumper){
            robot.ziptieIntake.setPower(-0.5);
            robot.beltIntake.setPower(-0.5);
        }else{
            robot.ziptieIntake.setPower(0);
            robot.beltIntake.setPower(0);
        }

    // lift motor
        if(gp.a) {
            robot.lift.setPower(-0.7);          // going down
        }else if(gp.y && robot.maxHeight.getState()) {
            robot.lift.setPower(0.7);           // going up
        }else {
            robot.lift.setPower(0);
        }

    // telemetry data
        telemetry.addData("Drive Speed", "left(%.2f) right(%.2f)", leftPower, rightPower);
        telemetry.addData("Shooter Speed", "limit(%.2f) actual(%.2f)", speedLimit, robot.shooter.getPower());
        telemetry.addData("Intake Motor Speed", "belt: %.2f ziptie intake: %.2f", robot.beltIntake.getPower(), robot.ziptieIntake.getPower());
        telemetry.addData("Lift Motor:", "power(%.2f)", robot.lift.getPower());
    }
    //todo delete after pull
    private void servosTests(Gamepad gp){
        // Sets index for servo array
        if (tgg.toggle(gp.dpad_right) && rotation < allServos.length){
            rotation ++;
        }else if(rotation >= allServos.length){
            rotation = 0;
        }

        if(tgg.toggle(gp.dpad_left) && rotation >= 0){
            rotation --;
        }else if(rotation < 0){
            rotation = allServos.length - 1;
        }

        //Gets servo and its position
        try {
            Servo servo = allServos[rotation];
            double servoPos = servo.getPosition();
            // Changes servo position
            if (tgg.toggle(gp.dpad_up))
                servoPos += 0.01;
            else if(tgg.toggle(gp.dpad_down))
                servoPos -= 0.01;
            servo.setPosition(servoPos);

            // telemetry data
            telemetry.addData("Servo: %s \nPosition: %.2f", servo.getDeviceName(), servoPos);
        }catch (NullPointerException e ){
            telemetry.clearAll();
            telemetry.addLine("Servo could not be found");
        }catch (ArrayIndexOutOfBoundsException e ){
            telemetry.clearAll();
            telemetry.addData("Servo pos could not be found", "%d", rotation);
        }
    }
    //  tests servos by using multiple controllers
    private void servoControllers(Gamepad gp){
    // loop setting the controller id
        if(tgg.toggle(gp.y)){
            if(controllerId == 0)
                controllerId = 1;
            else if(controllerId == 1)
                controllerId = 0;
        }
    //the controls going inside the statement
        if(controllerId == 0){
            double leftWingPos = robot.wingtipLeft.getPosition();
            double rightWingPos = robot.wingtipRight.getPosition();
            double pushPos  = robot.pushBall.getPosition();
        // left wing tip controls
            if(gp.dpad_left)
                robot.wingtipLeft.setPosition(leftWingPos + 0.0001);
            else if(gp.dpad_right)
                robot.wingtipLeft.setPosition(leftWingPos - 0.001);
        // right wing tip controls
            if(gp.x)
                robot.wingtipRight.setPosition(rightWingPos + 0.001);
            else if(gp.b)
                robot.wingtipRight.setPosition(rightWingPos - 0.001);
        // push ball into hopper controls
            if(gp.left_bumper)
                robot.pushBall.setPosition(pushPos + 0.001);
            else if(gp.right_bumper)
                robot.pushBall.setPosition(pushPos - 0.001);

        // instructions on how to use
            telemetry.addLine();
            telemetry.addData("Dpad left and right control the left wing tip", "Current Position: (%.4f)", leftWingPos);
            telemetry.addData("X and B buttons control the right wing tip","Current Position: (%.4f)", rightWingPos);
            telemetry.addData("Left and right bumpers control the servo pushing ball","Current Position: (%.4f)", pushPos);
        }else if(controllerId == 1){
            double tightenPos = robot.tightenSide.getPosition();
            double triggerPos = robot.shooterTrigger.getPosition();
            double brakePos   = robot.liftBrake.getPosition();
        // tight controls
            if(gp.dpad_left)
                robot.tightenSide.setPosition(tightenPos + 0.001);
            else if((gp.dpad_right))
                robot.tightenSide.setPosition(tightenPos - 0.001);
            //todo if this is continuous then the else statement is necessary
//            else
//                robot.wingtipLeft.setPosition(0);
        // trigger for shooter controls
            if(gp.x)
                robot.shooterTrigger.setPosition(triggerPos + 0.001);
            else if(gp.b)
                robot.shooterTrigger.setPosition(triggerPos - 0.001);
        // brake controls
            if(gp.left_bumper)
                robot.liftBrake.setPosition(brakePos + 0.001);
            else if(gp.right_bumper)
                robot.liftBrake.setPosition(brakePos - 0.001);

        // instructions on how to use
            telemetry.addLine();
            telemetry.addData("Dpad left and right control the tighten the tips", "Current Position: (%.4f)", tightenPos);
            telemetry.addData("X and B buttons control the shooter trigger", "Current Position: (%.4f)", triggerPos);
            telemetry.addData("Left and right bumpers control lift brake position", "Current Position: (%.4f)", brakePos);
        }
        telemetry.addLine();
        telemetry.addLine("Switch gamepad by pressing the Y button");
        telemetry.addData("Gamepad Id: ", "%d", controllerId);
    }
    private void autoPushBall(double distance, double targetDistance){
        double distanceToGroundCM = -1;      //todo find and set distance to ground
        if(distance <= targetDistance){
            robot.pushBall.setPosition(0.345); // pushes ball away
            // todo TEST if wait time is needed
            robot.pushBall.setPosition(0.16); // moves servo back
        }else if(distance < distanceToGroundCM){
            robot.beltIntake.setPower(0.5);
            robot.ziptieIntake.setPower(0.5);
        }
    }
    private void driveByAcceleration (double inputData, double maxPower, double velocityForward, double velocitySideways){
        // Set up variables
        maxPower = Math.abs(maxPower);
        // forward acceleration is used to stop drastic changes in motion
        double power = Range.clip(inputData, -maxPower, maxPower);
        double accel = Range.clip(velocityForward, -1 + maxPower, 1 - maxPower);
        // sideways acceleration is used to prevent the robot from drifting sideways
        double leftPower = (accel + power) - velocitySideways;
        double rightPower = (accel + power) + velocitySideways;
        // set motor power
        robot.leftDrive.setPower(leftPower);
        robot.rightDrive.setPower(rightPower);
    }
    private void turnByAcceleration (double power, int turn, double angularV){
        // use z axle as the axis of rotation (plus some shift)
    }
}
