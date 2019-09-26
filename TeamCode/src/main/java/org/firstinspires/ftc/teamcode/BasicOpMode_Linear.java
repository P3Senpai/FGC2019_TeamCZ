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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

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

@TeleOp(name="Basic: Linear OpMode", group="Linear Opmode")
//@Disabled
public class BasicOpMode_Linear extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private Bot robot = new Bot();
    private Toggle tgg = new Toggle();
    private static double speedLimit = 0.8;
    private boolean plusIntakePower = false;



    // servo set up
//    HashMap <Servo, Double> servoPosMap = new HashMap<>();
    Servo[] allServos = {robot.wingtipLeft, robot.wingtipRight, robot.pushBall, robot.tightenSide, robot.liftBrake, robot.shooterTrigger};
    static int rotation = 0;


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initializing");
        telemetry.update();

        // initialisation of components found in Bot class
        robot.init(hardwareMap);
//        initServoMap(allServos);

        // setup servo var
//        servoPosMap.putAll(intiServoMap(allServos));
        telemetry.addLine("To test the motors use Gamepad 1");
        telemetry.addLine("To test the servos use Gamepad 2");
        
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (true) {


            motorsTests(gamepad1);
            servoControllers(gamepad2);


        // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();

        // counter needs to go to zero in order for the loop to work!!!
            tgg.reset();
        if(!opModeIsActive()){break;}
        }
    }

    private void motorsTests(Gamepad gp) {

    // drive motors
        double leftPower = Range.clip(-gp.left_stick_y, -1.0, 1.0);
        double rightPower = Range.clip(-gp.right_stick_y, -1.0, 1.0);
        robot.leftDrive.setPower(leftPower);
        robot.rightDrive.setPower(rightPower);

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
        telemetry.addLine((plusIntakePower)?"Intake power is positive":"Intake power is negative");
        telemetry.addData("Lift Motor:", "power(%.2f)", robot.lift.getPower());
    }
    // todo null pointer exception found on line 176 as suggested by the ide
//    private void servosTests(Gamepad gp){
//    // Sets index for servo array
//        if (tgg.toggle(gp.dpad_up) && rotation < allServos.length){
//            rotation ++;
//        }else if(rotation >= allServos.length){
//            rotation = 0;
//        }
//
//        if(tgg.toggle(gp.dpad_down) && rotation > 0){
//            rotation --;
//        }else if(rotation <= 0){
//            rotation = allServos.length - 1;
//        }
//
//    //Gets servo and its position
//        Servo servo = allServos[rotation];
//        double servoPos;
//        try {
//            servoPos = servoPosMap.get(allServos[rotation]);
//            // Changes servo position
//            if (tgg.toggle(gp.y))
//                servoPos += 0.01;
//            else if(tgg.toggle(gp.a))
//                servoPos -= 0.01;
//            servo.setPosition(servoPos);
//            // Saves new position
//            servoPosMap.put(servo, servoPos);
//
//            // telemetry data
//            telemetry.addData("Servo: %s \nPosition: %.2f", servo.getDeviceName(), servoPos);
//        }catch (NullPointerException e){
//            telemetry.clearAll();
//            telemetry.addLine("Servo pos could not be found");
//            telemetry.update();
//        }
//    }
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
        int controllerId = gp.getGamepadId();

    // loop setting the controller id
        if(tgg.toggle(gp.a)){
            if(controllerId == 0)
                gp.setGamepadId(1);
            else if(controllerId == 1)
                gp.setGamepadId(0);
        }
    //the controls going inside the statement
        if(controllerId == 0){
            double leftWingPos = robot.wingtipLeft.getPosition();
            double rightWingPos = robot.wingtipRight.getPosition();
            double pushPos  = robot.pushBall.getPosition();
        // left wing tip controls
            if(tgg.toggle(gp.dpad_left))
                robot.wingtipLeft.setPosition(leftWingPos + 0.01);
            else if(tgg.toggle(gp.dpad_right))
                robot.wingtipLeft.setPosition(leftWingPos - 0.01);
        // right wing tip controls
            if(tgg.toggle(gp.x))
                robot.wingtipRight.setPosition(rightWingPos + 0.01);
            else if(tgg.toggle(gp.b))
                robot.wingtipRight.setPosition(rightWingPos - 0.01);
        // push ball into hopper controls
            if(tgg.toggle(gp.left_bumper))
                robot.wingtipLeft.setPosition(pushPos + 0.01);
            else if(tgg.toggle(gp.right_bumper))
                robot.wingtipLeft.setPosition(pushPos - 0.01);

        // instructions on how to use
            telemetry.addLine("Dpad left and right control the left wing tip");
            telemetry.addLine("X and B buttons control the right wing tip");
            telemetry.addLine("Left and right bumpers control the servo pushing ball");
        }else if(controllerId == 1){
            double tightenPos = robot.tightenSide.getPosition();
            double triggerPos = robot.shooterTrigger.getPosition();
            double brakePos   = robot.liftBrake.getPosition();
        // tight controls
            if(tgg.toggle(gp.dpad_left))
                robot.wingtipLeft.setPosition(tightenPos + 0.01);
            else if(tgg.toggle(gp.dpad_right))
                robot.wingtipLeft.setPosition(tightenPos - 0.01);
            //todo if this is continuous then the else statement is necessary
//            else
//                robot.wingtipLeft.setPosition(0);
        // trigger for shooter controls
            if(tgg.toggle(gp.x))
                robot.wingtipRight.setPosition(triggerPos + 0.01);
            else if(tgg.toggle(gp.b))
                robot.wingtipRight.setPosition(triggerPos - 0.01);
        // brake controls
            if(tgg.toggle(gp.left_bumper))
                robot.wingtipLeft.setPosition(brakePos + 0.01);
            else if(tgg.toggle(gp.right_bumper))
                robot.wingtipLeft.setPosition(brakePos - 0.01);

        // instructions on how to use
            telemetry.addLine("Dpad left and right control the tighten the tips");
            telemetry.addLine("X and B buttons control the shooter trigger");
            telemetry.addLine("Left and right bumpers control lift brake position");
        }
        telemetry.addLine("Switch gamepad by pressing the A button");
        telemetry.addData("Gamepad Id: ", "%s", controllerId);
    }

//    private HashMap <Servo, Double> initServoMap(Servo[] servos){
//        HashMap<Servo,Double> out = new HashMap<>();
//        for(Servo s : servos){
//            out.put(s, s.getPosition());
//        }
//        return out;
//    }
}
