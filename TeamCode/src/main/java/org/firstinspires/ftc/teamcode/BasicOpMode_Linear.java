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
    private static int controllerId = 0;
    private boolean inProcess = false;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initializing");

        // initialisation of components found in Bot class
        robot.init(hardwareMap);

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


            // if you hold th b button the intake will automatically work
            autoPushBall(sorterCurrentDistance, 500); // 1 second is guess



        // Show the elapsed game time and wheel power.
            telemetry.addLine();
            telemetry.addData("Distance sensor: ","%.1f cm", sorterCurrentDistance);
            telemetry.addData("Status", "Run Time: %.1f", runtime.seconds());
            telemetry.addData("Triggers pressed"," left: %.2f right: %.2f", gamepad1.left_trigger, gamepad1.right_trigger);
            telemetry.update();


        // counter needs to go to zero in order for the toggle method to work!!!
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
//        double leftPower = Range.clip(-gp.left_stick_y, -1.0, 1.0);    // set y stick to normal and not inverted
//        double rightPower = Range.clip(-gp.right_stick_y, -1.0, 1.0);  // set y stick to normal and not inverted

        double drive = -gamepad1.left_stick_y;
        double turning  =  gamepad1.left_stick_x;
        double rightPower   = Range.clip(drive - turning, -1.0, 1.0) ;
        double leftPower    = Range.clip(drive + turning, -1.0, 1.0) ;

        robot.leftDrive.setPower(leftPower);
        robot.rightDrive.setPower(rightPower);
    //shooter motor
        if (tgg.toggle(gp.dpad_up)) {
            robot.speedLimit += (robot.speedLimit < 1.0) ? 0.1 : 0; // increments by 0.1 if limit is under 1.0
        }
        if (tgg.toggle(gp.dpad_down)) {
            robot.speedLimit -= (robot.speedLimit > 0.1) ? 0.1 : 0; // increments by -0.1 if limit is above 0.1
        }

        // setting speed to shooter motor
        if (gp.x)
            robot.shooter.setPower(-robot.speedLimit);
        else
            robot.shooter.setPower(0);
    // intake motors

        if (gp.right_bumper){
            robot.ziptieIntake.setPower(0.5);   // ball going in
            robot.beltIntake.setPower(0.5);
        }else if (gp.left_bumper){
            robot.ziptieIntake.setPower(-0.5);  // ball going out
            robot.beltIntake.setPower(-0.5);
        }else{
            robot.ziptieIntake.setPower(0);
            robot.beltIntake.setPower(0);
        }

    // lift motor
        if(gp.a ) { //&& robot.minHeight.getState()
            robot.lift.setPower(-0.7);          // going down
        }else if(gp.y && robot.maxHeight.getState()) {
            robot.lift.setPower(0.7);           // going up
        }else {
            robot.lift.setPower(0);
        }

    // telemetry data
        telemetry.addData("Drive Speed", "left(%.2f) right(%.2f)", leftPower, rightPower);
        telemetry.addData("Shooter Speed", "limit(%.2f) actual(%.2f)", robot.speedLimit, robot.shooter.getPower());
        telemetry.addData("Intake Motor Speed", "belt: %.2f ziptie intake: %.2f", robot.beltIntake.getPower(), robot.ziptieIntake.getPower());
        telemetry.addData("Lift Motor:", "power(%.2f)", robot.lift.getPower());
    }
// tests servos by using multiple virtual controllers
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
                robot.wingtipLeft.setPosition(robot.OPEN_LEFT_WING);
            else if(gp.dpad_right)
                robot.wingtipLeft.setPosition(robot.OPEN_LEFT_WING);
        // right wing tip controls
            if(gp.x)
                robot.wingtipRight.setPosition(robot.OPEN_RIGHT_WING);
            else if(gp.b)
                robot.wingtipRight.setPosition(robot.CLOSED_RIGHT_WING);
        // push ball into hopper controls
            if(gp.left_bumper)
                robot.pushBall.setPosition(robot.OPEN_PUSH_BALL);
            else if(gp.right_bumper)
                robot.pushBall.setPosition(robot.PUSHED_PUSH_BALL);

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
        // trigger for shooter controls
            if(gp.x)
                robot.shooterTrigger.setPosition(robot.FIRE_TRIGGER_SERVO);
            else if(gp.b)
                robot.shooterTrigger.setPosition(robot.LOAD_TRIGGER_SERVO);
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
// uses distance sensor to push balls into the hopper
    private void autoPushBall(double distance, long time){
        ElapsedTime timer = new ElapsedTime();
        if(distance <= robot.DISTANCE_TO_TOP_CM && !inProcess){  // second condition ensures servo can return
                //todo add counter for ball limit
            robot.pushBall.setPosition(robot.PUSHED_PUSH_BALL); // pushes ball into hopper
            inProcess = true;
        }else if(inProcess){
            sleep(time);
            robot.pushBall.setPosition(robot.OPEN_PUSH_BALL); // moves servo back
            inProcess = false;
        }else if(distance < robot.DISTANCE_TO_GROUND_CM && distance > robot.DISTANCE_TO_TOP_CM){   // moves intake if there is a ball in the belt rails
            robot.beltIntake.setPower(0.5);
            robot.ziptieIntake.setPower(0.5);
        }else{
            robot.beltIntake.setPower(0);
            robot.ziptieIntake.setPower(0);
        }
        telemetry.addData("Servo Timer: ", "%.1f", timer.seconds());
    }
    private void driveByVelocity (double inputData, double maxPower, double velocityForward, double velocitySideways){
    // Set up variables
        double power, leftPower, rightPower, forwardV, sidewaysV, threshold, powerPercentWeight, sidewaysPercentWeight;
        maxPower = Math.abs(maxPower);
        threshold = maxPower * 1.5;     // todo TEST if threshold value is good (since it is random)
        powerPercentWeight = 0.95;      // todo TEST if weight strong enough
        sidewaysPercentWeight = 1 - powerPercentWeight;
    // Set range for values
        power = Range.clip(inputData, -maxPower, maxPower);
        forwardV = Range.clip(velocityForward, -maxPower, maxPower);
        sidewaysV = Range.clip(velocitySideways, -maxPower*sidewaysPercentWeight, maxPower*sidewaysPercentWeight);
    // limit power if predicted drastic changes in magnitude
        if (Math.abs(forwardV - power) > threshold){
            power *= 0.3;
        }else{
            power *= powerPercentWeight;
        }
        // positive axis is left and negative is right
        leftPower = power + sidewaysV;  // sideways velocity prevents drifting
        rightPower = power - sidewaysV; // sideways velocity prevents drifting
    // Set motor speeds
        robot.leftDrive.setPower(leftPower);
        robot.rightDrive.setPower(rightPower);
    }
// uses x and y velocity to adjust turning in order to center it
    private void turnByVelocity (double inputData, double maxPower, int turn, double xAxisV, double yAxisV){
    // Define key variables
        double power, errorX, errorY, leftPower, rightPower, powerSignificance, errorSignificance, error, errorMin, errorMax;
        maxPower = Math.abs(maxPower);
        powerSignificance = 0.9;    // % weight on the power value
        errorSignificance = (1 - powerSignificance);  // % weight on error value
        errorMin = (-maxPower * errorSignificance)/2;   // halves weight so that zero is center
        errorMax = (maxPower * errorSignificance)/2;    // halves weight so that zero is center
    // Set value ranges
        power = Range.clip(inputData, 0, maxPower * powerSignificance);
        errorX = Range.clip(xAxisV, errorMin, errorMax);
        errorY = Range.clip(yAxisV, errorMin, errorMax);
    // main calculation that centers robot while turning
        error = errorX + errorY*turn;
        leftPower = power - error;  // as if error > 0 then leftPowerOutput > rightPowerOutput
        rightPower = power + error; // as if error < 0 then leftPowerOutput < rightPowerOutput
    // set drive power
        robot.leftDrive.setPower(-leftPower * turn);    // turns left on default
        robot.rightDrive.setPower(rightPower * turn);
    }
}
