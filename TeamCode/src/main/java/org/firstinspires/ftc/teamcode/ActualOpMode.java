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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


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

@TeleOp(name="Actual OpMode - Linear", group="Linear Opmode")
//@Disabled
public class ActualOpMode extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private Bot robot = new Bot();
    private Toggle tgg = new Toggle();
    private boolean inProcess = false;
    private double sorterCurrentDistance ;

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

            playerMarek(gamepad1);
            playerHussain(gamepad2);

        // Show the elapsed game time and wheel power.
            telemetry.addLine();
            telemetry.addData("Distance sensor: ","%.1f cm", sorterCurrentDistance);
            telemetry.addData("Status", "Run Time: %.1f", runtime.seconds());
            telemetry.addData("Triggers pressed"," left: %.2f right: %.2f", gamepad1.left_trigger, gamepad1.right_trigger);
            telemetry.addData("Amount of micro pollutants: %d", robot.ballHopperQuantity);
            telemetry.update();


        // counter needs to go to zero in order for the toggle method to work!!!
            tgg.reset();
        }
    }

    private void playerHussain(Gamepad gp){
        lift(gp);
        shooter(gp);
    }
    private void playerMarek(Gamepad gp){
        drive(gp);
        intake(gp);
        robotLove(gp, gp.right_stick_x * 0.25);
    }
    private void drive(Gamepad gp){

        double drive = -gp.left_stick_y;
        double turning  =  gp.right_stick_x;
        double rightPower   = Range.clip(drive - turning, -1.0, 1.0) ;
        double leftPower    = Range.clip(drive + turning, -1.0, 1.0) ;
        double speedLimit = robot.SPEED_LIMIT;
        double boostSpeedLimit = robot.BOOST_SPEED_LIMIT;

        // dpad controls for drive
        if(gp.dpad_up){
            robot.leftDrive.setPower(speedLimit);
            robot.rightDrive.setPower(speedLimit);
        }
        else if(gp.dpad_down){
            robot.leftDrive.setPower(-speedLimit);
            robot.rightDrive.setPower(-speedLimit);
        }
        else if(gp.dpad_left){
            robot.leftDrive.setPower(-speedLimit);
            robot.rightDrive.setPower(speedLimit);
        }
        else if(gp.dpad_right){
            robot.leftDrive.setPower(speedLimit);
            robot.rightDrive.setPower(-speedLimit);
        }
    // Boosted driving speed controls
        else if(gp.left_stick_button || gp.right_stick_button){
            robot.leftDrive.setPower(leftPower * boostSpeedLimit);
            robot.rightDrive.setPower(rightPower * boostSpeedLimit);
        }
    // Standard Driving speed controls
        else {
            robot.leftDrive.setPower(leftPower * speedLimit);
            robot.rightDrive.setPower(rightPower * speedLimit);
        }
    }
    private void intake(Gamepad gp){
        sorterCurrentDistance = robot.distanceSensor.getDistance(DistanceUnit.CM);
    // Manually controls both belt and zip tie intake
        if(sorterCurrentDistance > robot.DISTANCE_TO_GROUND_CM){
            if (gp.right_bumper){
                robot.ziptieIntake.setPower(robot.INTAKE_SPEED);   // ball going in
                robot.beltIntake.setPower(robot.INTAKE_SPEED);
            }else if (gp.left_bumper){
                robot.ziptieIntake.setPower(-robot.INTAKE_SPEED);  // ball going out
                robot.beltIntake.setPower(-robot.INTAKE_SPEED);
            }else{
                robot.ziptieIntake.setPower(0);
                robot.beltIntake.setPower(0);
            }
    // Automatically controls belt intake and manually controls zip tie intake
        }else{
            autoPushBall(sorterCurrentDistance, 600); // todo check if 600 is better than 500
            if (gp.right_bumper){
                robot.ziptieIntake.setPower(robot.INTAKE_SPEED);   // ball going in
            }else if (gp.left_bumper){
                robot.ziptieIntake.setPower(-robot.INTAKE_SPEED);  // ball going out
            }else{
                robot.ziptieIntake.setPower(0);
            }
        }
    }
    private void lift(Gamepad gp){
        // lift motor
        if(gp.b) { //&& robot.minHeight.getState()
            robot.lift.setPower(-0.7);          // going down
        }else if(gp.y && robot.maxHeight.getState()) {
            robot.lift.setPower(0.9);           // going up
            //todo add wing closing code
        }else {
            robot.lift.setPower(0);
        }
    }
    private void shooter(Gamepad gp){
        boolean isTriggerPressed = tgg.toggle(gp.right_trigger > 0); // defined at top so that the toggle method is always called

        //shooter motor
        if (tgg.toggle(gp.dpad_up)) {
            robot.shooterSpeedLimit += (robot.shooterSpeedLimit < 1.0) ? 0.05 : 0; // increments by 0.05 if limit is under 1.0
        }
        if (tgg.toggle(gp.dpad_down)) {
            robot.shooterSpeedLimit -= (robot.shooterSpeedLimit > 0.1) ? 0.05 : 0; // increments by -0.05 if limit is above 0.1
        }

        // setting speed to shooter motor
        if (gp.x) {
            robot.shooter.setPower(-robot.shooterSpeedLimit);
        }else {
            robot.shooter.setPower(0);
        }

        if(gp.right_trigger > 0) {
            if (isTriggerPressed) {         // because this needs to happen once every click and not for the length of time the person is holding the button
                robot.ballHopperQuantity--;
            }
            robot.shooterTrigger.setPosition(robot.FIRE_TRIGGER_SERVO);
        }else {
            robot.shooterTrigger.setPosition(robot.LOAD_TRIGGER_SERVO);
        }


    }
    private void robotLove(Gamepad gp, double spinner){
        if(gp.left_trigger > 0){            // opening wings (putting them away into stowed position)
            robot.leftWing.setPower(0.15);
            robot.rightWing.setPower(0.15);
        }else if(gp.right_trigger > 0){     // hugging wings (closing them around the balls)
            robot.leftWing.setPower(-0.2 - Math.abs(spinner));
            robot.rightWing.setPower(-0.2 - Math.abs(spinner));
        }else{
            robot.leftWing.setPower(0);
            robot.rightWing.setPower(0);
        }
    }

// uses distance sensor to push balls into the hopper
    private void autoPushBall(double distance, long time){
        if(distance <= robot.DISTANCE_TO_TOP_CM && !inProcess){  // second condition ensures servo can return
            robot.ballHopperQuantity ++;    // todo see if toggle method required or if this action is done once per loop
            robot.beltIntake.setPower(0);   // stops belt
            robot.pushBall.setPosition(robot.PUSHED_PUSH_BALL); // pushes ball into hopper
            inProcess = true;   // changes the condition
        }else if(inProcess){
            sleep(time);    // waits for servo to reach end
            robot.pushBall.setPosition(robot.OPEN_PUSH_BALL); // moves servo back
            inProcess = false; // changes the condition
        }else if(distance < robot.DISTANCE_TO_GROUND_CM && distance > robot.DISTANCE_TO_TOP_CM){   // moves intake if there is a ball in the belt rails
            robot.beltIntake.setPower(robot.INTAKE_SPEED);
        }else{
            robot.beltIntake.setPower(0);
        }
    }
}
