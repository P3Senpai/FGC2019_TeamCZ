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

            playerHussain(gamepad1);
            playerMarek(gamepad2);



            // if you hold th b button the intake will automatically work
            autoPushBall(sorterCurrentDistance, 500);



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
    // todo fill methods with respective code
    private void playerHussain(Gamepad gp){
        drive(gp);
        intake(gp);
    }  // has drive and intake methods
    private void playerMarek(Gamepad gp){
        lift(gp);
        shooter(gp);
    } // has lift and shooter methods
    private void drive(Gamepad gp){
        // drive motors

        double drive = -gp.left_stick_y;
        double turning  =  gp.right_stick_x;
        double rightPower   = Range.clip(drive - turning, -1.0, 1.0) ;
        double leftPower    = Range.clip(drive + turning, -1.0, 1.0) ;
// dpad contols for drive
        if(gp.dpad_up){
            robot.leftDrive.setPower(1);
            robot.rightDrive.setPower(1);
        }
        else if(gp.dpad_down){
            robot.leftDrive.setPower(-1);
            robot.rightDrive.setPower(-1);
        }
        else if(gp.dpad_left){
            robot.leftDrive.setPower(-1);
            robot.rightDrive.setPower(1);
        }
        else if(gp.dpad_right){
            robot.leftDrive.setPower(1);
            robot.rightDrive.setPower(-1);
        }
        else {
            robot.leftDrive.setPower(leftPower);
            robot.rightDrive.setPower(rightPower);
        }
    }
    private void intake(Gamepad gp){
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
    }
    private void lift(Gamepad gp){
        // lift motor
        if(gp.a ) { //&& robot.minHeight.getState()
            robot.lift.setPower(-0.7);          // going down
        }else if(gp.y && robot.maxHeight.getState()) {
            robot.lift.setPower(0.7);           // going up
        }else {
            robot.lift.setPower(0);
        }
    }
    private void shooter(Gamepad gp){
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
}
