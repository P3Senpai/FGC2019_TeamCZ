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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="FGC 2019 Pushbot 1.0", group="Iterative Opmode")
//@Disabled
public class BasicOpMode_Iterative extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive   = null;
    private DcMotor rightDrive  = null;
    private DcMotor lift        = null;
    private Servo   openLeft    = null;
    private Servo   openRight   = null;
    private Servo   wingtipLeft = null;
    private Servo   wingtipRight = null;
    private double speedLimit   = 0.8;
    private boolean speedLimitToggle = false;
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        //  init of prototypes
        leftDrive = hardwareMap.get(DcMotor.class,"left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        lift       = hardwareMap.get(DcMotor.class, "lift");

        leftDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        rightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        lift.setDirection(DcMotorSimple.Direction.REVERSE);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); //TODO Test if this stops the lift motor from moving

        //init servos
        openLeft    = hardwareMap.get(Servo.class, "open_left");
        openRight   = hardwareMap.get(Servo.class, "open_right");
        wingtipLeft = hardwareMap.get(Servo.class, "wing_tip_left");
        wingtipRight = hardwareMap.get(Servo.class, "wing_tip_right");

        // closed position for servo
        openLeft.setPosition(0.7);  // open position is 0.3
        openRight.setPosition(0.3); // open position is 0.7

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {

        openLeft.setPosition(0.3);
        openRight.setPosition(0.7);


    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */

    //TODO bulldozer init in this method

    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        // changing speed limit with toggle controls
        if (!speedLimitToggle && gamepad1.dpad_up){
            speedLimit += (speedLimit < 1.0)? 0.1 : 0; // increments by 0.1 if limit is under 1.0
            speedLimitToggle = true;
        }else if(!speedLimitToggle && gamepad1.dpad_down){
            speedLimit -= (speedLimit > 0.1)? 0.1 : 0; // increments by -0.1 if limit is above 0.1 //TODO test if 0.1 goes to 0.0 as when 0.0 went to -0.1
            speedLimitToggle = true;
        }else if(speedLimitToggle && !gamepad1.dpad_up && !gamepad1.dpad_down)
            speedLimitToggle = false;   // resets toggle once button is pressed

        // driving
        double drive = gamepad1.left_stick_y;
        double turn  =  -gamepad1.right_stick_x;
        double leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
        double rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;

        leftDrive.setPower(leftPower * speedLimit);
        rightDrive.setPower(rightPower * speedLimit);


        // lifting
        if(gamepad1.a) // going down
            lift.setPower(-0.5);
        else if(gamepad1.y)     // going up
            lift.setPower(0.5);
        else
            lift.setPower(0);


        // servo stuffz
        double openLeftPos = openLeft.getPosition(); // TODO remove once ready and tested
        double openRightPos = openRight.getPosition(); // TODO remove once ready and tested
        // tested and set at top (maybe remove)
        double wingtipLeftPos   = wingtipLeft.getPosition();
        double wingtipRightPos  = wingtipRight.getPosition();

        // region gates open and close for the wing tips
        // todo remove once region gets tested and is ready
        if(gamepad2.a) { // open
            openLeft.setPosition(0.3);
            openRight.setPosition(0.7);
        }
        if(gamepad2.b){ // closed
            openLeft.setPosition(0.7); // closed position
            openRight.setPosition(0.3);
        }
        //endregion

        if(gamepad2.dpad_up){
//            wingtipLeft.setPosition(wingtipLeftPos + 0.001);
//            wingtipRight.setPosition(wingtipRightPos + 0.001);
            wingtipLeft.setPosition(0.27);
            wingtipRight.setPosition(0.67);
        }
        if(gamepad2.dpad_down){
//            wingtipLeft.setPosition(wingtipLeftPos - 0.001);
//            wingtipRight.setPosition(wingtipRightPos - 0.001);
            wingtipLeft.setPosition(0.48);
            wingtipRight.setPosition(0.45);
        }

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Speed Limit", "%.2f power", speedLimit);
        telemetry.addData("Drive Speed", "left(%.2f) right(%.2f)", leftPower, rightPower);
        telemetry.addData("Servo Position", "openL(%.2f) openR(%.2f) bulldozeL(%.2f) bulldozeR(%.2f)",openLeftPos,openRightPos,wingtipLeftPos,wingtipRightPos);
        String openState = ((openLeft.getPosition() == 0.3) && (openRight.getPosition() == 0.7))?"The gates are open and ready for use":"The gates are closed";
        telemetry.addLine(openState);
        // change made to test webhook at git push test 2
    }

    /*
     * Code to run ONCE after the driver hits STOP // todo find how this relates to rule E07 (
     */
    @Override
    public void stop() {
        //todo do the test of this method
        // region stop method test
//        ElapsedTime time = new ElapsedTime();
//        double totalTime = 5.0; // 5 seconds
//        double runtimeSeconds = time.seconds();
//        time.reset();
//
//        while (runtimeSeconds <= totalTime)
//        {
//            leftDrive.setPower(0.5);
//            rightDrive.setPower(-0.5);
//            telemetry.addLine("Houston we have a problem. \nThe driving motors just keep spinning");
//            telemetry.addData("Count down", "%.2f seconds (%.2f total)",runtimeSeconds,totalTime);
//            telemetry.update();
//        }
////        // todo if motors don't stop after the 5 seconds add manual stop
////        leftDrive.setPower(0);
////        rightDrive.setPower(0);
        //endregion
    }

}
