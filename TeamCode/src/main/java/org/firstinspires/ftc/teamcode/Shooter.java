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
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

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

@TeleOp(name="Test", group="Iterative Opmode")
//@Disabled
public class Shooter extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor spin = null;
    private double speedLimit = 0.1;
    private Toggle tgg = new Toggle();
    private DigitalChannel maxLimit = null;
    private DigitalChannel minLimit = null;


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initializing . . . ");
        
        //  init of prototypes
        spin = hardwareMap.get(DcMotor.class, "motor" );
        maxLimit = hardwareMap.get(DigitalChannel.class, "maxLimit");
        minLimit = hardwareMap.get(DigitalChannel.class, "minLimit");

        // set the digital channel to input.
        maxLimit.setMode(DigitalChannel.Mode.INPUT);
        minLimit.setMode(DigitalChannel.Mode.INPUT);
        spin.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {


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
        if (tgg.toggle(gamepad1.dpad_up))
            speedLimit += (speedLimit < 1.0)? 0.1 : 0; // increments by 0.1 if limit is under 1.0
        if(tgg.toggle(gamepad1.dpad_down))
            speedLimit -= (speedLimit > 0.1)? 0.1 : 0; // increments by -0.1 if limit is above 0.1

        // setting speed to shooter motor
        if(gamepad1.a)
            spin.setPower(-speedLimit);
        else
            spin.setPower(0);


        if(gamepad2.a && minLimit.getState()) // going down
            spin.setPower(-0.7);
        else if(gamepad2.y && maxLimit.getState())     // going up
            spin.setPower(0.7);
        else
            spin.setPower(0);



        // Telemetry Data
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Speed: ",speedLimit);
        telemetry.addData("Motor: ",spin.getPower());
        telemetry.addData("Magnetic Sense: ", !maxLimit.getState());


        // reset toggle values for next iteration
        tgg.reset();

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



