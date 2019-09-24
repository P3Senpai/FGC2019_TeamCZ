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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.util.HashMap;

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

@TeleOp(name="Basic: Iterative OpMode", group="Iterative Opmode")
//@Disabled
public class BasicOpMode_Iterative extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private Bot robot = new Bot();
    private Toggle tgg = new Toggle();
    private static double speedLimit = 0.8;
    private boolean plusIntakePower = false;
    private boolean lastPressed = false;
    private boolean lastPressed2 = false;
    double togglie = 0;

    // servo set up
    HashMap<Servo, Double> servoPosMap = new HashMap<>();
    Servo[] allServos = new Servo[] {robot.wingtipLeft, robot.wingtipRight, robot.pushBall, robot.tightenSide, robot.liftBrake, robot.shooterTrigger};
    int rotation = 0;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        robot.init(hardwareMap);

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
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        motorsTests(gamepad1);
//        servosTests(gamepad2);
        double servoS = robot.shooterTrigger.getPosition();

        if(gamepad1.dpad_left && !lastPressed){
//            robot.shooterTrigger.setPosition(servoS + 0.01); // 0.8
            robot.shooterTrigger.setPosition(0.86); // 0.8
            lastPressed = true;
        }else if(!gamepad1.dpad_left && lastPressed){
            lastPressed = false;
        }

        if(gamepad1.dpad_right && !lastPressed){
//            robot.shooterTrigger.setPosition(servoS - 0.01); // 0.32
            robot.shooterTrigger.setPosition(0.32); // 0.32
            lastPressed = true;
        }else if(!gamepad1.dpad_right && lastPressed){
            lastPressed = false;
        }
        telemetry.addData("",servoS);
        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
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
            togglie ++;
        }
        if (tgg.toggle(gp.dpad_down)) {
            speedLimit -= (speedLimit > 0.1) ? 0.1 : 0; // increments by -0.1 if limit is above 0.1
            togglie --;
        }

        // setting speed to shooter motor
        if (gp.x)
            robot.shooter.setPower(-speedLimit+0.1);
        else
            robot.shooter.setPower(0);
        // intake motors

        if (gp.right_bumper){
            robot.ziptieIntake.setPower(0.5);
            robot.beltIntake.setPower(0.5);
        }
        if (gp.left_bumper){
            robot.ziptieIntake.setPower(-0.5);
            robot.beltIntake.setPower(-0.5);
        }else{
            robot.ziptieIntake.setPower(0);
            robot.beltIntake.setPower(0);
        }

        // lift motor
        if(gp.a)
            robot.lift.setPower(-0.7);          // going down
        else if(gp.y && robot.maxHeight.getState())
            robot.lift.setPower(0.7);           // going up
        else
            robot.lift.setPower(0);

        // telemetry data
        telemetry.addData("Drive Speed", "left(%.2f) right(%.2f)", leftPower, rightPower);
        telemetry.addData("Shooter Speed", "limit(%.2f) actual(%.2f)", speedLimit, robot.shooter.getPower());
        telemetry.addLine((plusIntakePower)?"Intake power is positive":"Intake power is negative");
        telemetry.addData("Lift Motor:", "power(%.2f)", robot.lift.getPower());
        telemetry.addData("%d", togglie);
    }
    private void servosTests(Gamepad gp){
        // Sets index for servo array
        if (tgg.toggle(gp.dpad_up) && rotation < allServos.length){
            rotation ++;
        }else if(rotation >= allServos.length){
            rotation = 0;
        }
        if(tgg.toggle(gp.dpad_down) && rotation > -1){
            rotation --;
        }else if(rotation <= -1){
            rotation = allServos.length - 1;
        }
        //Gets servo and its position
        Servo servo = allServos[rotation];
        double servoPos = servoPosMap.get(allServos[rotation]);
        // Changes servo position
        if (tgg.toggle(gp.y))
            servoPos += 0.01;
        else if(tgg.toggle(gp.a))
            servoPos -= 0.01;
        servo.setPosition(servoPos);
        // Saves new position
        servoPosMap.put(servo, servoPos);

        // telemetry data
        telemetry.addData("Servo: %s \nPosition: %.2f", servo.getDeviceName(), servoPos);
        telemetry.update();
    }
    private HashMap <Servo, Double> intiServoMap(Servo[] servos){
        HashMap<Servo,Double> out = new HashMap<>();
        for(Servo s : servos){
            out.put(s, s.getPosition());
        }
        return out;
    }
}
