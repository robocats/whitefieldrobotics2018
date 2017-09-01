/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.configuration.I2cSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HardwareJacobot;

import static android.os.SystemClock.sleep;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Autored", group="Linear Opmode")  // @Autonomous(...) is the other common choice
//@Disabled
public class Autored
        extends LinearOpMode {


    HardwareJacobot robot       = new HardwareJacobot();

    ColorSensor colorSensor;

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    // DcMotor leftMotor = null;
    // DcMotor rightMotor = null;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        robot.init(hardwareMap);
        OpticalDistanceSensor ods = hardwareMap.opticalDistanceSensor.get("ods");
        colorSensor = hardwareMap.colorSensor.get("color sensor");

        float hsvValues[] = {0F, 0F, 0F};
        final float values[] = hsvValues;
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(R.id.RelativeLayout);

        /* eg: Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step (using the FTC Robot Controller app on the phone).
         */
        // leftMotor  = hardwareMap.dcMotor.get("left motor");
        // rightMotor = hardwareMap.dcMotor.get("right motor");

        // eg: Set the drive motor directions:
        // "Reverse" the motor that runs backwards when connected directly to the battery
        // leftMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        // rightMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();


            int leftencoderPos = robot.leftMotor.getCurrentPosition();
            int rightencoderPos = robot.rightMotor.getCurrentPosition();

            int leftencoder = robot.leftMotor.getCurrentPosition();
            int rightencoder = robot.rightMotor.getCurrentPosition();

            ////////////////////////////////////////////////Forward!/////////////////stop, run encode, # run to pos, # run using encorders, //////////////

            robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            robot.leftMotor.setTargetPosition(1050);
            robot.rightMotor.setTargetPosition(1050);

            robot.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.rightMotor.setPower(.25);
            robot.leftMotor.setPower(.25);
            sleep(2000);

            robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            robot.leftMotor.setPower(0);
            robot.rightMotor.setPower(0);

            ////////////////////////////////////////////////////////////////////////////////////////

            ////////////////////////////////////////////////Turn Right!/////////////////stop, run encode, # run to pos, # run using encorders, //////////////

            robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            robot.leftMotor.setTargetPosition(-325);
            robot.rightMotor.setTargetPosition(325);

            robot.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.leftMotor.setPower(-.20);
            robot.rightMotor.setPower(.20);
            sleep(3000);

            robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            robot.leftMotor.setPower(0);
            robot.rightMotor.setPower(0);

            ////////////////////////////////////////////////////////////////////////////////////////

            //////////////////////////////////////Uses sensor to find white line////////////////////

            while (ods.getRawLightDetected() < 1) {
                robot.rightMotor.setPower(.25);
                robot.leftMotor.setPower(.25);
                sleep(50);
            }
            robot.leftMotor.setPower(0);
            robot.rightMotor.setPower(0);


            ////////////////////////////////////////////////////////////////////////////////////////

            ////////////////////////////////////////////////Forward!/////////////////stop, run encode, # run to pos, # run using encorders, //////////////

            robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            robot.leftMotor.setTargetPosition(125);
            robot.rightMotor.setTargetPosition(125);

            robot.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.rightMotor.setPower(.25);
            robot.leftMotor.setPower(.25);
            sleep(2000);

            robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            robot.leftMotor.setPower(0);
            robot.rightMotor.setPower(0);

            ////////////////////////////////////////////////////////////////////////////////////////

            /////////////////////////////////////////Turns Left!/////////////////////////////////////////////
            while (ods.getRawLightDetected() < 1) {
                robot.rightMotor.setPower(.25);
                robot.leftMotor.setPower(-.25);
                sleep(50);
            }
            robot.leftMotor.setPower(0);
            robot.rightMotor.setPower(0);
            ////////////////////////////////////////////////////////////////////////////////////////


            ////////////////////////////////////////////////Turn right!/////////////////stop, run encode, # run to pos, # run using encorders, //////////////

            robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            robot.leftMotor.setTargetPosition(50);
            robot.rightMotor.setTargetPosition(-50);

            robot.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.leftMotor.setPower(.20);
            robot.rightMotor.setPower(.20);
            sleep(2000);

            robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            robot.leftMotor.setPower(0);
            robot.rightMotor.setPower(0);

            ////////////////////////////////////////////////////////////////////////////////////////


            if (colorSensor.red() > colorSensor.blue()) {
                robot.leftMotor.setPower(.3);
                sleep(200);
            } else if (colorSensor.blue() > colorSensor.red()) {
                robot.rightMotor.setPower(.3);
                sleep(200);

            }

            /*
            ////////////////////////////////////////////////Back!/////////////////stop, run encode, # run to pos, # run using encorders, //////////////

            robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            robot.leftMotor.setTargetPosition(-350);
            robot.rightMotor.setTargetPosition(-350);

            robot.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.leftMotor.setPower(-.25);
            robot.rightMotor.setPower(-.25);
            sleep(2000);

            robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            robot.leftMotor.setPower(0);
            robot.rightMotor.setPower(0);

            ////////////////////////////////////////////////////////////////////////////////////////

            ////////////////////////////////////////////////Turn Right!/////////////////stop, run encode, # run to pos, # run using encorders, //////////////

            robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            robot.leftMotor.setTargetPosition(550);
            robot.rightMotor.setTargetPosition(-550);

            robot.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.leftMotor.setPower(.20);
            robot.rightMotor.setPower(-.20);
            sleep(2000);

            robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            robot.leftMotor.setPower(0);
            robot.rightMotor.setPower(0);

            ////////////////////////////////////////////////////////////////////////////////////////

            //////////////////////////////////////Uses sensor to find white line////////////////////

            while (ods.getRawLightDetected() < 1.5) {
                robot.rightMotor.setPower(.3);
                robot.leftMotor.setPower(.3);
                sleep(50);
            }

            ////////////////////////////////////////////////////////////////////////////////////////

            ////////////////////////////////////////////////Back!/////////////////stop, run encode, # run to pos, # run using encorders, //////////////

            robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            robot.leftMotor.setTargetPosition(-150);
            robot.rightMotor.setTargetPosition(-150);

            robot.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.rightMotor.setPower(-.25);
            robot.leftMotor.setPower(-.25);
            sleep(2000);

            robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            robot.leftMotor.setPower(0);
            robot.rightMotor.setPower(0);

            ////////////////////////////////////////////////////////////////////////////////////////

            ////////////////////////////////////////////////Turn Left!/////////////////stop, run encode, # run to pos, # run using encorders, //////////////

            robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            robot.leftMotor.setTargetPosition(-615);
            robot.rightMotor.setTargetPosition(615);

            robot.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.leftMotor.setPower(-.20);
            robot.rightMotor.setPower(.20);
            sleep(2000);

            robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            robot.leftMotor.setPower(0);
            robot.rightMotor.setPower(0);

            ////////////////////////////////////////////////////////////////////////////////////////

            /*
            ////////////////////////////////////////////////Turn Right!/////////////////stop, run encode, # run to pos, # run using encorders, //////////////

            robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            robot.leftMotor.setTargetPosition(250);
            robot.rightMotor.setTargetPosition(-250);

            robot.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.leftMotor.setPower(.20);
            robot.rightMotor.setPower(-.20);
            sleep(2000);

            robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            robot.leftMotor.setPower(0);
            robot.rightMotor.setPower(0);

            ////////////////////////////////////////////////////////////////////////////////////////
            */


            idle();
            break;


        }
    }
}











