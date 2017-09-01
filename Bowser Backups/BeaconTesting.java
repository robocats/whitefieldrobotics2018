package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.ftcrobotcontroller.R;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

@Autonomous(name = "BeaconTesting", group = "Bowzer")
@Disabled
public class BeaconTesting extends LinearOpMode {

    ModernRoboticsI2cRangeSensor rangeSensor;


    HardwareBowzer robot = new HardwareBowzer(); // use the class created to define a Pushbot's hardware


    ColorSensor colorSensor;    // Hardware Device Object
    OpticalDistanceSensor odsSensor;  // Hardware Device Object

    @Override
    public void runOpMode() throws InterruptedException {
        float hsvValues[] = {0F, 0F, 0F};
        final float values[] = hsvValues;
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(R.id.RelativeLayout);

        colorSensor = hardwareMap.colorSensor.get("color sensor");
        odsSensor = hardwareMap.opticalDistanceSensor.get("ods");
        rangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range sensor");
        // wait for the start button to be pressed.
        waitForStart();

        robot.init(hardwareMap);

        while (opModeIsActive()) {

            //goes forward until it sees the line.
            while(odsSensor.getRawLightDetected() < 2 ){
                robot.motorLeft.setPower(.25);
                robot.motorRight.setPower(.25);
                idle();
            }


            robot.motorLeft.setPower(0);
            robot.motorRight.setPower(0);

            sleep(50);

            //turns to face beacon
            robot.motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            robot.motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            robot.motorLeft.setTargetPosition(1150);
            robot.motorRight.setTargetPosition(-1150);

            robot.motorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.motorLeft.setPower(.25);
            robot.motorRight.setPower(-.25);

            sleep(1100);

            robot.motorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.motorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            robot.motorLeft.setPower(0);
            robot.motorRight.setPower(0);

            sleep(50);


            //gets close the beacon using the ultrasonic sensor.
            while (rangeSensor.cmUltrasonic() > 13){
                robot.motorLeft.setPower(.25);
                robot.motorRight.setPower(.25);
                idle();
            }

            robot.motorLeft.setPower(0);
            robot.motorRight.setPower(0);

            if (robot.motorRight.isBusy() || robot.motorLeft.isBusy()){
                robot.motorLeft.setPower(0);
                robot.motorRight.setPower(0);
            }


            idle();

            break;


        }
    }
}