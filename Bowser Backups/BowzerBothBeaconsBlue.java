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

import static java.lang.Thread.sleep;
@Autonomous(name = "Dont run this2", group = "Bowzer")
//@disabled
public class BowzerBothBeaconsBlue extends LinearOpMode {
    HardwareBowzer robot = new HardwareBowzer();
    ModernRoboticsI2cRangeSensor rangeSensor;
    ColorSensor colorSensor;
    OpticalDistanceSensor odsSensor;
    ModernRoboticsI2cGyro gyro;
    @Override
    public void runOpMode() throws InterruptedException {
        colorSensor = hardwareMap.colorSensor.get("color sensor");
        odsSensor = hardwareMap.opticalDistanceSensor.get("ods");
        rangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range sensor");
        gyro = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("gyro");
        robot.init(hardwareMap);
        gyro.calibrate();
        waitForStart();
        while (opModeIsActive()) {
            while (gyro.isCalibrating())  {
                Thread.sleep(50);
                idle();
            }
            robot.sweepermotor.setPower(-1);
            robot.conveyerMotor.setPower(.93);
            sleep(1000);
            robot.sweepermotor.setPower(0);
            robot.conveyerMotor.setPower(0);
            int heading = gyro.getHeading();
            robot.motorLeft.setPower(1);
            robot.motorRight.setPower(.08);
            sleep(2000);
            while (rangeSensor.cmUltrasonic() > 44){
                idle();
            }
            stopMotors();
            sleep(200);
            runWithEncoders(-900, 900, .27, .27, 1200, "Turn to face first beacon");
            stopMotors();
            idle();
            findline();
            runWithEncoders(200, -200, .4, .4, 600, "small kick for gyro");
            ////////////
            gyroTurn(.34, -.34, 78);
            ////////////
            stopMotors();
            idle();
            findBeacon(10, 2);
            runWithEncoders(-500, -500, .4, .4, 700, "Back up from first beacon");
            stopMotors();
            heading = gyro.getHeading();
            while(heading != 0 && heading != 1 && heading != 2 && heading != 3 && heading != 4 && heading != 5){
                heading = gyro.getHeading();
                robot.motorLeft.setPower(-.37);
                robot.motorRight.setPower(.37);
                idle();
            }
            idle();
            stopMotors();
            robot.motorLeft.setPower(.65);
            robot.motorRight.setPower(.65);
            sleep(900);
            while (odsSensor.getRawLightDetected() < 1){
                robot.motorLeft.setPower(.21);
                robot.motorRight.setPower(.21);
                idle();
            }
            stopMotors();
            runWithEncoders(200, -200, .4, .4, 600, "small kick for gyro");
            ////////////////////
            gyroTurn(.35, -.35, 72);
            /////////////////////////
            stopMotors();
            findBeacon(10,2);
            idle();
            runWithEncoders(-500, -500, .4, .4, 700, "Back up from second beacon");

            heading = gyro.getHeading();
            runWithEncoders(900, -900, .4, .4, 600, "large kick for gyro");
            while (heading < 201
                    ){
                heading = gyro.getHeading();
                robot.motorLeft.setPower(.33);
                robot.motorRight.setPower(-.33);
                idle();
            }
            runWithEncoders(5200, 5200, .6, .6, 4000, "move towards centre");
            break;
        }
    }
    public void gyroTurn(double leftPower, double rightpower, int angle)throws InterruptedException{
        int heading = gyro.getHeading();
        while(heading < angle){
            heading = gyro.getHeading();
            robot.motorLeft.setPower(leftPower);
            robot.motorRight.setPower(rightpower);
            idle();
        }
    }
    public void findline()throws InterruptedException{
        while(odsSensor.getRawLightDetected() < 1){
            robot.motorLeft.setPower(.21);
            robot.motorRight.setPower(.21);
            idle();
        }
        stopMotors();
    }
    public void runWithEncoders(int leftDistance, int rightDistance, double leftPower, double rightPower, int duration, String task) throws InterruptedException {
        robot.motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();
        robot.motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        idle();
        robot.motorRight.setTargetPosition(rightDistance);
        robot.motorLeft.setTargetPosition(leftDistance);
        idle();
        robot.motorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        idle();
        robot.motorRight.setPower(rightPower);
        robot.motorLeft.setPower(leftPower);
        sleep(duration);
        idle();
        robot.motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        idle();
        stopMotors();
        idle();
    }
    public void findBeacon(int range, int colorStrength) throws InterruptedException {
        robot.motorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.motorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        while (rangeSensor.cmUltrasonic() > range) {
            robot.motorLeft.setPower(.25);
            robot.motorRight.setPower(.25);
            idle();
        }
        stopMotors();
        idle();
        if (colorSensor.blue() > colorStrength) {
            robot.motorLeft.setPower(.4);
            sleep(900);
        } else if (colorSensor.red() > colorStrength) {
            robot.motorRight.setPower(.4);
            sleep(900);
        }
       stopMotors();
        idle();
    }
    public void stopMotors() {
        robot.motorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.motorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.motorLeft.setPower(0);
        robot.motorRight.setPower(0);
    }
}