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
@Autonomous(name = "Bowzer Shoot Park Blue", group = "Bowzer")
//@disabled
public class BowzerShootParkBlue extends LinearOpMode {
    HardwareBowzer robot = new HardwareBowzer();
    ModernRoboticsI2cRangeSensor rangeSensor;
    ColorSensor colorSensor;
    OpticalDistanceSensor odsSensor;
    ModernRoboticsI2cGyro gyro;


    @Override
    public void runOpMode() throws InterruptedException {
        float hsvValues[] = {0F, 0F, 0F};
        final float values[] = hsvValues;
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(R.id.RelativeLayout);
        gyro.calibrate();
        waitForStart();
        robot.init(hardwareMap);
        colorSensor = hardwareMap.colorSensor.get("color sensor");
        odsSensor = hardwareMap.opticalDistanceSensor.get("ods");
        rangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range sensor");
        gyro = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("gyro");
        while (opModeIsActive()) {

            robot.sweepermotor.setPower(-1);
            robot.conveyerMotor.setPower(.95);
            sleep(1000);
            robot.sweepermotor.setPower(0);
            robot.conveyerMotor.setPower(0);

            runWithEncoders(300, 300, .5, .5, 700, "Go forward");

            runWithEncoders(500, -500, .5, -.5, 800, "turn to face vortex");

            runWithEncoders(6000,6000, .8, .8, 6000, "go and park");

            break;
        }
    }
    public void findline(){
        while(odsSensor.getRawLightDetected() < 2){
            robot.motorLeft.setPower(.33);
            robot.motorRight.setPower(.33);
        }
        robot.motorLeft.setPower(0);
        robot.motorRight.setPower(0);
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
        robot.motorRight.setPower(0);
        robot.motorLeft.setPower(0);
        idle();
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
}