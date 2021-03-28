package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import java.lang.Math;
//import java.lang.Float;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.Robot;

@TeleOp(name = "TeleOps 2020-2021", group = "")
public class TeleOps extends LinearOpMode {
    public DcMotor rightShooter;
    public TouchSensor liftButtonBot;
    public CRServo liftservo;
    public TouchSensor liftButtonTop;
    public DcMotor picker;
    public Servo pusher;
    public DcMotor leftShooter;
    public DcMotor Right_Front_Wheel;
    public DcMotor Left_Front_Wheel;
    public DcMotor Right_Rear_Wheel;
    public DcMotor Left_Rear_Wheel;
    public Servo claw;
    public TouchSensor armButtonBot;
    public DcMotor Arm;
    public TouchSensor armButtonTop;
    public Servo blocker;

    //private Robot robot;

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        rightShooter = hardwareMap.get(DcMotor.class, "rightShooter");
        liftButtonBot = hardwareMap.get(TouchSensor.class, "liftButtonBot");
        liftservo = hardwareMap.get(CRServo.class, "liftservo");
        liftButtonTop = hardwareMap.get(TouchSensor.class, "liftButtonTop");
        picker = hardwareMap.get(DcMotor.class, "picker");
        pusher = hardwareMap.get(Servo.class, "pusher");
        leftShooter = hardwareMap.get(DcMotor.class, "leftShooter");

        Left_Rear_Wheel = hardwareMap.get(DcMotor.class, "Left_Rear_Wheel");
        Left_Front_Wheel = hardwareMap.get(DcMotor.class, "Left_Front_Wheel");
        Right_Rear_Wheel = hardwareMap.get(DcMotor.class, "Right_Rear_Wheel");
        Right_Front_Wheel = hardwareMap.get(DcMotor.class, "Right_Front_Wheel");
        claw = hardwareMap.get(Servo.class, "claw");
        armButtonBot = hardwareMap.get(TouchSensor.class, "armButtonBot");
        Arm = hardwareMap.get(DcMotor.class, "Arm");
        armButtonTop = hardwareMap.get(TouchSensor.class, "armButtonTop");
        blocker = hardwareMap.get(Servo.class, "blocker");


        /*robot = new Robot();
        robot.Init(hardwareMap, telemetry, false);
         */


        waitForStart();
        Left_Rear_Wheel.setDirection(DcMotorSimple.Direction.REVERSE);
        Left_Front_Wheel.setDirection(DcMotorSimple.Direction.REVERSE);
        leftShooter.setDirection(DcMotorSimple.Direction.REVERSE);


        if (opModeIsActive()) {


            while (opModeIsActive()) {
                if (gamepad1.right_bumper) {
                    blocker.setPosition(1);
                }
                else if (gamepad1.left_bumper) {
                    blocker.setPosition(-1);
                }
                telemetry.update();
                //lift up
                if (gamepad2.a && !liftButtonBot.isPressed()) {
                    Move_Lift_Down_Continuous();
                }
                //lift down
                if (gamepad2.y && !liftButtonTop.isPressed()) {
                    Move_Lift_Up_Continuous();
                }
                //picker on
                if (gamepad2.x && liftButtonBot.isPressed()) {
                    picker.setPower(1);
                }
                //picker off
                if (gamepad2.b) {
                    picker.setPower(0);
                }
                //pusher
                if (gamepad2.left_bumper && liftButtonTop.isPressed()) {
                    pusher.setPosition(1);
                    sleep(500);
                    pusher.setPosition(-1);
                }
                //shooter
                if (liftButtonTop.isPressed()) {
                    Shooter_Start();
                } else {
                    Shooter_Stop();
                }

                //claw
                if (gamepad2.dpad_right) {
                    claw.setPosition(-1);
                }
                if (gamepad2.dpad_left) {
                    claw.setPosition(1);
                }
                //arm
                if (gamepad2.dpad_down && !armButtonBot.isPressed()) {
                    Arm.setPower(0.5);
                } else if (gamepad2.dpad_up && !armButtonTop.isPressed()) {
                    Arm.setPower(-0.5);
                } else {
                    Arm.setPower(0);
                }
                // Mecanum drive is controlled with three axes: drive (front-and-back),
                // strafe (left-and-right), and twist (rotating the whole chassis).
                double drive  = gamepad1.left_stick_y * -1;
                double strafe = gamepad1.left_stick_x;
                double twist  = gamepad1.right_stick_x;

                /*
                 * If we had a gyro and wanted to do field-oriented control, here
                 * is where we would implement it.
                 *
                 * The idea is fairly simple; we have a robot-oriented Cartesian (x,y)
                 * coordinate (strafe, drive), and we just rotate it by the gyro
                 * reading minus the offset that we read in the init() method.
                 * Some rough pseudocode demonstrating:
                 *
                 * if Field Oriented Control:
                 *     get gyro heading
                 *     subtract initial offset from heading
                 *     convert heading to radians (if necessary)
                 *     new strafe = strafe * cos(heading) - drive * sin(heading)
                 *     new drive  = strafe * sin(heading) + drive * cos(heading)
                 *
                 * If you want more understanding on where these rotation formulas come
                 * from, refer to
                 * https://en.wikipedia.org/wiki/Rotation_(mathematics)#Two_dimensions
                 */

                // You may need to multiply some of these by -1 to invert direction of
                // the motor.  This is not an issue with the calculations themselves.
                double[] speeds = {
                        (drive + strafe + twist),
                        (drive - strafe - twist),
                        (drive - strafe + twist),
                        (drive + strafe - twist)
                };

                // Because we are adding vectors and motors only take values between
                // [-1,1] we may need to normalize them.

                // Loop through all values in the speeds[] array and find the greatest
                // *magnitude*.  Not the greatest velocity.
                double max = Math.abs(speeds[0]);
                for(int i = 0; i < speeds.length; i++) {
                    if ( max < Math.abs(speeds[i]) ) max = Math.abs(speeds[i]);
                }

                // If and only if the maximum is outside of the range we want it to be,
                // normalize all the other speeds based on the given speed value.
                if (max > 1) {
                    for (int i = 0; i < speeds.length; i++) speeds[i] /= max;
                }

                // apply the calculated values to the motors.
                Left_Front_Wheel.setPower(speeds[0]);
                Right_Front_Wheel.setPower(speeds[1]);
                Left_Rear_Wheel.setPower(speeds[2]);
                Right_Rear_Wheel.setPower(speeds[3]);
            }

        }

        telemetry.update();
    }

    // all of our base movement functions
    private void StopBase() {
        Left_Front_Wheel.setPower(0);
        Right_Front_Wheel.setPower(0);
        Left_Rear_Wheel.setPower(0);
        Right_Rear_Wheel.setPower(0);
    }

    private void Move_Lift_Up_Continuous(){
        StopBase();
        while (!liftButtonTop.isPressed()) {
            liftservo.setPower(1);
            picker.setPower(0);

        }
        liftservo.setPower(0.2);
    }
    private void Move_Lift_Down_Continuous(){
        StopBase();
        while (!liftButtonBot.isPressed()) {
            liftservo.setPower(-1);
        }
        liftservo.setPower(0);
    }
    private void Shooter_Start(){
        leftShooter.setPower(1);
        rightShooter.setPower(1);
    }
    private void Shooter_Stop(){
        leftShooter.setPower(0);
        rightShooter.setPower(0);
    }
}
