package org.firstinspires.ftc.teamcode.drive.opmode;

import  com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

/**
 * This 2020-2021 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine the position of the Ultimate Goal game elements.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */


/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config

@Autonomous(group = "drive")


public class Autoprethi extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";
    private static final String VUFORIA_KEY =
            "AfNKhrX/////AAABmUBEXb6zGEKxsiFzrqjzR/lwcHlaAe4Kzi2jQqpWPYBIMequ0Kafs1C+YNhx3L5m2YV+qXgZQu/ZKxuV1qYCO6Pov3RCWbHaXVrWfqwJBoO5zLUYpOXM/AuivZd97OY0R1fdGvGWdzBe+5M34jqdSPSy3iV74mbUjaqia4qYoRFFikMY5uipMzuoMdRPMf/vPSFujM5QxnLdb7MjmKFyr0hoRKnEOpQBIV8j1o1tQdhU+KzuBQ5pmnCf0VHJ5Y3mRZk+oFIQRZ/Tp8oWlwphDSQSWjyppTdlkIVfq2b6p49Y0A6/OkD4OONSPA2GNHDk1K/aad9oS7BFPDbGNh1tYBmFQmEyBts4qM84IGR3WzqW";

    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
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
    @Override
    public void runOpMode() throws InterruptedException {

        rightShooter = hardwareMap.get(DcMotor.class, "rightShooter");
        liftButtonBot = hardwareMap.get(TouchSensor.class, "liftButtonBot");
        liftservo = hardwareMap.get(CRServo.class, "liftservo");
        liftButtonTop = hardwareMap.get(TouchSensor.class, "liftButtonTop");
        picker = hardwareMap.get(DcMotor.class, "picker");
        pusher = hardwareMap.get(Servo.class, "pusher");
        leftShooter = hardwareMap.get(DcMotor.class, "leftShooter");

        initVuforia();
        initTfod();
        Left_Rear_Wheel = hardwareMap.get(DcMotor.class, "Left_Rear_Wheel");
        Left_Front_Wheel = hardwareMap.get(DcMotor.class, "Left_Front_Wheel");
        Right_Rear_Wheel = hardwareMap.get(DcMotor.class, "Right_Rear_Wheel");
        Right_Front_Wheel = hardwareMap.get(DcMotor.class, "Right_Front_Wheel");
        claw = hardwareMap.get(Servo.class, "claw");
        armButtonBot = hardwareMap.get(TouchSensor.class, "armButtonBot");
        Arm = hardwareMap.get(DcMotor.class, "Arm");
        armButtonTop = hardwareMap.get(TouchSensor.class, "armButtonTop");
        blocker = hardwareMap.get(Servo.class, "blocker");


        waitForStart();

        if (opModeIsActive()) {

            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                tfod.activate();

                // The TensorFlow software will scale the input images from the camera to a lower resolution.
                // This can result in lower detection accuracy at longer distances (> 55cm or 22").
                // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
                // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
                // should be set to the value of the images used to create the TensorFlow Object Detection model
                // (typically 16/9).
                tfod.setZoom(2.5, 16.0/9.0);

                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions == null){
                    telemetry.addData("Rings Detected", "0");
                    telemetry.update();
                    /*claw.setPosition(-1);
                    Straight(52);

                    while (!armButtonBot.isPressed()) {
                        Arm.setPower(0.5);
                    }

                    Arm.setPower(0);
                    claw.setPosition(1);
                    Back(9); */
                }
                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());
                    // step through the list of recognitions and display boundary info.
                    int i = 0;
                    for (Recognition recognition : updatedRecognitions) {
                        telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                        telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                recognition.getLeft(), recognition.getTop());
                        telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                recognition.getRight(), recognition.getBottom());
                    }
                    telemetry.update();
                    int number = updatedRecognitions.size();
                    if (number == 1){
                        telemetry.addData("Rings Detected", "1");
                        telemetry.update();
                    }
                    if (number == 4){
                        telemetry.addData("Rings Detected", "4");
                        telemetry.update();
                    }
                }

            }

            if (tfod != null) {
                tfod.shutdown();
            }


/*                  Spline(0,40);

                    while (!liftButtonTop.isPressed()) {
                        liftservo.setPower(1);
                    }
                    liftservo.setPower(0);
                    while(liftButtonTop.isPressed()){
                        leftShooter.setPower(1);
                        rightShooter.setPower(1);
                    }
                    pusher.setPosition(1);
                    sleep(500);
                    pusher.setPosition(-1);
                    Spline(0,10);
                    pusher.setPosition(1);
                    sleep(500);
                    pusher.setPosition(-1);
                    Spline(0,10);
                pusher.setPosition(1);
                sleep(500);
                pusher.setPosition(-1);
                    Straight(7);
*/
        }
    }


    public void Straight(double DISTANCE) {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);


        Trajectory trajectory = drive.trajectoryBuilder(new Pose2d())
                .forward(DISTANCE)
                .build();

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectory(trajectory);

        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addData("finalX", poseEstimate.getX());
        telemetry.addData("finalY", poseEstimate.getY());
        telemetry.addData("finalHeading", poseEstimate.getHeading());
        telemetry.update();
    }
    public void Spline(double x, double y){
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        Trajectory traj = drive.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(x, y), 0)
                .build();

        drive.followTrajectory(traj);
    }

    public void Turn(double ANGLE){
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        drive.turn(Math.toRadians(ANGLE));
    }

    public void Back(double DISTANCE) {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);


        Trajectory trajectory = drive.trajectoryBuilder(new Pose2d())
                .back(DISTANCE)
                .build();

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectory(trajectory);

        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addData("finalX", poseEstimate.getX());
        telemetry.addData("finalY", poseEstimate.getY());
        telemetry.addData("finalHeading", poseEstimate.getHeading());
        telemetry.update();
    }

    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }
}

/*claw.setPosition(-1);
                    Straight(52);

                    while (!armButtonBot.isPressed()) {
                        Arm.setPower(0.5);
                    }

                    Arm.setPower(0);
                    claw.setPosition(1);
                    Back(9); */

