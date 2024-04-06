package org.firstinspires.ftc.teamcode.drive.Auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;



@Disabled
@Autonomous(name = "RedLeftClose", group = "RED")
public class RedLeftClose extends LinearOpMode {
    //armsNStuff arm;
    DcMotorEx Arm;
    //  public void setTargetPositionTolerance((int tolerance));
    Servo Box;
    Servo YPD;
    Servo intakeLeft;
    Servo intakeRight;
    CRServo Intake;
    OpenCvCamera webcam;
    GamePropLeft.gamePropPosition propPosition = GamePropLeft.gamePropPosition.LEFT;
    SampleMecanumDrive robot;
    @Override
    public void runOpMode() throws InterruptedException {

        // arm=new armsNStuff(hardwareMap);
        Arm=hardwareMap.get(DcMotorEx.class, "Arm");
        Box=hardwareMap.get(Servo.class, "Box");
        YPD=hardwareMap.get(Servo.class, "YPD");
        intakeRight=hardwareMap.get(Servo.class, "intakeRight");
        intakeLeft=hardwareMap.get(Servo.class, "intakeLeft");
        Intake=hardwareMap.get(CRServo.class, "Intake");
        Arm.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        Arm.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        Arm.setTargetPositionTolerance(20);
        Arm.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        Box.setPosition(0.84);
        intakeRight.setPosition(1);
        intakeLeft.setPosition(0);
        YPD.setPosition(0.87);
        telemetry.addData("Start OpMode", "BLUE LEFT");
        telemetry.update();
        startCamera();
        telemetry.addData("Selected Starting Position", propPosition);
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();

        while (!isStopRequested() && !opModeIsActive())
        {propPosition = getPropPosition();
            telemetry.addData("Identified Prop Location", propPosition);
            telemetry.update();
        }

        if (opModeIsActive() && !isStopRequested()) {
            //Build parking trajectory based on last detected target by vision
            webcam.stopStreaming(); //Stop Webcam to preserve Controlhub cycles.
            runAutonoumousMode();
        }
    }

    public void runAutonoumousMode() {

        Pose2d initPose = new Pose2d(0, 0, Math.toRadians(0.0)); // Starting Pose --Update CoOrdinates
        Pose2d nitPose = new Pose2d(0, 0, Math.toRadians(0.0));
        Pose2d innitPose = new Pose2d(0, 0, Math.toRadians(0.0)); // Starting Pose --Update CoOrdinates
        Pose2d wallPose1 = new Pose2d(0, 0, 0);
        Pose2d wallPose2 = new Pose2d(0, 0, 0);
        Pose2d returnpose = new Pose2d(0, 0, 0);
        Pose2d purplePixelPose = new Pose2d(0, 0, Math.toRadians(0.0));
        Pose2d leftPose  = new Pose2d(0, 0, Math.toRadians(0.0));
        switch (propPosition) { //UPDATE THESE POSITIONS
            case LEFT:
                telemetry.addData("Left", "Left");
                wallPose1 = new Pose2d(23,13, Math.toRadians(-90));
                wallPose2 = new Pose2d(23, 16.5, Math.toRadians(-90));
                returnpose = new Pose2d(4.5,-2, Math.toRadians(-80.5));
                nitPose = new Pose2d(2,-60, Math.toRadians(-87.5));
                innitPose = new Pose2d(40.3,-88.5, Math.toRadians(-90));
                purplePixelPose = new Pose2d(26, 8, Math.toRadians(0));
                leftPose = new Pose2d(18, 8, Math.toRadians(0));
                break;
            case CENTER:
                telemetry.addData("Center Position", "Center");
                wallPose1 = new Pose2d(23,12, Math.toRadians(-90));
                wallPose2 = new Pose2d(23,16.5, Math.toRadians(-90));
                returnpose = new Pose2d(4,-2, Math.toRadians(-100.5));
                nitPose = new Pose2d(2,-60, Math.toRadians(-87.5));
                innitPose = new Pose2d(35,-88.5, Math.toRadians(-90));
                purplePixelPose = new Pose2d(27, -3, Math.toRadians(0.0));
                leftPose = new Pose2d(25, -3, Math.toRadians(0.0));
                break;
            case RIGHT:
                wallPose1 = new Pose2d(23,12., Math.toRadians(-90));
                wallPose2 = new Pose2d(23,16.5, Math.toRadians(-90));
                returnpose = new Pose2d(4,-2, Math.toRadians(-90.5));
                nitPose = new Pose2d(2, -60,Math.toRadians(-87.5));
                innitPose = new Pose2d(26,-89.2, Math.toRadians(-90));
                leftPose = new Pose2d(29,-1, Math.toRadians(-90));
                purplePixelPose = new Pose2d(29,-5, Math.toRadians(-90));

                break;

        }

        // Pose2d parkingPose = new Pose2d(24, 70, Math.toRadians(0)); //UPDATE

        telemetry.update();
        sleep(200);

        robot = new SampleMecanumDrive(hardwareMap);
        robot.followTrajectorySequence(robot.trajectorySequenceBuilder(initPose) //Starting Pose
                // .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {Arm.setTargetPosition(0);Arm.setPower(0.2);Arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);})
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {Arm.setTargetPosition(300);Arm.setPower(0.2);Arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);})
                .lineToLinearHeading(leftPose)
                .lineToLinearHeading(purplePixelPose)//Drop Purple Pixel
                .back(8)
                .lineToLinearHeading(wallPose1)
                .lineToLinearHeading(wallPose2)
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {Arm.setTargetPosition(190);Arm.setPower(0.2);Arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);})
                .waitSeconds(.5)
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {Intake.setPower(0.75);})
                .setConstraints(SampleMecanumDrive.getVelocityConstraint(36.0, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL/5.0))
                .back(3)
                .waitSeconds(0.8)
                .resetConstraints()
                .forward(3)
                .setConstraints(SampleMecanumDrive.getVelocityConstraint(65.0, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL/1.2))
                .lineToLinearHeading(returnpose)
                .lineToLinearHeading(nitPose)
                .lineToLinearHeading(innitPose)
                .resetConstraints()
                .setConstraints(SampleMecanumDrive.getVelocityConstraint(35.0, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL/5.0))
                .forward(5)
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {YPD.setPosition(0.11);})
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {Arm.setTargetPosition(3200);Arm.setPower(0.7);Arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);})
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {Box.setPosition(0.3);})
                .waitSeconds(3.5)
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {Intake.setPower(-0.2);})
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {YPD.setPosition(0.84);})
                .resetConstraints()
                // .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {Intake.setPower(-0.2);})
                .waitSeconds(1.5)
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {Intake.setPower(0);})
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {Box.setPosition(0.84);})
                .waitSeconds(0.2)
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {Arm.setTargetPosition(0);Arm.setPower(0.5);Arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);})
                .waitSeconds(3)
                //.UNSTABLE_addTemporalMarkerOffset(0, () -> {225})
                //.UNSTABLE_addTemporalMarkerOffset(0, () -> {0.45;})
                /*.lineToLinearHeading(yellowPixelPose)
                //.waitSeconds(1)
                // .forward(4)
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {Arm.setTargetPosition(2250);Arm.setPower(0.4);Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);})
                .UNSTABLE_addTemporalMarkerOffset(3, () -> {Box.setPosition(0.45);})
                .waitSeconds(3)
                .forward(4.5)
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {Intake.setPower(-0.2);})
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {Intake.setPower(0);})
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {Box.setPosition(0.93);})
                .waitSeconds(0.75)
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {Arm.setTargetPosition(0);Arm.setPower(0.25);Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);})
                .waitSeconds(4)*/
                .build());

    }
    public void startCamera() {
        //Initialize Camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "FrontCam"), cameraMonitorViewId);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });

        webcam.setPipeline(new GamePropLeft());
    }

    public GamePropLeft.gamePropPosition getPropPosition() {
        return GamePropLeft.position;
    }

}