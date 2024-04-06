package org.firstinspires.ftc.teamcode.drive.Auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;



@Disabled
@Autonomous(name = "BlueRightMiddle", group = "BLUE")
public class BlueRight58 extends LinearOpMode {
    //armsNStuff arm;
    DcMotorEx Arm;
  //  public void setTargetPositionTolerance((int tolerance));
    Servo Box;
    Servo YPD;
    Servo intakeLeft;
    Servo intakeRight;
    CRServo Intake;
    OpenCvCamera webcam;
    GamePropRight.gamePropPosition propPosition = GamePropRight.gamePropPosition.LEFT;
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
        Pose2d purplePixelPose = new Pose2d(0, 0, Math.toRadians(0.0));
        Pose2d leftPose  = new Pose2d(0, 0, Math.toRadians(0.0));
        switch (propPosition) { //UPDATE THESE POSITIONS
            case LEFT:
            telemetry.addData("Left", "Left");
            wallPose1 = new Pose2d(24,-5, Math.toRadians(90));
            wallPose2 = new Pose2d(47,-16.5, Math.toRadians(90));
            nitPose = new Pose2d(56,78, Math.toRadians(90));
            innitPose = new Pose2d(24.3,88.5, Math.toRadians(90));
            purplePixelPose = new Pose2d(25, 2, Math.toRadians(-90.0));
            leftPose = new Pose2d(24.25, 0, Math.toRadians(0.0));
            break;
            case CENTER:
                telemetry.addData("Center Position", "Center");
                wallPose1 = new Pose2d(47,-15.5, Math.toRadians(90));
                wallPose2 = new Pose2d(47,-16.5, Math.toRadians(90));
                nitPose = new Pose2d(56,78, Math.toRadians(90));
                innitPose = new Pose2d(31,88.5, Math.toRadians(90));
                purplePixelPose = new Pose2d(47, 3, Math.toRadians(0.0));
                leftPose = new Pose2d(46.5, 3, Math.toRadians(0.0));
                break;
            case RIGHT:
                wallPose1 = new Pose2d(47,-15.5, Math.toRadians(90));
                wallPose2 = new Pose2d(47,-16.5, Math.toRadians(90));
                nitPose = new Pose2d(56,78, Math.toRadians(90));
                innitPose = new Pose2d(37,89.2, Math.toRadians(90));
                leftPose = new Pose2d(40.25,-7.5, Math.toRadians(0));
                purplePixelPose = new Pose2d(40.5,-7.5, Math.toRadians(0));

                break;

        }

       // Pose2d parkingPose = new Pose2d(24, 70, Math.toRadians(0)); //UPDATE

        telemetry.update();
        sleep(200);

        robot = new SampleMecanumDrive(hardwareMap);
        robot.followTrajectorySequence(robot.trajectorySequenceBuilder(initPose) //Starting Pose
               // .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {Arm.setTargetPosition(0);Arm.setPower(0.2);Arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);})
                .lineToLinearHeading(leftPose)
                .lineToLinearHeading(purplePixelPose)//Drop Purple Pixel
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {Arm.setTargetPosition(0);Arm.setPower(0.2);Arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);})
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {intakeLeft.setPosition(0.475);})
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {intakeRight.setPosition(0.375);})
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {Intake.setPower(-0.3);})
                .waitSeconds(0.75)
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {intakeLeft.setPosition(0);})
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {intakeRight.setPosition(1);})
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {Intake.setPower(0);})
                .forward(8)
                .lineToLinearHeading(wallPose1)
                .lineToLinearHeading(wallPose2)
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {Arm.setTargetPosition(230);Arm.setPower(0.2);Arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);})
                .waitSeconds(.5)
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {Intake.setPower(0.75);})
                .setConstraints(SampleMecanumDrive.getVelocityConstraint(36.0, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL/5.0))
                .back(2)
                .resetConstraints()
                .waitSeconds(0.8)
                /*.UNSTABLE_addTemporalMarkerOffset(0.0, () -> {intakeLeft.setPosition(0.62);})
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {intakeRight.setPosition(0.1);})
                .waitSeconds(1.5)
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {intakeLeft.setPosition(0);})
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {intakeRight.setPosition(1);})*/
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {Intake.setPower(0);})
                //.UNSTABLE_addTemporalMarkerOffset(0.1, () -> {YPD.setPosition(0.3);})
                .resetConstraints()
                .setConstraints(SampleMecanumDrive.getVelocityConstraint(80.0, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineToLinearHeading(nitPose)
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {Intake.setPower(0.75);})
                .lineToLinearHeading(innitPose)
                .resetConstraints()
                .setConstraints(SampleMecanumDrive.getVelocityConstraint(35.0, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL/5.0))
                .forward(5)
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {YPD.setPosition(0.11);})
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {Arm.setTargetPosition(3300);Arm.setPower(0.7);Arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);})
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {Box.setPosition(0.3);})
                .waitSeconds(3.5)
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {YPD.setPosition(0.84);})
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {Intake.setPower(-0.2);})
                .waitSeconds(1)
                .resetConstraints()
               // .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {Intake.setPower(-0.2);})
                .waitSeconds(1.5)
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {Intake.setPower(0);})
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {Box.setPosition(0.84);})
                .waitSeconds(0.2)
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {Arm.setTargetPosition(0);Arm.setPower(0.5);Arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);})
                .waitSeconds(4)
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

        webcam.setPipeline(new GamePropRight());
    }
	
	public GamePropRight.gamePropPosition getPropPosition() {
		return GamePropRight.position;
	}
	
}