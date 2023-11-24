package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "CENTERSTAGE BLUE RR BD SDP")
public class CENTERSTAGE_BLUE_RR_BD_DROP extends LinearOpMode {

    public static double armPower = 0.3;
    public static double armDropPos = -1600;
    public static double intakeDropAngle = 0.0;

    private OpenCvCamera webcam;

    private static final int CAMERA_WIDTH = 1280; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 720; // height of wanted camera resolution

    private BlueObjectPipeline.ElementPosition DropPos;
    Arm botArm = new Arm();

    @Override
    public void runOpMode() {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(8, 63, Math.toRadians(270));

        drive.setPoseEstimate(startPose);

        TrajectorySequence toLeftDrop = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(6, 26, Math.toRadians(0)))
                .build();

        TrajectorySequence toRightDrop = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(18, 32, Math.toRadians(180)))
                .build();

        TrajectorySequence toCenterDrop = drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(12, 38))
                .build();

        TrajectorySequence leftDropToPark = drive.trajectorySequenceBuilder(toLeftDrop.end())
                .lineTo(new Vector2d(5, 32))
                .splineToConstantHeading(new Vector2d(10, 48), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(48, 38, Math.toRadians(180)), Math.toRadians(270))
                .build();

        TrajectorySequence rightDropToPark = drive.trajectorySequenceBuilder(toRightDrop.end())
                .lineTo(new Vector2d(52, 26))
                .build();

        TrajectorySequence centerDropToPark = drive.trajectorySequenceBuilder(toCenterDrop.end())
                .lineTo(new Vector2d(12, 40))
                .lineToSplineHeading(new Pose2d(52, 36, Math.toRadians(180)))
                .build();


        botArm.init(hardwareMap, this);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam"), cameraMonitorViewId);

        //OpenCV Pipeline
        BlueObjectPipeline myPipeline;
        webcam.setPipeline(myPipeline = new BlueObjectPipeline());

        // Webcam Streaming
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT); // Was Upright

            }

            @Override
            public void onError(int errorCode) {
                telemetry.addLine("Camera Opening Error !!!!!");
                telemetry.update();
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });

        FtcDashboard.getInstance().startCameraStream(webcam, 0);

        telemetry.addData("Push Camera Stream and tap screen to update image \nAlign the square to the cone \n \n Wait at least 5 Sec before Pressing Start", "");
        telemetry.update();

        waitForStart();

        DropPos = myPipeline.getAnalysis();


        if (DropPos == BlueObjectPipeline.ElementPosition.LEFT) {
            botArm.setArmPos(2, -200);
            sleep(1000);
            drive.followTrajectorySequence(toLeftDrop);

            botArm.intakeReverse();
            sleep(1000);
            botArm.intakeOff();

            drive.followTrajectorySequence(leftDropToPark);

            sleep(2000);

            botArm.setArmPos(armPower, armDropPos-2000.0);
            telemetry.update();
            botArm.setIntakeTilt(intakeDropAngle);
            sleep(1000);
            botArm.setArmPos(armPower, armDropPos);
            telemetry.update();
            sleep(2000);


            botArm.intakeReverse();
            sleep(1000);
            botArm.intakeOff();
            sleep(1000);

            botArm.setArmPos(armPower, 0);
            botArm.resetIntakeTilt();
            sleep(5000);
            telemetry.update();

        } else if (DropPos == BlueObjectPipeline.ElementPosition.CENTER) {
            botArm.setArmPos(2, -200);
            sleep(1000);
            drive.followTrajectorySequence(toCenterDrop);

            botArm.intakeReverse();
            sleep(1000);
            botArm.intakeOff();

            drive.followTrajectorySequence(centerDropToPark);

            sleep(2000);

            botArm.setArmPos(armPower, armDropPos-2000.0);
            telemetry.update();
            botArm.setIntakeTilt(intakeDropAngle);
            sleep(1000);
            botArm.setArmPos(armPower, armDropPos);
            telemetry.update();
            sleep(2000);


            botArm.intakeReverse();
            sleep(1000);
            botArm.intakeOff();
            sleep(1000);

            botArm.setArmPos(armPower, 0);
            botArm.resetIntakeTilt();
            sleep(5000);
            telemetry.update();

        } else if (DropPos == BlueObjectPipeline.ElementPosition.RIGHT) {
            botArm.setArmPos(2, -200);
            sleep(1000);
            drive.followTrajectorySequence(toRightDrop);

            botArm.intakeReverse();
            sleep(1000);
            botArm.intakeOff();

            drive.followTrajectorySequence(rightDropToPark);

            sleep(2000);

            botArm.setArmPos(armPower, armDropPos-2000.0);
            telemetry.update();
            botArm.setIntakeTilt(intakeDropAngle);
            sleep(1000);
            botArm.setArmPos(armPower, armDropPos);
            telemetry.update();
            sleep(2000);


            botArm.intakeReverse();
            sleep(1000);
            botArm.intakeOff();
            sleep(1000);

            botArm.setArmPos(armPower, 0);
            botArm.resetIntakeTilt();
            sleep(5000);
            telemetry.update();
        }

        if (isStopRequested()) return;

    }

}
