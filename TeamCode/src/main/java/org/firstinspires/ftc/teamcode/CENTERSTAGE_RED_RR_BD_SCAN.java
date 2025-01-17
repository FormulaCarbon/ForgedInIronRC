package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "CENTERSTAGE RED RR BD SP")
public class CENTERSTAGE_RED_RR_BD_SCAN extends LinearOpMode {
    private OpenCvCamera webcam;

    private static final int CAMERA_WIDTH = 1280; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 960; // height of wanted camera resolution

    private RedObjectPipeline.ElementPosition DropPos;
    Arm botArm = new Arm();

    @Override
    public void runOpMode() {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(8, -63, Math.toRadians(90));

        drive.setPoseEstimate(startPose);

        TrajectorySequence toLeftDrop = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(12, -32 , Math.toRadians(180)))
                .build();

        TrajectorySequence toRightDrop = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(10, -26, Math.toRadians(0)))
                .build();

        TrajectorySequence toCenterDrop = drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(12, -35))
                .build();

        TrajectorySequence leftDropToPark = drive.trajectorySequenceBuilder(toLeftDrop.end())
                .lineToLinearHeading(new Pose2d(48, -32))
                .build();

        TrajectorySequence rightDropToPark = drive.trajectorySequenceBuilder(toRightDrop.end())
                .lineTo(new Vector2d(10, -32))
                .splineToConstantHeading(new Vector2d(10, -48), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(48, -32, Math.toRadians(180)), Math.toRadians(90))
                .build();

        TrajectorySequence centerDropToPark = drive.trajectorySequenceBuilder(toCenterDrop.end())
                .lineTo(new Vector2d(12, -38))
                .lineToSplineHeading(new Pose2d(60, -36, Math.toRadians(180)))
                .build();


        botArm.init(hardwareMap, this);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam"), cameraMonitorViewId);

        //OpenCV Pipeline
        RedObjectPipeline myPipeline;
        webcam.setPipeline(myPipeline = new RedObjectPipeline());

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


        if (DropPos == RedObjectPipeline.ElementPosition.LEFT) {
            drive.followTrajectorySequence(toLeftDrop);

            botArm.intakeReverse();
            sleep(1000);
            botArm.intakeOff();

            drive.followTrajectorySequence(leftDropToPark);

        } else if (DropPos == RedObjectPipeline.ElementPosition.CENTER) {
            drive.followTrajectorySequence(toCenterDrop);

            botArm.intakeReverse();
            sleep(1000);
            botArm.intakeOff();

            drive.followTrajectorySequence(centerDropToPark);

        } else if (DropPos == RedObjectPipeline.ElementPosition.RIGHT) {
            drive.followTrajectorySequence(toRightDrop);

            botArm.intakeReverse();
            sleep(1000);
            botArm.intakeOff();

            drive.followTrajectorySequence(rightDropToPark);
        }

        if (isStopRequested()) return;

    }

}
