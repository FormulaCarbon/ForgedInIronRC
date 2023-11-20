package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Arm;
@Autonomous(name = "intaketest")
public class intaketest extends LinearOpMode {
    @Override
    public void runOpMode() {
        Arm botArm = new Arm();
        botArm.init(hardwareMap, this);
        // Put initialization blocks here.
        waitForStart();
        isStopRequested();
        if (opModeIsActive()) {
            botArm.intakeOn();
            sleep(1000);
            botArm.intakeOff();
            // Put run blocks here.
            while (opModeIsActive()) {

            }
        }
    }
}
