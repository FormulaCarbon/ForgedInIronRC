package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Config
@Autonomous(name = "armtest")
public class armtest extends LinearOpMode {
    public static double armPower = 2.0;
    public static double armScale = 80.0;
    @Override
    public void runOpMode() {

        Arm botArm = new Arm();
        botArm.init(hardwareMap, this);
        // Put initialization blocks here.
        waitForStart();
        isStopRequested();
        if (opModeIsActive()) {

            while (opModeIsActive()) {
                botArm.setArmPos(armPower, armScale);
                botArm.setIntakeTilt(180);
                // Put run blocks here.
                telemetry.addData("armpos left", botArm.LeftArmMotor.getCurrentPosition());
                telemetry.addData("armpos right", botArm.RightArmMotor.getCurrentPosition());
                telemetry.addData("armtarget left", botArm.LeftArmMotor.getTargetPosition());
                telemetry.addData("armtarget right", botArm.RightArmMotor.getTargetPosition());
                telemetry.update();
            }
        }
    }
}
