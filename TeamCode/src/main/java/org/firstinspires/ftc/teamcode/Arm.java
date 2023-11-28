package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Arm {
    public DcMotorEx LeftArmMotor;
    public DcMotorEx RightArmMotor;
    public Servo LeftTiltServo;
    public Servo RightTiltServo;
    public CRServo LeftIntakeServo;
    public CRServo RightIntakeServo;

    static private double GEAR_RATIO = 71.2;

    static double ARM_COUNT_PER_DEGREE = 1993.6 / 360;

    HardwareMap hwMap = null;
    LinearOpMode MyOp = null;

    public Arm(){} // Constructor - Leave Blank

    // ------------ Claw ------------

    public void intakeOn()
    {
        LeftIntakeServo.setPower(-1);
        RightIntakeServo.setPower(1);
    }

    public void intakeOff()
    {
        LeftIntakeServo.setPower(0);
        RightIntakeServo.setPower(0);
    }

    public void intakeReverse()
    {
        LeftIntakeServo.setPower(1);
        RightIntakeServo.setPower(-1);
    }

    // ------------ Arm ------------

    public void setArmAngle(double angle, double speed)
    {
        LeftArmMotor.setTargetPosition( (int) (angle * ARM_COUNT_PER_DEGREE));
        RightArmMotor.setTargetPosition( (int) -(angle * ARM_COUNT_PER_DEGREE));

        LeftArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        LeftArmMotor.setVelocity(speed * ARM_COUNT_PER_DEGREE);
        RightArmMotor.setVelocity(speed * ARM_COUNT_PER_DEGREE);
    }

    public void setArmPos(double arm, double armPos)
    {
        LeftArmMotor.setTargetPosition((int) (armPos));
        RightArmMotor.setTargetPosition((int) (-armPos));

        LeftArmMotor.setPower(arm);
        RightArmMotor.setPower(arm);
    }

    // ------------ Tilt ------------

    public void setIntakeTilt(double angle)
    {
        LeftTiltServo.setPosition( (int) (angle / 180));
        RightTiltServo.setPosition( (int) 1.0-(angle / 180));
    }

    public void resetIntakeTilt()
    {
        LeftTiltServo.setPosition(0.48);
        RightTiltServo.setPosition(0.52);
    }

    public void init(HardwareMap map, LinearOpMode OpMode)
    {
        hwMap = map;
        MyOp = OpMode;

        LeftArmMotor = hwMap.get(DcMotorEx.class, "leftArm");
        RightArmMotor = hwMap.get(DcMotorEx.class, "rightArm");
        LeftTiltServo = hwMap.get(Servo.class, "leftIntakeTilt");
        RightTiltServo = hwMap.get(Servo.class, "rightIntakeTilt");
        LeftIntakeServo = hwMap.get(CRServo.class, "leftIntake");
        RightIntakeServo = hwMap.get(CRServo.class, "rightIntake");

        LeftArmMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        RightArmMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        LeftArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LeftArmMotor.setTargetPosition(0);
        RightArmMotor.setTargetPosition(0);

        LeftArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
}