package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
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

    public void intakeOn(long time)
    {
        LeftIntakeServo.setPower(1);
        RightIntakeServo.setPower(-1);

        if (time > 0)
        {
            MyOp.sleep(time);
        }

        LeftIntakeServo.setPower(0);
        RightIntakeServo.setPower(0);
    }

    public void intakeOff()
    {
        LeftIntakeServo.setPower(0);
        RightIntakeServo.setPower(0);
    }

    public void intakeReverse(long time)
    {
        LeftIntakeServo.setPower(-1);
        RightIntakeServo.setPower(1);

        if (time > 0)
        {
            MyOp.sleep(time);
        }

        LeftIntakeServo.setPower(0);
        RightIntakeServo.setPower(0);
    }

}