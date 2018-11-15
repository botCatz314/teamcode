package org.firstinspires.ftc.teamcode.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp(name = "Works Good", group = "Default")
public class teleop1114 extends LinearOpMode {

    private DcMotor left, right;
    @Override
    public void runOpMode()
    {
        left = hardwareMap.dcMotor.get("left"); //sets value to left motor
        right = hardwareMap.dcMotor.get("right"); //sets value to right motor
        right.setDirection(DcMotorSimple.Direction.REVERSE);


        waitForStart();
        while (opModeIsActive())
        {
            left.setPower(gamepad1.left_stick_y); //sets left motor power
            right.setPower(gamepad1.right_stick_y); //sets right motor power
            left.setPower(gamepad1.left_trigger); //sets left motor power
            right.setPower(gamepad1.left_trigger); //sets right motor power
            left.setPower(-gamepad1.right_trigger); //sets left motor power
            right.setPower(-gamepad1.right_trigger);
            idle();
        }
    }}
