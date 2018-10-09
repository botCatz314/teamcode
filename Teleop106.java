package org.firstinspires.ftc.teamcode.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp(name = "TeleOp 10/6", group = "Default")
public class Teleop106 extends LinearOpMode {

    private DcMotor leftMotor1,leftMotor2,rightMotor1, rightMotor2; //declares motors
    @Override
    public void runOpMode()
    {
        leftMotor1 = hardwareMap.dcMotor.get("leftMotor1"); //sets value to left motor
        leftMotor2 = hardwareMap.dcMotor.get("leftMotor2");//sets value to left motor
        rightMotor1 = hardwareMap.dcMotor.get("rightMotor1"); //sets value to right motor
        rightMotor2 = hardwareMap.dcMotor.get("rightMotor2"); //sets value to right motor

        waitForStart();
        while (opModeIsActive())
        {
            leftMotor1.setPower(gamepad1.left_stick_y); //sets left motor power
            leftMotor2.setPower(gamepad1.left_stick_y); //sets left motor power
            rightMotor1.setPower(-gamepad1.right_stick_y); //sets right motor power
            rightMotor2.setPower(-gamepad1.right_stick_y); //sets right motor power

            idle();
        }
    }








}
