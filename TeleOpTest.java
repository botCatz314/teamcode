package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp(name = "TeleOp Test", group = "Default")
public class TeleOpTest extends LinearOpMode {

    private DcMotor left, right; //declares motors
@Override
    public void runOpMode()
{

    left = hardwareMap.dcMotor.get("left"); //sets value to left motor
    right = hardwareMap.dcMotor.get("right"); //sets value to right motor

    waitForStart();
    while (opModeIsActive())
    {
        left.setPower(-gamepad1.left_stick_y); //sets left motor power
        right.setPower(-gamepad1.right_stick_y); //sets right motor power

        idle();
    }
}








}
