package org.firstinspires.ftc.teamcode.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "MTF TeleOp", group = "Default")
public class MTFTeleOp extends LinearOpMode {

    private DcMotor left, right,  pivotMotor, slideMotor; //declares
    private CRServo collector;

    private boolean collecting;
@Override
    public void runOpMode()
{
    left = hardwareMap.dcMotor.get("left"); //sets value to left motor
    right = hardwareMap.dcMotor.get("right"); //sets value to right motor
    pivotMotor = hardwareMap.dcMotor.get("pivotMotor");
    slideMotor = hardwareMap.dcMotor.get("slideMotor");
    collector = hardwareMap.crservo.get("collector");








    left.setDirection(DcMotorSimple.Direction.REVERSE);


    waitForStart();
    while (opModeIsActive())
    {
        left.setPower(gamepad1.left_stick_y); //sets left motor power

        right.setPower(gamepad1.right_stick_y); //sets right motor power

        if(gamepad2.x)
        {
            if (collecting){
                collecting = false;
                collector.setPower(0);
            }
            else if (!collecting){
                collecting = true;
                collector.setPower(1);
            }
    }

        slideMotor.setPower(gamepad2.right_stick_y);
        pivotMotor.setPower(gamepad2.left_stick_y);



        idle();
    }
}








}
