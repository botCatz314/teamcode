package org.firstinspires.ftc.teamcode.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
@Disabled
@TeleOp(name = "miniTeleOp", group = "Default")
public class miniTeleOp extends LinearOpMode {

    private DcMotor left, right;
    @Override
    public void runOpMode(){
        left = hardwareMap.dcMotor.get("left");
        right = hardwareMap.dcMotor.get("right");
        waitForStart();
        right.setPower(gamepad1.right_stick_y);
        left.setPower(gamepad1.left_stick_y);
        idle();
    }
}