package org.firstinspires.ftc.teamcode.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "RI3DTeleOp", group = "Default")
public class RI3DTeleOp extends LinearOpMode {
    //declares wheel motors
    private DcMotor leftWheel, rightWheel;
    private DcMotor attachment;
    //declares servos
    private Servo servoAttachment;
    //private CRServo servoAttachment;

    //controls whether there is one pad or two
    private boolean twoPads = false;


@Override
    public void runOpMode() {


    //defines drive motors
    leftWheel = hardwareMap.dcMotor.get("leftWheel");
    rightWheel = hardwareMap.dcMotor.get("rightWheel");
    //defines attachments
    attachment = hardwareMap.dcMotor.get("attachment");
    servoAttachment = hardwareMap.servo.get("servoAttachment");
    //servoAttachment = hardwareMap.crservo.get("servoAttachment");
    //changes from
    leftWheel.setDirection(DcMotorSimple.Direction.REVERSE);

    waitForStart();

    while (opModeIsActive()) {
        //if using two game pads
        if(twoPads){
            //moves wheels
            leftWheel.setPower(gamepad1.left_stick_y);
            rightWheel.setPower(gamepad1.right_stick_y);
            //moves arm
            attachment.setPower(gamepad2.right_stick_y);
            //opens and closes servo
            if(gamepad2.a){
                servoAttachment.setPosition(1);
            }else if(gamepad2.x) {
                servoAttachment.setPosition(0);
            }
            /*servoAttachment.setPower(gamepad1.right_trigger);
            servoAttachment.setPower(-gamepad1.left_trigger);*/
        }else{ //if using only one game pad
            //moves the wheels
            leftWheel.setPower(-gamepad1.left_stick_y);
            rightWheel.setPower(-gamepad1.right_stick_y);
            //moves the arm
            attachment.setPower(gamepad1.right_trigger);
            attachment.setPower(-gamepad1.left_trigger);
            //opens and closes the servo
            if(gamepad1.a){
                servoAttachment.setPosition(1);
            }else if(gamepad1.x){
                servoAttachment.setPosition(0);
            }
            /*
            if(gamepad1.a){
                servoAttachment.setPower(1);
            }else if(gamepad1.x){
                servoAttachment.setPower(-1);
            }else{
                servoAttachment.setPower(0);
            }
            */
        }
    }
}
}
