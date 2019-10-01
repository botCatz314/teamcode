package org.firstinspires.ftc.teamcode.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;

@TeleOp(name = "RelicRecoveryTeleOp", group = "Default")
public class RelicRecoveryTeleOp extends LinearOpMode {
    //declares wheel motors
    private DcMotor leftWheel, rightWheel;
    //declares intake motors
    private DcMotor leftIntake, rightIntake;
    //declares the servos
    private CRServo flipper;

    private Servo upDown;
    //declares variables holding common wheel values
    float fullPower = 1.0f;
    float halfPower = 0.5f;
    float reducedPower = 0.2f;
    //controls the speed of the robot
    private double maxSpeed = fullPower;

@Override
    public void runOpMode() {
    //defines drive motors
    leftWheel = hardwareMap.dcMotor.get("leftWheel");
    rightWheel = hardwareMap.dcMotor.get("rightWheel");
    //defines attachment motors
    leftIntake = hardwareMap.dcMotor.get("leftIntake");
    rightIntake = hardwareMap.dcMotor.get("rightIntake");
    //defines servos
    flipper = hardwareMap.crservo.get("flipper");
    upDown = hardwareMap.servo.get("upDown");
    //reverses motors to ensure they all rotate forwards
    rightWheel.setDirection(DcMotorSimple.Direction.REVERSE);
    leftIntake.setDirection(DcMotorSimple.Direction.REVERSE);

    waitForStart();
    while (opModeIsActive()) {
        //sets intake power
        //goes forward by default
        //a to reverse
        //left bumper to reverse on one side
        if(gamepad1.a){
            leftIntake.setPower(-fullPower*0.6);
            rightIntake.setPower(-fullPower*0.6);
        }else if(gamepad1.left_bumper){
            leftIntake.setPower(-fullPower*0.6);
        }else{
            leftIntake.setPower(fullPower*0.6);
            rightIntake.setPower(fullPower*0.6);
        }
        //ensures left wheel is within the desired range of speeds
        if(gamepad1.left_stick_y <= maxSpeed){
            leftWheel.setPower(gamepad1.left_stick_y);
        }else{
            leftWheel.setPower(maxSpeed);
        }
        //ensures the right wheel is within the desired range of speeds
        if(gamepad1.right_stick_y <= maxSpeed){
            rightWheel.setPower(gamepad1.right_stick_y);
        }else{
            rightWheel.setPower(maxSpeed);
        }
        //toggles between max wheel power of 100% power and 20% power
        if(gamepad1.right_bumper && maxSpeed == reducedPower){
            maxSpeed = fullPower;
        }else if(gamepad1.right_bumper && maxSpeed == fullPower){
            maxSpeed = reducedPower;
        }
        //controls the flipper
        flipper.setPower(gamepad1.right_trigger);
        flipper.setPower(-gamepad1.left_trigger);

        if(gamepad1.x){
            upDown.setPosition(0);
        }else if(gamepad1.y){
            upDown.setPosition(1);
        }
    }
}
}
