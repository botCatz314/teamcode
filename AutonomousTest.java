package org.firstinspires.ftc.teamcode.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous (name = "AutonomousTest")
public class AutonomousTest extends LinearOpMode {

    private DcMotor leftMotor1, leftMotor2, rightMotor1, rightMotor2; // declare drive motor variables
    public double stopBot = 0;
    private Servo phoneServo;
@Override
    public void runOpMode() {
    leftMotor1 = hardwareMap.dcMotor.get("leftMotor1"); //set left drive motor
    leftMotor2 = hardwareMap.dcMotor.get("leftMotor2"); //set left drive motor

    rightMotor1 = hardwareMap.dcMotor.get("rightMotor1"); //set right drive motor
    rightMotor2 = hardwareMap.dcMotor.get("rightMotor2"); //set right drive motor

    phoneServo = hardwareMap.servo.get("rejector");

    leftMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); //sets left motor to run without encoder
    leftMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); //sets left motor to run without encoder

    rightMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); //sets right motor to run without encoder
    rightMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); //sets right motor to run without encoder

    leftMotor1.setDirection(DcMotorSimple.Direction.REVERSE);
    leftMotor2.setDirection(DcMotorSimple.Direction.REVERSE);
    phoneServo.setDirection(Servo.Direction.REVERSE);


    waitForStart();
phoneServo.setPosition(1);
sleep(1000);
phoneServo.setPosition(0);
}
    private void MoveRobot(double leftPower, double rightPower){
    rightMotor1.setPower(rightPower);
    leftMotor1.setPower(leftPower);
    rightMotor2.setPower(rightPower);
    leftMotor2.setPower(leftPower);

    }
    private void StopBot(){
    leftMotor1.setPower(stopBot);
    rightMotor2.setPower(stopBot);
    leftMotor2.setPower(stopBot);
    rightMotor1.setPower(stopBot);
    }
}
