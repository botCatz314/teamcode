package org.firstinspires.ftc.teamcode.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous (name = "AutonomousTest")
public class Ezras_atonamous extends LinearOpMode {

    private DcMotor left, right; // declare drive motor variables
    public double stopBot = 0;
@Override
    public void runOpMode() {
    left = hardwareMap.dcMotor.get("left"); //set left drive motor
    right = hardwareMap.dcMotor.get("right"); //set right drive motor
    left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); //sets left motor to run without encoder
    right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); //sets right motor to run without encoder

    waitForStart();
    MoveRobot(-1, 1); //left need to be negetive to go foreward.
    sleep(2000);
    StopBot();
    MoveRobot(1, 1);
    sleep(1000);
    MoveRobot(-1,1);
    sleep(2000);
    StopBot();
}
    private void MoveRobot(double leftPower, double rightPower){
    right.setPower(rightPower);
    left.setPower(leftPower);
    }
    private void StopBot(){
    left.setPower(stopBot);
    right.setPower(stopBot);
    }
}
