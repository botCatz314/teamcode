package org.firstinspires.ftc.teamcode.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
@Disabled
@Autonomous (name = "AutonomousEzra")
public class autonomuosEzra extends LinearOpMode {

    private DcMotor leftF, rightF, leftB, rightB,hangingMotor ; // declare drive motor variables
    public double stopBot = 0;
    @Override
    public void runOpMode() {
        leftF = hardwareMap.dcMotor.get("leftF"); //set leftF drive motor
        rightF = hardwareMap.dcMotor.get("rightF"); //set rightF drive motor
        leftB = hardwareMap.dcMotor.get("leftB"); //set leftB drive motor
        rightB = hardwareMap.dcMotor.get("rightB"); //set rightB drive motor
        hangingMotor =hardwareMap.dcMotor.get("hangingMotor");

        leftF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); //sets leftF motor to run without encoder
        rightF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); //sets rightF motor to run without encoder
        leftB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); //sets leftB motor to run without encoder
        rightB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); //sets rightB motor to run without encoder
        hangingMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    waitForStart();

    //drop
    hangingMotor.setPower(1);
    sleep(1000);
    StopBot();

    //move foreword
    MoveRobot(1,1,1,1);
    sleep(1000);
    StopBot();

    //strafe
    MoveRobot(1,-1,-1,1);
    sleep(1000);
    StopBot();


    }
    private void MoveRobot(double leftFPower, double rightFPower, double leftBPower, double rightBPower){
        rightF.setPower(rightFPower);// give power to the right motor
        leftF.setPower(leftFPower);// give left motor power
        rightB.setPower(rightBPower);// give power to the right motor
        leftB.setPower(leftBPower);// give left motor power
    }
    private void StopBot(){
        leftF.setPower(stopBot);// stops left motor
        rightF.setPower(stopBot);// stops right motor
        leftB.setPower(stopBot);// stops left motor
        rightB.setPower(stopBot);// stops right motor
        hangingMotor.setPower(stopBot);
    }
}