package org.firstinspires.ftc.teamcode.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
@Disabled
@Autonomous (name= "LukesBruteForce")
public class LukesBruteForce extends LinearOpMode {
    private DcMotor leftF;
    private DcMotor rightF;
    private  DcMotor leftB;
    private  DcMotor rightB;
    private Servo rejectionServo;

    public void runOpMode() {
        leftF = hardwareMap.dcMotor.get("leftF");
        rightF = hardwareMap.dcMotor.get("rightF");
        leftB = hardwareMap.dcMotor.get("leftB");
        rightB = hardwareMap.dcMotor.get("rightB");
        rejectionServo = hardwareMap.servo.get("rejectionServo");
        rightB.setDirection(DcMotorSimple.Direction.REVERSE);
        rightF.setDirection(DcMotorSimple.Direction.REVERSE);
        telemetry.addData("leftF: ", leftF);
        telemetry.addData("rightF: ", rightF);
        telemetry.addData("leftB: ", leftB);
        telemetry.addData("rightB", rightB);
        telemetry.addData("servoA", rejectionServo);
        telemetry.update();

        waitForStart();
     runMotors(-0.5);
     sleep(250);
     turnleftandright( .5, true);
     sleep(1000);
     runMotors( -1);
     sleep(100);
     runMotors(0);
     setServoA();
     sleep(100);
     runMotors( -1);
     sleep(2500);
    }
    private void runMotors(double power) {
        leftF.setPower(power);
        rightF.setPower(power);
        leftB.setPower(power);
        rightB.setPower(power);
    }
    private void setServoA(){
        rejectionServo.setPosition(.556);
    }

        private void turnleftandright (double power, boolean isRight){
        if(isRight){
            leftF.setPower(power);
            rightF.setPower(-power);
            leftB.setPower(power);
            rightB.setPower(-power);
        }
        else{
            leftF.setPower(-power);
            rightF.setPower(power);
            leftB.setPower(-power);
            rightB.setPower(power);
        }

    }

}
