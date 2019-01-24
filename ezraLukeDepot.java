package org.firstinspires.ftc.teamcode.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.robotcore.util.Hardware;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.AnalogInput;

@Autonomous (name = "ezraLukeDepot")
public class ezraLukeDepot extends LinearOpMode {
    private DcMotor leftF, rightF, leftB, rightB, pivotMotor;

    @Override
    public void runOpMode() {
        leftF = hardwareMap.dcMotor.get("leftF");
        rightF = hardwareMap.dcMotor.get("rightF");
        leftB = hardwareMap.dcMotor.get("leftB");
        rightB = hardwareMap.dcMotor.get("rightB");
        pivotMotor = hardwareMap.dcMotor.get("pivotMotor");
        leftF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightF.setDirection(DcMotorSimple.Direction.REVERSE);
        rightB.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();
        streightGo(0.15,0.8,0.15,0.8);//starts here from right sample
        sleep(2000);
        StopBot();
        streightGo(0.6,0.6,0.6,0.6);//kinda turns left
        sleep(1000);
        StopBot();
        streightGo(-0.4,-0.4,-0.4,-0.4);//aligns to wall
        sleep(200);
        StopBot();
        pivotMotor.setPower(0.3);//releases cat
        sleep(100);
        pivotMotor.setPower(0);
        streightGo(-0.3,-0.3,-0.3,-0.3);//goes to crater
        sleep(10000);
        StopBot();
    }
    private void streightGo(double leftFPower, double rightFPower, double leftBPower, double rightBPower) {
        rightF.setPower(rightFPower);// give power to the right motor
        leftF.setPower(leftFPower);// give left motor power
        rightB.setPower(rightBPower);// give power to the right motor
        leftB.setPower(leftBPower);// give left motor power
    }
    private void leftStrayfe(double leftFPower, double rightFPower, double leftBPower, double rightBPower) {
        rightF.setPower(rightFPower);// give power to the right motor
        leftF.setPower(-leftFPower);// give left motor power
        rightB.setPower(-rightBPower);// give power to the right motor
        leftB.setPower(leftBPower);// give left motor power
    }
    private void rightStrayfe(double leftFPower, double rightFPower, double leftBPower, double rightBPower) {
        rightF.setPower(-rightFPower);// give power to the right motor
        leftF.setPower(leftFPower);// give left motor power
        rightB.setPower(rightBPower);// give power to the right motor
        leftB.setPower(-leftBPower);// give left motor power
    }
    private void StopBot(){
        leftF.setPower(0);// stops left motor
        rightF.setPower(0);// stops right motor
        leftB.setPower(0);// stops left motor
        rightB.setPower(0);// stops right motor
    }
}