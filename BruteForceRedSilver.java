package org.firstinspires.ftc.teamcode.teamcode;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous (name = "BruteForceRedSilver")
public class BruteForceRedSilver extends LinearOpMode {

    private DcMotor left, right; // declare drive motor variables

    private double powerOff = 0; //declares common powers that we use
@Override
    public void runOpMode() {
    //sets drive motors
    left = hardwareMap.dcMotor.get("left");
    right = hardwareMap.dcMotor.get("right");
   //sets up drive motors to our specification
    left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    left.setDirection(DcMotorSimple.Direction.REVERSE);

    waitForStart();
    left.setPower(.5);
    right.setPower(1);
    sleep(1500);
left.setPower(-1);
right.setPower(1);
sleep(465);
left.setPower(1);
right.setPower(1);
sleep(2500);
right.setPower(1);
sleep(250);
left.setPower(-1);
right.setPower(-1);
sleep(2800);
}

}


