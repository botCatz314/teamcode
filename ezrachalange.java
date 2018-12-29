package org.firstinspires.ftc.teamcode.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous (name= "ezrachalange")
public class ezrachalange extends LinearOpMode {
    private ColorSensor colorRight;
    private DcMotor leftF;
    private DcMotor rightF;
    private DistanceSensor rangeHigh;

    public void runOpMode() {
        leftF = hardwareMap.dcMotor.get("leftF");
        leftF = hardwareMap.dcMotor.get("rightF");
        colorRight = hardwareMap.get(ColorSensor.class, "colorRight");
        rangeHigh = hardwareMap.get(DistanceSensor.class,"rangeHigh");
        waitForStart();
        leftF.setPower(-0.3);
        rightF.setPower(0.3);
     while(colorRight.blue() <160 || colorRight.red() <310){
         //do stuff
     }
     leftF.setPower(0.0);
     rightF.setPower(0.0);
     telemetry.addData("range: ", rangeHigh.getDistance(DistanceUnit.INCH));
     telemetry.update();
    }

}
