package org.firstinspires.ftc.teamcode.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous (name = "TouchTheSensors")
public class TouchTheSensor extends LinearOpMode {

    private DcMotor left, right; // declare drive motor variables

    private TouchSensor touchLeft, touchRight;

    private double powerOff = 0; //declares common powers that we use
@Override
    public void runOpMode() {
    //sets drive motors
    left = hardwareMap.dcMotor.get("left");
    right = hardwareMap.dcMotor.get("right");

// set touch sensor
    touchLeft = hardwareMap.get(TouchSensor.class,"touchLeft");
    touchRight = hardwareMap.get(TouchSensor.class, "touchRight");

   //sets up drive motors to our specification
    left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    left.setDirection(DcMotorSimple.Direction.REVERSE);

    waitForStart();
    DriveUntilTouch();

    }

    private void DriveUntilTouch(){
    while(!touchLeft.isPressed() && !touchRight.isPressed()){
        right.setPower(-0.1);
        left.setPower(-0.1);
    }
        left.setPower(powerOff);
        right.setPower(powerOff);
    }
}
