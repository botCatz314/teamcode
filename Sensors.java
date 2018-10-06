package org.firstinspires.ftc.teamcode.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous (name = "AutonomousTest")
public class Sensors extends LinearOpMode {

    private DcMotor left, right; // declare drive motor variables
    private DistanceSensor range;
    public double stopBot = 0;
@Override
    public void runOpMode() {
    left = hardwareMap.dcMotor.get("left"); //set left drive motor
    right = hardwareMap.dcMotor.get("right"); //set right drive motor
    range = hardwareMap.get(DistanceSensor.class, "range");
    left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); //sets left motor to run without encoder
    right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); //sets right motor to run without encoder

    waitForStart();
    while(opModeIsActive()) {
        telemetry.addData("Range in inches: ", range.getDistance(DistanceUnit.INCH));
        telemetry.addData("Range in CM: ", range.getDistance(DistanceUnit.CM));
        telemetry.addData("Range in Meters: ", range.getDistance(DistanceUnit.METER));
        telemetry.addData("Range in MM: ", range.getDistance(DistanceUnit.MM));
        telemetry.update();

    }
    }

}
