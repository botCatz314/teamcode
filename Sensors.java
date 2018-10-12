package org.firstinspires.ftc.teamcode.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous (name = "Sensors")
public class Sensors extends LinearOpMode {

    private DcMotor left, right; // declare drive motor variables
    private DistanceSensor rangeLeft, rangeRight;
@Override
    public void runOpMode() {
    left = hardwareMap.dcMotor.get("left"); //set left drive motor
    right = hardwareMap.dcMotor.get("right"); //set right drive motor
    rangeLeft = hardwareMap.get(DistanceSensor.class, "rangeLeft");//sets range left sensor
    rangeRight = hardwareMap.get(DistanceSensor.class, "rangeRight");
    left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); //sets left motor to run without encoder
    right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); //sets right motor to run without encoder
    left.setDirection(DcMotorSimple.Direction.REVERSE);//sets left motor to reverse

    waitForStart();

    DistancetoRate(20, DistanceUnit.INCH, 20);
    sleep(2000);
    }

    private boolean InRangeLeft(double target, DistanceUnit units){
    double distance; //creates a variable to hold the range sensor's reading
    distance = rangeLeft.getDistance(units); //sets the distance variable to the value that the range sensor reads
    return (distance <= target); //returns true if the robot is closer to the target than the target position
    }

    private boolean InRangeRight(double target, DistanceUnit units){
    double distanceRight;
    distanceRight = rangeRight.getDistance(units);
    return ( distanceRight <= target);
    }

    private void DriveUntilDistance(double target, DistanceUnit unit){

    while(!InRangeLeft(target +10, unit) ) {
        left.setPower(0.5);
        right.setPower(0.5);
    }
    while (!InRangeLeft(target, unit )){
        left.setPower(0.1);
        right.setPower(0.1);
    }
    left.setPower(0);
    right.setPower(0);
    }

    private void DistancetoRate(double stoptarget, DistanceUnit unit, double time){
    double distanceRight, distanceLeft;
    double linearRateLeft, linearRateRight;
    double angularRateLeft, angularRateRight;
    double radius = 2;
    while(!InRangeLeft(stoptarget, unit)){
        distanceLeft = rangeLeft.getDistance(unit) - stoptarget;
        distanceRight = rangeRight.getDistance(unit) - stoptarget;
        linearRateLeft = distanceLeft / time;
        linearRateRight = distanceRight / time;
        angularRateLeft = linearRateLeft / radius;
        angularRateRight = linearRateRight / radius;

        left.setPower(angularRateLeft);
        right.setPower(angularRateRight);

        telemetry.addData("left: ", left.getPower());
        telemetry.addData("right: ", right.getPower());
        telemetry.addData("range: ", rangeLeft.getDistance(unit));
        telemetry.addData("range right: ", rangeRight.getDistance(unit));
        telemetry.update();

    }
    right.setPower(0);
    left.setPower(0);
    sleep(1000);
    if(rangeRight.getDistance(unit ) > rangeLeft.getDistance(unit)){
        while(!InRangeLeft(rangeRight.getDistance(unit), unit)) {
            left.setPower(0.1);
            }
        left.setPower(0);
        }

    if(rangeLeft.getDistance(unit) > rangeRight.getDistance(unit)){
        while(!InRangeRight(rangeLeft.getDistance(unit), unit)){
            right.setPower(0.1);
        }
        right.setPower(0);
    }
    }



}
