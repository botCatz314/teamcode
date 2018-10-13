package org.firstinspires.ftc.teamcode.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;


import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous (name = "Sensors")
public class Sensors extends LinearOpMode {

    private DcMotor left, right; // declare drive motor variables

    private DistanceSensor rangeLeft, rangeRight; //declares range sensor variables

    private ColorSensor colorLeft, colorRight; //declares color sensor variables

    private double powerOff = 0; //declares common powers that we use
@Override
    public void runOpMode() {
    //sets drive motors
    left = hardwareMap.dcMotor.get("left");
    right = hardwareMap.dcMotor.get("right");
    //sets range sensors
    rangeLeft = hardwareMap.get(DistanceSensor.class, "rangeLeft");
    rangeRight = hardwareMap.get(DistanceSensor.class, "rangeRight");
    //sets color sensors
    colorLeft = hardwareMap.get(ColorSensor.class, "colorLeft");
    colorRight = hardwareMap. get(ColorSensor.class, "colorRight");

   //sets up drive motors to our specification
    left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    left.setDirection(DcMotorSimple.Direction.REVERSE);

    waitForStart();

   // DrivetoLine(45,29, 0.2, 0.2, true); //drives to line in auto

   //  sleep(5000);

   // lineUp(45, 29, 0.1, 0.1); //straightens robot
  /*  telemetry.addData("RangeLeft: ", rangeLeft.getDistance(DistanceUnit.INCH));
    telemetry.update();

    while(!InRangeLeft(35, DistanceUnit.INCH)){
        right.setPower(.8);
        left.setPower(.8);
    }

    left.setPower(0);
    right.setPower(0);
*/
    Straighten(DistanceUnit.INCH);
    telemetry.addData("left: ", rangeLeft.getDistance(DistanceUnit.INCH));
    telemetry.addData("right: ", rangeRight.getDistance(DistanceUnit.INCH));
    telemetry.update();
    sleep(2000);

    //DrivetoLine(35,20,0.15,0.15,true);

   // left.setPower(-1);
   // right.setPower(-1);

    //sleep(3000);

   // right.setPower(0);
   // left.setPower(0);

    //  DrivetoLine(30, 20, 0.4, 0.4, false); //drives to team zone

    telemetry.addData("right: ", colorRight.blue());
    telemetry.update();

    sleep(4000);
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
    //converts the distance read by the range sensor to a speed for drive motors
    private void DistancetoRate(double stoptarget, DistanceUnit unit, double time){
    //sets variables to hold the values of the distance that the robot needs to travel
    double distanceRight, distanceLeft;
    //declares a variable to hold the linear rate
    double linearRateLeft, linearRateRight;
    //declares a variable to hold the angular rate
    double angularRateLeft, angularRateRight;
    //the radius of the wheel in inches
    double radius = 2;
    //runs until range sensor reads a specified distance at which point the robot stops
    while(!InRangeLeft(stoptarget, unit)){
        //sets the distance variables to the range sensor to the difference of the range sensor reading and target location
        distanceLeft = rangeLeft.getDistance(unit) - stoptarget;
        distanceRight = rangeRight.getDistance(unit) - stoptarget;
        //sets linear rate to the distance we want to travel divided by the target time
        linearRateLeft = distanceLeft / time;
        linearRateRight = distanceRight / time;
        //converts linear rate into angular rate by dividing it by the radius of the wheel
        angularRateLeft = linearRateLeft / radius;
        angularRateRight = linearRateRight / radius;
        //sets drive motors to the angular rate value
        left.setPower(angularRateLeft);
        right.setPower(angularRateRight);
    }
    //turns off drive motors
    right.setPower(powerOff);
    left.setPower(powerOff);
    //stopped here for cleaning up-direction = up
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
    //determines if the color sensor's reading is in between two values
    private boolean WithinColorRange(int max, int min, ColorSensor sensor){
    //declares and sets a variable equal to the color sensor reading
    int color = sensor.blue();
    //returns true if the color sensor is less than or equal to the max value and less than or equal to the min value
    return(color <= max && color >= min);

    }
    //uses color sensors to square along a colored line
    private void lineUp(int max, int min, double leftPower, double rightPower){
   //moves right wheel until it is on the color sensor reads the line's color range
    while(!WithinColorRange(max, min, colorRight)){
        //sets right drive power
        right.setPower(rightPower);
     }
     //turns off right drive power
    right.setPower(powerOff);
   //moves left motor until the color sensor reads the line's color range
    while(!WithinColorRange(max, min, colorLeft)){
            //sets left drive power
            left.setPower(leftPower);
        }
        //turns off left drive power
        left.setPower(powerOff);
    //moves left motor until the color sensor reads the non-line's color range
    while(!WithinColorRange(23, 11, colorLeft)){
        //sets left drive power
        left.setPower(leftPower);
    }
    //turns off left drive power
    left.setPower(powerOff);
    //moves right motor forward and left motor backwards until right motor reads the non-line's color range
    while(!WithinColorRange(21, 11, colorRight)){
        //sets drive motor's powers
        right.setPower(0.1);
        left.setPower(-0.1);
    }
    //turns off drive motors
    left.setPower(powerOff);
    right.setPower(powerOff);
    }

    //stops the robot when the color sensor reads within the set range
    private void DrivetoLine(int max, int min, double leftPower, double rightPower, boolean backup){
    //runs until it is within the set range
    while(!WithinColorRange(max, min, colorRight)){
        //sets drive motors to respective powers set in the calling of the method
        right.setPower(rightPower);
        left.setPower(leftPower);
        }
        //turns power off
        left.setPower(powerOff);
        right.setPower(powerOff);
        //if backup variable set in calling of the method is true
        if(backup){
            //back up until right behind the line we drove to
            while(!WithinColorRange(23,11, colorRight)){
                right.setPower(-0.1);
                left.setPower(-0.1);
            }
            //turns drive power off
            left.setPower(powerOff);
            right.setPower(powerOff);
        }
    }
    //uses the range sensor to square robot relative to a surface
    private void Straighten(DistanceUnit units){
    //declares variables to hold range sensor reading
    double rightRange, leftRange;
    //sets the initial values of the variables to the range sensors reading
    rightRange = rangeRight.getDistance(units);
    leftRange = rangeLeft.getDistance(units);
    //runs until we are straight
    while(rightRange != leftRange){
        //updates values of range variables
        rightRange = rangeRight.getDistance(units);
        leftRange = rangeLeft.getDistance(units);
        //turns right if the right distance is less than the left
        if(rightRange < leftRange){
            //sets motor powers
            right.setPower(-0.2);
            left.setPower(0.2);
        }
        //turns off motor power
        right.setPower(powerOff);
        left.setPower(powerOff);
        //turns left if the left range is less than the right
        if(leftRange < rightRange){
            //sets motor power
            left.setPower(-0.2);
            right.setPower(0.2);
        }
        //turns drive motor power off
        right.setPower(powerOff);
        left.setPower(powerOff);
    }

    }
}
