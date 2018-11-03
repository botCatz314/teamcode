package org.firstinspires.ftc.teamcode.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous (name = "MtFAuto")
public class MtFAuto extends LinearOpMode {

    private DcMotor left, right, left1, right1; // declare drive motor variables

    private DcMotor hangingMotor;

    private DistanceSensor rangeLeft, rangeRight; //declares range sensor variables

    private ColorSensor colorLeft, colorRight; //declares color sensor variables

    private DigitalChannel touchLower, touchUpper; //declares touch sensor variables

    private Servo phoneServo;

    private GoldAlignDetector detector;

    private String position = null;

    private double powerOff = 0; //declares common powers that we use
@Override
    public void runOpMode() {
    //sets drive motors
    left = hardwareMap.dcMotor.get("left");
    right = hardwareMap.dcMotor.get("right");
    left1 = hardwareMap.dcMotor.get("left1");
    right1 = hardwareMap.dcMotor.get("right1");

    hangingMotor = hardwareMap.dcMotor.get("hangingMotor");

    //sets range sensors
    rangeLeft = hardwareMap.get(DistanceSensor.class, "rangeLeft");
    rangeRight = hardwareMap.get(DistanceSensor.class, "rangeRight");
    //sets color sensors
    colorLeft = hardwareMap.get(ColorSensor.class, "colorLeft");
    colorRight = hardwareMap. get(ColorSensor.class, "colorRight");
    //sets touch sensor variables
    touchLower = hardwareMap.get(DigitalChannel.class, "touchLower");
    touchUpper = hardwareMap.get(DigitalChannel.class, "touchUpper");

   //sets up drive motors to our specification
    left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    left1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    right1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    left.setDirection(DcMotorSimple.Direction.REVERSE);
    left1.setDirection(DcMotorSimple.Direction.REVERSE);
    hangingMotor.setDirection(DcMotorSimple.Direction.REVERSE);

    touchUpper.setMode(DigitalChannel.Mode.INPUT);
    touchLower.setMode(DigitalChannel.Mode.INPUT);

    //set up Doge CV Detector
    detector = new GoldAlignDetector();
    detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
    detector.useDefaults();

    detector.alignSize = 10;
    detector.alignPosOffset = 5000;
    detector.downscale = 0.4;

    detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA;
    detector.maxAreaScorer.weight = 0.005;

    detector.ratioScorer.weight = 5;
    detector.ratioScorer.perfectRatio = 0.8;

    detector.setAlignSettings(0, 1000);
    detector.enable();

   // phoneServo.setPosition(1);
    sleep(1000);

    waitForStart();
    HangingApparatus(); //lowers from hanging position
    /*DrivetoLine(23, 11, 0.4,0.4,true);//goes to blue line
    lineUp(23, 11, 0.2,0.2); //straightens
    GetGoldPos(); //determines where gold is
    Sampling(position); //hits the gold*/


    //get working through at least here
  /*  sleep(1000);
    DriveUntilDistanceReverse(10, DistanceUnit.INCH);
    TurnLeft(0.2, 200);
    DistancetoRate(15,DistanceUnit.INCH, 10);
    Straighten(DistanceUnit.INCH);
    TurnLeft(0.2, 300);
    DriveUntilDistance(30, DistanceUnit.INCH);
    DrivetoLine(23,11,0.4,0.4,false);
    */

   // DistancetoRate(20, DistanceUnit.INCH, 10);
   // Straighten(DistanceUnit.INCH);

    }
    private void TurnLeft(double power, long time){
        left.setPower(-power);
        right.setPower(power);
        right1.setPower(power);
        left1.setPower(-power);
        sleep(time);
        left.setPower(powerOff);
        right.setPower(powerOff);
        right1.setPower(powerOff);
        left1.setPower(powerOff);
    }
    private void TurnRight(double power, long time){
        left.setPower(power);
        right.setPower(-power);
        right1.setPower(-power);
        left1.setPower(power);
        sleep(time);
        left.setPower(powerOff);
        right.setPower(powerOff);
        right1.setPower(powerOff);
        left1.setPower(powerOff);
    }
    //returns whether the left range sensor is reading less than a certain value
    private boolean InRangeLeft(double target, DistanceUnit units){
    //creates a variable to hold the range sensor's reading
    double distance;
    //sets the distance variable to the value that the range sensor reads
    distance = rangeLeft.getDistance(units);
    //returns true if the robot is closer to the target than the target position
    return (distance <= target);
    }
    //returns whether the right range sensor is reading less than a certain value
    private boolean InRangeRight(double target, DistanceUnit units){
    //declares a variable to hold the range sensor's reading
    double distanceRight;
    //sets the distance variable to the value that the range sensor reads
    distanceRight = rangeRight.getDistance(units);
    //returns true if the robot is closer to the target than the target position
    return ( distanceRight <= target);
    }
    //powers drive motors until range sensor reads a certain target
    private void DriveUntilDistance(double target, DistanceUnit unit){
    //goes until ten sensor is ten units from the target
    while(!InRangeLeft(target +10, unit) ) {
        //powers drive motors
        left.setPower(0.5);
        right.setPower(0.5);
        right1.setPower(0.5);
        left1.setPower(0.5);
    }
    //goes until the sensor is within the range of the target
    while (!InRangeLeft(target, unit )){
        //sets drive power
        left.setPower(0.1);
        right.setPower(0.1);
        right1.setPower(0.1);
        left1.setPower(0.1);

    }
    //turns off drive motors
    left.setPower(powerOff);
    right.setPower(powerOff);
    right1.setPower(powerOff);
    left1.setPower(powerOff);
    }
    private void DriveUntilDistanceReverse(double target, DistanceUnit units){
    while(InRangeLeft(target + 10, units)){
        left.setPower(-0.5);
        right.setPower(-0.5);
        right1.setPower(-0.5);
        left1.setPower(-0.5);
        }
        while(InRangeLeft(target, units)){
        left.setPower(-0.1);
        right.setPower(-0.1);
        right1.setPower(-0.1);
        left1.setPower(-0.1);
        }
        left.setPower(powerOff);
        right.setPower(powerOff);
        right1.setPower(powerOff);
        left1.setPower(powerOff);
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
    while(!InRangeLeft(stoptarget, unit) && !InRangeRight(stoptarget, unit)){
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
        left1.setPower(angularRateLeft);
        right1.setPower(angularRateRight);
    }
    //turns off drive motors
    right.setPower(powerOff);
    left.setPower(powerOff);
    right1.setPower(powerOff);
    left1.setPower(powerOff);
    //waits one second to give robot to fully stop
    sleep(1000);
    //if right wheel is further away than left wheel, turns the right wheel
    if(rangeRight.getDistance(unit ) > rangeLeft.getDistance(unit)){

        while(!InRangeLeft(rangeRight.getDistance(unit), unit)) {
            //turns right wheel
            right.setPower(0.1);
            right1.setPower(0.1);
            }
        //turns off right drive motor
        right.setPower(powerOff);
        right1.setPower(powerOff);
        }
    //if left wheel is further away than right wheel, turns the left wheel
    if(rangeLeft.getDistance(unit) > rangeRight.getDistance(unit)){

        while(!InRangeRight(rangeLeft.getDistance(unit), unit)){
            //turns left wheel
            left.setPower(0.1);
            left1.setPower(0.1);
            }
        //turns off left drive motor
        left.setPower(powerOff);
        left1.setPower(powerOff);
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
        right1.setPower(rightPower);
     }
     //turns off right drive power
    right.setPower(powerOff);
    right1.setPower(powerOff);
   //moves left motor until the color sensor reads the line's color range
    while(!WithinColorRange(max, min, colorLeft)){
            //sets left drive power
            left.setPower(leftPower);
            left1.setPower(leftPower);
        }
        //turns off left drive power
        left.setPower(powerOff);
        left1.setPower(powerOff);
    //moves left motor until the color sensor reads the non-line's color range
    while(!WithinColorRange(23, 11, colorLeft)){
        //sets left drive power
        left.setPower(leftPower);
        left1.setPower(leftPower);
    }
    //turns off left drive power
    left.setPower(powerOff);
    left1.setPower(powerOff);
    //moves right motor forward and left motor backwards until right motor reads the non-line's color range
    while(!WithinColorRange(21, 11, colorRight)){
        //sets drive motor's powers
        right.setPower(0.1);
        left.setPower(-0.1);
        right1.setPower(0.1);
        left1.setPower(-0.1);
    }
    //turns off drive motors
    left.setPower(powerOff);
    right.setPower(powerOff);
    right1.setPower(powerOff);
    left1.setPower(powerOff);
    }

    //stops the robot when the color sensor reads within the set range
    private void DrivetoLine(int max, int min, double leftPower, double rightPower, boolean backup){
    //runs until it is within the set range
    while(!WithinColorRange(max, min, colorRight)){
        //sets drive motors to respective powers set in the calling of the method
        right.setPower(rightPower);
        left.setPower(leftPower);
        right1.setPower(rightPower);
        left1.setPower(leftPower);
        }
        //turns power off
        left.setPower(powerOff);
        right.setPower(powerOff);
        left1.setPower(powerOff);
        right1.setPower(powerOff);
        //if backup variable set in calling of the method is true
        if(backup){
            //back up until right behind the line we drove to
            while(!WithinColorRange(23,11, colorRight)){
                right.setPower(-0.1);
                left.setPower(-0.1);
                left1.setPower(-0.1);
                right1.setPower(-0.1);
            }
            //turns drive power off
            left.setPower(powerOff);
            right.setPower(powerOff);
            right1.setPower(powerOff);
            left1.setPower(powerOff);
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
            right.setPower(-0.25);
            left.setPower(0.25);
            left1.setPower(0.25);
            right1.setPower(-0.25);
        }
        //turns off motor power
        right.setPower(powerOff);
        left.setPower(powerOff);
        left1.setPower(powerOff);
        right1.setPower(powerOff);
        //turns left if the left range is less than the right
        if(leftRange < rightRange){
            //sets motor power
            left.setPower(-0.25);
            right.setPower(0.25);
            right1.setPower(0.25);
            left.setPower(-0.25);
        }
        //turns drive motor power off
        right.setPower(powerOff);
        left.setPower(powerOff);
        right1.setPower(powerOff);
        left1.setPower(powerOff);
    }

    }
    private boolean LowerPressed(){
    return !touchLower.getState();
    }
    private boolean UpperPressed(){
    return !touchUpper.getState();
    }
    //a method that drives until both touch sensors are pressed
    /*private void DriveUntilTouch(){

    while (!LeftPressed() || !RightPressed()){
    telemetry.addData("left pressed: ", LowerPressed());
    telemetry.addData("right pressed: ", UpperPressed());
    telemetry.update();
    left.setPower(-0.2);
    right.setPower(-0.2);

    }
    left.setPower(powerOff);
    right.setPower(powerOff);

    }
*/
    private String GetGoldPos(){
    if(detector.getAligned()){
        telemetry.addData("sees gold: ", true);
        telemetry.update();
        phoneServo.setPosition(phoneServo.getPosition() - 0.01);
        if(detector.getAligned()){
            telemetry.addData("Gold's position determined: ", true);
            telemetry.update();
            position = "Right";
        }
    }
    phoneServo.setPosition(0.5);
    sleep(1000);
    if(detector.getAligned() && position ==null){
        telemetry.addData("sees gold: ", true);
        telemetry.update();
        phoneServo.setPosition(phoneServo.getPosition() - 0.01);
        if(detector.getAligned()){
            telemetry.addData("Gold's position determined: ", true);
            telemetry.update();
            position = "left";
        }
    }
    else if(position ==null){
        position = "center";
    }
    detector.disable();
    return position;
    }

    private void HangingApparatus(){
        while(!UpperPressed()){
            hangingMotor.setPower(1);
        }
        hangingMotor.setPower(powerOff);
    }
    private void Sampling(String pos){
        if(pos == "Left"){
            left.setPower(-0.2);
            right.setPower(0.2);
            right1.setPower(0.2);
            left1.setPower(-0.2);
            sleep(450);
            right.setPower(powerOff);
            left.setPower(powerOff);
            right1.setPower(powerOff);
            left1.setPower(powerOff);
            DriveUntilDistance(5,DistanceUnit.INCH);
        }
        if(pos =="Right"){
            left.setPower(0.2);
            right.setPower(-0.2);
            left1.setPower(0.2);
            right1.setPower(-0.2);
            sleep(375);
            left.setPower(powerOff);
            right.setPower(powerOff);
            right1.setPower(powerOff);
            left1.setPower(powerOff);
            DriveUntilDistance(5, DistanceUnit.INCH);
        }
        if(pos == "Center"){
            DriveUntilDistance(5, DistanceUnit.INCH);
        }
    }
}


