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

@Autonomous (name = "AutonomousTest")
public class AutonomousTest extends LinearOpMode {
    private ColorSensor colorRight;
    private DistanceSensor rangeLeft, rangeHigh;
    private DcMotor left, right;
    private Servo phoneServo;
    private BNO055IMU imu;
    private Orientation lastAngles = new Orientation();
    private double correction, globalAngle, powerOff = 0;
    private DigitalChannel touchLeft, touchRight;
    private GoldAlignDetector detector;
    private String position = null;
@Override
    public void runOpMode() {
   // colorLeft = hardwareMap.get(ColorSensor.class, "colorLeft");
    colorRight = hardwareMap.get(ColorSensor.class, "colorRight");
    left = hardwareMap.dcMotor.get("left");
    right = hardwareMap.dcMotor.get("right");
    phoneServo = hardwareMap.servo.get("phoneServo");
    touchLeft = hardwareMap.get(DigitalChannel.class, "touchLeft");
    touchRight = hardwareMap.get(DigitalChannel.class, "touchRight");
    rangeLeft = hardwareMap.get(DistanceSensor.class, "rangeLeft");
    rangeHigh = hardwareMap.get(DistanceSensor.class, "rangeHigh");
    left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    right.setDirection(DcMotorSimple.Direction.REVERSE);

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
    detector.setAlignSettings(0,1000);
    detector.enable();

    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
    parameters.mode = BNO055IMU.SensorMode.IMU;
    parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
    parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
    parameters.loggingEnabled = false;
    imu = hardwareMap.get(BNO055IMU.class, "imu");
    imu.initialize(parameters);

    telemetry.addData("mode: ", "calibrating...");
    telemetry.update();

    while(!isStopRequested() && !imu.isGyroCalibrated()){
        sleep(50);
        idle();
    }
    telemetry.addData("mode: ", "ready");
    telemetry.update();
    right.setDirection(DcMotorSimple.Direction.FORWARD);
    left.setDirection(DcMotorSimple.Direction.REVERSE);

    phoneServo.setPosition(0.9);

    waitForStart();

    DrivebyColor(0.4, colorRight);
    Sampling();
    telemetry.addData("gold:", GetPosition());
    telemetry.update();
    phoneServo.setPosition(0.5);
    GyroTurn(70, 0.2);
    DrivebyRange(10,0.4, rangeLeft);
    DriveUntilTouch(0.4);
    ResetAngles();
    DrivebyRangeReverse(3, 0.4, rangeLeft);
    GyroTurn(90, 0.2);
    DrivebyRange(23, 1.0, rangeLeft);
    sleep(5000);
    right.setPower(-0.9);
    left.setPower(-1);
    sleep(3000);
    right.setPower(0);
    left.setPower(0);
    }
//gets the reading from the imu and converts the angle to be cumulative
private double GetAngles(){
    //declares and sets a variable to the reading of the imu
    Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    //declares and sets a variable to the change of the angle that is and the angle that was before
    double deltaAngle = angles.firstAngle - lastAngles.firstAngle;
    //sets delta angle itself plus 360 if it is less than -180 degrees
    if (deltaAngle < -180){
        deltaAngle += 360;
    }
    //sets delta angle to itself minus 360 if it is greater than 180 degrees
    else if(deltaAngle > 180) {
        deltaAngle -= 360;
    }
    //sets globalAngle to itself plus deltaAngle
    globalAngle += deltaAngle;
    //sets last angle to the current value of angles
    lastAngles = angles;
    //returns globalAngle
    return globalAngle;
}
//returns the value that the robot must correct for the robot to maintain a straight heading
private double CheckDirection(){
    //sets three variables to hold the value that the robot is off course, the reading of the imu, and the value for how much error allowed
    double correction, angle, gain = 0.1;
    //sets the value of angles equal to the reading of the imu
    angle = GetAngles();
    //if the robot is on course, correction equals 0
    if (angle == 0){
        correction = 0;
    }
    //otherwise, the correction is the opposite of the imu reading
    else{
        correction = -angle;
    }
    //account for a small amount of error
    correction = correction * gain;
    //return the value of correction
    return correction;
}
//sets all angles to starting values
private void ResetAngles(){
    //sets last angles to the current reading of the imu
    lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    //sets global angle to 0
    globalAngle = 0;
}
//turns using the gyro to determine distance; works best at 0.2 power
private void GyroTurn(int degrees, double power){
    int offset = 5;
    degrees = degrees - offset;
    //declares two variables to hold the power of the left and right drive motors
    double leftPower, rightPower;
    //sets angle variables to starting values
    ResetAngles();
    //if it is less than 0, sets drive motors to turn right
    if(degrees < 0){
        leftPower = power;
        rightPower = -power;
    }
    //otherwise, if the value of degrees is greater than 0, sets drive motors to turn left
    else if(degrees > 0){
         leftPower = -power;
         rightPower = power;
    }
    //otherwise do nothing
    else return;
    //sets the power of the drive motors
    left.setPower(leftPower);
    right.setPower(rightPower);

    if(degrees < 0){
        //turns until complete. First while method is to get robot off value of 0
        while(opModeIsActive() && GetAngles() == 0){}

        while(opModeIsActive() && GetAngles() > degrees){}
    }
    else {
        //otherwise, turns until complete
        while(opModeIsActive() && GetAngles() < degrees){}
    }
    //turns off power to drive motors
    left.setPower(powerOff);
    right.setPower(powerOff);
    //waits half a second
    sleep(500);
    //resets the value of the angle variables
    ResetAngles();
}
//adjusts the drive power to keep robot on a 0 degree heading
private void GyroStraightening(double power){
    //sets correction to CheckDirection
    correction = CheckDirection();
    //sets drive power relative to gyro reading
    left.setPower(power - correction);
    right.setPower(power);
}
    //returns true if touch sensor is pressed
    private boolean LeftPressed(){return !touchLeft.getState(); }
    private boolean RightPressed(){ return !touchRight.getState();}
    //a method that drives until both touch sensors are pressed
    private void DriveUntilTouch(double power) {
        //while either left or right is not pressed, drives with gyro straightening
        while (!LeftPressed() || !RightPressed()) {
            telemetry.addData("left pressed: ", LeftPressed());
            telemetry.addData("right pressed: ", RightPressed());
            telemetry.update();
            right.setPower(0.2);
            left.setPower(0.4);
        }
        //turns off drive power
        left.setPower(powerOff);
        right.setPower(powerOff);
    }
    private boolean InRange(double target, DistanceUnit units, DistanceSensor range){
        //creates a variable to hold the range sensor's reading
        double distance;
        //sets the distance variable to the value that the range sensor reads
        distance = range.getDistance(units);
        //returns true if the robot is closer to the target than the target position
        return (distance <= target);
    }
    private void DrivebyRange(double distance, double power, DistanceSensor range){
    while(!InRange(distance, DistanceUnit.INCH, range)){
        GyroStraightening(power);
    }
    right.setPower(powerOff);
    left.setPower(powerOff);
}
private void DrivebyRangeReverse(double distance, double power, DistanceSensor range){
    while(InRange(distance, DistanceUnit.INCH, range)){
        GyroStraightening(-power);
    }
    left.setPower(powerOff);
    right.setPower(powerOff);
}
private void DrivebyColor(double power, ColorSensor colorSensor){
    while(!WithinColorRange(70, 42, colorSensor)){
        GyroStraightening(power);
    }
    left.setPower(powerOff);
    right.setPower(powerOff);

}
private boolean WithinColorRange(int max, int min, ColorSensor sensor) {
    //declares and sets a variable equal to the color sensor reading
    int color = sensor.blue();
    //returns true if the color sensor is less than or equal to the max value and less than or equal to the min value
    return (color <= max && color >= min);

}
private String GetPosition(){
    if(detector.getAligned()){
        position = "Right";
    }
    if(position == null) {
        phoneServo.setPosition(0.5);
        if(detector.getAligned()){
            position = "Left";
        }
        else if(!detector.getAligned()){
            position = "Center";
        }
    }
    return position;
}
private void ColorStraightenSimple(double power){
    if(WithinColorRange(50, 42, colorRight)){
        /*while(!WithinColorRange(50, 42, colorLeft)){
            left.setPower(power);
            right.setPower(-power);
        }*/
        left.setPower(powerOff);
        right.setPower(powerOff);
    }
    /*else if(WithinColorRange(50, 42, colorLeft)){
        while(!WithinColorRange(50, 42, colorRight)){
            right.setPower(power);
            left.setPower(-power);
        }
        right.setPower(powerOff);
        left.setPower(powerOff);
     }*/
}
private void DriveByLander(double target, double power){
    if(power < 0){
        while(!InRange(target, DistanceUnit.INCH, rangeHigh)){
            left.setPower(power);
            right.setPower(power);
        }
        right.setPower(powerOff);
        left.setPower(powerOff);
    }
    else if(power > 0){
        while(InRange(target, DistanceUnit.INCH, rangeHigh)){
            left.setPower(power);
            right.setPower(power);
        }
        left.setPower(powerOff);
        right.setPower(powerOff);
    }
}

private void Sampling(){
    String position = GetPosition();
    switch (position){
        case("Center"):
            DriveByLander(27, 0.4);
            DriveByLander(13, -0.4);
            break;
        case("Left"):
            GyroTurn(25, 0.2);
            DriveByLander(29, 0.4);
            DriveByLander(13, -0.4);
            GyroTurn(-20, 0.2);
            break;
        case("Right"):
            GyroTurn(-25, 0.2);
            DrivebyRangeReverse(29, -0.4, rangeHigh);
            DrivebyRange(17, -0.4, rangeHigh);
            GyroTurn(35, 0.2);
            break;
        }
}
}
