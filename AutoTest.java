package org.firstinspires.ftc.teamcode.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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
@Disabled
@Autonomous (name = "AutoTest")
public class AutoTest extends LinearOpMode {
    private ColorSensor colorRight, colorLeft;
    private DcMotor slideMotor;
    private DistanceSensor rangeLeft, rangeRight, rangeHigh;
    private DcMotor leftF, rightF, leftB, rightB;
    private DcMotor hangingMotor, pivotMotor;
    private Servo phoneServo;// catLauncher;
    private BNO055IMU imu;
    private Orientation lastAngles = new Orientation();
    private double correction, globalAngle, powerOff = 0;
    private DigitalChannel touchUpper, magnetLower;
    private AnalogInput armPos;
    private GoldAlignDetector detector;
    private String position = null;
    @Override
    public void runOpMode() {
        // colorLe711hardwareMap.get(ColorSensor.class, "colorLeft");
        colorRight = hardwareMap.get(ColorSensor.class, "colorRight");
        colorLeft = hardwareMap.get(ColorSensor.class, "colorLeft");
        leftF = hardwareMap.dcMotor.get("leftF");
        rightF = hardwareMap.dcMotor.get("rightF");
        leftB = hardwareMap.dcMotor.get("leftB");
        rightB = hardwareMap.dcMotor.get("rightB");
        hangingMotor = hardwareMap.dcMotor.get("hangingMotor");
        pivotMotor = hardwareMap.dcMotor.get("pivotMotor");
        phoneServo = hardwareMap.servo.get("phoneServo");
        slideMotor = hardwareMap.dcMotor.get("slideMotor");
        touchUpper = hardwareMap.get(DigitalChannel.class, "touchUpper");
        magnetLower = hardwareMap.get(DigitalChannel.class, "magnetLower");
        armPos = hardwareMap.get(AnalogInput.class, "armPos");
        // catLauncher = hardwareMap.servo.get("catLauncher");
        //touchLeft = hardwareMap.get(DigitalChannel.class, "touchLeft");
        // touchRight = hardwareMap.get(DigitalChannel.class, "touchRight");
        // magneticSwitch = hardwareMap.get(DigitalChannel.class, "magneticSwitch");
        rangeLeft = hardwareMap.get(DistanceSensor.class, "rangeLeft");
        rangeRight = hardwareMap.get(DistanceSensor.class, "rangeRight");
        rangeHigh = hardwareMap.get(DistanceSensor.class, "rangeHigh");
        leftF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightF.setDirection(DcMotorSimple.Direction.REVERSE);
        rightB.setDirection(DcMotorSimple.Direction.REVERSE);
        hangingMotor.setDirection(DcMotorSimple.Direction.REVERSE);

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
        detector.setAlignSettings(0,800); //1000
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

        telemetry.addData("servo: ", phoneServo.getPosition());
        telemetry.update();

        phoneServo.setPosition(0.8);

        waitForStart();


//leftB.setPower(1);
//rightF.setPower(1);
//leftF.setPower(-0.3);
//rightB.setPower(-0.3);
//sleep(2000);
//leftB.setPower(0);
//rightF.setPower(0);
//leftF.setPower(0);
//rightB.setPower(0);


        leftF.setPower(1);



        leftB.setPower(1);



        rightF.setPower(1);


        rightB.setPower(1);
        sleep(2000);
        rightB.setPower(0);
        leftF.setPower(0);
        leftB.setPower(0);
        rightF.setPower(0);
        //testing auto
       // deploy();
        //lineUpByColorSimple();
      //  sleep(1000);
       // sampling2();
        //driveToDepot();
        //dropCat();
        //park();
        //lineUpByColorSimple();




        //bioscience auto
   /* deploy();
    pivotMotor.setPower(0.5);
    sleep(200);
    pivotMotor.setPower(powerOff);
    driveByLander(11, 0.3);
    sampling2();*/

        //future auto
    /*driveToDepot();
    powerMotorsOff();
    leftF.setPower(0);
    leftB.setPower(0);
    dropCat();
    park();*/
        //getAngles();
        // telemetry.addData("position of gold: ", position);
//   //drivebyColor(0.3, colorRight);
        // driveByChangeInRange(false);
        //sleep(1000);
     /* DrivebyColor(0.4, colorLeft);
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
        catLauncher.setPosition(1);

        right.setPower(-0.9);
        left.setPower(-1);
        sleep(3000);
        right.setPower(powerOff);
        left.setPower(powerOff);*/
    }

    private void setRotationPower(boolean isRight, double power){
        if(isRight){
            setMotorPowers(-power, power,
                    -power, power);
        }
        else{
            setMotorPowers(power, -power,
                    power, -power);
        }
    }
    private void setPowerInDirection(double degrees, double power){
        degrees = checkDirection();
        double leftPwr = power - (degrees *0.1);
        setMotorPowers(leftPwr, power, leftPwr, power);
    }
    private void setPowerStraight(double power){
        setPowerInDirection(0, power);
    }
    private void powerMotorsOff(){
        setPowerInDirection(0,powerOff);
    }
    //gets the reading from the imu and converts the angle to be cumulative
    private double getAngles(){
        //declares and sets a variable to the reading of the imu
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        //declares and sets a variable to the change of the angle that is and the angle that was before
        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;
        //sets delta angle itself plus 360 if it is less than -180 degrees
        if (deltaAngle < -180){
            deltaAngle += 360;
        }
        //sets delta angle to itself minus 360 if it is greater than 180 degrees
        else if(deltaAngle > 180){
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
    private double checkDirection(){
        //sets three variables to hold the value that the robot is off course, the reading of the imu, and the value for how much error allowed
        double correction, angle;
        //sets the value of angles equal to the reading of the imu
        angle = getAngles();
        //if the robot is on course, correction equals 0
        if (angle == 0){
            correction = 0;
        }
        //otherwise, the correction is the opposite of the imu reading
        else{
            correction = -angle;
        }
        //return the value of correction
        return correction;
    }
    //sets all angles to starting values
    private void resetAngles(){
        //sets last angles to the current reading of the imu
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        //sets global angle to 0
        globalAngle = 0;
    }
    //turns using the gyro to determine distance; works best at 0.2 power
    private void gyroTurn(int degrees, double power){
        int offset = 5;
        //declares two variables to hold the power of the left and right drive motors
        double leftPower, rightPower;
        //sets angle variables to starting values
        resetAngles();
        //if it is less than 0, sets drive motors to turn right
        boolean isRight = degrees > 0;
        setRotationPower(isRight, power);
        if(isRight){
            degrees = degrees - offset;
            //turns until complete. First while method is to get robot off value of 0
            while(opModeIsActive() && getAngles() == 0){}
            while(opModeIsActive() && getAngles() < degrees){
                telemetry.addData("is Right: ", isRight);
                telemetry.addData("angle: ", getAngles());
                telemetry.update();
            }
        }
        else {
            degrees = degrees + offset;
            //otherwise, turns until complete
            while(opModeIsActive() && getAngles() > degrees){}
        }
        //turns off power to drive motors
        powerMotorsOff();
        setRotationPower(false, powerOff);
        telemetry.addData("got here", true);
        telemetry.update();
        //waits half a second
        sleep(500);
        //resets the value of the angle variables
        resetAngles();
    }
    //returns true if touch sensor is pressed
    //  private boolean leftPressed(){return !touchLeft.getState(); }
    //private boolean rightPressed(){ return !touchRight.getState();}
    //a method that drives until both touch sensors are pressed
 /*   private void driveUntilTouch(double power) {
        //while either left or right is not pressed, drives with gyro straightening
        while (!leftPressed() || !rightPressed()) {
            telemetry.addData("left pressed: ", leftPressed());
            telemetry.addData("right pressed: ", rightPressed());
            telemetry.update();
            setPowerInDirection(2, 0.4);
        }
        //turns off drive power
        powerMotorsOff();
    }*/
    private boolean inRange(double target, DistanceUnit units, DistanceSensor range){
        //creates a variable to hold the range sensor's reading
        double distance;
        //sets the distance variable to the value that the range sensor reads
        distance = range.getDistance(units);
        //returns true if the robot is closer to the target than the target position
        return (distance <= target);
    }
    private void drivebyRange(double distance, double power, DistanceSensor range){
        while(!inRange(distance, DistanceUnit.INCH, range)){
            setPowerStraight(power);
        }
        powerMotorsOff();
    }
    private void drivebyRangeReverse(double distance, double power, DistanceSensor range){
        while(inRange(distance, DistanceUnit.INCH, range)){
            setPowerStraight(-power);
        }
        powerMotorsOff();
    }
    private void drivebyColor(double power, ColorSensor colorSensor){
        while(!withinColorRange(30, 20, colorSensor)){
            setPowerStraight(power);
        }
        powerMotorsOff();

    }
    //returns true if color sensor reads within the maximum and minimum values
    private boolean withinColorRange(int max, int min, ColorSensor sensor) {
        //declares and sets a variable equal to the color sensor reading
        int color = sensor.blue();
        //returns true if the color sensor is less than or equal to the max value and less than or equal to the min value
        return (color <= max && color >= min);
    }
    //returns string with position of gold
    private String getPosition(){
        //sets position to right
        if(detector.getAligned()){
            position = "Right";
        }
        //if the position is not already set
        if(position == null) {
            //move servo to left mineral position
            phoneServo.setPosition(0.5);
            sleep(1000);
            //sets position to left
            if(detector.getAligned()){
                position = "Left";
            }
            //otherwise sets position to center
            else if(!detector.getAligned()){
                position = "Center";
            }
        }
        //returns the position
        return position;
    }
    //drive by range for the rangeHigh relative to the lander
    private void driveByLander(double target, double power){
        telemetry.addData("doing this", true);
        telemetry.update();
        //determines direction robot wants to travel
        if(power < 0){
            //if traveling backwards, drive until robot is within a range
            while(!inRange(target, DistanceUnit.INCH, rangeHigh)){
                setPowerStraight(power);
            }
            powerMotorsOff();
        }
        //if the robot wants to drive forwards
        else if(power > 0){
            //drive until the range sensor is not within a range
            while(inRange(target, DistanceUnit.INCH, rangeHigh)){
                setPowerStraight(power);
            }
            powerMotorsOff();
        }
    }
    //navigates to hit the correct mineral
    private void sampling(){
        //finds where the gold is
        String position = getPosition();
        switch (position){
            //if center, hit the center position TO DO: test
            case("Center"):
                driveByLander(27, 0.3);

                driveByLander(20, -0.3);

                break;
            //if left, hit the left position TO DO: test
            case("Left"):

                break;
            //if right, hit the right position TO DO: test
            case("Right"):
                strafe(0.4, true);
                sleep(1700);
                strafe(0, true);
                setMotorPowers(0.3, 0.3, 0.3,0.3);
                sleep(850);
                powerMotorsOff();
                setMotorPowers(-0.3, -0.3, -0.3, -0.3);
                sleep(900);
                strafe(0.4, false);
                sleep(1700);
                strafe(0,false);

                break;
        }
    }
    //sets all the motors power to the inputs
    private void setMotorPowers(double leftFPwr, double rightFPwr, double leftBPwr, double rightBPwr){
        leftF.setPower(leftFPwr);
        rightF.setPower(rightFPwr);
        leftB.setPower(leftBPwr);
        rightB.setPower(rightBPwr);

    }
    //strafes the robot
    private void strafe(double power, boolean right){
        //if strafing right
        if(right){
            //sets front left and back right motors to negative power, sets front right and back left motors to positive power
            setMotorPowers(power, -power,
                    -power, power);
        }
        //if strafing left
        else{
            //sets front left and back right motors to positive power and sets front right and back left motors to negative power
            setMotorPowers(-power, power,
                    power, -power);
        }
    }
    //
    private void distanceToRate(double stoptarget, DistanceUnit unit, double time){
        //sets variables to hold the values of the distance that the robot needs to travel
        double distanceRight, distanceLeft;
        //declares a variable to hold the linear rate
        double linearRateLeft, linearRateRight;
        //declares a variable to hold the angular rate
        double angularRateLeft, angularRateRight;
        //the radius of the wheel in inches
        double radius = 2;
        //runs until range sensor reads a specified distance at which point the robot stops
        while(!inRange(stoptarget, unit, rangeLeft) && !inRange(stoptarget, unit, rangeRight)){
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
            setMotorPowers(angularRateLeft, angularRateRight,
                    angularRateLeft, angularRateRight);
        }
        //turns off drive motors
        powerMotorsOff();
        //waits one second to give robot to fully stop
        sleep(1000);
    }
    private void straighten(DistanceUnit units){
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
                setMotorPowers(0.25, -0.25,
                        0.25, -0.25);
            }
            //turns off motor power
            powerMotorsOff();
            //turns left if the left range is less than the right
            if(leftRange < rightRange){
                //sets motor power
                setMotorPowers(-0.25, 0.25,
                        -0.25, 0.25);
            }
            //turns drive motor power off
            powerMotorsOff();
        }
    }
    private void driveToDepot(){
        gyroTurn(70, 0.3);
        distanceToRate(10, DistanceUnit.INCH, 10);
        straighten(DistanceUnit.INCH);
        gyroTurn(90, 0.3);
        drivebyRange(15, 0.5, rangeLeft);
    }
    private void dropCat(){
        pivotMotor.setPower(0.3);
        sleep(1000);
        pivotMotor.setPower(powerOff);
    }
    private void park(){
        setMotorPowers(-1, -1, -1, -1);
    }
    private void sampling2(){
        driveByLander(7, 0.3);
        strafe(0.3, true);
        sleep(1000);
        strafe(0, false);
        if(detector.getAligned()){
            position = "Right";
            driveByEncoder(9, 0.2);
            sleep(100);
            driveByEncoder(-6, 0.3);
            strafeByEncoder(35, 0.3, false);
        }/*
        else if(position == null){
            telemetry.addData("got here:", true);
            telemetry.update();
            strafeByEncoder(15, 0.3, false);
            sleep(500);
            if(detector.getAligned()){
                position = "Center";
                driveByEncoder(10, 0.3);
                sleep(100);
                driveByEncoder(-10, 0.3);
                strafeByEncoder(20, 0.3, false);
            }
            else if(position == null){       // this thing does check if gold is in left.
                strafeByEncoder(20, 0.3, false);
                sleep(100);
                if(detector.getAligned()){
                    driveByEncoder(10, 0.3);
                }
            }
        }
        driveByEncoder(-3, 0.3); //goes back to ensure robot does not hit left mineral*/
    }
    private boolean isTouched(DigitalChannel touch){
        return !touch.getState();
    }
    private void goToTouch(double power, DigitalChannel sensor){
        while(!isTouched(sensor)){
            telemetry.addData("get here", true);
            telemetry.update();
            hangingMotor.setPower(power);
        }
        hangingMotor.setPower(powerOff);
    }
    private boolean foundMagnet(DigitalChannel sensor){return !sensor.getState();}
    private void goToMagnetLimitSensor(double power, DigitalChannel sensor){
        while(!foundMagnet(sensor)){
            hangingMotor.setPower(power);
        }
        hangingMotor.setPower(powerOff);
    }
    private void deploy(){
        goToTouch(1, touchUpper);
        strafe( 0.5, true);
        sleep(1000);
        strafe(0, false);
    }
    private void driveByEncoder(double position, double power){
        leftF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        double wheelDiameter = 4*3.14;
        position = position / wheelDiameter;
        position *= 1120;
        telemetry.addData("target position: ", position);
        telemetry.update();
        if(position >=0) {
            while (leftF.getCurrentPosition() < position) {
                setPowerStraight(power);
            }
            powerMotorsOff();
        }
        else{
            while(leftF.getCurrentPosition() > position){
                setPowerStraight(-power);
            }
        }
    }
    private void strafeByEncoder(double position, double power, boolean isRight){
        leftF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        double wheelDiameter = 4*3.14, degrees, altPwr;
        position = position / wheelDiameter;
        position *= 1120;
        if (isRight) {
            while(leftF.getCurrentPosition() < position) {
                degrees = checkDirection();
                altPwr = power - (degrees * 0.1);
                setMotorPowers(power, -power,
                        -power, power);
            }
        }
        else{
            while(-leftF.getCurrentPosition() < position) {
                degrees = checkDirection();
                altPwr = power + (degrees * 0.1);
                setMotorPowers(-power, power,
                        power, -power);

                telemetry.addData("left pos: ", leftF.getCurrentPosition());
                telemetry.update();
            }
        }
        powerMotorsOff();
    }
    private void lineUpByColorSimple(){
        while(!withinColorRange(32, 20, colorRight) && !withinColorRange(30, 20, colorLeft)){
            if(!withinColorRange(32, 20, colorRight)){
                rightF.setPower(0.3);
                rightB.setPower(0.3);
            }
            else{
                rightF.setPower(-0.3);
                rightB.setPower(-0.3);
            }

            if(!withinColorRange(32, 20, colorLeft)){
                leftF.setPower(0.3);
                leftB.setPower(0.3);
            }
            else{
                leftF.setPower(-0.3);
                leftB.setPower(-0.3);
            }
        }
        powerMotorsOff();
    }

    private void lineUpAgainstWall(double power, long time){
        strafe(power, false);
        sleep(time);
        leftB.setPower(0);
        leftF.setPower(0);
        rightB.setPower(0);
        rightF.setPower(0);
    }

    private void dropCat( double distance, double secondDistance, double angle){
        while(rangeLeft.getDistance(DistanceUnit.INCH) >= distance ){
            setMotorPowers(1,1,1,1);
        }
        setMotorPowers(0,0,0,0);
        strafe(100,true);
        sleep(200);
        setMotorPowers(0,0,0,0);
        pivotMotor.setPower(-.3);
        sleep(200);
        pivotMotor.setPower(0);
        while(rangeLeft.getDistance(DistanceUnit.INCH) <= secondDistance ){
            setMotorPowers(-1,-1,-1,-1);
        }
        setMotorPowers(0,0,0,0);
        setMotorPowers(-1,1,-1,1);
        sleep(1000);
        setMotorPowers(0,0,0,0);
        slideMotor.setPower(100);
        sleep(5000);
        slideMotor.setPower(0);

    }

}

//P algorithm turn only ok
/*
        while (getAngles() != degrees && opModeIsActive()) {
            error = degrees - getAngles();
            turnRobot(error * kP, isRight);

            if (Math.abs(error) < 1) {
                sleep(500);
                if (Math.abs(error) < 1) {
                    break;
                }
            }
            if (getAngles() > degrees - 1 && getAngles() < degrees + 1) {
                while (getAngles() != degrees && opModeIsActive()) {
                    error = degrees - getAngles();
                    turnRobot(error * kP, isRight);

                    if (Math.abs(error) < 1) {
                        sleep(500);
                        if (Math.abs(error) < 1) {
                            break;
                        }
                    }
                }
            }*/