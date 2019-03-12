package org.firstinspires.ftc.teamcode.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
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

@Autonomous (name = "AutonomousTest")
public class AutonomousTest extends LinearOpMode {
    //motors
    private DcMotor leftF, rightF, leftB, rightB; //declares drive motors
    private DcMotor hangingMotor, pivotMotor, slideMotor; //declares attachment motors
    //sensors
    private ColorSensor colorRight, colorLeft; //declares color sensors
    private DistanceSensor rangeLeft, rangeRight; //declares range sensors
    private BNO055IMU imu; //declares REV imu
    private DigitalChannel up, down;
 //   private DigitalChannel touchUpper, magnetLower; //declares touch sensor and magnetic limit sensor
    private AnalogInput armPos; //declares potentiometer
    private GoldAlignDetector detector; // declares Doge CV detector
    //servos
    private Servo catapult;
    private CRServo collector;
    //other variables
    private Orientation lastAngles = new Orientation(); //variable for imu to hold its previous reading
    private double correction, globalAngle; //imu related doubles.
    private int inertiaCorrection;
    private double powerOff = 0; //turns power off
    private String position = null; //string to hold the gold's position
@Override
    public void runOpMode() {
    //sets value of drive motors
    leftF = hardwareMap.dcMotor.get("leftF");
    rightF = hardwareMap.dcMotor.get("rightF");
    leftB = hardwareMap.dcMotor.get("leftB");
    rightB = hardwareMap.dcMotor.get("rightB");
    //sets value of attachment motor
    hangingMotor = hardwareMap.dcMotor.get("hangingMotor");
    pivotMotor = hardwareMap.dcMotor.get("pivotMotor");
    slideMotor = hardwareMap.dcMotor.get("slideMotor");
    //sets value of color sensors
    colorRight = hardwareMap.get(ColorSensor.class, "colorRight");
    colorLeft = hardwareMap.get(ColorSensor.class, "colorLeft");
    up = hardwareMap.get(DigitalChannel.class, "up");
    down = hardwareMap.get(DigitalChannel.class, "down");
    armPos = hardwareMap.get(AnalogInput.class, "armPos");
    rangeLeft = hardwareMap.get(DistanceSensor.class, "rangeLeft");
    rangeRight = hardwareMap.get(DistanceSensor.class, "rangeRight");
    //rangeHigh = hardwareMap.get(DistanceSensor.class, "rangeHigh");
    //sets value of servos
    catapult = hardwareMap.servo.get("catapult");
    collector = hardwareMap.crservo.get("collector");

    //set parameters of motors
    leftF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    rightF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    rightF.setDirection(DcMotorSimple.Direction.REVERSE);
    rightB.setDirection(DcMotorSimple.Direction.REVERSE);
    hangingMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    //set parameters of Doge CV detector
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

    //sets parameters of Rev imu
    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
    parameters.mode = BNO055IMU.SensorMode.IMU;
    parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
    parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
    parameters.loggingEnabled = false;
    imu = hardwareMap.get(BNO055IMU.class, "imu");
    imu.initialize(parameters);

    //calibrates gyro
    telemetry.addData("mode: ", "calibrating...");
    telemetry.update();

    while(!isStopRequested() && !imu.isGyroCalibrated()){
        sleep(50);
        idle();
    }
    telemetry.addData("mode: ", "ready");
    telemetry.update();
//7726 MAX HANG
    waitForStart();

    deploy();
    sleep(1000);
    sampling4();
    sleep(1000);
     goToWall();
     sleep(1000);
    driveToDepot();
    sleep(1000);
    dropCat();
    sleep(1000);
    park();
    }
    //turns without gyro
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
    //corrects using gyro if in loop
    private void setPowerInDirection(double degrees, double power){
    //gets gyro value
    degrees = checkDirection();
    //gets a corrected power value that is equal to power - one tenth of the error
    double leftPwr = power - (degrees *0.1);
    //applies power
    setMotorPowers(leftPwr, power, leftPwr, power);
    }
    //method above always set to zero
    //must be used in while loop
    private void setPowerStraight(double power){
    setPowerInDirection(0, power);
 }
 //turns all the motors off
    private void powerMotorsOff(){
    setMotorPowers(0,0,0,0);
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
            //finishes turn
            while(opModeIsActive() && getAngles() < degrees){}
        }
        else {
            degrees = degrees + offset;
            //otherwise, turns until complete
            while(opModeIsActive() && getAngles() > degrees){}
        }
        //turns off power to drive motors
        powerMotorsOff();
        //resets the value of the angle variables
        resetAngles();
    }
    //determines if the range sensor is closer to a target than a specified value
    private boolean inRange(double target, DistanceUnit units, DistanceSensor range){
        //creates a variable to hold the range sensor's reading
        double distance;
        //sets the distance variable to the value that the range sensor reads
        distance = range.getDistance(units);
        //returns true if the robot is closer to the target than the target position
        return (distance <= target);
    }
    private void drivebyRange(double distance, double power, DistanceSensor range){
        //determines whether the robot is travelling towards or away from the target.
        if(power > 0) {
            //if driving towards target, keeps driving until the actual range is less than the target range
            while (!inRange(distance, DistanceUnit.INCH, range) && opModeIsActive()) {
                setPowerStraight(power);
            }
            //turns off drive motors
            powerMotorsOff();
        }
        else{
            //if moving away from the target, drive until the actual distance is greater than the target position
            while(inRange(distance, DistanceUnit.INCH, range) && opModeIsActive()){
                setPowerStraight(-power);
            }
            //powers off drive motors
            powerMotorsOff();
        }
    }
    //returns true if color sensor reads within the maximum and minimum values
    private boolean withinColorRange(int max, int min, ColorSensor sensor) {
        //declares and sets a variable equal to the color sensor reading
        int color = sensor.blue();
        //returns true if the color sensor is less than or equal to the max value and less than or equal to the min value
        return (color <= max && color >= min);
    }
   //drives robot until color sensor reads within two specified values
    private void drivebyColor(double power, int max, int min, ColorSensor colorSensor){
        while(!withinColorRange(max, min, colorSensor) && opModeIsActive()){
            setPowerStraight(power);
        }
        powerMotorsOff();

    }
    //drive by range for the rangeHigh relative to the lander
   /* private void driveByLander(double target, double power){
        //determines direction robot wants to travel
        if(power < 0){
            //if traveling backwards, drive until robot is within a range
            //while(!inRange(target, DistanceUnit.INCH, rangeHigh) && opModeIsActive()){
                setPowerStraight(power);
            }
            powerMotorsOff();
        }
        //if the robot wants to drive forwards
        else if(power > 0){
            //drive until the range sensor is not within a range
          //  while(inRange(target, DistanceUnit.INCH, rangeHigh) && opModeIsActive()){
                setPowerStraight(power);
            }
            powerMotorsOff();
        }
*/
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
    //moves the robot at a rate porportional to its distance from the target position
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
        while(!inRange(stoptarget, unit, rangeLeft) && !inRange(stoptarget, unit, rangeRight) && opModeIsActive()){
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
    //straightens the robot using the range sensors
    private void straighten(DistanceUnit units){
        //declares variables to hold range sensor reading
        double rightRange, leftRange;
        //sets the initial values of the variables to the range sensors reading
        rightRange = rangeRight.getDistance(units);
        leftRange = rangeLeft.getDistance(units);
        //runs until we are straight
        while(rightRange != leftRange && opModeIsActive()){
            //updates values of range variables
            rightRange = rangeRight.getDistance(units);
            leftRange = rangeLeft.getDistance(units);
            //turns right if the right distance is less than the left
            if(rightRange < leftRange){
                //sets motor powers
                setMotorPowers(-0.3, 0.3,
                               -.3, 0.3);
            }
            //turns off motor power
            powerMotorsOff();
            //turns left if the left range is less than the right
            if(leftRange < rightRange){
                //sets motor power
                setMotorPowers(0.3, -0.3,
                               0.3, -0.3);
            }
            //turns drive motor power off
            powerMotorsOff();
        }
    }
    //moves the robot into position to drop the team marker
    private void driveToDepot(){
        //turns towards the wall
        gyroTurn(80, 0.4);
        if(position == "Left"){
            gyroTurn(-5, 0.4);
        }
        sleep(500);
        telemetry.addData("range Left: ", rangeLeft.getDistance(DistanceUnit.INCH));
        telemetry.update();
        drivebyRange(20,.6,rangeRight);
        sleep(1000);
    }
    private void dropCat(){
        sleep(1000);
        catapult.setPosition(0);
        sleep(2000);
    }
    //drives to and parks on crater
    private void park(){
    gyroTurn(7, 0.4);
    driveByEncoder(-37, 1);
    }
    private void sampling2() {
       // driveByLander(7, 0.3);// moves away from lander
        strafe(0.5, true);// move to the right gold
        sleep(1700);// stop 1.7 seconds
        strafe(0, false);// stops
        if (detector.getAligned()) {// if sees gold
            position = "Right";//sets position
            //scores gold and returns to original position
            setMotorPowers(0.3, 0.3, 0.3, 0.3);
            sleep(800);
            setMotorPowers(-0.3, -0.3, -0.3, -0.3);
            sleep(800);
            //goes to common known position
            strafe(0.5, false);
            sleep(2400);
            strafe(0, false);

        }
        //strafes to center mineral
        strafe(0.5, false);
        sleep(1700);
        strafe(0, false);
        //if it sees gold
        if(detector.getAligned()){
            //sets position to Center mineral
            position = "Center";
            //scores center mineral and returns to original position
            setMotorPowers(0.3, 0.3, 0.3, 0.3);
            sleep(550);
            setMotorPowers(-0.3, -0.3, -0.3, -0.3);
            sleep(550);
            powerMotorsOff();
            //strafes to known place
            strafe(0.5, false);
            sleep(1400);
            strafe(0, false);
        }
        //else the position is left
        else if(position == null){
            //strafe to left mineral
            strafe(0.5, false);
            sleep(1000);
            strafe(0, false);
            //double checks
            if(detector.getAligned()){
                //scores the mineral and returns to original position
                setMotorPowers(0.3, 0.3, 0.3, 0.3);
                sleep(540);
                setMotorPowers(-0.3, -0.3, -0.3, -0.3);
                sleep(620);
                setMotorPowers(0,0,0,0);
            }
        }
    }
    private void sampling3(){
        lineUpByColorSimple();
       // driveByLander(rangeHigh.getDistance(DistanceUnit.INCH)+3, 0.4);
        gyroTurn(-30, 0.4);
        if(detector.getAligned()){
            position="Right";
            gyroTurn(-10, 0.4);
            driveByEncoder(15, 0.6);
            driveByEncoder(-10,0.6);//15............!
            gyroTurn(30,0.4);
        }
        else if(position == null){
            gyroTurn(75,.4);
            driveByEncoder(5, 0.6);
            if (detector.getAligned()){
                telemetry.addData("got here", true);
                telemetry.update();
                sleep(1000);
                position="Left";
                driveByEncoder(8,.6);
                driveByEncoder(-13,.6);
                gyroTurn(-45,.4);
            }
            else{
                driveByEncoder(-3, 0.6);
                gyroTurn(-45, 0.4);
                driveByEncoder(10, 0.6);
                driveByEncoder(-10, 0.6);
            }
        }
    }
    private void sampling4(){
    telemetry.addData("got here", true);
    telemetry.update();
        lineUpByColorSimple();
        sleep(200);
        if(detector.getAligned()){
            if(detector.getAligned()) {
                telemetry.addData("Sees center: ", true);
                telemetry.update();
                position = "Center";
                sleep(300);
                driveByEncoder(15, 0.6);
                sleep(100);
                driveByEncoder(-8, 0.6);
                sleep(300);
            }
        }
        else if(position == null) {

            sleep(100);
           gyroTurn(-25, 0.4);
           sleep(500);
           if(detector.getAligned()){
               if(detector.getAligned()) {
                   telemetry.addData("sees right: ", true);
                   position = "Right";
                   gyroTurn(-15, 0.4);
                   driveByEncoder(20, 0.6);
                   driveByEncoder(-10 , 0.6);//15............!
                   gyroTurn(25, 0.4);
               }

           }
        }

        if(position == null){
            position = "Left";
            telemetry.addData("going left: ", true);
            gyroTurn(60,.4);
            driveByEncoder(15, 0.6);
            driveByEncoder(-9 , 0.6);
            gyroTurn(-25, 0.4);
        }
    }

    private void goToWall(){
        //driveByEncoder(-2, 0.3);
        gyroTurn(60, 0.4);
        drivebyRange(10, 0.6, rangeRight);
        if(position != "Left") {
            driveByEncoder(2, 0.4);
        }
        driveByEncoder(-1, 0.6);
       // straighten(DistanceUnit.INCH);
    }
    //returns the opposite of the state of a specified touch sensor
    private boolean isTouched (DigitalChannel touch){
        return !touch.getState();
    }

    private void goToTouch ( double power, DigitalChannel sensor){
        //drives hanging motors until touch sensor is pressed
        while (!isTouched(sensor) && opModeIsActive()) {
            hangingMotor.setPower(power);
        }
        hangingMotor.setPower(powerOff);
    }
    //returns the opposite of the magnetic limit switch's state
    private boolean foundMagnet (DigitalChannel sensor){
            return !sensor.getState();
    }
    //moves hangingmotor until the magnetic limit sensor on the hanging motor reads true
    private void goToMagnetLimitSensor ( double power, DigitalChannel sensor){
        while (!foundMagnet(sensor) && opModeIsActive()) {
            hangingMotor.setPower(power);
        }
        hangingMotor.setPower(powerOff);
    }
    //moves the robot off the hook
    private void deploy () {
        hangingMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hangingMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //drops
        while(opModeIsActive() && hangingMotor.getCurrentPosition() > 7700) {
            hangingMotor.setPower(-1);
        }
        hangingMotor.setPower(powerOff);

        //strafes just a tiny bit to ensure we don't catch
        //strafeByEncoder(5, 0.3, true );
    }
    //drives the robot using the value of the front left wheel's encoder as a reference to the robot's position
    private void driveByEncoder ( double position, double power){
        //sets motors drive mode
        leftF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //converts encoder ticks to approximately inches
        double wheelDiameter = 4 * 3.14;
        position = position / wheelDiameter;
        position *= 1120;
        //prints target to phone

        //determines the direction
        if (position >= 0) {
            //if forwards, sets motor's power until the actual position is greater than the target position
            while (-leftF.getCurrentPosition() < position && opModeIsActive()) {
                //setPowerStraight(power);
                setPowerStraight(power);
            }
            powerMotorsOff();
        } else {
            //if backwards, sets the motor to a negative power until the actual position is greater than the target position
            while (-leftF.getCurrentPosition() > position && opModeIsActive()) {
                //setPowerStraight(-power);
                setPowerStraight(-power);
            }
        powerMotorsOff();
            inertiaCorrection = (int) getAngles();
            gyroTurn(inertiaCorrection, 0.4);
        }
    }
    //strafes using the front left motor's encoder as a reference point to the robot's position
    private void strafeByEncoder ( double position, double power, boolean isRight){
        //sets the motor's modes
        leftF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //converts encoder ticks into approximate inches
        double wheelDiameter = 4 * 3.14, degrees, altPwr;
        position = position / wheelDiameter;
        position *= 1120;
        //if strafing right
        if (isRight) {
            //strafes right correcting with gyro
            while (-leftF.getCurrentPosition() < position && opModeIsActive()) {
                degrees = checkDirection();
                altPwr = power - (degrees * 0.1);
                setMotorPowers(power, -power,
                               -power, power);
            }
        }
        else {
            //drives until the negative of the left encoder's value is less than the position correcting with the gyro
            while (leftF.getCurrentPosition() < position && opModeIsActive()) {
                degrees = checkDirection();
                altPwr = power + (degrees * 0.1);
                setMotorPowers(-power, power,
                                power, -power);
            }
        }
        powerMotorsOff();
    }
    //straightens the robot using the two color sensors
    private void lineUpByColorSimple() {
        //while both color sensors are within a certain range, the following process continues to happen
        while (!withinColorRange(32, 20, colorRight) && !withinColorRange(30, 20, colorLeft) && opModeIsActive()) {
            //if the right color sensor is not in the desired value, drive forwards.
            if (!withinColorRange(32, 20, colorRight) && opModeIsActive()){
                rightF.setPower(0.3);
                rightB.setPower(0.3);
            } else { //if it is not within the specified range, it drives backwards
                rightF.setPower(-0.3);
                rightB.setPower(-0.3);
            }
            //if the left color sensor is not within the specified color range, the robot drives forwards
            if (!withinColorRange(32, 20, colorLeft) && opModeIsActive()) {
                leftF.setPower(0.3);
                leftB.setPower(0.3);
            } else { //if the left color sensor is not within the specified color range, the robot drives backwards
                leftF.setPower(-0.3);
                leftB.setPower(-0.3);
            }
            telemetry.addData("left", leftF.getCurrentPosition());
            telemetry.addData(" actual power: ", leftF.getPower());
            telemetry.addData("rightF", rightF.getPower());
            telemetry.update();
        }
        //turns off motors
        powerMotorsOff();
    }
    private void setScoringArmToPosition(double position, double power){
    slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if(slideMotor.getCurrentPosition() < position){
            while(slideMotor.getCurrentPosition() < position && opModeIsActive()){
                slideMotor.setPower(power);
            }
        }
        else{
            while(slideMotor.getCurrentPosition() > position && opModeIsActive()){
                slideMotor.setPower(-power);
            }
        }
    }
    /*private void gyroTurn2(int turnDegree,double power ){
    while getAngles() != angleStart
    }*/
}