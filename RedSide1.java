package org.firstinspires.ftc.teamcode.teamcode;

import android.drm.DrmStore;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.relicrecovery.JewelDetector;
import com.disnodeteam.dogecv.filters.LeviColorFilter;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;


@Autonomous (name = "RedSide1")
public class RedSide1 extends LinearOpMode {
    //motors
    private DcMotor leftWheel, rightWheel;
    private DcMotor leftIntake, rightIntake;
    //servos
    private Servo upDown, leftRight;
    private CRServo flipper;
    //declares sensors
    private BNO055IMU imu;
    //declares the Doge CV object for the jewels
    private JewelDetector jewelDetector;

    //auto setup vars
    String teamColor = null;
    int boardNum = 1;
    boolean confirm = false;

    //vumark vars
    String collumn = "Center";
   /* public static final String TAG = "Vuforia VuMark Sample";
    OpenGLMatrix lastLocation = null;
    VuforiaLocalizer vuforia;
    VuforiaTrackables relicTrackables;
    VuforiaTrackable relicTemplate;*/

    //servo position constants
    float servoUp = 1, servoDown = 0;
    float servoRight = 1, servoLeft = 0;
    //variables for the gyro
    private Orientation lastAngles = new Orientation();
    private double globalAngle;

    @Override
    public void runOpMode() {
        //sets value of drive motors
        leftWheel = hardwareMap.dcMotor.get("leftWheel");
        rightWheel = hardwareMap.dcMotor.get("rightWheel");

        //sets values of attachment motors
        leftIntake = hardwareMap.dcMotor.get("leftIntake");
        rightIntake = hardwareMap.dcMotor.get("rightIntake");

        upDown = hardwareMap.servo.get("upDown");
        leftRight = hardwareMap.servo.get("leftRight");
        flipper = hardwareMap.crservo.get("flipper");

        rightWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        leftIntake.setDirection(DcMotorSimple.Direction.REVERSE);




        telemetry.addData("Pls Select your team color: ", "right bumper = blue, left bumper = red");
        telemetry.update();

        while (teamColor == null && !isStarted()) {
            if (gamepad1.right_bumper) {
                teamColor = "BLUE";
            }
            if (gamepad1.left_bumper) {
                teamColor = "RED";
            }
        }

        telemetry.addData("You are " + teamColor + " team", "If this is not corect, select stop and start this process over");
        telemetry.addData("Pls select your board", "press bumper to change your board and a to confirm");
        telemetry.addData("you are on board:", boardNum);
        telemetry.update();

        while (!confirm) {
            if (gamepad1.right_bumper) {
                if (boardNum == 1) {
                    boardNum++;
                } else {
                    boardNum--;
                }
                telemetry.addData("the board is now set to " + boardNum, "press a to confirm o right bumper to change");
                telemetry.update();
            }

            if (gamepad1.a) {
                confirm = true;
            }
        }

        telemetry.addData("Your match is set", " you are " + teamColor + " and on board " + boardNum);
        telemetry.addData("Pls wait for your sensors to calibate", "waiting...");
        telemetry.update();


        jewelDetector = new JewelDetector();
        jewelDetector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        jewelDetector.colorFilterRed = new LeviColorFilter(LeviColorFilter.ColorPreset.RED);
        jewelDetector.colorFilterBlue = new LeviColorFilter(LeviColorFilter.ColorPreset.BLUE);
        jewelDetector.enable();




        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        telemetry.addData("Gyro is calibrating", "Pls wait");
        telemetry.update();

        while (!isStopRequested() && imu.isGyroCalibrated() && opModeIsActive()) {
            sleep(50);
            idle();
        }

        telemetry.addData("Ready for the round to begin", "good luck!");
        telemetry.update();

        waitForStart();
        sleep(1000);
        if(teamColor.equals("RED")){
            knockJewel();
            //gyroTurn(-90, 0.3);
            DriveOffBoard();
            sleep(500);
            gyroTurn(90, 0.3);
            sleep(100);

            switch(collumn){
                case "Center":
                    driveRobot(-6, 0.4);
                    break;
                case "Right":
                    driveRobot(-12, 0.4);
                    break;
                case "Left":
                    driveRobot(1.15, 0.4);
                    //gyroTurn(-10, 0.3);
                    sleep(5);
                    break;
            }
            sleep(500);
            gyroTurn(87, 0.3);
            sleep(90);
            if(collumn != "LEFT"){
                driveRobot(5, 0.4);
            }else{
                gyroTurn(-18, 0.4);
                sleep(50);
                driveRobot(3.5, 0.3);
            }
            sleep(200);
            flipper.setPower(-1);
            sleep(1100);
            flipper.setPower(0);
            sleep(1500);
            driveRobot(2, 0.3);
            sleep(500);
            driveRobot(-2, 0.3);
            sleep(1000);
            driveRobot(3, 0.3);
            sleep(500);
            driveRobot(-5, 0.3);

            flipper.setPower(1);
            sleep(1000);
            flipper.setPower(0);
        }else{
            knockJewel();

            driveRobot(7, 0.5);
            sleep(500);
            gyroTurn(87, 0.3);
            sleep(100);


            switch(collumn){
                case "Center":
                    driveRobot(-7.28, 0.4);
                    break;
                case "Right":
                    driveRobot(-3, 0.4);
                    sleep(5);
                    break;
                case "Left":
                    driveRobot(-12, 0.4);
                    //gyroTurn(-10, 0.3);
                    break;
            }
            sleep(500);
            gyroTurn(-87, 0.3);
            sleep(90);
            if(collumn != "RIGHT"){
                driveRobot(4, 0.4);
            }else{
                gyroTurn(-18, 0.4);
                sleep(50);
                driveRobot(4, 0.3);
            }
            sleep(500);
            flipper.setPower(-1);
            sleep(1000);
            flipper.setPower(0);
            sleep(1500);
            driveRobot(2.5, 0.3);
            sleep(500);
            driveRobot(-2, 0.3);
            sleep(1000);
            driveRobot(3, 0.3);
            sleep(500);
            driveRobot(-5, 0.3);

            flipper.setPower(1);
            sleep(1000);
            flipper.setPower(0);

        }


    }

    //uses the servo stick and Doge CV to knock the correct jewel off the platform
    private void knockJewel() {

        //if it recognizes the order of the jewels, then the sevo stick goes down
        //otherwise, it sends a message and leaves this function
        if (jewelDetector.getCurrentOrder() != JewelDetector.JewelOrder.UNKNOWN) {
            upDown.setPosition(servoDown);
            sleep(1500);
        } else {
            telemetry.addData("Cannot determine Jewel order", "Skipping procedure");
            telemetry.update();
            sleep(300);
            return;
        }
        //moves the servo to knock off the correct jewel
        if (jewelDetector.getCurrentOrder() == JewelDetector.JewelOrder.RED_BLUE) {
            if (teamColor.equals("RED")) {
                leftRight.setPosition(servoRight);
                sleep(1000);
            } else if (teamColor.equals("BLUE")) {
                leftRight.setPosition(servoLeft);
                sleep(1000);
            }
        } else if (jewelDetector.getCurrentOrder() == JewelDetector.JewelOrder.BLUE_RED) {
            if (teamColor.equals("RED")) {
                leftRight.setPosition(servoLeft);
                sleep(1000);
            } else if (teamColor.equals("BLUE")) {
                leftRight.setPosition(servoRight);
                sleep(1000);
            }
        }
        //resets everything
        jewelDetector.disable();
        upDown.setPosition(servoUp);
        sleep(500);
        jewelDetector.disable();
    }

    //gets the reading from the imu and converts the angle to be cumulative
    private double getAngles() {
        //declares and sets a variable to the reading of the imu
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        //declares and sets a variable to the change of the angle that is and the angle that was before
        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;
        //sets delta angle itself plus 360 if it is less than -180 degrees
        if (deltaAngle < -180) {
            deltaAngle += 360;
        }
        //sets delta angle to itself minus 360 if it is greater than 180 degrees
        else if (deltaAngle > 180) {
            deltaAngle -= 360;
        }
        //sets globalAngle to itself plus deltaAngle
        globalAngle += deltaAngle;
        //sets last angle to the current value of angles
        lastAngles = angles;
        //returns globalAngle
        return globalAngle;
    }

    private void gyroTurn(int degrees, double power) {
        int offset = 20;
        if(degrees < 0){
            degrees +=20;
        }else{
            degrees-=20;
        }
        resetAngles();
        resetAngles();
        resetAngles();

        //if it is less than 0, sets drive motors to turn right
        boolean isRight = degrees > 0;

        while(Math.abs(getAngles()) < Math.abs(degrees)){
            turnRobot(power, isRight);
        }

        //sets angle variables to starting values
        resetAngles();
    }

    private void resetAngles() {
        //sets last angles to the current reading of the imu
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        //sets global angle to 0
        globalAngle = 0;
    }

    private void turnRobot(double power, boolean turnRight) {
        if (turnRight) {
            leftWheel.setPower(power);
            rightWheel.setPower(-power);
        } else {
            leftWheel.setPower(-power);
            rightWheel.setPower(power);
        }
    }

    private void driveRobot(double distance, double power) {
        leftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double wheelCircumference = 3.75 * 3.14159;
        distance = distance / wheelCircumference;
        distance *= 1120;

        if (distance < 0) {
            power = -power;
        }
        leftWheel.setPower(power);
        rightWheel.setPower(power);

        while (Math.abs(leftWheel.getCurrentPosition()) <= Math.abs(distance) && Math.abs(rightWheel.getCurrentPosition()) <= Math.abs(distance) && opModeIsActive()) {
            if (Math.abs(leftWheel.getCurrentPosition()) < Math.abs(distance)) {
                leftWheel.setPower(power);
            }else{
                leftWheel.setPower(0);
            }
            if (Math.abs(rightWheel.getCurrentPosition()) <= Math.abs(distance)) {
                rightWheel.setPower(power);
            }else{
                rightWheel.setPower(0);
            }
        }
        leftWheel.setPower(0);
        rightWheel.setPower(0);

        leftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sleep(200);
        leftWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void turnOnIntake(){
        leftIntake.setPower(0.6);
        rightIntake.setPower(0.6);
    }

    private void DriveOffBoard() {
        driveRobot(-7, 0.5);
    }
    private void GoToCryptobox(){
        gyroTurn(90, 0.3);

    }

}