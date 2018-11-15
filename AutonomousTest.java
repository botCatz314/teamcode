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
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import com.qualcomm.robotcore.hardware.ColorSensor;


@Autonomous (name = "AutonomousTest")
public class AutonomousTest extends LinearOpMode {
    private ColorSensor colorLeft, colorRight;
    private DcMotor left, right;
    BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    double correction, globalAngle, powerOff = 0;
@Override
    public void runOpMode() {
    colorLeft = hardwareMap.get(ColorSensor.class, "colorLeft");
    colorRight = hardwareMap.get(ColorSensor.class, "colorRight");
    left = hardwareMap.dcMotor.get("left");
    right = hardwareMap.dcMotor.get("right");
    left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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
    waitForStart();
    telemetry.addData("Color Left: ", colorLeft.blue());
    telemetry.addData("Color Right: ", colorRight.blue());
    telemetry.update();
    sleep(20000);
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
//turns using the gyro to determine distance
private void GyroTurn(int degrees, double power){
    //declares two variables to hold the power of the left and right drive motors
    double leftPower, rightPower;
    //sets angle variables to starting values
    ResetAngles();
    //if it is less than 0, sets drive motors to turn right
    if(degrees < 0){
        leftPower = -power;
        rightPower = power;
    }
    //otherwise, if the value of degrees is greater than 0, sets drive motors to turn left
    else if(degrees > 0){
         leftPower = power;
         rightPower = -power;
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
    left.setPower(-power + correction);
    right.setPower(-power);
}
}
