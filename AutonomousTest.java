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
    double correction, globalAngle;
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
private double GetAngles(){
    Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

    if (deltaAngle < -180){
        deltaAngle += 360;
    }
    else if(deltaAngle > 180) {
        deltaAngle -= 360;
    }

        globalAngle += deltaAngle;
        lastAngles = angles;

        return globalAngle;
}
private double CheckDirection(){
    double correction, angle, gain = 0.1;
    angle = GetAngles();
    if (angle == 0){
        correction = 0;
    }
    else{
        correction = -angle;
    }

    correction = correction * gain;
    
    return correction;
}

}
