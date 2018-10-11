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
    private DistanceSensor range;
@Override
    public void runOpMode() {
    left = hardwareMap.dcMotor.get("left"); //set left drive motor
    right = hardwareMap.dcMotor.get("right"); //set right drive motor
    range = hardwareMap.get(DistanceSensor.class, "range");//
    left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); //sets left motor to run without encoder
    right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); //sets right motor to run without encoder
    left.setDirection(DcMotorSimple.Direction.REVERSE);//sets left motor to reverse

    waitForStart();

    DrivebyDistance(20, 10, DistanceUnit.INCH);
    sleep(20000);
    /*while(!InRange(50, DistanceUnit.CM)){
        right.setPower(.3);
        left.setPower(0.3);
        telemetry.addData("Not in range, ", range.getDistance(DistanceUnit.CM));
        telemetry.update();
    }
    right.setPower(0);
    left.setPower(0);
    telemetry.addData("in range, ", ":)");
    telemetry.addData("Final Range: ", range.getDistance(DistanceUnit.CM));
    telemetry.update();
    sleep(10000);*/
    }

    private boolean InRange(double target, DistanceUnit units){
    double distance; //creates a variable to hold the range sensor's reading
    distance = range.getDistance(units); //sets the distance variable to the value that the range sensor reads
    return (distance <= target); //returns true if the robot is closer to the target than the target position
    }


    private void DrivebyDistance( double completeTime, double stopTarget, DistanceUnit units){
    double distance;
    double linearRate, angularRate, motorRate, prgmRate, off = 0;
    double radius, RPS = 9000, percent = 100;

    radius = 2;
    while(!InRange(stopTarget, units)){ //repeats until we reach the desired range
        distance = range.getDistance(units); //finds the distance between the sensor and nearest object
        linearRate = distance / completeTime; //finds the rate in human measurement
        angularRate = linearRate / radius; //solves for the rotations per second
        motorRate = angularRate / RPS; //solves for the percentage that the motor needs to move at
        prgmRate = motorRate / percent; //puts the percentage between -1 and 1 for the motors
        right.setPower(prgmRate); //sets motor power
        left.setPower(prgmRate);
        telemetry.addData("leftPower: ", left.getPower()); //sends data for the left and right motor power and the distance sensor
        telemetry.addData("right Power", right.getPower());
        telemetry.addData("range: ", range.getDistance(DistanceUnit.INCH));
        telemetry.update();
    }
    prgmRate = off; //turns power of motors off
    right.setPower(prgmRate);
    left.setPower(prgmRate);
    }
}
