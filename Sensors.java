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
    right.setDirection(DcMotorSimple.Direction.REVERSE);//sets left motor to reverse

    waitForStart();

    DriveUntilDistance(10, DistanceUnit.INCH);
    sleep(20000);
    }

    private boolean InRange(double target, DistanceUnit units){
    double distance; //creates a variable to hold the range sensor's reading
    distance = range.getDistance(units); //sets the distance variable to the value that the range sensor reads
    return (distance <= target); //returns true if the robot is closer to the target than the target position
    }

    private void DriveUntilDistance(double target, DistanceUnit unit){

    while(!InRange(target +10, unit)) {
        left.setPower(0.5);
        right.setPower(0.5);
    }
    while (!InRange(target, unit )){
        left.setPower(0.1);
        right.setPower(0.1);
    }
    left.setPower(0);
    right.setPower(0);
    }

}
