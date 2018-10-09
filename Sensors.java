package org.firstinspires.ftc.teamcode.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
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
    range = hardwareMap.get(DistanceSensor.class, "range");
    left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); //sets left motor to run without encoder
    right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); //sets right motor to run without encoder

    waitForStart();
    /*while(opModeIsActive()) {
        telemetry.addData("Range in inches: ", range.getDistance(DistanceUnit.INCH));
        telemetry.addData("Range in CM: ", range.getDistance(DistanceUnit.CM));
        telemetry.addData("Range in Meters: ", range.getDistance(DistanceUnit.METER));
        telemetry.addData("Range in MM: ", range.getDistance(DistanceUnit.MM));
        telemetry.update();
        }*/
    left.setPower(DriveByDistance(10, DistanceUnit.INCH, 10));
    right.setPower(DriveByDistance(10, DistanceUnit.INCH, 10));
    telemetry.addData("Left: ", left.getPower());
    telemetry.addData("Right: ", right.getPower());
    telemetry.addData("Distance: ", range.getDistance(DistanceUnit.INCH));
    telemetry.update();
   /* while(!InRange(25, DistanceUnit.MM)){
        telemetry.addData("Not in range, ", range.getDistance(DistanceUnit.MM));
        telemetry.update();
    }
    telemetry.addData("in range, ", ":)");
    telemetry.update();
*/
    }

    public boolean InRange(double target, DistanceUnit units){
    boolean inRange;
    double distance;

    distance = range.getDistance(units);
    if(distance <= target){
        inRange = true;
    }
    else{
        inRange = false;
        }
    return inRange;
    }

    private double DriveByDistance(double desiredTime, DistanceUnit units, double stopDistance ){
    double distance; //sets variable to hold value that range sensor reads
    double rate = 0; //sets the variable to hold the value that ill be the wheels power
    while (InRange(stopDistance, DistanceUnit.INCH)) { //happens until we reach the desired proximity from the target object
        distance = range.getDistance(units); //sets the distance variable to the sensors reading
        rate = distance / desiredTime; //converts distance to a rate baed on how long we want to take
        return rate; //returns the rate
        }
    return rate;//returns rate of zero
    }
}
