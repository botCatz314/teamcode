package org.firstinspires.ftc.teamcode.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;
@Autonomous (name = "AutonomousTest")
public class AutonomousTest extends LinearOpMode {
    private ColorSensor colorLeft, colorRight;
@Override
    public void runOpMode() {
    colorLeft = hardwareMap.get(ColorSensor.class, "colorLeft");
    colorRight = hardwareMap.get(ColorSensor.class, "colorRight");
    waitForStart();
    telemetry.addData("Color Left: ", colorLeft.blue());
    telemetry.addData("Color Right: ", colorRight.blue());
    telemetry.update();
}
}
