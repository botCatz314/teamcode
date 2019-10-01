package org.firstinspires.ftc.teamcode.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
@Disabled
@TeleOp(name = "botCatzTeleOpTim", group = "Default")
public class botCatzTeleOpTim extends LinearOpMode {

    private DcMotor leftF, rightF, leftB, rightB; //hangingMotor, pivotMotor, slideMotor, collector; //declares
    private DcMotor hangingMotor, pivotMotor, slideMotor, collector;
    double velX = 0, velY, velR;
    boolean motorIsUsed = false, driveAtAngle;
    private DigitalChannel magneticSwitch;
    private DigitalChannel touchUpper;
    private DigitalChannel up, down;
    private AnalogInput armPos;
    private double powerOff = 0;
    private boolean collecting;
@Override
    public void runOpMode()
{
    //finds wheel motors in Hardware Map
    leftF = hardwareMap.dcMotor.get("leftF");
    rightF = hardwareMap.dcMotor.get("rightF");
    leftB = hardwareMap.dcMotor.get(("leftB"));
    rightB = hardwareMap.dcMotor.get("rightB");
    //finds magnetic switch and touch sensors in hardware map
    magneticSwitch = hardwareMap.get(DigitalChannel.class, "magneticSwitch");
    touchUpper = hardwareMap.get(DigitalChannel.class, "touchUpper");
    armPos = hardwareMap.get(AnalogInput.class, "armPos");
    up = hardwareMap.get(DigitalChannel.class, "up");
    down = hardwareMap.get(DigitalChannel.class, "down");
    //finds the attachment motors in the hardware map
    hangingMotor = hardwareMap.dcMotor.get("hangingMotor");
    pivotMotor = hardwareMap.dcMotor.get("pivotMotor");
    slideMotor = hardwareMap.dcMotor.get("slideMotor");
    collector = hardwareMap.dcMotor.get("collector");

    touchUpper.setMode(DigitalChannel.Mode.INPUT);

    rightF.setDirection(DcMotorSimple.Direction.REVERSE);
    rightB.setDirection(DcMotorSimple.Direction.REVERSE);
    hangingMotor.setDirection(DcMotorSimple.Direction.REVERSE);

    waitForStart();
    telemetry.addData("happening: ", true);
    telemetry.update();
    while (opModeIsActive())
    {
        slideMotor.setPower(0);
        //drive mode 1

        if(gamepad2.right_trigger > 0.1) {
            if(gamepad2.right_bumper) {
                rightF.setPower(-gamepad2.right_trigger);
            } else {
                rightF.setPower(gamepad2.right_trigger);
            }
        }
        if(gamepad2.left_trigger > 0.1) {
            if(gamepad2.left_bumper) {
                leftF.setPower(-gamepad2.left_trigger);
            } else {
                leftF.setPower(gamepad2.left_trigger);
            }
        }
        if(gamepad2.right_stick_x > 0.1) {
            rightB.setPower(gamepad2.right_stick_x);
        }
        if(gamepad2.left_stick_x > 0.1) {
            leftB.setPower(gamepad2.left_stick_x);
        }

        telemetry.addData("leftF: ", leftF.getPower());
        telemetry.addData("leftB: ", leftB.getPower());
        telemetry.addData("rightF: ", rightF.getPower());
        telemetry.addData("rightB: ", leftB.getPower());
        telemetry.update();






        idle();
    }
}

    private void driveMotors(double powerLF, double powerRF, double powerLB, double powerRB){
        leftF.setPower(powerLF);
        rightF.setPower(powerRF);
        leftB.setPower(powerLB);
        rightB.setPower(powerRB);
    }
    private void setControlBools(boolean mtrIsUsed, boolean drvAtAngle){
        motorIsUsed = mtrIsUsed;
        driveAtAngle = drvAtAngle;
    }
    private boolean getTouch(DigitalChannel touchSensor){ return !touchSensor.getState();}
    private void driveUntilTouch(double power, DcMotor driveMotor, DigitalChannel touchSensor){
        while(!getTouch(touchSensor)) {
            driveMotor.setPower(power);
        }
        driveMotor.setPower(powerOff);
    }
    private boolean getMagneticSwitch(){ return !magneticSwitch.getState();}
    private void armToScoringPosition(double power){
        while(!getMagneticSwitch()){
            slideMotor.setPower(power);
        }
        slideMotor.setPower(powerOff);
    }

}
