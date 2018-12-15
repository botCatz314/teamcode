package org.firstinspires.ftc.teamcode.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name = "botCatzTeleOp", group = "Default")
public class botCatzTeleOp extends LinearOpMode {

    private DcMotor leftF, rightF, leftB, rightB; //hangingMotor, pivotMotor, slideMotor, collector; //declares
    private DcMotor hangingMotor, pivotMotor, slideMotor, collector;
    double velX = 0, velY, velR;
    boolean motorIsUsed = false, driveAtAngle;
    private DigitalChannel magneticSwitch;
    private DigitalChannel touchUpper;
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
        motorIsUsed = false;
        velX = 0;
        velY = 0;
        if(gamepad1.dpad_up){
            velY = -0.5;
            setControlBools(false, true);
        }
        else if(gamepad1.dpad_down){
            velY = 0.5;
            setControlBools(false, true);
        }
        else if(gamepad1.dpad_right){
            velX = -0.5;
            setControlBools(false, true);
        }
        else if(gamepad1.dpad_left){
            velX = 0.5;
            setControlBools(false, true);
        }
        if(driveAtAngle){
            driveMotors(velY + velX, velY - velX, velY - velX, velY + velX);
            setControlBools(true, false);
        }
        if((gamepad1.right_trigger >=0.1 && !motorIsUsed) || (gamepad2.dpad_right && !motorIsUsed)){
            motorIsUsed = true;
            velX = gamepad1.right_trigger;
            driveMotors( -velX,  velX,
                          velX,  -velX);
        }

        if((gamepad1.left_trigger >= 0.1 && !motorIsUsed) || (gamepad2.dpad_left && !motorIsUsed)){
            motorIsUsed = true;
            velX = gamepad1.left_trigger;
            driveMotors(  velX,  -velX,
                         -velX,  velX);
        }
    if(!motorIsUsed) {
            double velL = 0;
            double velR = 0;
            motorIsUsed = true;
        if(gamepad2.dpad_up){
            velL = 0.2;
            velR = 0.2;
        }
        if(gamepad2.dpad_down){
            velL = -0.2;
            velR = -0.2;
        }
        if (gamepad1.left_stick_y >= 0.1 || gamepad1.left_stick_y <= -0.1) {
            velL = -gamepad1.left_stick_y;
        }

        if (gamepad1.right_stick_y >= 0.1 || gamepad1.right_stick_y <= -0.1) {
            velR = -(gamepad1.right_stick_y); //sets right motor power
        }
        driveMotors(velL, velR,
                    velL, velR);
    }

        if(gamepad2.right_bumper){
            collector.setPower(9.0);
        }
        else if(gamepad2.left_bumper){
            collector.setPower(powerOff);
        }
        else if(gamepad2.a){
            collector.setPower(-0.9);
        }
            slideMotor.setPower(gamepad2.left_stick_y);

            pivotMotor.setPower(-gamepad2.right_stick_y);
        if(gamepad2.b){
            armToScoringPosition(0.5);
        }
        if(gamepad2.x){
            armToScoringPosition(-0.5);
        }

        if(gamepad2.y && !getTouch(touchUpper)){
            driveUntilTouch(1.0, hangingMotor, touchUpper);
        }
        if(gamepad2.right_trigger > 0.1) {
            hangingMotor.setPower(gamepad2.right_trigger);
        }
        else if(gamepad2.left_trigger > 0.1){
            hangingMotor.setPower(-gamepad2.left_trigger);
        }
        else{hangingMotor.setPower(0);}

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
