package org.firstinspires.ftc.teamcode.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;

@TeleOp(name = "MTF TeleOp", group = "Default")
public class MTFTeleOp extends LinearOpMode {

    private DcMotor leftF, rightF, leftB, rightB, hangingMotor, pivotMotor, slideMotor, collector; //declares
    double velX = 0, velY, velR;
    boolean motorIsUsed = false, driveAtAngle;
    private DigitalChannel touchLower, touchUpper;

    private boolean collecting;
@Override
    public void runOpMode()
{
    leftF = hardwareMap.dcMotor.get("leftF"); //sets value to left motor
    rightF = hardwareMap.dcMotor.get("rightF"); //sets value to right motor
    leftB = hardwareMap.dcMotor.get(("leftB"));
    rightF = hardwareMap.dcMotor.get("rightB");
    hangingMotor = hardwareMap.dcMotor.get("hangingMotor");
    touchLower = hardwareMap.get(DigitalChannel.class, "touchLeft");
    touchUpper = hardwareMap.get(DigitalChannel.class, "touchRight");
    pivotMotor = hardwareMap.dcMotor.get("pivotMotor");
    slideMotor = hardwareMap.dcMotor.get("slideMotor");
    collector = hardwareMap.dcMotor.get("collector");





    touchLower.setMode(DigitalChannel.Mode.INPUT);
    touchUpper.setMode(DigitalChannel.Mode.INPUT);


    rightF.setDirection(DcMotorSimple.Direction.REVERSE);
    rightB.setDirection(DcMotorSimple.Direction.REVERSE);
    hangingMotor.setDirection(DcMotorSimple.Direction.REVERSE);

    waitForStart();
    while (opModeIsActive())
    {
        motorIsUsed = false;
        if(gamepad1.dpad_up){
            velY = 0.5;
            setControlBools(false, true);
        }
        else if(gamepad1.dpad_down){
            velY = -0.5;
            setControlBools(false, true);
        }
        else if(gamepad1.dpad_right){
            velX = 0.5;
            setControlBools(false, true);
        }
        else if(gamepad1.dpad_left){
            velX = -0.5;
            setControlBools(false, true);
        }
        if(driveAtAngle){
            driveMotors(velY + velX, velY - velX, velY - velX, velY + velX);
            setControlBools(true, false);
        }
        if(gamepad1.right_trigger >=0.1 && !motorIsUsed){
            motorIsUsed = true;
            velX = gamepad1.right_trigger;
            driveMotors( velX, -velX,
                        -velX,  velX);
        }

        if(gamepad1.left_trigger >= 0.1 && !motorIsUsed){
            motorIsUsed = true;
            velX = -gamepad1.left_trigger;
            driveMotors(-velX,  velX,
                         velX, -velX);
        }
    if(!motorIsUsed) {
            double velL = 0;
            double velR = 0;
            motorIsUsed = true;
        if (gamepad1.left_stick_y >= 0.1 || gamepad1.left_stick_y <= -0.1) {
            velL = gamepad1.left_stick_y;
        }

        if (gamepad1.right_stick_y >= 0.1 || gamepad1.right_stick_y <= -0.1) {
            velR = (gamepad1.right_stick_y); //sets right motor power
        }
        driveMotors(velL, velR,
                    velL, velR);
    }

        collector.setPower(gamepad2.right_trigger);
        collector.setPower(-gamepad2.left_trigger);

        slideMotor.setPower(gamepad2.right_stick_y);
        pivotMotor.setPower(-gamepad2.left_stick_y);

        if(gamepad1.right_bumper){
            hangingMotor.setPower(0.2);
        }
        else if(gamepad1.left_bumper){
            hangingMotor.setPower(-0.2);
        }
        else{
            hangingMotor.setPower(0);
        }

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
}
