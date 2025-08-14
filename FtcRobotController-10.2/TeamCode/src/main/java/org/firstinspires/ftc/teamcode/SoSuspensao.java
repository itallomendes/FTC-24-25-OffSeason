package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class SoSuspensao extends OpMode {

    DcMotor SE, SD;

    @Override
    public void init() {

        SE = hardwareMap.dcMotor.get("SE");
        SD = hardwareMap.dcMotor.get("SD");

        SE.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SD.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        SE.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        SD.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        SE.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        SD.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop() {
        SE.setPower(gamepad2.left_stick_y);
        SD.setPower(gamepad2.right_stick_y);
    }
}
