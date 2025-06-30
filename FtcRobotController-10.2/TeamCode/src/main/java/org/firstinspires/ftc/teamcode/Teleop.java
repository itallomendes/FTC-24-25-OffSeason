package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class Teleop extends OpMode {
    DcMotor motorDID, motorDIE, motorTD, motorTE, motorSE, motorSD;

    SistemasDoRobo sistemas;

    float forcaA, forcaB, valorMarchinhaDireitaY, valorMarchinhaDireitaX, forcaDeGiro, forcaTE, forcaDIE, forcaTD, forcaDID;
    double multiplicadorVelocidade = 1, posicaoVerticalGarra = 0.25, denominator, frontLeftPower, frontRightPower, backRightPower, backLeftPower, botHeading, x, y, rx, rotX, rotY;
    long tempo, tempodunk;

    int posicaoBraco;

    IMU imu;

    @Override
    public void init() {
        imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));

        imu.initialize(parameters);

        motorDID = hardwareMap.get(DcMotor.class, "DiD");
        motorDIE = hardwareMap.get(DcMotor.class, "DiE");
        motorTD = hardwareMap.get(DcMotor.class, "TD");
        motorTE = hardwareMap.get(DcMotor.class, "TE");

        /*
        motorSE = hardwareMap.get(DcMotor.class, "SE");
        motorSD = hardwareMap.get(DcMotor.class, "SD");
        motorSE.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorSD.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorSE.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorSD.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorSD.setDirection(DcMotorSimple.Direction.REVERSE);

        motorSE.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorSD.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        */


        motorTD.setDirection(DcMotorSimple.Direction.REVERSE);
        motorDID.setDirection(DcMotorSimple.Direction.REVERSE);

        motorDID.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorDIE.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorTD.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorTE.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        tempo = System.currentTimeMillis();
        tempodunk = System.currentTimeMillis();
    }

    @Override
    public void loop() {
        telemetry.update();

        if (gamepad2.dpad_up && (System.currentTimeMillis() - tempo) > 50) {
            posicaoBraco -= 10;
            tempo = System.currentTimeMillis();
        } else if (gamepad2.dpad_down && (System.currentTimeMillis() - tempo) > 50) {
            posicaoBraco += 10;
            tempo = System.currentTimeMillis();
        }

        if (gamepad2.dpad_right && (System.currentTimeMillis() - tempo) > 100) {
            posicaoVerticalGarra -= 0.1;
            tempo = System.currentTimeMillis();
        } else if (gamepad2.dpad_left && (System.currentTimeMillis() - tempo) > 100) {
            posicaoVerticalGarra += 0.1;
            tempo = System.currentTimeMillis();
        }

        posicaoVerticalGarra = Math.min(Math.max(posicaoVerticalGarra, 0.1), 0.9);

        /*

        if (gamepad2.left_bumper) {
            motorSE.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorSD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            motorSD.setTargetPosition(-200);
            motorSE.setTargetPosition(-200);

            motorSD.setPower(-1);
            motorSE.setPower(-1);

            motorSD.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorSE.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            posicaoBraco = 80;
        } else if (gamepad2.right_bumper) {

            motorSE.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorSD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            motorSD.setTargetPosition(-6000);
            motorSE.setTargetPosition(-6000);

            motorSD.setPower(1);
            motorSE.setPower(1);

            motorSD.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorSE.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
         */

        if (gamepad2.x) { //POSIÇÃO DA COLETA TÁ SENDO 230, vai pra 200 pra n dar uma porrada no chão
            posicaoBraco = 200;
            posicaoVerticalGarra = 0.1;
        } else if (gamepad2.y) {
            posicaoBraco = 0;
            posicaoVerticalGarra = 0.8;
        } else if (gamepad2.b) {
            posicaoBraco = 100;
            posicaoVerticalGarra = 0.4;
        }

        y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
        x = gamepad1.left_stick_x; //robô tava andando lateralmente pro lado errado, rever isso
        rx = gamepad1.right_trigger - gamepad1.left_trigger;

        if (gamepad1.y) {
            imu.resetYaw();
        }

        botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1.3;  // Counteract imperfect strafing

        denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        frontLeftPower = (rotY + rotX + rx) / denominator;
        backLeftPower = (rotY - rotX + rx) / denominator;
        frontRightPower = (rotY - rotX - rx) / denominator;
        backRightPower = (rotY + rotX - rx) / denominator;

        motorDIE.setPower(frontLeftPower*multiplicadorVelocidade);
        motorTE.setPower(backLeftPower*multiplicadorVelocidade);
        motorDID.setPower(frontRightPower*multiplicadorVelocidade);
        motorTD.setPower(backRightPower*multiplicadorVelocidade);

        if (gamepad1.a) {
            multiplicadorVelocidade = 0.5;
        } else if (gamepad1.b) {
            multiplicadorVelocidade = 1;
        }

        if (gamepad1.right_bumper) {
            sistemas.cesta.Depositar();
        }

        if (gamepad2.right_trigger > 0.2) {
            servoGarraModular.setPower(-gamepad2.right_trigger);
        } else {
            servoGarraModular.setPower(gamepad2.left_trigger);
        }

        if (gamepad1.dpad_up) {
            motor_Expansao.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor_Expansao.setTargetPosition(4100); // JÁ TÁ OK
            motor_Expansao.setPower(1);
            motor_Expansao.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        } else if (gamepad1.dpad_down) {
            motor_Expansao.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor_Expansao.setTargetPosition(0);
            motor_Expansao.setPower(-1);
            motor_Expansao.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        } else if (gamepad1.dpad_right) {
            motor_Expansao.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor_Expansao.setTargetPosition(1900);
            motor_Expansao.setPower(1);
            motor_Expansao.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        } else if (gamepad1.dpad_left) {
            motor_Expansao.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor_Expansao.setTargetPosition(1650);
            motor_Expansao.setPower(-1);
            motor_Expansao.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        } else if (gamepad1.x) {
            motor_Expansao.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor_Expansao.setTargetPosition(420);
            motor_Expansao.setPower(1);
            motor_Expansao.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        if (Math.abs(gamepad1.right_stick_y) > 0.2 || motor_Expansao.getMode() == DcMotor.RunMode.RUN_WITHOUT_ENCODER) {
            motor_Expansao.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor_Expansao.setPower(-gamepad1.right_stick_y);
        }

        if (gamepad2.left_stick_button) {
            modularBraco.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            modularBraco.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        } else if (gamepad1.left_stick_button) {
            motor_Expansao.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor_Expansao.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        servoModularGarraVerticalD.setPosition(posicaoVerticalGarra);
        servoModularGarraVerticalE.setPosition(1.1-posicaoVerticalGarra);

        servoGarraPinca.setPower(gamepad2.left_stick_x * 0.5); //Abre e fecha a pinça

        servoGarraBraco.setPower(-gamepad2.right_stick_x * 0.35);
    }
}