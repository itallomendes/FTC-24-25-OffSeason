package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class Teleop extends OpMode {
    DcMotor DIE, DID, TE, TD, motorExpansaoV, modularcoleta, modulardunk;
    CRServo servopinca, servogarra, servoModularPincaHorizontal;
    Servo modularPincaVerticalE, modularPincaVerticalD, expansaoHE, expansaoHD;
    IMU imu;

    int expansaoVerticalCompleta = 4100, posicaoBracoColeta = 0, posicaoBracoDunk;
    long temporizadorTransferencia, temporizadorAjusteVertical, temporizadorAjusteHorizontal;
    double posicaoVerticalPinca = 0.9, posicaoExpansaoH = 0.5;
    boolean soltouMarchinhaAgora = true, soltouMarchinhaExpansaoHorizontal = true;

    public long tempoDecorrido(long temporizador) {
        return System.currentTimeMillis() - temporizador;
    }

    @Override
    public void init() {
        modulardunk = hardwareMap.dcMotor.get("modulardunk");
        modulardunk.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        modulardunk.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //modulardunk.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); POR ENQUANTO VAI USAR ENCODER, DPS VEMOS PID

        modularcoleta = hardwareMap.dcMotor.get("modularcoleta");
        modularcoleta.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        modularcoleta.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //modularcoleta.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); POR ENQUANTO VAI USAR ENCODER, DPS VEMOS PID
        modularcoleta.setDirection(DcMotorSimple.Direction.REVERSE);

        motorExpansaoV = hardwareMap.dcMotor.get("ExpansaoV");
        motorExpansaoV.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorExpansaoV.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorExpansaoV.setDirection(DcMotorSimple.Direction.REVERSE);

        servopinca = hardwareMap.crservo.get("Pinca");
        servogarra = hardwareMap.crservo.get("Garra");
        servoModularPincaHorizontal = hardwareMap.crservo.get("ModularH");

        modularPincaVerticalD = hardwareMap.servo.get("ModularD");
        modularPincaVerticalE = hardwareMap.servo.get("ModularE");

        expansaoHE = hardwareMap.servo.get("expansaoesqr");
        expansaoHD = hardwareMap.servo.get("expansaodir");

        DIE = hardwareMap.dcMotor.get("DiE");
        TE = hardwareMap.dcMotor.get("TE");
        DID = hardwareMap.dcMotor.get("DiD");
        TD = hardwareMap.dcMotor.get("TD");
        imu = hardwareMap.get(IMU.class, "imu");

        DID.setDirection(DcMotorSimple.Direction.REVERSE);
        TD.setDirection(DcMotorSimple.Direction.REVERSE);
        modularcoleta.setDirection(DcMotorSimple.Direction.REVERSE);

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        motorExpansaoV.setTargetPosition(0);

        modularcoleta.setTargetPosition(0);
        modulardunk.setTargetPosition(0);

        temporizadorTransferencia = System.currentTimeMillis();
        temporizadorAjusteHorizontal = System.currentTimeMillis();
        temporizadorAjusteVertical = System.currentTimeMillis();
    }

    @Override
    public void loop() {
        modularcoleta.setPower(0.8);
        //modulardunk.setPower(0.8);

        expansaoHD.setPosition(1-posicaoExpansaoH);
        expansaoHE.setPosition(posicaoExpansaoH);

        double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double x = gamepad1.left_stick_x; //robô tava andando lateralmente pro lado errado, rever isso
        double rx = gamepad1.right_trigger - gamepad1.left_trigger;

        // This button choice was made so that it is hard to hit on accident,
        // it can be freely changed based on preference.
        // The equivalent button is start on Xbox-style controllers.
        if (gamepad1.y) {
            imu.resetYaw();
        }

        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1.3;  // Counteract imperfect strafing

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        DIE.setPower(frontLeftPower);
        TE.setPower(backLeftPower);
        DID.setPower(frontRightPower);
        TD.setPower(backRightPower);

        //motorExpansaoH.setMode(DcMotor.RunMode.RUN_TO_POSITION); //fica mandando os motores de expansão irem pra última posição que foi definida
        //todas as vezes que o looping é executado
        modularcoleta.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        modulardunk.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if (gamepad2.x) {
            posicaoBracoDunk = 20; //levanta um pouco o braco de deposito pra não atrapalhar
            posicaoBracoColeta = 360; //expansão horizontal para coleta do bloco no chão
            posicaoVerticalPinca = 0.75;
        } else if (gamepad2.y) {
            posicaoBracoColeta = -10;  //volta o braço de coleta pra fazer a transferencia
            temporizadorTransferencia = System.currentTimeMillis(); //reseta temporizador para saber o momento de baixar o braço de deposito
            posicaoVerticalPinca = 0.5;
        }

        //modulardunk.setTargetPosition(posicaoBracoDunk);
        modularcoleta.setTargetPosition(posicaoBracoColeta);

        if (250 < tempoDecorrido(temporizadorTransferencia) && tempoDecorrido(temporizadorTransferencia) < 500) {
            posicaoBracoDunk = 10;
        }

        if (gamepad2.left_bumper) {
            servogarra.setPower(gamepad2.right_stick_x);
        } else {
            servopinca.setPower(gamepad2.right_stick_x);
        }

        servoModularPincaHorizontal.setPower(gamepad2.left_trigger - gamepad2.right_trigger);

        modularPincaVerticalD.setPosition(posicaoVerticalPinca);
        modularPincaVerticalE.setPosition(1 - posicaoVerticalPinca);


        if (gamepad2.dpad_up && (System.currentTimeMillis() - temporizadorAjusteVertical) > 100) {
            posicaoVerticalPinca = Math.min(posicaoVerticalPinca+0.1, 0.9);
            temporizadorAjusteVertical = System.currentTimeMillis();
        } else if (gamepad2.dpad_down && (System.currentTimeMillis() - temporizadorAjusteVertical) > 100) {
            posicaoVerticalPinca = Math.max(posicaoVerticalPinca-0.1, 0.1);
            temporizadorAjusteVertical = System.currentTimeMillis();
        }

        if (gamepad2.dpad_right && (System.currentTimeMillis() - temporizadorAjusteHorizontal > 100)) {
            posicaoExpansaoH = Math.min(posicaoExpansaoH+0.1, 0.9);
            temporizadorAjusteHorizontal = System.currentTimeMillis();
        } else if (gamepad2.dpad_left && (System.currentTimeMillis() - temporizadorAjusteHorizontal > 100)) {
            posicaoExpansaoH = Math.max(posicaoExpansaoH-0.1, 0.1);
            temporizadorAjusteHorizontal = System.currentTimeMillis();
        }

        telemetry.addData("Posição vertical pinça", posicaoVerticalPinca);
        telemetry.addData("ExpansaoV", motorExpansaoV.getCurrentPosition());
        telemetry.addData("Dunk", modulardunk.getCurrentPosition());
        telemetry.addData("PosiçãoH", posicaoExpansaoH);
        telemetry.update();

        if (Math.abs(gamepad1.right_stick_y) > 0.25) {
            soltouMarchinhaAgora = false;
            motorExpansaoV.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motorExpansaoV.setPower(-gamepad1.right_stick_y);
        } else if (Math.abs(gamepad1.right_stick_y) < 0.25 && !soltouMarchinhaAgora) {
            soltouMarchinhaAgora = true;
            motorExpansaoV.setTargetPosition(motorExpansaoV.getCurrentPosition());
            motorExpansaoV.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorExpansaoV.setPower(0.8);
            motorExpansaoV.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        if (gamepad1.dpad_up) {
            motorExpansaoV.setTargetPosition(expansaoVerticalCompleta);
            motorExpansaoV.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }
}
