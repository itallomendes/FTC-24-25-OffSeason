package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
@Autonomous
public class TesteAutonoma extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        SistemasDoRobo sistemas = new SistemasDoRobo(hardwareMap, true, true);
        SampleMecanumDrive chassi = new SampleMecanumDrive(hardwareMap);

        Pose2d posicaoInicial = new Pose2d(37.5,60, Math.toRadians(0));
        chassi.setPoseEstimate(posicaoInicial);

        waitForStart();

        Trajectory InitialScore = chassi.trajectoryBuilder(posicaoInicial, true)
                .lineToLinearHeading(new Pose2d(55,54,Math.toRadians(45)))
                .build();

        Trajectory Sample1 = chassi.trajectoryBuilder(InitialScore.end(), true)
                .lineToLinearHeading(new Pose2d(48,37,Math.toRadians(90)))
                .build();

        Trajectory Score1 = chassi.trajectoryBuilder(Sample1.end(), true)
                .lineToLinearHeading(new Pose2d(55,54,Math.toRadians(45)))
                .build();

        Trajectory Sample2 = chassi.trajectoryBuilder(Score1.end(), true)
                .lineToLinearHeading(new Pose2d(58,37,Math.toRadians(90)))
                .build();

        Trajectory Score2 = chassi.trajectoryBuilder(Sample2.end(), true)
                .lineToLinearHeading(new Pose2d(55,54,Math.toRadians(45)))
                .build();

        Trajectory Sample3 = chassi.trajectoryBuilder(Score2.end(), true)
                .lineToLinearHeading(new Pose2d(55,37,Math.toRadians(135)))
                .build();

        Trajectory Score3 = chassi.trajectoryBuilder(Sample3.end(), true)
                .lineToLinearHeading(new Pose2d(55,54,Math.toRadians(45)))
                .build();

        sistemas.bracoMeiaAltura();

        chassi.followTrajectory(InitialScore); //TEM QUE FICAR CHAMANDO O SISTEMAS.UPDATE DURANTE TODAS AS TRAJETÓRIAS SIMULTANEAMENTE

        sistemas.depositar();

        chassi.followTrajectory(Sample1);

        sistemas.coletarSample(); //JÁ PEGA NO CHÃO, SOLTA O SAMPLE NA CESTA E BAIXA O BRAÇO MEIA ALTURA

        chassi.followTrajectory(Score1);

        sistemas.depositar();

        chassi.followTrajectory(Sample2);

        sistemas.coletarSample();

        chassi.followTrajectory(Score2);

        sistemas.depositar();

        chassi.followTrajectory(Sample3);

        sistemas.coletarSample();

        chassi.followTrajectory(Score3);

        sistemas.depositar();
    }


}
