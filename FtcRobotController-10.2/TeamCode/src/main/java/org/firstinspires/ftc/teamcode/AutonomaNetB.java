package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous
public class AutonomaNetB extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        SistemasDoRobo sistemas = new SistemasDoRobo(hardwareMap, true, true);
        SampleMecanumDrive chassi = new SampleMecanumDrive(hardwareMap);

        Pose2d posicaoInicial = new Pose2d(37.5,60, Math.toRadians(0));
        chassi.setPoseEstimate(posicaoInicial);

        waitForStart();

        Trajectory InitialScore = chassi.trajectoryBuilder(posicaoInicial, true)
                .lineToLinearHeading(new Pose2d(53,52,Math.toRadians(45)))
                .build();

        Trajectory Sample1 = chassi.trajectoryBuilder(InitialScore.end(), true)
                .lineToLinearHeading(new Pose2d(46.5,38.5,Math.toRadians(90)))
                .build();

        Trajectory Score1 = chassi.trajectoryBuilder(Sample1.end(), true)
                .lineToLinearHeading(new Pose2d(51,51,Math.toRadians(45)))
                .build();

        Trajectory Sample2 = chassi.trajectoryBuilder(Score1.end(), true)
                .lineToLinearHeading(new Pose2d(57.5,38.5,Math.toRadians(90)))
                .build();

        Trajectory Score2 = chassi.trajectoryBuilder(Sample2.end(), true)
                .lineToLinearHeading(new Pose2d(51,51,Math.toRadians(45)))
                .build();

        Trajectory Sample3 = chassi.trajectoryBuilder(Score2.end(), true)
                .lineToLinearHeading(new Pose2d(55,51,Math.toRadians(90)))
                .build();


        Runnable updateLoop = () -> {
            while (opModeIsActive() && chassi.isBusy()) {
                chassi.update();
                sistemas.update();
            }
        };

        sistemas.bracoMeiaAltura();
        sistemas.CestaAlta();
        chassi.followTrajectoryAsync(InitialScore); //TEM QUE FICAR CHAMANDO O SISTEMAS.UPDATE DURANTE TODAS AS TRAJETÓRIAS SIMULTANEAMENTE
        updateLoop.run();
        sistemas.depositar();


        chassi.followTrajectoryAsync(Sample1);
        updateLoop.run();

        sistemas.coletarSample(); //JÁ PEGA NO CHÃO, SOLTA O SAMPLE NA CESTA E BAIXA O BRAÇO MEIA ALTURA

        sistemas.CestaAlta();
        chassi.followTrajectoryAsync(Score1);
        updateLoop.run();
        sistemas.depositar();


        chassi.followTrajectoryAsync(Sample2);
        updateLoop.run();

        sistemas.coletarSample();

        sistemas.CestaAlta();
        chassi.followTrajectoryAsync(Score2);
        updateLoop.run();
        sistemas.depositar();

        chassi.followTrajectoryAsync(Sample3);
        updateLoop.run();

        /*
        chassi.followTrajectoryAsync(Sample3);
        updateLoop.run();

        sistemas.coletarSample();

        chassi.followTrajectory(Score3);
        updateLoop.run();

        sistemas.depositar();

         */
    }


}
