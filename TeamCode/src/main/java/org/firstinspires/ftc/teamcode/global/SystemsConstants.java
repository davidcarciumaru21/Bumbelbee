package org.firstinspires.ftc.teamcode.global;

public final class SystemsConstants {

    public static final class StopperConstants {
        public static final double STOPPER_OPENED_POSITON = 0.15;
        public static final double STOPPER_CLOSED_POSITON = 0.625;
        public static final double TIME_TO_OPEN = 500;
        public static final double TIME_TO_CLOSE = 200;
    }

    public static final class IntakeConstants {
        public static final double PULL_POWER = 1.0;
        public static final double PUSH_POWER = -1.0;
        public static final double INTAKE_STALL_CURRENT = 10.0;
    }

    public static final class IndexerConstants {
        public static final double PULL_SPEED = 2595;
        public static final double PUSH_SPEED = -2595;
        public static final double P = 0.0;
        public static final double I = 0.0;
        public static final double D = 0.0;
        public static final double F = 12.0;
    }

    public static final class DeflectorConstants {
        public static final double MIN_ANGLE = Math.toRadians(20);
        public static final double MAX_ANGLE = Math.toRadians(40);
    }

    public static final class OuttakeConstants {
        public static double KP = 0.1;
        public static double KI = 0.0;
        public static double KS = 0.6;
        public static double KV = 0.0021;
    }

    public static final class TurretConstants {
        public static final double TICKS_PER_REV = 537.7;
        public static final double KP = 0.1;
        public static final double KI = 0.0;
        public static final double KD = 0.0;
    }

    public static final class ShooterConstants {
        public static final double THREE_BALLS_TIME = 700;
    }
 }
