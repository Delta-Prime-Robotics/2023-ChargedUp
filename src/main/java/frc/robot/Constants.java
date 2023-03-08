// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants { 

    // Connections to the RoboRio
    public static final class RoboRio {
        public final class PwmPort {
            // No PWM ports currently defined
        }
        public final class DioPort {
            public static final int kBeamBreak = 5;
            public static final int kLimitSwitch = 2;
        }
        public final class AnalogPort {
            public static final int kUltraSonic = 0;
        }
        public final class CanID {
            public static final int kLeftLeader = 11;
            public static final int kLeftFollower = 12;
            public static final int kRightLeader = 13;
            public static final int kRightFollower = 14;
            public static final int kArmLeader = 19;
            public static final int kArmFollower = 21;
            public static final int kIntakeControler = 20;
            public static final int kPDP = 0;
            public static final int kPCMcontroller = 1;
        }
    }    

    // Connections to the Drivers' Station Laptop
    public static final class Laptop {
        public final class UsbPort {
            public static final int kGamePad = 2;
            public static final int kFlightJoystick = 3;
        }
    }

    public static final class RobotConfig {
        // public static final class Camera {
        //     public static final double kHeightInMeters = Units.inchesToMeters(12);
        //     public static final double kPitchInRadians = Units.degreesToRadians(0);
        // }
        /* The following is an estimate. We need to confirm with the actual robot and remove this comment. */
        public static final double kDistancePerRotation = 2 * Math.PI * 6.0;
    }

    public static final class RobotCharacterization {
        // Need to characterize the robot to determine the actual values
        public static final double kAngularP = 0.1;
        public static final double kAngularD = 0.0;
        public static final double kLinearP = 0.1;
        public static final double kLinearD = 0.0;
    }

    // public static final class Field {
    //     public static final class Target1 {
    //         public static final double kHeightInMeters = Units.feetToMeters(3);
    //     }
    // } 

    // 3D Joystick (rotating stick)
    public static final class FlightStick{
        public final class Axis {
            public static final int kLeftRight = 0;
            public static final int kFwdBack = 1;
            public static final int kRotate = 2;
            public static final int kThrottle = 3;
        }
        // Buttons are already numbered, so no translation needed
    }
    
    // Constants for the gamepad joysticks & buttons
    public static final class GamePad {
        // Joysticks and their axes
        public final class LeftStick {
            public static final int kLeftRight = 0;
            public static final int kUpDown = 1;
        }
        public final class RightStick {
            public static final int kLeftRight = 2;
            public static final int kUpDown = 3;
        }
    
        public final class Button {
            public static final int kX = 1;
            public static final int kA = 2;
            public static final int kB = 3;
            public static final int kY = 4;
            public static final int kLB = 5;
            public static final int kRB = 6;
            public static final int kLT = 7;
            public static final int kRT = 8;
            /* kBack = 9; kStart = 10 */
            /* Joystick click: kLeftStick = 11, kRightStick = 12 */
        }    
    }

    
}
