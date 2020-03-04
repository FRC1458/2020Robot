package frc.team1458.robot

import frc.team1458.lib.actuator.SRX
import frc.team1458.lib.actuator.Solenoid
import frc.team1458.lib.pathing.PathGenerator
import frc.team1458.lib.pid.PIDConstants
import frc.team1458.lib.sensor.NavX
import frc.team1458.lib.sensor.interfaces.AngleSensor


class RobotMap {

    val colorSense = ColorSensorConnect()

    val leftMaster = SRX(canID = 3, encoderPPR = 18000.0, invert = true, invertEncoder = false)
    val rightMaster = SRX(canID = 2, encoderPPR = 18000.0, invert = false, invertEncoder = false)

    //TODO: canID needed
    val colorMotor  = SRX(canID = -1, invert = false)

    val drivetrain = Drivetrain(
            leftMaster = leftMaster, rightMaster = rightMaster,
            leftMotor  = SRX(canID = 1, master = leftMaster, invert = true),
            rightMotor = SRX(canID = 4, master = rightMaster, invert = false),

            closedLoop = true,
            wheelDiameter = 0.51,
            trackWidth = 2.17,

            // kS is value from characterization tool with / 12.0 added
            // kF is calculated by this formula
            // (kV from char. tool) * ((852.5 * pi * wheel_diameter) / ppr)
            pidConstantsLowGearLeft   = PIDConstants(kP = 0.10, kI = 0.0, kD = 0.0, kF = 0.07117573508, kS = 1.17 / 12.0),
            pidConstantsLowGearRight  = PIDConstants(kP = 0.10, kI = 0.0, kD = 0.0, kF = 0.06980988942, kS = 1.19 / 12.0),

            pidConstantsHighGearLeft  = PIDConstants.DISABLE,
            pidConstantsHighGearRight = PIDConstants.DISABLE,

            shifter = null,
            gyro = NavX.MXP_I2C().yaw,
            invertGyro = true,

            maxVoltage = 11.0
    )

    val lowGearPathGenerator = PathGenerator(
            left_kS = 1.17, right_kS = 1.19,
            left_kV = 0.938, right_kV = 0.920,
            left_kA = 0.172, right_kA = 0.187,

            trackWidth = 2.17,
            maxControlEffortVolts = 10.0,
            maxVelocity = 6.0,
            maxAcceleration = 3.0,
            maxCentripitalAcceleration = 0.5
    )

    /**
     * Track width = 2.17 ft
     *
     * LOW GEAR, PRACTICE CHASSIS
     *
     * Left:
     * kS = 1.17 V
     * kV = 0.938 V / (ft / sec)
     * kA = 0.172 V / (ft / sec^2)
     *
     * Right:
     * kS = 1.19 V
     * kV = 0.92 V / (ft / sec)
     * kA = 0.187 V / (ft / sec^2)
     *
     *
     *
     */
}