package frc.team1458.robot

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX

class Drivetrain(val leftMaster: WPI_TalonSRX,
                 val rightMaster: WPI_TalonSRX,
                 val leftMotor: WPI_TalonSRX,
                 val rightMotor: WPI_TalonSRX,

                 var closedLoop: Boolean = False,
                 wheelDiameter: Double? = null,

                 ) {


// Features needed TODO TODO TODO
    // - raw
    // - direct control voltage (raw with voltage comp
    // - closed loop velocity
    // - able to disable encoders for open loop voltage
    // - angle / distance compensator for vision
        // - while repeatedly called: shift to low, start angle PID, start distance control, report if aligned (and derivative is sufficiently small)
    // - turn in place
    // - ramseteeeee
}