package frc.team1458.robot

import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.motorcontrol.FeedbackDevice
import com.ctre.phoenix.motorcontrol.NeutralMode
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX
import edu.wpi.first.wpilibj.TimedRobot
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import frc.team1458.lib.util.flow.delay

class Robot : TimedRobot() {

    private val oi: OI = OI()
    //private val robot = RobotMap() // TODO Might cause issues if not in robotInit
    //private var drivetrainReversed = false

    val leftMaster = WPI_TalonSRX(2);
    val rightMaster = WPI_TalonSRX(3);
    val leftSlave = WPI_TalonSRX(4);
    val rightSlave = WPI_TalonSRX(1);

    override fun robotInit() {
        println("Robot Initialized")

        leftMaster.setInverted(true);
        leftMaster.setSensorPhase(false);
        leftMaster.setNeutralMode(NeutralMode.Brake);

        rightMaster.setInverted(true);
        rightMaster.setSensorPhase(true);
        rightMaster.setNeutralMode(NeutralMode.Brake);

        leftSlave.setInverted(true);
        leftSlave.follow(leftMaster);
        leftSlave.setNeutralMode(NeutralMode.Brake);

        rightSlave.setInverted(true);
        rightSlave.follow(rightMaster);
        rightSlave.setNeutralMode(NeutralMode.Brake);


        leftMaster.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);
        rightMaster.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);

        leftMaster.setSelectedSensorPosition(0);
        rightMaster.setSelectedSensorPosition(0);

        //leftMaster.sensor


        //leftMaster.config_kP(0, 1024 * 0.00103);
        //leftMaster.config_kF(0, 1024 * 0.924);

        //rightMaster.config_kP(0, 1024 * 0.000967);
        //rightMaster.config_kF(0, 1024 * 0.962);
    }

    override fun autonomousInit() {
    }

    override fun autonomousPeriodic() {

    }

    override fun teleopInit() {
        // do nothing maladors
    }

    override fun teleopPeriodic() {

        val left = 1.0 * (oi.throttle.value - oi.steer.value) * 0.924;
        val right = 1.0 * (oi.throttle.value + oi.steer.value) * 0.962;

        leftMaster.set(ControlMode.PercentOutput, left);
        rightMaster.set(ControlMode.PercentOutput, right);

        SmartDashboard.putNumber("leftmalador", left)
        SmartDashboard.putNumber("rightmalador", right)

        //SmartDashboard.putNumber("leftError", leftMaster.getClosedLoopError().toDouble())
        //SmartDashboard.putNumber("rightError", rightMaster.getClosedLoopError().toDouble())
    }

    override fun testInit() {

    }

    override fun testPeriodic() {
        // hi maladors
    }

    override fun disabledInit() {

    }

    override fun disabledPeriodic() {

    }
}

