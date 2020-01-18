package frc.team1458.lib.pid

/**
 * Class to hold PID constants
 *
 * @param kP Proportional gain
 * @param kI Integral gain
 * @param kD Derivative gain
 * @param kF Feedforward gain
 * @param kS Static friction feedforward
 */
data class PIDConstants(val kP: Double, val kI: Double = 0.0, val kD: Double = 0.0, val kF: Double = 0.0, val kS: Double = 0.0) {

    val openLoop: PIDConstants
        get() = PIDConstants(0.0, 0.0, 0.0, kF, kS)

    companion object {
        val DISABLE = PIDConstants(0.0, 0.0, 0.0, 0.0, 0.0)
    }
}