package frc.team1458.lib.sensor

import edu.wpi.first.wpilibj.PowerDistributionPanel

object PDP {
    val pdp = PowerDistributionPanel()

    val voltage : Double
        get() = pdp.voltage

    val temperature : Double
        get() = pdp.temperature

    val totalCurrent : Double
        get() = pdp.totalCurrent

    val totalPower : Double
        get() = pdp.totalPower

    val totalEnergy : Double
        get() = pdp.totalEnergy

    fun resetTotalEnergy() {
        pdp.resetTotalEnergy()
    }

    fun clearStickyFaults() {
        pdp.clearStickyFaults()
    }

    fun getCurrent(channel: Int) : Double {
        return pdp.getCurrent(channel)
    }
}