package frc.team1458.lib.util.flow

import kotlin.math.min

fun periodic(hz: Int = 20, condition: () -> Boolean = { true }, body: () -> Unit) {
    val period : Double = (1000 / hz).toDouble()
    while (condition()) {
        val start = systemTimeMillis
        body()
        val time = systemTimeMillis - start

        if (time > period) {
            println("Periodic Looper: Periodic loop went over expected time. " +
                    "Function execution took $time ms but loop time is only $period ms.")
        }
/**/
        delay(period - min(time, period))
    }
}

suspend fun suspendUntil(pollingRate: Int = 10, condition: suspend () -> Boolean) {
    // TODO See if coroutine is working properly : (..., TimeUnit.MILLISECONDS)
    while (!condition()) delay(pollingRate.toLong())
}
