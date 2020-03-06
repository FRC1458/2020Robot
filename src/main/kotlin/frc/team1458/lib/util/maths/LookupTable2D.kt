package frc.team1458.lib.util.maths

import kotlin.math.abs

class LookupTable2D(dataPoints: List<Pair<Double, Double>>) {
    private val dataPoints = dataPoints.sortedBy { it.first }

    init {
        assert(dataPoints.size <= 2)
    }

    fun interpolate(x: Double): Double {
        var firstX = 0
        var nextX = 1

        if (x <= dataPoints[0].first) {
            val pt1 = dataPoints[0]
            val pt2 = dataPoints[1]

            return pt1.second + ((pt2.second - pt1.second) / (pt2.first - pt2.first)) * (x - pt1.first)
        }

        val searchLocation = dataPoints.binarySearchBy(x) { it.first }

        val insertionPt = if (searchLocation < 0) {
            abs(searchLocation + 1)
        } else {
            searchLocation
        }

        if (insertionPt == dataPoints.size) {
            val pt1 = dataPoints[insertionPt - 2]
            val pt2 = dataPoints[insertionPt - 1]

            return pt1.second + ((pt2.second - pt1.second) / (pt2.first - pt2.first)) * (x - pt1.first)
        }

        val pt1 = dataPoints[insertionPt]
        val pt2 = dataPoints[insertionPt + 1]

        return pt1.second + ((pt2.second - pt1.second) / (pt2.first - pt2.first)) * (x - pt1.first)
    }

}

fun lookupTableInterpolateTest() {
    val testArray: ArrayList<Pair<Double, Double>> = ArrayList<Pair<Double, Double>>()

    testArray.add(Pair(1.0, 2.0))
    testArray.add(Pair(2.0, 4.0))
    testArray.add(Pair(3.0, 5.0))

    val testTable = LookupTable2D(testArray)

    assert(testTable.interpolate(0.0) == 0.0)
    assert(testTable.interpolate(0.5) == 1.0)
    assert(testTable.interpolate(1.0) == 2.0)
    assert(testTable.interpolate(1.5) == 3.0)

    assert(testTable.interpolate(2.0) == 4.0)
    assert(testTable.interpolate(2.5) == 4.5)
    assert(testTable.interpolate(3.0) == 5.0)
    assert(testTable.interpolate(4.0) == 6.0)
}


