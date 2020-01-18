package frc.team1458.lib.util

import edu.wpi.first.networktables.NetworkTableEntry
import edu.wpi.first.networktables.NetworkTableInstance

// Interface with FalconDashboard
object LiveDashboard {
    var _robotX : NetworkTableEntry? = null
    var _robotY : NetworkTableEntry? = null
    var _robotHeading : NetworkTableEntry? = null

    var _followingPath : NetworkTableEntry? = null
    var _pathX : NetworkTableEntry? = null
    var _pathY : NetworkTableEntry? = null
    var _pathHeading : NetworkTableEntry? = null

    var visionOffset: NetworkTableEntry? = null
    var visionAngle: NetworkTableEntry? = null

    var _offsetX: Double = 0.0
    var _offsetY: Double = 0.0

    fun setup(offsetX: Double = 0.0, offsetY: Double = 0.0) {
        val table = NetworkTableInstance.getDefault().getTable("Live_Dashboard")
        _robotX = table.getEntry("robotX")
        _robotY = table.getEntry("robotY")
        _robotHeading = table.getEntry("robotHeading")

        _followingPath = table.getEntry("isFollowingPath")
        _pathX = table.getEntry("pathX")
        _pathY = table.getEntry("pathY")
        _pathHeading = table.getEntry("pathHeading")

        _offsetX = offsetX
        _offsetY = offsetY

        endPath()
        putOdom(0.0, 0.0, 0.0)
    }

    fun putOdom(robotX: Double, robotY: Double, robotHeading: Double) {
        // IF THIS THROWS A NULLPOINTER YOU DIDN'T CALL SETUP()
        _robotX!!.setDouble(robotX + _offsetX)
        _robotY!!.setDouble(robotY + _offsetY)
        _robotHeading!!.setDouble(robotHeading)
    }

    fun putPath(pathX: Double, pathY: Double, pathHeading: Double) {
        // IF THIS THROWS A NULLPOINTER YOU DIDN'T CALL SETUP()
        _pathX!!.setDouble(pathX + _offsetX)
        _pathY!!.setDouble(pathY + _offsetY)
        _pathHeading!!.setDouble(pathHeading)
        _followingPath!!.setBoolean(true)
    }

    fun endPath() {
        _followingPath!!.setBoolean(false)
    }
}