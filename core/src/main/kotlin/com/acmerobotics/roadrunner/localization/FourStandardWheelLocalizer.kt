package com.acmerobotics.roadrunner.localization

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.kinematics.Kinematics
import org.apache.commons.math3.linear.Array2DRowRealMatrix
import org.apache.commons.math3.linear.DecompositionSolver
import org.apache.commons.math3.linear.LUDecomposition
import org.apache.commons.math3.linear.MatrixUtils

/**
 * Localizer based on three unpowered tracking omni wheels.
 *
 * @param wheelPoses wheel poses relative to the center of the robot (positive X points forward on the robot)
 */
abstract class FourStandardWheelLocalizer(
    wheelPoses: List<Pose2d>
) : Localizer {
    private var _poseEstimate = Pose2d()
    override var poseEstimate: Pose2d
        get() = _poseEstimate
        set(value) {
            lastWheelPositions = emptyList()
            _poseEstimate = value
        }
    private var lastWheelPositions = emptyList<Double>()

    init {
        require(wheelPoses.size == 4) { "4 wheel positions must be provided" }
    }

    override fun update() {
        val wheelPositions = getWheelPositions()
        if (lastWheelPositions.isNotEmpty()) {
            val wheelDeltas = wheelPositions
                    .zip(lastWheelPositions)
                    .map { it.first - it.second }
            val l = wheelDeltas[0]
            val r = wheelDeltas[1]
            val bl = wheelDeltas[2]
            val br = wheelDeltas[3]
            val xvel = (l + r + bl + br) * (0.0375 / 4) * 39.37
            val yvel = (-l + r + bl - br) * (0.0375 / 4) * 39.37
            val tvel = (-l + r - bl + br) * (0.0375 / (4 * 0.3429)) * 39.37
            val robotPoseDelta = Pose2d(xvel, yvel, tvel)

            _poseEstimate = Kinematics.relativeOdometryUpdate(_poseEstimate, robotPoseDelta)
        }
        lastWheelPositions = wheelPositions
    }

    /**
     * Returns the positions of the tracking wheels in the desired distance units (not encoder counts!)
     */
    abstract fun getWheelPositions(): List<Double>
}
