package org.firstinspires.ftc.teamcode.ALGORITHMS

import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.floor

//a series of useful math functions i use throughout the code
object Math {

    //the mod operation for float/doubles
    fun floatMod(a: Double, b: Double) = a - floor(a / b) * b

    //difference between floats/doubles (in this code, positions) with some tolerance due to how the values are stored in memory
    fun posDiff(pos1: Double, pos2: Double) = abs(pos1-pos2) < 0.0001

    //angle difference calculated but so that it gets the shortest arc between the 2
    fun angDiff(ang1: Double, ang2: Double) = floatMod(ang2 - ang1 + PI, 2 * PI) - PI
}