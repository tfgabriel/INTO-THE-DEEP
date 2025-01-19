package org.firstinspires.ftc.teamcode.AUTO

import com.acmerobotics.dashboard.config.Config
import org.firstinspires.ftc.teamcode.ALGORITHMS.Pose
import org.firstinspires.ftc.teamcode.ALGORITHMS.Vec2D
import org.firstinspires.ftc.teamcode.ALGORITHMS.Vec4D
import kotlin.math.PI

@Config
object SpecimenVars {

    @JvmField
    var Ascore_preload =
        Pose(0.0, -85.0, 0.0, Vec2D(50.0, 55.0), 25.0, Vec4D(20.0, 0.4, 200.0, 200.0))


    @JvmField
    var Bwait_theydontloveyoulikeiloveyou =
        Pose(0.0, -50.0, 2.2, Vec2D(0.0, 0.0), 0.0, Vec4D(50.0, 0.5, 200.0, 200.0))

    @JvmField
    var s_8 = Pose(-70.0, -35.0, PI, Vec2D(0.0, 0.0), 0.0, Vec4D(30.0, 0.2, 200.0, 200.0))
    @JvmField
    var oppositionclaims = Pose(-70.0, -35.0, 0.0, Vec2D(0.0, 0.0), 0.0, Vec4D(30.0, 0.8, 200.0, 200.0)).setHeading(true)
    @JvmField
    var oppositionclaimsRetard = Pose(0.0, -35.0, 0.0, Vec2D(0.0, 0.0), 0.0, Vec4D(30.0, 0.8, 200.0, 200.0))
    @JvmField
    var score_gen = Pose(0.0, -83.0, 0.0, Vec2D(50.0, 55.0), 40.0, Vec4D(20.0, 0.4, 200.0, 200.0)).setff(0.025)
    @JvmField
    var rotate = Pose(0.0, -83.0, PI, Vec2D(0.0, 0.0), 0.0, Vec4D(34.0, 1.1, 200.0, 200.0)).setHeading(true)


    @JvmField
    var first_spin = Pose(-72.0, -52.0, 2.15, Vec2D(), 5.0, Vec4D(5.0, 0.1, 200.0, 200.0)).setHeading(true)
    @JvmField
    var second_spin = Pose(-72.0, -52.0, 2.35, Vec2D(), 5.0, Vec4D(5.0, 0.1, 200.0, 200.0)).setHeading(true)
    @JvmField
    var drop_speci = -530
    @JvmField
    var wait_move = 0.00

    @JvmField
    var wrist_one = 0.1

    @JvmField
    var wrist_two = 0.06

    @JvmField
    var wrist_three = 0.9

    @JvmField
    var spinny_baby = Pose(-72.0, -58.0, 0.7, Vec2D(0.0, 0.0), 00.0, Vec4D(6.0, 0.2, 200.0, 200.0)).setHeading(true)
    @JvmField
    var wait_take = 0.35

    @JvmField
    var the_third_children = Pose(-85.0, -93.0, Math.toRadians(90.0), Vec2D(20.0, 20.0), 15.0).setff(0.025)

    @JvmField
    var spinny_baby3 = Pose(-74.0, -58.0, 0.5, Vec2D(0.0, 0.0), 0.0, Vec4D(10.0, 0.3, 200.0, 200.0))

    @JvmField
    var sleepy_extend_from_preload = 0.2

    @JvmField
    var sleepy_extend_third_impact = 0.3

    @JvmField
    var testp = Pose(0.0, -40.0, 0.0, 1.0)
    @JvmField
    var bomboclaat = Pose(-74.0, -58.0, 2.36, Vec2D(0.0, 0.0), 4.0, Vec4D(4.0, 0.15, 200.0, 200.0)).setHeading(true)

    @JvmField
   var bomboclaat_0 = Pose(-74.0, -58.0, 2.56, Vec2D(50.0, 50.0), 10.0, Vec4D(4.0, 0.1, 200.0, 200.0))

    @JvmField
    var extension_0 = -500
    @JvmField
    var steal_withff = Pose(-75.0, -18.0, PI, Vec2D(20.0, 20.0), 35.0, Vec4D(3.5, 0.2, 200.0, 200.0)).setff(0.025)

    @JvmField
    var wo1 = Pose()
    @JvmField
    var wo2 = Pose()
    @JvmField
    var wo3 = Pose()
    @JvmField
    var co1 = Pose()
    @JvmField
    var co2 = Pose()
    @JvmField
    var co3 = Pose()

    @JvmField
    var sleep_steal = 0.1
}



@Config
object sample_vars {
    @JvmField
    var dunk = Pose(-17.0, -48.0, Math.toRadians(45.0), Vec2D(14.0, 15.0), 10.0, Vec4D(10.0, 0.1, 200.0, 200.0)).setName("dunk")

    @JvmField
    var sample_1 = Pose(-20.5, -31.0, 1.62 , Vec2D(20.0, 20.0), 9.0).setName("sample1")

    @JvmField
    var sample_2 = Pose(-37.0, -51.0, 1.6, Vec2D(17.0, 17.0), 10.0).setName("sample2")

    @JvmField
    var sample2_examination: Int = -540

    @JvmField
    var sample_three = Pose(-85.0, -13.0, Math.toRadians(180.0), Vec2D(20.0, 20.0), 13.0).setName("sample3").setff(0.025)

    @JvmField
    var wait_takeS = 0.27
    @JvmField
    var waitaminute = 0.7

    @JvmField
    var park2 = Pose(-145.0, -20.0, 3.3, Vec2D(), 0.0, Vec4D(90.0, 2.0, 200.0, 200.0)).setName("park2")

    @JvmField
    var park3 = Pose(-145.0, 35.0, 3.3, Vec2D(), 10.0).setName("park3")

    @JvmField
    var dunk2 = Pose(-22.0, -46.0, Math.toRadians(45.0), Vec2D(40.0, 40.0), 35.0, Vec4D(10.0, 0.1,200.0, 200.0)).setName("dun2")

    @JvmField
    var rotate1 = Pose(
        -24.0,
        -25.7,
        Math.toRadians(45.0),
        Vec2D(15.0, 30.0),
        15.0,
        Vec4D(20.0, 0.2, 200.0, 200.0)
    ).setName("rotatate1")

    @JvmField
    var rotate1v2 = Pose(
        -25.0,
        -25.7,
        Math.toRadians(45.0),
        Vec2D(15.0, 30.0),
        15.0,
        Vec4D(10.0, 0.2, 200.0, 200.0)
    ).setName("rotate1v2")

    @JvmField
    var rotate2 = Pose(
        -25.0,
        -59.2,
        Math.toRadians(45.0),
        Vec2D(15.0, 30.0),
        15.0,
        Vec4D(20.0, 0.2, 200.0, 200.0)
    ).setName("rotate2")

    @JvmField
    var rotate3 = Pose(
        -25.0,
        -59.7,
        Math.toRadians(90.0),
        Vec2D(0.0, 0.0),
        0.0,
        Vec4D(50.0, 0.3, 200.0, 200.0),
    ).setName("rotate3")


    @JvmField

    var dunkmid = Pose(-18.0, -48.0, Math.toRadians(45.0), Vec2D(14.0, 15.0), 10.0, Vec4D(10.0, 0.1, 200.0, 200.0)).setName("dunk")

    @JvmField
    var rotatemid = Pose(-22.0, -44.0, 1.62, Vec2D(14.0, 15.0), 10.0, Vec4D(5.0, 0.1, 200.0, 200.0)).setName("rotateMid")

    @JvmField
    var sleepExtendoThird = 0.0
}
