package org.firstinspires.ftc.teamcode.AUTO

import com.acmerobotics.dashboard.config.Config
import org.firstinspires.ftc.teamcode.ALGORITHMS.Pose
import org.firstinspires.ftc.teamcode.ALGORITHMS.Vec2D
import org.firstinspires.ftc.teamcode.ALGORITHMS.Vec4D
import kotlin.math.PI

@Config
object SpecimenVars {
    @JvmField
    var score_preload =
        Pose(0.0, -80.0, 0.0, Vec2D(50.0, 55.0), 25.0, Vec4D(20.0, 0.4, 200.0, 200.0))

    @JvmField
    var sleep_preload = 0.00

    @JvmField
    var sleep_score = 0.3


    @JvmField
    var wait_theydontloveyoulikeiloveyou =
        Pose(0.0, -64.0, 2.2, Vec2D(0.0, 0.0), 0.0, Vec4D(50.0, 0.5, 200.0, 200.0))

    @JvmField
    var s_8 = Pose(-73.0, -40.0, PI, Vec2D(0.0, 0.0), 0.0, Vec4D(200.0, 0.4, 200.0, 200.0))

    @JvmField
    var s_s = Pose(-73.0, -40.0, PI, Vec2D(0.0, 0.0), 0.0, Vec4D(80.0, 20.0, 200.0, 200.0))

    @JvmField
    var s_9 = Pose(-73.0, -12.7, PI, Vec2D(10.0, 15.0), 10.0)

    @JvmField
    var wait_move = 0.00

    @JvmField
    var from_preload_to_samples = Pose(-50.0, -60.0, 2.2, Vec2D(20.0, 21.0), 20.0)

    @JvmField
    var second_sample = Pose(-76.0, -61.0, 2.2, Vec2D(14.0, 15.0), 0.0)

    @JvmField
    var wrist_one = 0.1

    @JvmField
    var wrist_two = 0.1

    @JvmField
    var wrist_three = 0.9

    @JvmField
    var spinny_baby = Pose(-50.0, -60.0, 0.5, Vec2D(0.0, 0.0), 0.0, Vec4D(10.0, 0.3, 200.0, 200.0))

    @JvmField
    var spinny_baby2 = Pose(-76.0, -61.0, 0.5, Vec2D(0.0, 0.0), 0.0, Vec4D(10.0, 0.3, 200.0, 200.0))

    @JvmField
    var wait_take = 0.15

    @JvmField
    var the_third_children = Pose(-85.0, -95.0, Math.toRadians(90.0), Vec2D(8.0, 10.0), 7.0)

    @JvmField
    var spinny_baby3 = Pose(-76.0, -61.0, 0.5, Vec2D(0.0, 0.0), 0.0, Vec4D(10.0, 0.3, 200.0, 200.0))

    @JvmField
    var matei_clumsy = Pose(-20.0, -55.0, 0.7, 1.0)

    @JvmField
    var alexia_offset = Pose(-15.0, -65.0, 0.7, 1.0)

    @JvmField
    var matei_sleepy = 0.2

    @JvmField
    var score_offset = Pose(-1.0, 1.0, 0.0, 0.6)

    @JvmField
    var sleepy_extend_from_preload = 0.2

    @JvmField
    var sleepy_extend_third_impact = 0.6

    @JvmField
    var alexia_wait = 0.2

    @JvmField
    var score_with_rotation =
        Pose(20.0, -66.0, 0.0, Vec2D(10.0, 15.0), 15.0, Vec4D(50.0, 0.2, 200.0, 200.0))

    @JvmField
    var rotate = Pose(-70.0, -40.0, PI, Vec2D(0.0, 0.0), 0.0, Vec4D(34.0, 1.1, 200.0, 200.0))

    @JvmField
    var testp = Pose(-80.0, 60.0, PI, 1.0)

    @JvmField
    var sp1 = Pose(-70.0, -55.0, 2.7, Vec2D(20.0, 21.0), 20.0)

    @JvmField
    var sp2 = Pose(-70.0, -55.0, 2.2, Vec2D(20.0, 21.0), 20.0)

    @JvmField
    var sp_r = Pose(-70.0, -55.0, 2.2, Vec2D(20.0, 21.0), 20.0)

}



@Config
object sample_vars {
    @JvmField
    var dunk = Pose(-17.0, -44.0, Math.toRadians(45.0), Vec2D(14.0, 15.0), 10.0, Vec4D(10.0, 0.1, 200.0, 200.0)).setName("dunk")

    @JvmField
    var sample_1 = Pose(-25.0, -34.0, 1.62, Vec2D(10.0, 10.0), 10.0).setName("sample1")

    @JvmField
    var sample_2 = Pose(-42.0, -60.0, 1.6, Vec2D(17.0, 17.0), 13.0).setName("sample2")

    @JvmField
    var sample2_examination: Int = -540

    @JvmField
    var sample_three = Pose(-92.0, -11.5, Math.toRadians(180.0), Vec2D(20.0, 20.0), 13.0).setName("sample3")

    @JvmField
    var wait_takeS = 0.27

    @JvmField
    var waitaminute = 0.7

    @JvmField
    var park2 = Pose(-140.0, -20.0, 3.3, Vec2D(), 0.0, Vec4D(20.0, 2.0, 200.0, 200.0)).setName("park2")

    @JvmField
    var park3 = Pose(-140.0, 35.0, 3.3, Vec2D(), 10.0).setName("park3")

    @JvmField
    var dunk2 = Pose(-26.0, -45.0, Math.toRadians(45.0), Vec2D(15.0, 30.0), 15.0).setName("dun2")

    @JvmField
    var rotate1 = Pose(
        -27.0,
        -25.7,
        Math.toRadians(45.0),
        Vec2D(15.0, 30.0),
        15.0,
        Vec4D(20.0, 0.2, 200.0, 200.0)
    ).setName("rotatate1")

    @JvmField
    var rotate1v2 = Pose(
        -27.0,
        -25.7,
        Math.toRadians(45.0),
        Vec2D(15.0, 30.0),
        15.0,
        Vec4D(10.0, 0.2, 200.0, 200.0)
    ).setName("rotate1v2")

    @JvmField
    var rotate2 = Pose(
        -27.0,
        -59.2,
        Math.toRadians(45.0),
        Vec2D(15.0, 30.0),
        15.0,
        Vec4D(20.0, 0.2, 200.0, 200.0)
    ).setName("rotate2")

    @JvmField
    var rotate3 = Pose(
        -27.0,
        -59.7,
        Math.toRadians(90.0),
        Vec2D(0.0, 0.0),
        0.0,
        Vec4D(50.0, 0.3, 200.0, 200.0),
    ).setName("rotate3")

    @JvmField
    var rotatemid = Pose(-25.0, -44.0, 1.62, Vec2D(14.0, 15.0), 10.0, Vec4D(5.0, 0.1, 200.0, 200.0)).setName("rotateMid")

    @JvmField
    var sleepExtendoThird = 0.3
}
