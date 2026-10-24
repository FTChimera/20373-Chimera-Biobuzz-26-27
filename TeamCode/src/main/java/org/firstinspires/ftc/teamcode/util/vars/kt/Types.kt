package org.firstinspires.ftc.teamcode.util.vars.kt

import org.firstinspires.ftc.teamcode.util.poses.Pose2d
import org.firstinspires.ftc.teamcode.util.vars.Var


public typealias DoubleVar = Var<Double>
public typealias IntVar = Var<Int>
public typealias StrVar = Var<String>
public typealias PosVar = Var<Pose2d>
public typealias BoolVar = Var<Boolean>

public inline fun DoubleVar(name:String, value:Double): DoubleVar = Var<Double> (name, value)

public inline fun IntVar(name:String, value:Int): IntVar = Var<Int> (name, value)

public inline fun StrVar(name:String, value:String): StrVar = Var<String> (name, value)

public inline fun PosVar(name:String, value: Pose2d): PosVar = Var<Pose2d> (name, value)

public inline fun BoolVar(name:String, value: Boolean): BoolVar = Var<Boolean> (name, value)
