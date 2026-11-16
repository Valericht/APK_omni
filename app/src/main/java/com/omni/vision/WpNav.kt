package com.omni.vision

import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.sin

/** Koreksi pose hasil re-anchor landmark (HP tidak bisa menulis pose firmware, jadi disimpan sebagai ofset). */
class PoseCorr { @Volatile var dx = 0f; @Volatile var dy = 0f; @Volatile var dyaw = 0f
    fun reset() { dx = 0f; dy = 0f; dyaw = 0f } }

/**
 * FR12: navigasi waypoint dead reckoning (pose encoder+IMU dari telemetri) + fusi.
 * Konvensi firmware: dunia = badan diputar yaw (CCW +); badan +x kanan, +y depan. Protokol HP: VX = maju (+y badan), VY = geser kanan (+x badan), W + = CCW.
 * Robot omni: galat jarak diuraikan ke komponen maju (VX) dan geser (VY), W dari galat yaw. Murni logika, tanpa OpenCV.
 */
object WpNav {
    class Out(val cmd: Cmd, val arrived: Boolean, val note: String, val distCm: Float, val dyawDeg: Float, val anchored: Boolean)

    fun wrapDeg(a: Float): Float { var d = a % 360f; if (d > 180f) d -= 360f; if (d < -180f) d += 360f; return d }

    /** pose = (x cm, y cm, yaw deg) dari telemetri; us = [depan, kiri, kanan] cm (999 = tak terbaca). */
    fun step(wp: Waypoint, pose: FloatArray, us: IntArray, dets: List<Det>, corr: PoseCorr, p: Profile): Out {
        var cx = pose[0] + corr.dx; var cy = pose[1] + corr.dy; var cyaw = wrapDeg(pose[2] + corr.dyaw)
        var ex = wp.x - cx; var ey = wp.y - cy
        var dist = hypot(ex, ey)

        // fusi 1: landmark waypoint menempati region tersimpan -> re-anchor pose + konfirmasi tiba
        var anchored = false
        val box = wp.box; val cls = wp.lmClass
        if (box != null && cls != null && dist <= Tuning.ANCHOR_RADIUS_CM) {
            val best = dets.filter { it.base == cls }.maxOfOrNull { LmMatch.overlapBox(it, box) } ?: 0f
            if (best >= Tuning.ANCHOR_OVERLAP) {
                corr.dx = wp.x - pose[0]; corr.dy = wp.y - pose[1]; corr.dyaw = wrapDeg(wp.yaw - pose[2])
                anchored = true
                cx = wp.x; cy = wp.y; cyaw = wp.yaw; ex = 0f; ey = 0f; dist = 0f
            }
        }
        val dyaw = wrapDeg(wp.yaw - cyaw)
        val arrived = anchored || (dist <= p.wpTolCm && abs(dyaw) <= p.wpTolDeg)
        if (arrived) return Out(Cmd.ZERO, true, if (anchored) "tiba (landmark)" else "tiba", dist, dyaw, anchored)

        // uraikan galat dunia -> badan
        val yr = Math.toRadians(cyaw.toDouble())
        val right = (ex * cos(yr) + ey * sin(yr)).toFloat()
        val fwd = (-ex * sin(yr) + ey * cos(yr)).toFloat()
        var vx = 0f; var vy = 0f
        if (dist > p.wpTolCm) {
            val sp = (Tuning.WP_KP * dist).coerceIn(Tuning.WP_MIN_RPM, Tuning.WP_MAX_RPM)
            vx = sp * fwd / dist; vy = sp * right / dist
        }
        // fusi 2: ultrasonik kiri/kanan menjaga jarak dinding referensi di dekat waypoint
        var note = ""
        if (dist <= Tuning.WP_US_RADIUS) {
            var sum = 0f; var n = 0
            if (wp.usL >= 0 && us.size >= 3 && us[1] in 1..300) { val d = us[1] - wp.usL; if (abs(d) <= Tuning.WP_US_SANITY) { sum += -d; n++ } }
            if (wp.usR >= 0 && us.size >= 3 && us[2] in 1..300) { val d = us[2] - wp.usR; if (abs(d) <= Tuning.WP_US_SANITY) { sum += d; n++ } }
            if (n > 0) { vy += (Tuning.WP_US_KP * sum / n).coerceIn(-Tuning.WP_US_MAX, Tuning.WP_US_MAX); note = "koreksi dinding" }
        }
        val w = if (abs(dyaw) < 1.5f) 0f else (Tuning.WP_KW * dyaw).coerceIn(-Tuning.WP_MAX_W, Tuning.WP_MAX_W)
        return Out(Cmd(vx, vy, w), false, note, dist, dyaw, false)
    }
}
