package com.omni.vision

object Tuning {
    const val BAUD = 460800
    const val FORWARD_RPM = 40f       // VX dasar saat menuju target
    const val KP = 150f               // VY = KP * galat (-1..1)
    const val MAX_STRAFE_RPM = 80f
    const val DEADBAND = 0.05f        // galat dianggap nol di bawah ini
    const val MIN_AREA = 0.005f       // blob lebih kecil dari ini (rasio layar) diabaikan
    const val STOP_AREA = 0.25f       // area tiba bawaan bila kelas belum dikalibrasi ("Set area tiba")
    const val SEND_PERIOD_MS = 50L    // 20 Hz (failsafe firmware 500 ms)
    const val PROC_WIDTH = 320        // lebar frame untuk diproses

    // watchdog & keepalive HP (FR6)
    const val CMD_STALE_MS = 300L     // perintah otonom lebih tua dari ini -> kirim nol
    const val KEEPALIVE_MS = 500L     // MODE/TARGET/CAPS diulang; menjaga telemetri ringkas firmware tetap mengalir (jendela 2 s)
    const val MANUAL_MAX_RPM = 200f   // kecepatan D-pad manual (tekan-tahan)

    // halangan lapis 2 (FR4)
    const val OBS_SLOW = 0.12f        // skor perubahan zona depan -> SLOW
    const val OBS_STOP = 0.30f        // -> STOP
    const val OBS_GROWTH = 0.08f      // kenaikan skor antar-frame yang dianggap "mendekat"
    const val SLOW_SCALE = 0.4f       // pengali kecepatan saat SLOW

    // training (FR10)
    const val SAMPLE_TOL = 30         // toleransi sampling S/V (H = tol/4); diturunkan dari 50 (mask banjir, uji 16 Nov)
    const val SPAN_H = 40             // cakupan kumulatif satu kelas di atas ini -> peringatan "mask banjir" (jumlah nilai H dari 180)
    const val SPAN_S = 170            // idem S (dari 256)
    const val SPAN_V = 170            // idem V (dari 256)
    const val MATCH_MS = 180_000L     // pertandingan 3 menit (§4.2.2)

    // FR11: landmark bernama dari coretan
    const val LM_MIN_POS = 2          // wajib disampling dari >= 2 posisi kamera robot
    const val LM_MATCH = 0.30f        // irisan / luas bbox terkecil agar deteksi dianggap menempati region tersimpan (dituning saat uji M2)
    const val LM_MOVE_CM = 5f         // posisi robot dianggap beda bila bergeser >= ini ...
    const val LM_MOVE_DEG = 10f       // ... atau berputar >= ini

    // FR12: navigasi waypoint (odometri encoder+IMU dari telemetri)
    const val WP_TOL_CM = 10f         // tiba bila galat posisi <= ini dan galat yaw <= WP_TOL_DEG (bawaan PRD, dikalibrasi lewat profil)
    const val WP_TOL_DEG = 10f
    const val WP_KP = 0.6f            // RPM per cm galat jarak
    const val WP_MAX_RPM = 60f
    const val WP_MIN_RPM = 12f        // di bawah ini roda tidak bergerak
    const val WP_KW = 0.8f            // RPM putar per derajat galat yaw
    const val WP_MAX_W = 30f
    const val WP_US_KP = 0.5f         // RPM geser per cm selisih jarak dinding terhadap referensi
    const val WP_US_MAX = 15f
    const val WP_US_RADIUS = 100f     // koreksi dinding aktif hanya dalam jarak ini dari waypoint (referensi diambil di waypoint)
    const val WP_US_SANITY = 40f      // selisih dinding > ini dianggap dinding berubah, abaikan
    const val ANCHOR_OVERLAP = 0.60f  // landmark waypoint menempati region tersimpan >= ini -> re-anchor + konfirmasi tiba
    const val ANCHOR_RADIUS_CM = 40f  // ... hanya bila odometri bilang masih dalam jarak ini (anti-salah-kenal)
}
