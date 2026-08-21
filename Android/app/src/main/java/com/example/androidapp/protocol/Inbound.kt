package com.example.androidapp.protocol

import com.example.androidapp.arena.Facing

/**
 * Messages arriving from the robot. Pure Kotlin, so the whole parser is covered
 * by JVM unit tests with no device attached.
 *
 * The parser is deliberately forgiving. The written checklist gives the target
 * format as "TARGET, <Obstacle Number>, <Target ID>" while the ARCM briefing
 * slides give it as "TARGET,B2,11" — and a supervisor typing into the AMD tool
 * by hand will produce whichever they remember, possibly with stray spaces.
 * Accepting only one of those loses a demo we had already earned.
 */
sealed class Inbound {

    /** ROBOT,<x>,<y>,<dir> — C.10 */
    data class Robot(val x: Int, val y: Int, val facing: Facing) : Inbound()

    /** TARGET,<n>,<id> or TARGET,<n>,<id>,<dir> — C.9 */
    data class Target(val obstacleId: Int, val targetId: Int, val face: Facing?) : Inbound()

    /** MSG,[text] or STATUS,<text> — C.4, the status box. The map ignores these. */
    data class Message(val text: String) : Inbound()

    /**
     * STM,<reply> — the Pi bridge relaying whatever the STM board said, one of
     * READY, DONE, STALL, TIMEOUT, ACK, BUSY, ERR, or NO_REPLY when the bridge
     * gave up waiting. See rpi/a1_bridge.py.
     */
    data class StmReply(val reply: String) : Inbound()

    /** STATUS,SENT,<command> — the bridge confirming it forwarded a command. */
    data class Forwarded(val command: String) : Inbound()

    /**
     * STATUS,MAP,<message> — the bridge acknowledging one of our own obstacle
     * edits. A receipt for something we sent, so it belongs in the traffic log
     * and not in the status box: C.4 is explicit that the box shows selected
     * information rather than everything on the wire.
     */
    data class MapAck(val message: String) : Inbound()

    /** ERR,<reason> — the bridge refused something we sent. */
    data class Rejected(val reason: String) : Inbound()

    /** Anything we did not recognise. Logged to the debug drawer, never thrown. */
    data class Unknown(val raw: String) : Inbound()
}

/**
 * Tolerances applied to every field:
 *  - surrounding whitespace trimmed
 *  - keyword and direction letter matched case-insensitively
 *  - obstacle numbers accepted as "B2" or "2"
 *  - coordinates accepted as "(10,6)" or "10,6"
 *  - a trailing empty field ignored, so "ROBOT,7,2,W," still parses
 */
fun parseInbound(raw: String): Inbound {
    val line = raw.trim()
    if (line.isEmpty()) return Inbound.Unknown(raw)

    val head = line.substringBefore(',').trim().uppercase()
    val tail = if (line.contains(',')) line.substringAfter(',') else ""

    return when (head) {
        "ROBOT" -> parseRobot(tail) ?: Inbound.Unknown(raw)
        "TARGET" -> parseTarget(tail) ?: Inbound.Unknown(raw)
        "MSG" -> Inbound.Message(unwrapBrackets(tail))
        "STATUS" -> parseStatus(tail)
        "STM" -> tail.trim().uppercase()
            .takeIf { it.isNotEmpty() }
            ?.let { Inbound.StmReply(it) }
            ?: Inbound.Unknown(raw)
        "ERR" -> Inbound.Rejected(tail.trim().ifEmpty { "unspecified" })
        else -> Inbound.Unknown(raw)
    }
}

/**
 * The Pi bridge sends both plain notices ("STATUS,RPi bridge ready") and a
 * forwarding receipt ("STATUS,SENT,FW010") down the same keyword.
 */
private fun parseStatus(tail: String): Inbound {
    val f = fields(tail)
    val kind = f.firstOrNull().orEmpty()
    return when {
        f.size >= 2 && kind.equals("SENT", ignoreCase = true) ->
            Inbound.Forwarded(f[1].uppercase())

        f.size >= 2 && kind.equals("MAP", ignoreCase = true) ->
            Inbound.MapAck(tail.trim().substringAfter(',').trim())

        else -> Inbound.Message(unwrapBrackets(tail))
    }
}

private fun parseRobot(tail: String): Inbound.Robot? {
    val f = fields(tail)
    if (f.size < 3) return null
    val x = f[0].toIntOrNull() ?: return null
    val y = f[1].toIntOrNull() ?: return null
    val facing = Facing.from(f[2]) ?: return null
    return Inbound.Robot(x, y, facing)
}

private fun parseTarget(tail: String): Inbound.Target? {
    val f = fields(tail)
    if (f.size < 2) return null
    val obstacleId = obstacleNumber(f[0]) ?: return null
    val targetId = f[1].toIntOrNull() ?: return null
    val face = if (f.size >= 3) Facing.from(f[2]) else null
    return Inbound.Target(obstacleId, targetId, face)
}

/**
 * Splits on commas, drops the parentheses some formats wrap coordinates in,
 * trims each field and discards empties. "B1,(10,6)" becomes ["B1","10","6"].
 */
internal fun fields(s: String): List<String> =
    s.replace('(', ',')
        .replace(')', ',')
        .split(',')
        .map { it.trim() }
        .filter { it.isNotEmpty() }

/** Accepts "B2", "b2" and "2". */
internal fun obstacleNumber(token: String): Int? {
    val t = token.trim()
    val digits = if (t.length > 1 && (t[0] == 'B' || t[0] == 'b')) t.substring(1) else t
    return digits.toIntOrNull()
}

private fun unwrapBrackets(s: String): String {
    val t = s.trim()
    return if (t.length >= 2 && t.startsWith('[') && t.endsWith(']')) t.substring(1, t.length - 1) else t
}
