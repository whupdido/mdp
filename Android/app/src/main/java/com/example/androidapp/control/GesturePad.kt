package com.example.androidapp.control

import android.content.Context
import android.graphics.Canvas
import android.graphics.Color
import android.graphics.Paint
import android.graphics.Path
import android.graphics.RectF
import android.util.AttributeSet
import android.util.TypedValue
import android.view.HapticFeedbackConstants
import android.view.MotionEvent
import android.view.View
import com.example.androidapp.protocol.Move
import kotlin.math.atan2
import kotlin.math.hypot
import kotlin.math.min

/**
 * Drag-from-centre drive control for C.3.
 *
 * The checklist accepts labelled buttons as the minimum and says gestures score
 * better. This is the gesture: pull away from the centre and the sector under
 * your thumb lights up; release to fire it. Releasing back in the middle, or
 * outside the ring, cancels — so an accidental touch costs nothing, which
 * matters when a stray command during a timed run is expensive.
 *
 * Six sectors, laid out the way the car actually behaves. There is no plain
 * "left" or "right" because this chassis cannot turn on the spot: every turn
 * also carries it forwards or backwards.
 *
 * A single tap in the middle is STOP — the one control you want reachable
 * without looking.
 */
class GesturePad @JvmOverloads constructor(
    context: Context,
    attrs: AttributeSet? = null,
    defStyle: Int = 0,
) : View(context, attrs, defStyle) {

    /** Fired on release, with the sector the thumb ended in. */
    var onMove: (Move) -> Unit = { }

    private var touchX = 0f
    private var touchY = 0f
    private var pressed = false
    private var active: Move? = null

    private fun dp(v: Float) = v * resources.displayMetrics.density
    private fun sp(v: Float) =
        TypedValue.applyDimension(TypedValue.COMPLEX_UNIT_SP, v, resources.displayMetrics)

    private val ringPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = RING; style = Paint.Style.STROKE; strokeWidth = dp(1.2f)
    }
    private val sectorPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply { color = SECTOR }
    private val liveSectorPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply { color = SECTOR_LIVE }
    private val hubPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply { color = HUB }
    private val hubEdge = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = RING; style = Paint.Style.STROKE; strokeWidth = dp(1.2f)
    }
    private val hubLive = Paint(Paint.ANTI_ALIAS_FLAG).apply { color = STOP_LIVE }
    private val labelPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = LABEL; textAlign = Paint.Align.CENTER; isFakeBoldText = true
    }
    private val hubLabel = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = LABEL; textAlign = Paint.Align.CENTER; isFakeBoldText = true
    }
    private val thumbPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply { color = THUMB }

    private val rect = RectF()
    private val path = Path()

    /** Sector centre bearings, clockwise from straight up. */
    private val sectors = listOf(
        Move.FORWARD to 0f,
        Move.FWD_RIGHT to 60f,
        Move.BACK_RIGHT to 120f,
        Move.BACKWARD to 180f,
        Move.BACK_LEFT to 240f,
        Move.FWD_LEFT to 300f,
    )

    private var cx = 0f
    private var cy = 0f
    private var outer = 0f
    private var inner = 0f

    override fun onSizeChanged(w: Int, h: Int, oldw: Int, oldh: Int) {
        super.onSizeChanged(w, h, oldw, oldh)
        cx = w / 2f
        cy = h / 2f
        outer = min(w, h) / 2f - dp(2f)
        inner = outer * 0.36f
        labelPaint.textSize = min(sp(11f), outer * 0.19f)
        hubLabel.textSize = min(sp(13f), inner * 0.44f)
    }

    override fun onDraw(canvas: Canvas) {
        if (outer <= 0f) return

        sectors.forEach { (move, bearing) ->
            val live = pressed && active == move
            path.reset()
            // Canvas sweeps measure from 3 o'clock; bearings from 12.
            val start = bearing - 30f - 90f
            rect.set(cx - outer, cy - outer, cx + outer, cy + outer)
            path.arcTo(rect, start + 2f, 56f, true)
            rect.set(cx - inner, cy - inner, cx + inner, cy + inner)
            path.arcTo(rect, start + 58f, -56f, false)
            path.close()
            canvas.drawPath(path, if (live) liveSectorPaint else sectorPaint)
            canvas.drawPath(path, ringPaint)

            val mid = (inner + outer) / 2f
            val rad = Math.toRadians(bearing.toDouble() - 90.0)
            val lx = cx + (Math.cos(rad) * mid).toFloat()
            val ly = cy + (Math.sin(rad) * mid).toFloat()
            labelPaint.color = if (live) Color.WHITE else LABEL
            canvas.drawText(labelFor(move), lx, ly + labelPaint.textSize * 0.35f, labelPaint)
        }

        val stopLive = pressed && active == null
        canvas.drawCircle(cx, cy, inner, if (stopLive) hubLive else hubPaint)
        canvas.drawCircle(cx, cy, inner, hubEdge)
        hubLabel.color = if (stopLive) Color.WHITE else LABEL
        canvas.drawText("STOP", cx, cy + hubLabel.textSize * 0.35f, hubLabel)

        if (pressed) {
            val d = hypot(touchX - cx, touchY - cy).coerceAtMost(outer)
            val a = atan2(touchY - cy, touchX - cx)
            canvas.drawCircle(
                cx + (Math.cos(a.toDouble()) * d).toFloat(),
                cy + (Math.sin(a.toDouble()) * d).toFloat(),
                dp(11f), thumbPaint,
            )
        }
    }

    private fun labelFor(move: Move) = when (move) {
        Move.FORWARD -> "FWD"
        Move.BACKWARD -> "BACK"
        Move.FWD_LEFT -> "F-L"
        Move.FWD_RIGHT -> "F-R"
        Move.BACK_LEFT -> "B-L"
        Move.BACK_RIGHT -> "B-R"
        Move.STOP -> "STOP"
    }

    /** Null means the hub (STOP) or outside the ring entirely. */
    private fun sectorAt(x: Float, y: Float): Move? {
        val d = hypot(x - cx, y - cy)
        if (d < inner || d > outer) return null
        var bearing = Math.toDegrees(atan2((x - cx).toDouble(), (cy - y).toDouble())).toFloat()
        if (bearing < 0) bearing += 360f
        return sectors.minByOrNull { (_, centre) ->
            val diff = Math.abs(((bearing - centre + 540f) % 360f) - 180f)
            180f - diff
        }?.first
    }

    override fun onTouchEvent(event: MotionEvent): Boolean {
        when (event.actionMasked) {
            MotionEvent.ACTION_DOWN -> {
                parent?.requestDisallowInterceptTouchEvent(true)
                pressed = true
                touchX = event.x; touchY = event.y
                active = sectorAt(event.x, event.y)
                invalidate()
                return true
            }
            MotionEvent.ACTION_MOVE -> {
                touchX = event.x; touchY = event.y
                val next = sectorAt(event.x, event.y)
                if (next != active) {
                    active = next
                    // A tick as the thumb crosses into a new sector, so the
                    // control can be used without looking at it.
                    performHapticFeedback(HapticFeedbackConstants.CLOCK_TICK)
                }
                invalidate()
                return true
            }
            MotionEvent.ACTION_UP -> {
                val d = hypot(event.x - cx, event.y - cy)
                val chosen = active
                pressed = false
                active = null
                invalidate()
                when {
                    chosen != null -> { performClick(); onMove(chosen) }
                    d < inner -> { performClick(); onMove(Move.STOP) }
                    else -> Unit // released outside the ring: deliberate cancel
                }
                return true
            }
            MotionEvent.ACTION_CANCEL -> {
                pressed = false; active = null; invalidate(); return true
            }
        }
        return super.onTouchEvent(event)
    }

    override fun performClick(): Boolean {
        super.performClick()
        return true
    }

    private companion object {
        const val RING = 0xFF24303A.toInt()
        const val SECTOR = 0xFF1B242D.toInt()
        const val SECTOR_LIVE = 0xFFF0A82E.toInt()
        const val HUB = 0xFF1B242D.toInt()
        const val STOP_LIVE = 0xFFE0785A.toInt()
        const val LABEL = 0xFF8A9AA6.toInt()
        const val THUMB = 0x66F0A82E
    }
}
