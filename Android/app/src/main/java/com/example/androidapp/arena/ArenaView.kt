package com.example.androidapp.arena

import android.content.Context
import android.graphics.Canvas
import android.graphics.Color
import android.graphics.Paint
import android.graphics.Path
import android.graphics.RectF
import android.util.AttributeSet
import android.util.TypedValue
import android.view.MotionEvent
import android.view.View
import android.view.ViewConfiguration
import kotlin.math.atan2
import kotlin.math.hypot
import kotlin.math.min
import kotlin.math.roundToInt

/**
 * The arena canvas. C.5 draws it, C.6 and C.7 make it interactive, C.9 and
 * C.10 are driven by whatever the view model feeds into [state].
 *
 * The view knows how to draw a grid and which cell a finger is over. It does
 * not know what a Bluetooth message is, and it decides nothing — every touch
 * that means something is handed upward through a callback. Keeping it dumb is
 * what lets the whole look change for the video without any behaviour moving.
 */
class ArenaView @JvmOverloads constructor(
    context: Context,
    attrs: AttributeSet? = null,
    defStyle: Int = 0,
) : View(context, attrs, defStyle) {

    // --- callbacks ------------------------------------------------------

    var onAddObstacle: (x: Int, y: Int) -> Unit = { _, _ -> }
    var onMoveObstacle: (id: Int, x: Int, y: Int) -> Unit = { _, _, _ -> }
    var onRemoveObstacle: (id: Int) -> Unit = { }
    var onSetTargetFace: (id: Int, face: Facing) -> Unit = { _, _ -> }

    /** Cell under the finger, or null when it leaves the arena. For the readout chip. */
    var onHover: (x: Int, y: Int) -> Unit = { _, _ -> }
    var onHoverEnd: () -> Unit = { }

    // --- state ----------------------------------------------------------

    var state: ArenaState = ArenaState()
        set(value) {
            field = value
            if (selectorFor != null && value.obstacle(selectorFor!!) == null) selectorFor = null
            invalidate()
        }

    /** Obstacle whose face selector is open (C.7), or null. */
    private var selectorFor: Int? = null

    private var dragId: Int? = null
    private var dragging = false
    private var dragX = 0f
    private var dragY = 0f
    private var downX = 0f
    private var downY = 0f
    private val slop = ViewConfiguration.get(context).scaledTouchSlop

    // --- geometry -------------------------------------------------------

    private var cell = 0f
    private var gridLeft = 0f
    private var gridTop = 0f
    private var gridBottom = 0f
    private var gutter = 0f

    // --- paint ----------------------------------------------------------

    private fun dp(v: Float) = v * resources.displayMetrics.density
    private fun sp(v: Float) =
        TypedValue.applyDimension(TypedValue.COMPLEX_UNIT_SP, v, resources.displayMetrics)

    private val boardPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply { color = COLOR_BOARD }
    private val gridPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = COLOR_GRID; style = Paint.Style.STROKE; strokeWidth = dp(0.6f)
    }
    private val majorPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = COLOR_GRID_MAJOR; style = Paint.Style.STROKE; strokeWidth = dp(1f)
    }
    private val framePaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = COLOR_FRAME; style = Paint.Style.STROKE; strokeWidth = dp(2f)
    }
    private val startPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply { color = COLOR_START }
    private val axisPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = COLOR_AXIS; textAlign = Paint.Align.CENTER
    }
    private val obstaclePaint = Paint(Paint.ANTI_ALIAS_FLAG).apply { color = COLOR_OBSTACLE }
    private val obstacleEdge = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = COLOR_OBSTACLE_EDGE; style = Paint.Style.STROKE; strokeWidth = dp(1f)
    }
    private val facePaint = Paint(Paint.ANTI_ALIAS_FLAG).apply { color = COLOR_TARGET_FACE }
    private val numberPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = Color.WHITE; textAlign = Paint.Align.CENTER; isFakeBoldText = true
    }
    private val robotPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply { color = COLOR_ROBOT }
    private val robotEdge = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = COLOR_ROBOT_EDGE; style = Paint.Style.STROKE; strokeWidth = dp(1.5f)
    }
    private val nosePaint = Paint(Paint.ANTI_ALIAS_FLAG).apply { color = COLOR_ROBOT_NOSE }
    private val trailPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply { color = COLOR_TRAIL }
    private val highlightPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = COLOR_HIGHLIGHT; style = Paint.Style.STROKE; strokeWidth = dp(2f)
    }
    private val dangerPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply { color = COLOR_DANGER }
    private val scrimPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply { color = COLOR_SCRIM }
    private val quadrantPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply { color = COLOR_QUADRANT }
    private val quadrantEdge = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = COLOR_QUADRANT_EDGE; style = Paint.Style.STROKE; strokeWidth = dp(1.5f)
    }
    private val quadrantLabel = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = Color.WHITE; textAlign = Paint.Align.CENTER; isFakeBoldText = true
    }

    private val rect = RectF()
    private val path = Path()

    // -----------------------------------------------------------------
    // Measurement — the arena is always square
    // -----------------------------------------------------------------

    override fun onMeasure(widthMeasureSpec: Int, heightMeasureSpec: Int) {
        val w = MeasureSpec.getSize(widthMeasureSpec)
        val h = MeasureSpec.getSize(heightMeasureSpec)
        val side = min(w, h).coerceAtLeast(1)
        setMeasuredDimension(side, side)
    }

    override fun onSizeChanged(w: Int, h: Int, oldw: Int, oldh: Int) {
        super.onSizeChanged(w, h, oldw, oldh)
        gutter = dp(18f)
        val usable = min(w - gutter, h - gutter)
        cell = usable / Arena.SIZE
        gridLeft = gutter
        gridTop = 0f
        gridBottom = gridTop + cell * Arena.SIZE
        axisPaint.textSize = min(sp(10f), cell * 0.62f)
        quadrantLabel.textSize = dp(15f)
    }

    // -----------------------------------------------------------------
    // Drawing
    // -----------------------------------------------------------------

    override fun onDraw(canvas: Canvas) {
        if (cell <= 0f) return
        drawBoard(canvas)
        drawAxes(canvas)
        drawTrail(canvas)
        drawObstacles(canvas)
        drawRobot(canvas)
        drawDragGhost(canvas)
        drawFaceSelector(canvas)
    }

    private fun drawBoard(canvas: Canvas) {
        val right = gridLeft + cell * Arena.SIZE
        rect.set(gridLeft, gridTop, right, gridBottom)
        canvas.drawRect(rect, boardPaint)

        // Start zone, bottom-left.
        rect.set(
            gridLeft,
            gridBottom - cell * Arena.START_ZONE_SPAN,
            gridLeft + cell * Arena.START_ZONE_SPAN,
            gridBottom,
        )
        canvas.drawRect(rect, startPaint)

        for (i in 0..Arena.SIZE) {
            val paint = if (i % 5 == 0) majorPaint else gridPaint
            val x = gridLeft + i * cell
            canvas.drawLine(x, gridTop, x, gridBottom, paint)
            val y = gridTop + i * cell
            canvas.drawLine(gridLeft, y, right, y, paint)
        }
        rect.set(gridLeft, gridTop, right, gridBottom)
        canvas.drawRect(rect, framePaint)
    }

    private fun drawAxes(canvas: Canvas) {
        // Every label when there is room, otherwise every other one.
        val step = if (cell >= dp(16f)) 1 else 2
        val baseline = gridBottom + axisPaint.textSize + dp(3f)
        for (i in 0 until Arena.SIZE step step) {
            canvas.drawText(i.toString(), cellCentreX(i), baseline, axisPaint)
        }
        axisPaint.textAlign = Paint.Align.RIGHT
        for (i in 0 until Arena.SIZE step step) {
            canvas.drawText(
                i.toString(),
                gridLeft - dp(4f),
                cellCentreY(i) + axisPaint.textSize * 0.36f,
                axisPaint,
            )
        }
        axisPaint.textAlign = Paint.Align.CENTER
    }

    private fun drawTrail(canvas: Canvas) {
        val r = cell * 0.16f
        state.trail.forEach { (x, y) ->
            canvas.drawCircle(cellCentreX(x), cellCentreY(y), r, trailPaint)
        }
    }

    private fun drawObstacles(canvas: Canvas) {
        state.obstacles.forEach { o ->
            if (dragging && o.id == dragId) return@forEach
            drawObstacle(canvas, o, cellLeft(o.x), cellTop(o.y), 1f)
        }
    }

    private fun drawObstacle(canvas: Canvas, o: Obstacle, left: Float, top: Float, scale: Float) {
        val size = cell * scale
        val inset = size * 0.06f
        rect.set(left + inset, top + inset, left + size - inset, top + size - inset)
        val radius = size * 0.16f
        canvas.drawRoundRect(rect, radius, radius, obstaclePaint)
        canvas.drawRoundRect(rect, radius, radius, obstacleEdge)

        // C.7 / C.9 — thick line on the face holding the target image.
        o.targetFace?.let { face ->
            val t = size * 0.20f
            val bar = when (face) {
                Facing.N -> RectF(rect.left, rect.top, rect.right, rect.top + t)
                Facing.S -> RectF(rect.left, rect.bottom - t, rect.right, rect.bottom)
                Facing.E -> RectF(rect.right - t, rect.top, rect.right, rect.bottom)
                Facing.W -> RectF(rect.left, rect.top, rect.left + t, rect.bottom)
            }
            canvas.drawRect(bar, facePaint)
        }

        val label = o.targetId?.toString() ?: o.id.toString()
        // C.5 wants the obstacle number small; C.9 wants the target ID large.
        numberPaint.textSize = if (o.targetId != null) size * 0.62f else size * 0.42f
        canvas.drawText(
            label,
            rect.centerX(),
            rect.centerY() + numberPaint.textSize * 0.36f,
            numberPaint,
        )
    }

    private fun drawRobot(canvas: Canvas) {
        val r = state.robot
        val span = cell * Arena.ROBOT_SPAN
        val half = Arena.ROBOT_SPAN / 2
        val left = cellLeft(r.x - half)
        val top = cellTop(r.y + half)
        val inset = span * 0.07f
        rect.set(left + inset, top + inset, left + span - inset, top + span - inset)
        val radius = span * 0.14f
        canvas.drawRoundRect(rect, radius, radius, robotPaint)
        canvas.drawRoundRect(rect, radius, radius, robotEdge)

        // A nose that leaves no doubt which way it faces.
        val cx = rect.centerX()
        val cy = rect.centerY()
        val reach = span * 0.34f
        val wing = span * 0.24f
        path.reset()
        when (r.facing) {
            Facing.N -> { path.moveTo(cx, cy - reach); path.lineTo(cx - wing, cy + wing * 0.5f); path.lineTo(cx + wing, cy + wing * 0.5f) }
            Facing.S -> { path.moveTo(cx, cy + reach); path.lineTo(cx - wing, cy - wing * 0.5f); path.lineTo(cx + wing, cy - wing * 0.5f) }
            Facing.E -> { path.moveTo(cx + reach, cy); path.lineTo(cx - wing * 0.5f, cy - wing); path.lineTo(cx - wing * 0.5f, cy + wing) }
            Facing.W -> { path.moveTo(cx - reach, cy); path.lineTo(cx + wing * 0.5f, cy - wing); path.lineTo(cx + wing * 0.5f, cy + wing) }
        }
        path.close()
        canvas.drawPath(path, nosePaint)
    }

    private fun drawDragGhost(canvas: Canvas) {
        val id = dragId ?: return
        if (!dragging) return
        val o = state.obstacle(id) ?: return
        val outside = !inArena(dragX, dragY)

        if (outside) {
            rect.set(dragX - cell * 0.6f, dragY - cell * 0.6f, dragX + cell * 0.6f, dragY + cell * 0.6f)
            canvas.drawRoundRect(rect, cell * 0.2f, cell * 0.2f, dangerPaint)
            numberPaint.textSize = cell * 0.42f
            canvas.drawText("×", rect.centerX(), rect.centerY() + numberPaint.textSize * 0.36f, numberPaint)
            return
        }

        val gx = cellXAt(dragX)
        val gy = cellYAt(dragY)
        if (gx in 0 until Arena.SIZE && gy in 0 until Arena.SIZE) {
            rect.set(cellLeft(gx), cellTop(gy), cellLeft(gx) + cell, cellTop(gy) + cell)
            canvas.drawRect(rect, highlightPaint)
        }
        drawObstacle(canvas, o, dragX - cell * 0.6f, dragY - cell * 0.6f, 1.2f)
    }

    /**
     * C.7. An obstacle is one cell out of twenty — about a fingertip wide in
     * total — so its four edges cannot be hit reliably. The checklist allows
     * another touch-based method, and this is it: tap the block and a
     * magnified compass opens over it.
     */
    private fun drawFaceSelector(canvas: Canvas) {
        val id = selectorFor ?: return
        val o = state.obstacle(id) ?: return

        canvas.drawRect(gridLeft, gridTop, gridLeft + cell * Arena.SIZE, gridBottom, scrimPaint)

        val cx = selectorCentreX(o)
        val cy = selectorCentreY(o)
        val outer = selectorRadius()
        val inner = outer * 0.34f

        Facing.entries.forEach { face ->
            path.reset()
            val sweepStart = when (face) {
                Facing.N -> 225f
                Facing.E -> 315f
                Facing.S -> 45f
                Facing.W -> 135f
            }
            rect.set(cx - outer, cy - outer, cx + outer, cy + outer)
            path.arcTo(rect, sweepStart, 90f, true)
            rect.set(cx - inner, cy - inner, cx + inner, cy + inner)
            path.arcTo(rect, sweepStart + 90f, -90f, false)
            path.close()
            canvas.drawPath(path, if (o.targetFace == face) facePaint else quadrantPaint)
            canvas.drawPath(path, quadrantEdge)

            val mid = outer * 0.68f
            val lx = when (face) { Facing.E -> cx + mid; Facing.W -> cx - mid; else -> cx }
            val ly = when (face) { Facing.N -> cy - mid; Facing.S -> cy + mid; else -> cy }
            canvas.drawText(face.name, lx, ly + quadrantLabel.textSize * 0.35f, quadrantLabel)
        }

        // The obstacle itself sits in the hub so you can see what you are annotating.
        drawObstacle(canvas, o, cx - inner * 0.7f, cy - inner * 0.7f, (inner * 1.4f) / cell)
    }

    // -----------------------------------------------------------------
    // Touch
    // -----------------------------------------------------------------

    override fun onTouchEvent(event: MotionEvent): Boolean {
        if (cell <= 0f) return false
        val ex = event.x
        val ey = event.y

        when (event.actionMasked) {

            MotionEvent.ACTION_DOWN -> {
                parent?.requestDisallowInterceptTouchEvent(true)
                downX = ex; downY = ey; dragX = ex; dragY = ey
                dragging = false

                if (selectorFor != null) return true // resolved on UP

                val gx = cellXAt(ex); val gy = cellYAt(ey)
                dragId = if (inArena(ex, ey)) state.obstacleAt(gx, gy)?.id else null
                if (inArena(ex, ey)) onHover(gx, gy)
                return true
            }

            MotionEvent.ACTION_MOVE -> {
                dragX = ex; dragY = ey
                if (selectorFor == null) {
                    if (!dragging && dragId != null &&
                        hypot(ex - downX, ey - downY) > slop
                    ) dragging = true
                    if (inArena(ex, ey)) onHover(cellXAt(ex), cellYAt(ey)) else onHoverEnd()
                }
                invalidate()
                return true
            }

            MotionEvent.ACTION_UP -> {
                onHoverEnd()
                val open = selectorFor
                if (open != null) {
                    resolveSelector(open, ex, ey)
                    selectorFor = null
                    dragId = null; dragging = false
                    invalidate()
                    return true
                }

                val id = dragId
                if (dragging && id != null) {
                    if (inArena(ex, ey)) {
                        onMoveObstacle(id, cellXAt(ex), cellYAt(ey))
                    } else {
                        onRemoveObstacle(id)
                    }
                } else if (!dragging) {
                    if (inArena(ex, ey)) {
                        val gx = cellXAt(ex); val gy = cellYAt(ey)
                        val hit = state.obstacleAt(gx, gy)
                        if (hit != null) {
                            selectorFor = hit.id
                            performClick()
                        } else {
                            onAddObstacle(gx, gy)
                        }
                    }
                }
                dragId = null; dragging = false
                invalidate()
                return true
            }

            MotionEvent.ACTION_CANCEL -> {
                onHoverEnd()
                dragId = null; dragging = false; selectorFor = null
                invalidate()
                return true
            }
        }
        return super.onTouchEvent(event)
    }

    override fun performClick(): Boolean {
        super.performClick()
        return true
    }

    private fun resolveSelector(id: Int, ex: Float, ey: Float) {
        val o = state.obstacle(id) ?: return
        val cx = selectorCentreX(o)
        val cy = selectorCentreY(o)
        val outer = selectorRadius()
        val inner = outer * 0.34f
        val d = hypot(ex - cx, ey - cy)
        if (d < inner || d > outer) return // hub or outside: dismiss without changing anything

        // Screen y grows downward, so negate it to get compass bearings.
        val degrees = Math.toDegrees(atan2((cy - ey).toDouble(), (ex - cx).toDouble()))
        val face = when {
            degrees >= 45 && degrees < 135 -> Facing.N
            degrees >= -45 && degrees < 45 -> Facing.E
            degrees >= -135 && degrees < -45 -> Facing.S
            else -> Facing.W
        }
        onSetTargetFace(id, face)
    }

    // -----------------------------------------------------------------
    // Geometry helpers
    // -----------------------------------------------------------------

    private fun cellLeft(x: Int) = gridLeft + x * cell
    private fun cellTop(y: Int) = gridBottom - (y + 1) * cell
    private fun cellCentreX(x: Int) = cellLeft(x) + cell / 2f
    private fun cellCentreY(y: Int) = cellTop(y) + cell / 2f

    private fun cellXAt(px: Float): Int = ((px - gridLeft) / cell).toInt().let {
        if (px < gridLeft) -1 else it
    }

    private fun cellYAt(py: Float): Int = ((gridBottom - py) / cell).toInt().let {
        if (py > gridBottom) -1 else it
    }

    private fun inArena(px: Float, py: Float): Boolean =
        px >= gridLeft && px < gridLeft + cell * Arena.SIZE && py >= gridTop && py < gridBottom

    private fun selectorRadius(): Float = maxOf(cell * 2.6f, dp(58f))

    /** Keeps the compass fully on screen even for an obstacle against an edge. */
    private fun selectorCentreX(o: Obstacle): Float {
        val r = selectorRadius()
        return cellCentreX(o.x).coerceIn(gridLeft + r, gridLeft + cell * Arena.SIZE - r)
    }

    private fun selectorCentreY(o: Obstacle): Float {
        val r = selectorRadius()
        return cellCentreY(o.y).coerceIn(gridTop + r, gridBottom - r)
    }

    /** Physical distance across the arena in centimetres, for the readout. */
    fun cellToCm(v: Int): Int = (v * Arena.CELL_CM.toFloat()).roundToInt()

    private companion object {
        const val COLOR_BOARD = 0xFF0E1317.toInt()
        const val COLOR_GRID = 0xFF1E2830.toInt()
        const val COLOR_GRID_MAJOR = 0xFF2C3B45.toInt()
        const val COLOR_FRAME = 0xFF44555F.toInt()
        const val COLOR_START = 0x24F0A82E
        const val COLOR_AXIS = 0xFF7B8A93.toInt()
        const val COLOR_OBSTACLE = 0xFF232B33.toInt()
        const val COLOR_OBSTACLE_EDGE = 0xFF4A5A66.toInt()
        const val COLOR_TARGET_FACE = 0xFFFF5A3C.toInt()
        const val COLOR_ROBOT = 0xFF1D6F94.toInt()
        const val COLOR_ROBOT_EDGE = 0xFF3FA8DE.toInt()
        const val COLOR_ROBOT_NOSE = 0xFF9BE0FF.toInt()
        const val COLOR_TRAIL = 0x552FA8DE
        const val COLOR_HIGHLIGHT = 0xFFF0A82E.toInt()
        const val COLOR_DANGER = 0xCCB0402A.toInt()
        const val COLOR_SCRIM = 0xB00A0E12.toInt()
        const val COLOR_QUADRANT = 0xF01F2A33.toInt()
        const val COLOR_QUADRANT_EDGE = 0xFF4A5A66.toInt()
    }
}
