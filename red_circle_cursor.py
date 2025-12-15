#!/usr/bin/env python3
import sys
import signal
import os
import time
from collections import deque
import math

from PyQt5.QtCore import Qt, QTimer, QRect, QPoint
from PyQt5.QtWidgets import QApplication, QWidget
from PyQt5.QtGui import QPainter, QColor, QBrush, QCursor, QPen

os.environ['QT_QPA_PLATFORM'] = 'xcb'


# ============================================================
#   STATIC BLUE RECTANGLE OVERLAY
# ============================================================
class StaticRectangleOverlay(QWidget):
    def __init__(self):
        super().__init__()

        self.rect_x1, self.rect_x2 = 815, 3150
        self.rect_y1, self.rect_y2 = 980, 1960

        self.setWindowFlags(
            Qt.FramelessWindowHint |
            Qt.WindowStaysOnTopHint |
            Qt.Tool |
            Qt.X11BypassWindowManagerHint |
            Qt.WindowDoesNotAcceptFocus |
            Qt.WindowTransparentForInput
        )

        self.setAttribute(Qt.WA_TranslucentBackground)
        self.setAttribute(Qt.WA_TransparentForMouseEvents)
        self.setGeometry(
            self.rect_x1,
            self.rect_y1,
            self.rect_x2 - self.rect_x1,
            self.rect_y2 - self.rect_y1
        )

        self.pen = QPen(QColor(0, 0, 255, 200), 2)

    def paintEvent(self, event):
        p = QPainter(self)
        p.setPen(self.pen)
        p.setBrush(Qt.NoBrush)
        p.drawRect(0, 0, self.width(), self.height())


# ============================================================
#   CURSOR + TRAILS + GREEN RECT + SINE WAVES
# ============================================================
class SmoothCursorOverlay(QWidget):
    def __init__(self):
        super().__init__()

        self.circle_radius = 40
        self.green_trace_radius = 50
        self.yellow_trace_radius = 50

        self.fade_duration = 0.30
        self.max_alpha = 30

        self.traces = deque(maxlen=80)
        self.green_traces = deque()
        self.yellow_traces = deque()

        self.last_pos = QCursor.pos()

        self.border_x1, self.border_x2 = 685, 3300
        self.border_y1, self.border_y2 = 315, 2060

        margin = 0
        self.window_x = self.border_x1 - margin
        self.window_y = self.border_y1 - margin

        self.setGeometry(
            self.window_x,
            self.window_y,
            (self.border_x2 - self.border_x1) + 2 * margin,
            (self.border_y2 - self.border_y1) + 2 * margin
        )

        self.setWindowFlags(
            Qt.FramelessWindowHint |
            Qt.WindowStaysOnTopHint |
            Qt.Tool |
            Qt.X11BypassWindowManagerHint |
            Qt.WindowDoesNotAcceptFocus |
            Qt.WindowTransparentForInput
        )

        self.setAttribute(Qt.WA_TranslucentBackground)
        self.setAttribute(Qt.WA_TransparentForMouseEvents)

        self.circle_brush = QBrush(QColor(255, 0, 0, 180))
        self.last_window_pos = self.mapFromGlobal(self.last_pos)

        # ---------- GREEN RECT ----------
        self.green_rect_visible = False

        bx1, bx2 = 815, 3150
        by1, by2 = 980, 1960

        self.green_rect_w = (bx2 - bx1) / 2
        self.green_rect_h = (by2 - by1) / 2

        cx = (bx1 + bx2) / 2 - self.window_x
        cy = (by1 + by2) / 2 - self.window_y

        self.green_rect_x = int(cx - self.green_rect_w / 2)
        self.green_rect_y = int(cy - self.green_rect_h / 2)

        # ---------- SINE WAVES ----------
        self.sine_wave_visible = False
        self.sine_shift = 60
        self.sine_steps = 200

        self.timer = QTimer()
        self.timer.timeout.connect(self.update_cursor)
        self.timer.start(33)

    # ----------------------------------------------------------
    def sine_y_at_x(self, x, shift):
        t = (x - self.green_rect_x) / self.green_rect_w
        angle = 2 * math.pi * 2 * t
        base_y = self.green_rect_y + self.green_rect_h / 2
        return base_y - math.sin(angle) * (self.green_rect_h / 2) + shift

    # ----------------------------------------------------------
    def update_cursor(self):
        pos = self.mapFromGlobal(QCursor.pos())
        now = time.time()

        if pos != self.last_window_pos:
            self.traces.append((pos.x(), pos.y(), now))

            # ---- GREEN RECT TRACE ----
            if self.green_rect_visible:
                green_rect = QRect(
                    int(self.green_rect_x),
                    int(self.green_rect_y),
                    int(self.green_rect_w),
                    int(self.green_rect_h)
                )
                if green_rect.contains(pos):
                    self.green_traces.append(QPoint(pos.x(), pos.y()))

            # ---- SINE REGION TRACE ----
            if self.sine_wave_visible:
                x = pos.x()
                y = pos.y()

                if self.green_rect_x <= x <= self.green_rect_x + self.green_rect_w:
                    y_upper = self.sine_y_at_x(x, -self.sine_shift)
                    y_lower = self.sine_y_at_x(x, +self.sine_shift)

                    if y_upper <= y <= y_lower:
                        self.yellow_traces.append(QPoint(x, y))

            self.last_window_pos = pos

        cutoff = now - self.fade_duration
        while self.traces and self.traces[0][2] < cutoff:
            self.traces.popleft()

        self.update()

    # ----------------------------------------------------------
    def paintEvent(self, event):
        p = QPainter(self)
        p.setCompositionMode(QPainter.CompositionMode_Clear)
        p.fillRect(event.rect(), Qt.transparent)
        p.setCompositionMode(QPainter.CompositionMode_SourceOver)

        now = time.time()

        # --- red fading trail ---
        for x, y, t in self.traces:
            age = now - t
            if age > self.fade_duration:
                continue
            k = age / self.fade_duration
            r = int(self.circle_radius * (1 - k))
            a = int(self.max_alpha * (1 - k))
            if r > 0 and a > 0:
                p.setBrush(QColor(255, 0, 0, a))
                p.setPen(Qt.NoPen)
                p.drawEllipse(QRect(x - r, y - r, r * 2, r * 2))

        # --- green permanent trace ---
        if self.green_rect_visible:
            p.setBrush(QColor(0, 255, 0, 35))
            p.setPen(Qt.NoPen)
            r = self.green_trace_radius
            for pt in self.green_traces:
                p.drawEllipse(pt.x() - r, pt.y() - r, r * 2, r * 2)

        # --- yellow sine-region trace ---
        if self.sine_wave_visible:
            p.setBrush(QColor(255, 255, 0, 35))
            p.setPen(Qt.NoPen)
            r = self.yellow_trace_radius
            for pt in self.yellow_traces:
                p.drawEllipse(pt.x() - r, pt.y() - r, r * 2, r * 2)

        # --- cursor ---
        r = self.circle_radius
        p.setBrush(self.circle_brush)
        p.setPen(Qt.NoPen)
        p.drawEllipse(
            self.last_window_pos.x() - r,
            self.last_window_pos.y() - r,
            r * 2, r * 2
        )

        # --- green rectangle ---
        if self.green_rect_visible:
            p.setPen(QPen(QColor(0, 255, 0, 200), 2))
            p.setBrush(Qt.NoBrush)
            p.drawRect(
                int(self.green_rect_x),
                int(self.green_rect_y),
                int(self.green_rect_w),
                int(self.green_rect_h)
            )

        # --- sine waves ---
        if self.sine_wave_visible:
            base_y = self.green_rect_y + self.green_rect_h / 2
            for shift in (-self.sine_shift, self.sine_shift):
                prev = None
                for i in range(self.sine_steps + 1):
                    t = i / self.sine_steps
                    x = self.green_rect_x + t * self.green_rect_w
                    y = base_y - math.sin(2 * math.pi * 2 * t) * (self.green_rect_h / 2) + shift
                    pt = QPoint(int(x), int(y))
                    if prev:
                        p.setPen(QPen(QColor(255, 255, 0, 200), 2))
                        p.drawLine(prev, pt)
                    prev = pt

    def clear_green_traces(self):
        self.green_traces.clear()

    def clear_yellow_traces(self):
        self.yellow_traces.clear()


# ============================================================
#   KEY LISTENER
# ============================================================
class KeyListener(QWidget):
    def __init__(self, overlay):
        super().__init__()
        self.overlay = overlay
        self.setWindowFlags(Qt.FramelessWindowHint)
        self.setGeometry(0, 0, 1, 1)
        self.show()
        self.setFocus()

    def keyPressEvent(self, e):
        if e.key() == Qt.Key_R:
            self.overlay.green_rect_visible = not self.overlay.green_rect_visible
            if not self.overlay.green_rect_visible:
                self.overlay.clear_green_traces()
            self.overlay.update()

        elif e.key() == Qt.Key_S:
            self.overlay.sine_wave_visible = not self.overlay.sine_wave_visible
            if not self.overlay.sine_wave_visible:
                self.overlay.clear_yellow_traces()
            self.overlay.update()


# ============================================================
#   MAIN
# ============================================================
def main():
    signal.signal(signal.SIGINT, lambda *a: sys.exit(0))

    app = QApplication(sys.argv)

    blue = StaticRectangleOverlay()
    blue.show()

    cursor = SmoothCursorOverlay()
    cursor.show()

    keys = KeyListener(cursor)

    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
