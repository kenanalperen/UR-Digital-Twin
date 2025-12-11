#!/usr/bin/env python3
import sys
import signal
import time
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtWidgets import QApplication, QWidget
from PyQt5.QtGui import QPainter, QColor, QBrush, QCursor

# Set Qt attribute early
import os
os.environ['QT_QPA_PLATFORM'] = 'xcb'

class CursorCircle(QWidget):
    def __init__(self):
        super().__init__()

        self.circle_radius = 40
        self.last_pos = QCursor.pos()
        
        # Window setup
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
        self.setAttribute(Qt.WA_ShowWithoutActivating)
        
        diameter = self.circle_radius * 2
        self.setFixedSize(diameter, diameter)
        
        self.circle_brush = QBrush(QColor(255, 0, 0, 180))
        
        # Timer for updates
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_position)
        self.timer.start(40)  # ~25 FPS
    
    def update_position(self):
        current_pos = QCursor.pos()
        if current_pos != self.last_pos:
            self.last_pos = current_pos
            self.move(current_pos.x() - self.circle_radius, 
                     current_pos.y() - self.circle_radius)
            self.update()
    
    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing, False)
        painter.setBrush(self.circle_brush)
        painter.setPen(Qt.NoPen)
        painter.drawEllipse(0, 0, self.circle_radius * 2, self.circle_radius * 2)
    
    def clean_up(self):
        """Clean up resources"""
        self.timer.stop()
        self.close()

def signal_handler(signum, frame):
    """Handle Ctrl+C signal"""
    print("\n\nCtrl+C pressed - Closing cursor overlay...")
    
    # Find and close all Qt windows
    app = QApplication.instance()
    if app:
        # Close all windows
        for widget in app.topLevelWidgets():
            if hasattr(widget, 'clean_up'):
                widget.clean_up()
            widget.close()
        app.quit()
    
    sys.exit(0)

def main():
    # Set up signal handler for Ctrl+C
    signal.signal(signal.SIGINT, signal_handler)
    
    # Create application
    app = QApplication(sys.argv)
    
    # Create overlay
    overlay = CursorCircle()
    overlay.show()
    
    # Print instructions
    print("=" * 60)
    print("RED CIRCLE CURSOR OVERLAY")
    print("=" * 60)
    print("Press Ctrl+C in THIS terminal to quit")
    print("DO NOT press Ctrl+Z")
    print("=" * 60)
    print("\nThe red circle is now following your cursor...\n")
    
    # Run the application
    try:
        sys.exit(app.exec_())
    except SystemExit:
        pass
    finally:
        # Ensure cleanup
        if 'overlay' in locals():
            overlay.clean_up()

if __name__ == "__main__":
    main()