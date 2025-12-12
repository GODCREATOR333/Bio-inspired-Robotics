import pyqtgraph.opengl as gl
from PyQt5 import QtGui
from PyQt5.QtCore import Qt, QTimer


class MyView(gl.GLViewWidget):
    def __init__(self):
        super().__init__()

        self.zoomFactor = 0.9
        self.currentDist = 300
        self.panStep = 5

        self.default_pos = self.cameraPosition()
        self.default_center = QtGui.QVector3D(
            self.opts['center'].x(),
            self.opts['center'].y(),
            self.opts['center'].z(),
        )

        # Set camera base position
        self.setCameraPosition(distance=self.currentDist,
                               elevation=0, azimuth=0)

        # Set projection, window title, focus
        self.opts['projection'] = 'perspective'
        self.setWindowTitle("Bio-Inspired Robotics")
        self.setFocusPolicy(Qt.StrongFocus)
        self.setFocus()

        # Apply rotation AFTER the GL context is created
        QTimer.singleShot(0, self.apply_initial_rotation)

    def apply_initial_rotation(self):
        # Rotate camera -90 degrees around Z axis
        self.orbit(-90, 0)

    # Camera follow function
    def follow(self, x, y, z=0):
        self.opts['center'] = QtGui.QVector3D(x, y, z)
        self.update()

    # Keybindings
    def keyPressEvent(self, ev):
        key = ev.key()
        c = self.opts['center']

        # --- ZOOM ---
        if key == Qt.Key_I:
            self.currentDist *= self.zoomFactor
            self.setCameraPosition(distance=self.currentDist)
            return

        if key == Qt.Key_O:
            self.currentDist /= self.zoomFactor
            self.setCameraPosition(distance=self.currentDist)
            return

        # --- PAN ---
        if key == Qt.Key_W:
            c.setX(c.x() - self.panStep)
            self.opts['center'] = c
            self.update()
            return

        if key == Qt.Key_S:
            c.setX(c.x() + self.panStep)
            self.opts['center'] = c
            self.update()
            return

        if key == Qt.Key_A:
            c.setY(c.y() - self.panStep)
            self.opts['center'] = c
            self.update()
            return

        if key == Qt.Key_D:
            c.setY(c.y() + self.panStep)
            self.opts['center'] = c
            self.update()
            return

        # --- RESET ---
        if key == Qt.Key_R:
            self.setCameraPosition(pos=self.default_pos)
            self.opts['center'] = QtGui.QVector3D(
                self.default_center.x(),
                self.default_center.y(),
                self.default_center.z()
            )
            self.update()

            # Reset agent position and scale
            self.main.agent.spawn(0, 0, 2.5)
            return

        # --- AGENT MOVEMENT (Arrow Keys) ---
        if key == Qt.Key_Up:
            self.main.agent.move(0, +5)
        elif key == Qt.Key_Down:
            self.main.agent.move(0, -5)
        elif key == Qt.Key_Left:
            self.main.agent.move(-5, 0)
        elif key == Qt.Key_Right:
            self.main.agent.move(+5, 0)
        else:
            super().keyPressEvent(ev)
            return

        px, py, pz = self.main.agent.position
        self.follow(px, py, pz)
