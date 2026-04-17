import matplotlib.pyplot as plt

from engine.py_kinematics.point3d import Point3D

class Visualizer:
    def __init__(self):
        plt.ion()
        fig = plt.figure()
        self.ax = fig.add_subplot(111, projection='3d')

    def update(self, body):

        floor_z = min((leg.smoothed_abs_pos.z for leg in body.legs), default=0.0) - 5

        self.ax.cla()
        self.ax.set_proj_type('persp', focal_length=0.4)
        self.ax.set_xlim(-500, 500)
        self.ax.set_ylim(-300, 700)
        self.ax.set_zlim(floor_z, floor_z + 1000)
        self.ax.set_xlabel('X Axis')
        self.ax.set_ylabel('Y Axis')
        self.ax.set_zlabel('Z Axis')
        self.scatter_points([leg.abs_pos for leg in body.legs], alpha=0.25)
        self.scatter_points([leg.smoothed_abs_pos for leg in body.legs])

        # Shadow emulation
        self.scatter_points([Point3D(leg.smoothed_abs_pos.x, leg.smoothed_abs_pos.y, floor_z) for leg in body.legs], color='gray')
        plt.pause(0.00001)

    def finish(self):
        plt.ioff()
        plt.close('all')

    def scatter_points(self, points: list[Point3D], color: str = 'red', size: int = 50, alpha: float = 1.0):
        if not points:
            return

        xs = [p.x for p in points]
        ys = [p.y for p in points]
        zs = [p.z for p in points]

        self.ax.scatter(xs, ys, zs, c=color, marker='o', s=size, alpha=alpha)

