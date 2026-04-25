from system_detector import DESKTOP

if DESKTOP:
    import matplotlib.pyplot as plt

from engine.py_kinematics.point3d import Point3D

class Visualizer:
    def __init__(self):
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d')
        
        plt.ion()
        plt.show(block=False)

    def update(self, body):
        if not plt.fignum_exists(self.fig.number):
            return False

        floor_z = min((leg.smoothed_abs_pos.z for leg in body.legs), default=0.0) - 5

        self.ax.cla()
        self.ax.set_proj_type('persp', focal_length=0.4)
        self.ax.set_xlim(-500, 500)
        self.ax.set_ylim(-300, 700)
        self.ax.set_zlim(floor_z, floor_z + 1000)
        self.ax.set_xlabel('X Axis')
        self.ax.set_ylabel('Y Axis')
        self.ax.set_zlabel('Z Axis')

        # The unsmoother leg tip
        self.scatter_points([leg.abs_pos for leg in body.legs], alpha=0.25)

        # Smoothed leg tip
        self.scatter_points([leg.smoothed_abs_pos for leg in body.legs])

        # Shadow emulation
        self.scatter_points([Point3D(leg.smoothed_abs_pos.x, leg.smoothed_abs_pos.y, floor_z) for leg in body.legs], color='gray')

        # Body center/origin
        self.scatter_points([Point3D(0, 0, 0)], color='blue')

        if not body.is_stance_mode():
            # Walk direction arrow
            step_dir = body.gait.get_direction() * 300
            self.ax.quiver(
                0, 0, 0, 
                step_dir.x, step_dir.y, step_dir.z, 
                color='blue', 
                arrow_length_ratio=0.2,  # Size of the arrow head relative to length
                linewidth=1.5
            )

            # Turn rotation center if cilcular walk engaged
            if body.gait.is_circular():
                any_stepper = next(iter(body.gait._steppers.values()))
                self.scatter_points([Point3D(any_stepper.cx, 0, floor_z)], color='green')

        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()

        return True

    def finish(self):
        plt.ioff()
        
        if plt.fignum_exists(self.fig.number):
            plt.close(self.fig)

    def scatter_points(self, points: list[Point3D], color: str = 'red', size: int = 50, alpha: float = 1.0):
        if not points:
            return

        xs = [p.x for p in points]
        ys = [p.y for p in points]
        zs = [p.z for p in points]

        self.ax.scatter(xs, ys, zs, c=color, marker='o', s=size, alpha=alpha)

