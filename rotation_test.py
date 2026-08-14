# rotation_test.py - fast spin (omega0 = 10) under gravity + viscous friction torque
#   gravity torque:  engine's gravitation_torque
#   friction torque: viscous, proportional to angular speed (strong while fast,
#                    weak near rest, so gravity visibly accelerates the swing down)
import tkinter as tk
import math
from Physics_Engine import RotationalMechanics
from vectogebra import vector

m, k, I_com = 1.0, 2.0, 0.0
c = 0.5                             # viscous friction coefficient (smaller: less damping)
AXIS = vector(0, 0, 0)
I = I_com + m * k * k               # 4.0, whole moment of inertia is m*k^2
omega0 = 10.0                       # initial angular speed

body = RotationalMechanics(mass=m, position=AXIS + vector(0, -k, 0),  # hanging at bottom
                           moment_of_inertia_com=I_com,
                           friction_co=0.0, restitution_co=0.0,
                           gravity=vector(0, -9.81, 0))
body.axis_of_rotation = AXIS
body.angle = -math.pi / 2
body.angular_momentum = vector(0, 0, I * omega0)   # spin it up

dt = 0.001
SUBSTEPS = 20
t = 0.0
last_report = 0.0

W, H = 600, 600
SCALE = 90
CX, CY = W // 2, H // 2

def to_screen(p):
    return CX + p.i * SCALE, CY - p.j * SCALE

root = tk.Tk()
root.title("Fast spin under gravity + viscous friction")
cv = tk.Canvas(root, width=W, height=H, bg="white")
cv.pack()
cv.create_oval(CX - 4, CY - 4, CX + 4, CY + 4, fill="red")   # axis

def step():
    global t, last_report
    for _ in range(SUBSTEPS):
        omega = body.angular_momentum.k / I

        body.gravitational_force()                  # stores force_of_gravity
        body.gravitation_torque()                   # gravity torque (engine)
        body.add_torque(vector(0, 0, -c * omega))   # viscous friction torque

        body.update(dt)
        # constraint: body rides the circle at the engine's angle
        body.position = AXIS + vector(k * math.cos(body.angle), k * math.sin(body.angle), 0)
        t += dt

    if t - last_report >= 5.0:
        last_report = t
        print(f"t = {t:5.1f}s   omega = {body.angular_momentum.k / I:8.2f} rad/s")

    x, y = to_screen(body.position)
    cv.delete("arm", "bob")
    cv.create_line(CX, CY, x, y, fill="gray", tags="arm")
    cv.create_oval(x - 7, y - 7, x + 7, y + 7, fill="blue", tags="bob")
    root.after(10, step)

step()
root.mainloop()
