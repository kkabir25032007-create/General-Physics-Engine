# particle_collision_test.py - many balls, small variation in mass/radius,
# elastic collisions (e=1), no gravity/friction/electric fields.
# Verification: pair collisions conserve momentum and energy, no NaN,
#               bounded overlap, no explosion.
import tkinter as tk
import math
import random
from Physics_Engine import Dynamics
from vectogebra import vector

random.seed(42)
W, H = 8.0, 5.5          # box, physics units
N = 25
RESTITUTION = 1.0        # elastic
RADIUS = 0.25            # all balls identical, small
MASS = 1.0

def make_balls():
    balls = []
    cols, rows = 10, 5
    for idx in range(N):
        gx, gy = idx % cols, idx // cols
        x = 0.4 + gx * (W - 0.8) / (cols - 1) + random.uniform(-0.12, 0.12)
        y = 0.55 + gy * (H - 1.1) / (rows - 1) + random.uniform(-0.12, 0.12)
        b = Dynamics(mass=MASS, position=vector(x, y, 0),
                     friction_co=0.0, restitution_co=RESTITUTION, gravity=vector(0, 0, 0))
        b.radius = RADIUS
        b.momentum = vector(random.uniform(-3, 3), random.uniform(-3, 3), 0)
        balls.append(b)
    return balls

def walls(balls):
    for b in balls:
        if b.position.j <= b.radius and b.velocity.j < 0:
            b.collision(vector(0, 1, 0)); b.position = vector(b.position.i, b.radius, 0)
        if b.position.j >= H - b.radius and b.velocity.j > 0:
            b.collision(vector(0, -1, 0)); b.position = vector(b.position.i, H - b.radius, 0)
        if b.position.i <= b.radius and b.velocity.i < 0:
            b.collision(vector(1, 0, 0)); b.position = vector(b.radius, b.position.j, 0)
        if b.position.i >= W - b.radius and b.velocity.i > 0:
            b.collision(vector(-1, 0, 0)); b.position = vector(W - b.radius, b.position.j, 0)

def pairs(balls):
    for i in range(len(balls)):
        for j in range(i + 1, len(balls)):
            balls[i].particle_colllision(balls[j], vector(1, 0, 0))

def substep(balls, dt):
    for b in balls:
        b.update(dt)
    walls(balls)
    pairs(balls)

def verify():
    balls = make_balls()
    dt = 0.002
    t = 0.0
    max_overlap = 0.0
    nan = False
    p_pair_drift = 0.0
    E_pair_drift = 0.0
    KE0 = sum(0.5 * (b.momentum * b.momentum) / b.mass for b in balls)
    while t < 2.0:      # shorter run with 50 balls, 1225 pairs per step
        for b in balls:
            b.update(dt)
        walls(balls)
        p_before = sum((b.momentum for b in balls), vector(0, 0, 0))
        KE_before = sum(0.5 * (b.momentum * b.momentum) / b.mass for b in balls)
        pairs(balls)
        p_after = sum((b.momentum for b in balls), vector(0, 0, 0))
        KE_after = sum(0.5 * (b.momentum * b.momentum) / b.mass for b in balls)
        p_pair_drift = max(p_pair_drift, abs((p_after - p_before).i) + abs((p_after - p_before).j))
        E_pair_drift = max(E_pair_drift, abs(KE_after - KE_before))
        for i in range(len(balls)):
            for j in range(i + 1, len(balls)):
                over = balls[i].radius + balls[j].radius - abs(balls[i].position - balls[j].position)
                max_overlap = max(max_overlap, over)
            b = balls[i]
            if any(math.isnan(c) for c in (b.position.i, b.position.j, b.momentum.i, b.momentum.j)):
                nan = True
        t += dt
    KE_end = sum(0.5 * (b.momentum * b.momentum) / b.mass for b in balls)
    vmax = max(abs(b.velocity) for b in balls)
    print(f"NaN present:               {nan}")
    print(f"max overlap ever:          {max_overlap:.4f}  (expect < ~0.05)")
    print(f"pair momentum drift:       {p_pair_drift:.6f}  (expect ~0, pair collisions conserve)")
    print(f"pair energy drift:         {E_pair_drift:.6f}  (expect ~0, elastic)")
    print(f"total KE drift over run:   {KE_end - KE0:+.5f}  (expect small)")
    print(f"max speed:                 {vmax:.3f}  (no explosion, expect < ~10)")
    return all([not nan, max_overlap < 0.05, p_pair_drift < 0.01, E_pair_drift < 0.01, vmax < 10.0])

if __name__ == "__main__":
    ok = verify()
    print("PASS" if ok else "FAIL")

# ---- tkinter visualization ----
if __name__ == "__main__":
    SCALE = 100
    X0, Y0 = 50, 650
    Wv, Hv = 900, 700

    balls = make_balls()
    root = tk.Tk()
    root.title("50 identical balls, elastic, no gravity")
    cv = tk.Canvas(root, width=Wv, height=Hv, bg="white")
    cv.pack()
    cv.create_rectangle(X0, Y0 - H * SCALE, X0 + W * SCALE, Y0, outline="gray")

    def to_screen(p):
        return X0 + p.i * SCALE, Y0 - p.j * SCALE

    dt = 0.001
    def step():
        for _ in range(10):                    # 0.01 s of sim per frame
            substep(balls, dt)                 # update + walls + pair collisions
        cv.delete("ball")
        for b in balls:
            x, y = to_screen(b.position)
            r = b.radius * SCALE
            cv.create_oval(x - r, y - r, x + r, y + r, outline="black", tags="ball")
        root.after(10, step)

    step()
    root.mainloop()
