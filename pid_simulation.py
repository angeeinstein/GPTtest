import numpy as np
import matplotlib.pyplot as plt

# Simple PID controller simulation
# Setpoint is 1.0, plant is a first-order system

def simulate_pid(kp, ki, kd, dt, steps):
    setpoint = 1.0
    x = 0.0
    integral = 0.0
    prev_error = 0.0
    history = []

    for _ in range(steps):
        error = setpoint - x
        integral += error * dt
        derivative = (error - prev_error) / dt
        u = kp * error + ki * integral + kd * derivative
        x += u * dt  # simple plant: dx/dt = u
        history.append(x)
        prev_error = error
    return history

if __name__ == "__main__":
    dt = 0.01
    steps = 1000
    response = simulate_pid(kp=2.0, ki=1.0, kd=0.1, dt=dt, steps=steps)
    time = np.arange(steps) * dt
    plt.plot(time, response, label="Output")
    plt.axhline(1.0, color="r", linestyle="--", label="Setpoint")
    plt.xlabel("Time [s]")
    plt.ylabel("Output")
    plt.legend()
    plt.title("PID Controller Response")
    plt.tight_layout()
    plt.savefig("pid_response.png")
    plt.show()
