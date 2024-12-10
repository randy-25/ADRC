import serial
import time
import numpy as np
import control as ctrl
import matplotlib.pyplot as plt

# Initialize serial communication with Arduino
arduino = serial.Serial(port='COM10', baudrate=115200, timeout=1)
time.sleep(2)  # Wait for the connection to establish
arduino.reset_input_buffer()

# Define plant transfer function (example: a second-order system)
numerator = [3]
denominator = [1, 2, 1]  # s^2 + 2s + 1
plant_tf = ctrl.TransferFunction(numerator, denominator)

# Convert the transfer function to state-space representation
plant_ss = ctrl.tf2ss(plant_tf)

# Discretize the state-space system
t_sampling = 0.01  # Sampling time in seconds
plant_d = ctrl.c2d(plant_ss, t_sampling, method='zoh')

# Extract discrete state-space matrices
A = plant_d.A
B = plant_d.B
C = plant_d.C
D = plant_d.D

# Simulation variables
x = np.zeros((A.shape[0],))  # Plant states
setpoint = 1.0  # Desired output
control_signal = 0.0  # Initial control input

# Arrays to store simulation results for plotting
time_steps = []
plant_outputs = []
control_signals = []

# Simulation loop
try:
    for i in range(1000):  # Run for 1000 steps
        t = i * t_sampling
        # Compute plant output
        y = C @ x + D * control_signal

        # Send plant output to Arduino
        arduino.write(f"{y[0][0]:.6f}\n".encode())  # Use item() to ensure a scalar value

        # Read control signal from Arduino
        control_signal = float(arduino.readline().decode().strip())
        x_hat_values = arduino.readline().decode().strip().split(",")
        x_hat = np.array([float(x) for x in x_hat_values])
        # Update plant state
        x = A @ x + B * control_signal

        # Log simulation results
        print(f"Time: {t:.2f}s, Plant Output: {y[0][0]:.6f}, Control Signal: {control_signal:.6f}, Estimated State: {x_hat}")

        # Store results for plotting
        time_steps.append(t)
        plant_outputs.append(y[0][0])
        control_signals.append(control_signal)

        time.sleep(t_sampling)

except KeyboardInterrupt:
    print("Simulation stopped.")
finally:
    arduino.close()

# Settling time calculation
tolerance = 0.02  # 2% tolerance band
upper_bound = setpoint * (1 + tolerance)
lower_bound = setpoint * (1 - tolerance)

settling_time = None
for t, output in zip(time_steps, plant_outputs):
    if all(lower_bound <= o <= upper_bound for o in plant_outputs[time_steps.index(t):]):
        settling_time = t
        break

# Display settling time
if settling_time is not None:
    print(f"Settling Time: {settling_time:.2f} seconds")
else:
    print("The system did not settle within the specified tolerance.")

# Plot the results
plt.figure(figsize=(10, 6))

# Plot plant output
plt.plot(time_steps, plant_outputs, label="Plant Output", color="blue")
plt.axhline(setpoint, color="red", linestyle="--", label="Setpoint")
plt.axhline(upper_bound, color="green", linestyle="--", label="Tolerance Band")
plt.axhline(lower_bound, color="green", linestyle="--")
plt.title("Plant Output vs Time")
plt.xlabel("Time (s)")
plt.ylabel("Output")
plt.legend()
plt.grid()

# Highlight settling time
if settling_time is not None:
    plt.axvline(settling_time, color="orange", linestyle="--", label=f"Settling Time = {settling_time:.2f}s")

plt.tight_layout()
plt.legend()
plt.show()
