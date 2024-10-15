import tkinter as tk
import serial
import time
import math
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure

# Set up serial communication (adjust the 'COM3' or '/dev/ttyUSB0' to your port)
arduino = serial.Serial(port='/dev/cu.usbmodem1101', baudrate=115200, timeout=1)
time.sleep(2)  # Wait for the connection to establish

# Global variables
counter = 0
last_pass_time = None
rpm = 0
temperature = 0  # Variable to store temperature
rpm_history = []  # List to keep track of RPM values for plotting
temperature_history = []  # List to keep track of temperature values for plotting

# Function to calculate RPM
def calculate_rpm():
    global last_pass_time, rpm
    current_time = time.time()
    if last_pass_time:
        time_difference = current_time - last_pass_time
        rpm = 60 / (time_difference * 3)  # 3 fins = 1 full revolution
    last_pass_time = current_time

# Function to listen for serial data from Arduino
def read_from_arduino():
    global counter, temperature
    if arduino.in_waiting > 0:
        data = arduino.readline().decode('utf-8').strip()
        if data == "PASS":  # Fin has passed in front of the sensor
            counter += 1
            calculate_rpm()
            update_windmill()
        else:
            try:
                temperature = float(data)  # Read temperature from serial
                update_temperature_plot()  # Update temperature plot
            except ValueError:
                pass  # Ignore any non-float values
    root.after(50, read_from_arduino)  # Check every 50ms for serial data

# Function to draw the windmill with a rotating turbine (pole and fins)
def draw_windmill(canvas, angle):
    canvas.delete("all")  # Clear the canvas
    cx, cy = 150, 250  # Adjusted to center the windmill better
    r = 100  # Length of each fin

    # Rotate the turbine base (pole) as well
    pole_length = 120
    pole_angle = math.radians(angle)  # Angle for pole rotation
    pole_top_x = cx + pole_length * math.cos(pole_angle)
    pole_top_y = cy + pole_length * math.sin(pole_angle)

    # Draw the rotating pole
    canvas.create_line(cx, cy, pole_top_x, pole_top_y, fill="red", width=4)

    # Draw the central circle (hub)
    canvas.create_oval(cx - 20, cy - 20, cx + 20, cy + 20, fill="gray")

    # Draw the vertical line from the hub downwards
    canvas.create_line(cx, cy + 20, cx, cy + pole_length + 20, fill="black", width=4)

    # Calculate positions of the fins (3 fins, 120 degrees apart)
    for i in range(3):
        fin_angle = math.radians(angle + i * 120)  # Angle of the current fin
        x = cx + r * math.cos(fin_angle)
        y = cy + r * math.sin(fin_angle)
        canvas.create_line(cx, cy, x, y, fill="black", width=4)

# Function to update the windmill's rotation and RPM display
def update_windmill():
    global counter, rpm
    angle = (counter % 3) * 120  # Rotate by 120 degrees for each fin pass
    draw_windmill(canvas, angle)
    rpm_label.config(text=f"RPM: {rpm:.2f}")

    # Update RPM history for plotting
    rpm_history.append(rpm)
    if len(rpm_history) > 20:  # Limit history to the last 20 entries
        rpm_history.pop(0)

    # Update the plot
    update_plot()

# Function to update the RPM plot
def update_plot():
    ax.clear()  # Clear the current plot
    ax.plot(rpm_history, label="RPM", color='blue', marker='o', linestyle='-', markersize=5)
    ax.set_title("RPM Over Time", fontsize=14)
    ax.set_xlabel("Samples", fontsize=12)
    ax.set_ylabel("RPM", fontsize=12)
    ax.legend(loc='upper right')
    ax.grid(True, linestyle='--', alpha=0.5)
    ax.set_facecolor('white')
    ax.set_xlim(0, 19)
    ax.set_ylim(0, max(rpm_history) + 10 if rpm_history else 100)
    canvas_plot.draw()

# Function to update the temperature plot
def update_temperature_plot():
    global temperature, temperature_history
    temperature_history.append(temperature)
    if len(temperature_history) > 20:  # Limit history to the last 20 entries
        temperature_history.pop(0)

    ax2.clear()  # Clear the current temperature plot
    ax2.plot(temperature_history, label="Temperature (°C)", color='red', marker='o', linestyle='-', markersize=5)
    ax2.set_title("Temperature Over Time", fontsize=14)
    ax2.set_xlabel("Samples", fontsize=12)
    ax2.set_ylabel("Temperature (°C)", fontsize=12)
    ax2.legend(loc='upper right')
    ax2.grid(True, linestyle='--', alpha=0.5)
    ax2.set_facecolor('white')
    ax2.set_xlim(0, 19)
    ax2.set_ylim(min(temperature_history) - 5 if temperature_history else 0, 
                 max(temperature_history) + 5 if temperature_history else 100)
    canvas_plot.draw()

# Create the Tkinter app
root = tk.Tk()
root.title("Windturbine Fin Counter")

# Create a frame for the windmill and plot
frame = tk.Frame(root)
frame.pack(fill=tk.BOTH, expand=True)

# Create a canvas for drawing the windmill
canvas = tk.Canvas(frame, width=300, height=500)
canvas.pack(side=tk.LEFT)

# Create the RPM label and place it below the windmill canvas
rpm_label = tk.Label(frame, text=f"RPM: {rpm:.2f}", font=("Helvetica", 24))
rpm_label.pack(side=tk.LEFT, pady=20)

# Create a Matplotlib figure for plotting RPM and Temperature
fig = Figure(figsize=(10, 5), dpi=100)
ax = fig.add_subplot(121)  # RPM plot
ax2 = fig.add_subplot(122)  # Temperature plot
canvas_plot = FigureCanvasTkAgg(fig, master=frame)
canvas_plot.get_tk_widget().pack(side=tk.RIGHT, padx=20)

# Start reading from Arduino
root.after(50, read_from_arduino)

# Start the Tkinter main loop
root.mainloop()

# Close the serial connection when the app is closed
arduino.close()