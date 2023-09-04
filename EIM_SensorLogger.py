import os
from datetime import datetime
import serial
from serial.tools import list_ports
import matplotlib.pyplot as plt
from collections import deque
import time
import csv
from collections import deque as circular_buffer
import tkinter as tk
from tkinter import ttk
from scipy.fft import rfft, rfftfreq
import numpy as np
from scipy import signal

def calculate_sample_spacing(t):
    if len(t) >= 2:
        # Calculate the differences between consecutive timestamps
        differences = [t[i + 1] - t[i] for i in range(len(t) - 1)]

        # Calculate the average sample spacing
        T = sum(differences) / len(differences)
        return T
    else:
        print("Not enough data to calculate sample spacing")
        return None

def update_plot():
    # Restoring background
    fig.canvas.restore_region(ax1_background)

    # Update time series plots
    for ax, data_dict, background in zip((ax1, ax2, ax3), (accel_data, gyro_data, torque_data),
                                         (ax1_background, ax2_background, ax3_background)):
        for line, data_queue in zip(ax.get_lines(), data_dict.values()):
            line.set_data(t, data_queue)

        # Rescale the view and redraw the lines
        ax.relim()
        ax.autoscale_view()
        ax.draw_artist(ax.lines[0])

    # Update FFT plots
    T = calculate_sample_spacing(t)
    n = len(data_queue)
    for ax_fft, data_dict, background in zip((ax1_fft, ax2_fft, ax3_fft), (accel_data, gyro_data, torque_data),
                                             (ax1_fft_background, ax2_fft_background, ax3_fft_background)):
        for line, (signal_name, data_queue) in zip(ax_fft.get_lines(), data_dict.items()):

            # Compute the FFT
            window = signal.windows.hann(n)
            fft_result = rfft(np.array(data_queue) * window)
            fft_result /= n

            freqs = rfftfreq(n, T)
            line.set_data(freqs, 2*np.abs(fft_result))

        # Rescale the view and redraw the lines
        ax_fft.relim()
        ax_fft.autoscale_view()
        ax_fft.draw_artist(ax_fft.lines[0])

    fig.canvas.blit(ax1.bbox)
    fig.canvas.blit(ax2.bbox)
    fig.canvas.blit(ax3.bbox)
    fig.canvas.flush_events()

# Create directory for data if it doesn't exist
directory = 'Arduino_Data'
if not os.path.exists(directory):
    os.makedirs(directory)

# Prepare timestamped filename for data storage
timestamp = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
filename = f'{directory}/{timestamp}_data.csv'

# Initialize CSV writer for data storage
with open(filename, 'w', newline='') as f:
    print("CSV file opened:", filename)
    writer = csv.writer(f, delimiter=',')

    headers = ['Time[s]', 'RateRoll[deg/s]', 'RatePitch[deg/s]', 'RateYaw[deg/s]', 'AccX[m/s^2]', 'AccY[m/s^2]',
               'AccZ[m/s^2]', 'Torque[Ncm]']
    writer.writerow(headers)

    # Find Arduino port
    arduino_port = None
    ports = list_ports.comports()
    for port in ports:
        if 'Arduino' in port.description:
            arduino_port = port.device
            print("Connected to:", port.description)
            break

    if arduino_port is None:
        print("Arduino port not found. Make sure the Arduino is connected.")
        ser = serial.Serial('COM5', 57600)  # If necessary, set the port manually

    # Establish serial connection
    ser = serial.Serial(arduino_port, 57600)

    ser.setDTR(False)
    time.sleep(1)
    ser.flushInput()
    ser.setDTR(True)

    # Create GUI
    root = tk.Tk()
    root.title("Data Logger")
    root.geometry("650x250")  # Larger size

    # Setting up a style
    style = ttk.Style()
    style.theme_use('clam')
    style.configure('TButton', font=('Arial', 12), foreground='black')
    style.configure('TLabel', font=('Arial', 12), background='lightgrey', padding=10)
    style.configure('TScale', background='lightgrey')

    # Variables to track button states
    logging_enabled = True
    plotting_enabled = True
    program_stopped = False
    decimation_value = tk.IntVar(value=10)  # New IntVar for decimation value

    def start_logging():
        global logging_enabled
        logging_enabled = True
        logging_status_label.config(text="Sensor data is recorded again.")  # Update logging status

    def stop_logging():
        global logging_enabled
        logging_enabled = False
        logging_status_label.config(text="")  # Clear logging status
        logging_status_label.config(text="Sensor data recording has been stopped.")  # Update logging status

    def start_plotting():
        global plotting_enabled
        plotting_enabled = True

    def stop_plotting():
        global plotting_enabled
        plotting_enabled = False

    def stop_program():
        global program_stopped
        program_stopped = True
        root.destroy()

    # Create frames for plotting and logging buttons
    plotting_frame = tk.Frame(root)
    plotting_frame.pack(side='left', fill='both', expand=True)
    logging_frame = tk.Frame(root)
    logging_frame.pack(side='right', fill='both', expand=True)

    # Create buttons using ttk.Button
    start_logging_button = ttk.Button(logging_frame, text="Start Logging", command=start_logging)
    start_logging_button.grid(row=0, column=0, pady=5, padx=5)  # padding between the buttons
    start_plotting_button = ttk.Button(plotting_frame, text="Start Plotting", command=start_plotting)
    start_plotting_button.grid(row=0, column=0, pady=5, padx=5)  # padding between the buttons
    stop_logging_button = ttk.Button(logging_frame, text="Stop Logging", command=stop_logging)
    stop_logging_button.grid(row=1, column=0, pady=5, padx=5)  # padding between the buttons
    stop_plotting_button = ttk.Button(plotting_frame, text="Stop Plotting", command=stop_plotting)
    stop_plotting_button.grid(row=1, column=0, pady=5, padx=5)  # padding between the buttons

    # Create label to display logged lines using ttk.Label
    logged_lines_label = ttk.Label(root, text="")
    logged_lines_label.pack(pady=5)  # padding around the label

    # Create label to display logging status using ttk.Label
    logging_status_label = ttk.Label(root, text="Sensor data is recorded.")
    logging_status_label.pack(pady=5)  # padding around the label

    # StringVar variable to store and display the current decimation value
    decimation_value_str = tk.StringVar()

    # Function to update the current decimation value
    def update_decimation_value(val):
        # Round the value to the nearest integer
        int_val = round(float(val))

        # Update the slider's value to the rounded integer value
        decimation_value.set(int_val)
        decimation_value_str.set("Current decimation value: " + str(int_val))

    # Create decimation slider
    decimation_slider = ttk.Scale(root, from_=1, to=100, variable=decimation_value, command=update_decimation_value,
                                  orient='horizontal', length=500)
    decimation_slider.pack(pady=10)  # padding around the slider

    # Label to display the current decimation value
    decimation_value_label = ttk.Label(root, textvariable=decimation_value_str)
    decimation_value_label.pack()

    stop_program_button = ttk.Button(root, text="Stop Program", command=stop_program)
    stop_program_button.pack(pady=10)  # padding between the buttons

    # Prepare the figure before entering the main loop
    plt.style.use('fast')  # For more styles, see print(plt.style.available)
    plt.ion()
    fig, ((ax1, ax1_fft), (ax2, ax2_fft), (ax3, ax3_fft)) = plt.subplots(3, 2, figsize=(20, 10))

    # Draw everything that won't change, and then start blit
    fig.canvas.draw()
    ax1_background = fig.canvas.copy_from_bbox(ax1.bbox)
    ax2_background = fig.canvas.copy_from_bbox(ax2.bbox)
    ax3_background = fig.canvas.copy_from_bbox(ax3.bbox)
    ax1_fft_background = fig.canvas.copy_from_bbox(ax1_fft.bbox)
    ax2_fft_background = fig.canvas.copy_from_bbox(ax2_fft.bbox)
    ax3_fft_background = fig.canvas.copy_from_bbox(ax3_fft.bbox)

    N = 300
    t = deque(maxlen=N)
    accel_data = {'AccX': deque(maxlen=N), 'AccY': deque(maxlen=N), 'AccZ': deque(maxlen=N)}
    gyro_data = {'RateRoll': deque(maxlen=N), 'RatePitch': deque(maxlen=N), 'RateYaw': deque(maxlen=N)}
    torque_data = {'Torque': deque(maxlen=N)}

    # Plot initial data
    for label, deque in accel_data.items():
        ax1.plot(t, deque, label=label)
        ax1_fft.plot(t, deque, label=label)
    ax1.set_ylabel('Acceleration [m/s^2]')
    ax1.legend(loc='upper left')
    ax1.grid(True)
    ax1_fft.set_ylabel('FFT Acceleration')
    ax1_fft.legend(loc='upper left')
    ax1_fft.grid(True)

    for label, deque in gyro_data.items():
        ax2.plot(t, deque, label=label)
        ax2_fft.plot(t, deque, label=label)
    ax2.set_ylabel('Rotation rate [°/s]')
    ax2.legend(loc='upper left')
    ax2.grid(True)
    ax2_fft.set_ylabel('FFT Rotation rate')
    ax2_fft.legend(loc='upper left')
    ax2_fft.grid(True)

    for label, deque in torque_data.items():
        ax3.plot(t, deque, label=label)
        ax3_fft.plot(t, deque, label=label)
    ax3.set_xlabel('Time [s]')
    ax3.set_ylabel('Torque [Ncm]')
    ax3.legend(loc='upper left')
    ax3.grid(True)
    ax3_fft.set_xlabel('Frequency [Hz]')
    ax3_fft.set_ylabel('FFT Torque')
    ax3_fft.legend(loc='upper left')
    ax3_fft.grid(True)

    plt.tight_layout()
    plt.show()
    plt.pause(0.1)

    # Pre-define the data_dict before the loop
    data_dict = {**gyro_data, **accel_data, **torque_data}

    start_time = time.time()
    # Start data acquisition and plotting loop
    try:
        decimation = decimation_value.get()  # Get the initial value
        data_rows = circular_buffer(maxlen=decimation)
        while True:
            if ser.is_open:
                try:
                    line = ser.readline().decode('utf-8', 'ignore').strip()
                except UnicodeDecodeError:
                    print("A UnicodeDecodeError occurred.")
                data = line.split(',')
                if len(data) == 7:
                    try:
                        t.append(time.time() - start_time)
                        data_row = [round(time.time() - start_time, 2)] + [float(val) for val in data]
                        for i, (label, deque) in enumerate(data_dict.items()):
                            deque.append(data_row[i + 1])
                        data_rows.append(data_row)
                        if len(data_rows) == decimation and plotting_enabled:
                            update_plot()
                            Ts = calculate_sample_spacing(t)
                            print("Estimated sample time: {:.2f} ms".format(Ts*1000))
                        if logging_enabled and not program_stopped:
                            formatted_time = "Time: {:.2f}".format(time.time() - start_time)
                            logged_lines_label.config(text=formatted_time + ': ' + line)
                    except ValueError:
                        formatted_time = "Time: {:.2f}".format(time.time() - start_time)
                        print(f"{formatted_time} - Received malformed data: {data}")
                else:
                    formatted_time = "Time: {:.2f}".format(time.time() - start_time)
                    print(f"{formatted_time} - Received data of incorrect length: {data}")

            # Check if decimation value has changed. If so, create new data_rows buffer.
            new_decimation = decimation_value.get()
            if new_decimation != decimation:
                decimation = new_decimation
                data_rows = circular_buffer(maxlen=decimation)

            if len(data_rows) == decimation and logging_enabled:
                writer.writerows(data_rows)
                f.flush()
                data_rows.clear()

            root.update()  # Update the GUI to handle button clicks

    except KeyboardInterrupt:
        pass

    finally:
        # Clean up after loop is finished or interrupted
        ser.close()
        f.close()
        if not program_stopped:
            root.destroy()
