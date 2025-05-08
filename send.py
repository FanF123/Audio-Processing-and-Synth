import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation

ser = serial.Serial('COM6', 38400)

fig, ax = plt.subplots()
x_data, y_data = [], []

def update(frame):
    line = ser.readline().decode().strip()
    try:
        
        adc_value = int(line)
        print(adc_value)
        y_data.append(adc_value)
        x_data.append(len(y_data))
    except Exception as e:
        print("Parsing error:", e)
    
    ax.clear()
    ax.plot(x_data[-1000:], y_data[-1000:])  # Plot last 200 samples
    ax.set_title("ADC Data")
    ax.set_xlabel("Sample")
    ax.set_ylabel("ADC Value")

ani = animation.FuncAnimation(fig, update, interval=100)
plt.show()