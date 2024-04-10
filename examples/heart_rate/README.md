# Heart Rate Monitor Example

## Overview

The `HeartRateMonitor` class is designed to calculate heart rate from the raw sensor readings of a MAX30102 pulse oximeter and heart-rate sensor, tailored for use in a MicroPython environment on an ESP32 board. It continuously processes a stream of raw integer readings from the sensor, identifies heartbeats by detecting peaks in the signal, and calculates the heart rate based on the intervals between consecutive peaks.

## How It Works

- Input: The class expects individual raw sensor readings (integer values) as input, provided to it through the `add_sample` method. These readings should come from the IR or green LEDs of the MAX30102 sensor, continuously polled at a consistent rate.

- Signal Processing:

  - **Smoothing**: The input signal is first smoothed using a moving average filter to reduce high-frequency noise. This step is crucial for accurate peak detection.

  - **Peak Detection**: The algorithm then identifies peaks in the smoothed signal using a dynamic thresholding method. A peak represents a heartbeat.

- Heart Rate Calculation: Once peaks are identified, the class calculates the heart rate by averaging the time intervals between consecutive peaks. The result is expressed in beats per minute (BPM).

## Parameters

- `sample_rate` (int): Defines the rate at which samples are collected from the sensor, in samples per second (Hz). This rate should match the polling frequency of the sensor in your application.

- `window_size` (int): Determines the number of samples over which to perform peak detection and heart rate calculation. A larger `window_size` can improve accuracy by considering more data but may also increase computation time and reduce responsiveness to changes in heart rate. Typically set based on the expected range of heart rates and the sample rate.

- `smoothing_window` (int): Specifies the size of the moving average filter window for signal smoothing. A larger window will produce a smoother signal but may also dilute the signal's peaks, potentially affecting peak detection accuracy. The optimal size often depends on the level of noise in the signal and the sample rate.

### Setting the Parameters

- `sample_rate`: Set this to match the frequency at which you're polling the MAX30102 sensor. Common values are 50, 100, or 200 Hz, depending on your application's requirements for data granularity and responsiveness.

- `window_size`: Start with a value that covers 1 to 2 seconds of data, based on your sample_rate. For example, at 100 Hz, a window size of 100 to 200 samples might be appropriate. Adjust based on testing, considering the balance between accuracy and responsiveness.

- `smoothing_window`: Begin with a small window, such as 5 to 10 samples, and adjust based on the noise level observed in your sensor data. The goal is to smooth out high-frequency noise without significantly delaying the detection of true heartbeats.

## Expected Input and Results

- Input: Continuous integer readings from the MAX30102 sensor, added one at a time via the add_sample method.

- Output: The heart rate in BPM, calculated periodically by calling the calculate_heart_rate method. This method returns None if not enough data is present to accurately calculate the heart rate.

## Example Usage

For a complete example, see `./main.py`.

```python
# Initialize the heart rate monitor
hr_monitor = HeartRateMonitor(
    # Select a sample rate that matches the sensor's acquisition rate
    sample_rate=actual_acquisition_rate,
    # Select a significant window size to calculate the heart rate (2-5 seconds)
    window_size=int(actual_acquisition_rate * 3),
)

# Add samples in a loop (replace the sample polling with actual sensor data retrieval)
for _ in range(1000):  # Example loop
    sample = ...  # Poll the MAX30102/5 sensor to get a new sample
    hr_monitor.add_sample(sample)
    # Optionally, sleep or wait based on your polling frequency

# Calculate and print the heart rate
heart_rate = hr_monitor.calculate_heart_rate()
if heart_rate is not None:
    print(f"Heart Rate: {heart_rate:.2f} BPM")
else:
    print("Not enough data to calculate heart rate")
```
