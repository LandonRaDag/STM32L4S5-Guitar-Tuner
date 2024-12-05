import numpy as np

def generate_sine_wave(frequency, amplitude, num_samples, sample_rate):
    """
    Generates amplitude values for a sine wave.

    Parameters:
        frequency (float): Frequency of the sine wave in Hz.
        amplitude (float): Amplitude of the sine wave (peak value).
        num_samples (int): Number of samples to generate.
        sample_rate (int): Number of samples per second.

    Returns:
        np.ndarray: Array of amplitude values normalized to fit uint32_t.
    """
    t = np.linspace(0, num_samples / sample_rate, num_samples, endpoint=False)  # Time vector
    sine_wave = np.sin(2 * np.pi * frequency * t)  # Sine wave

    # Normalize to uint32_t range (0 to 2^32-1)
    sine_wave_normalized = ((sine_wave + 1) / 2) *amplitude  # Map [-1, 1] to [0, amplitude]
    return sine_wave_normalized.astype(np.uint32)

def format_as_c_array(data, var_name):
    """
    Formats data as a C array.

    Parameters:
        data (np.ndarray): Array of data values.
        var_name (str): Name of the C array variable.

    Returns:
        str: C array as a formatted string.
    """
    formatted_data = ", ".join(f"{x}" for x in data)
    return f"{var_name} = {{ {formatted_data} }};"

# Parameters
frequency = 110  # Frequency in Hz (e.g., A4)
amplitude = 4000    # Peak amplitude
num_samples = 1024  # Number of samples
sample_rate = 48000  # Sampling rate in Hz (standard for audio)
var_name = "sine_samples[1]"  # Name of the C array

# Generate sine wave
sine_wave = generate_sine_wave(frequency, amplitude, num_samples, sample_rate)

# Format as C array
c_array = format_as_c_array(sine_wave, var_name)

# Print the C array
print(c_array)

# Optional: Save to a file
output_filename = "sine_wave.c"
with open(output_filename, "w") as file:
    file.write(c_array)
print(f"C array saved to {output_filename}")
