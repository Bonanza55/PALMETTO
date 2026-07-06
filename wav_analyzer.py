#!/usr/bin/env python3
import os
import sys
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.ticker import MultipleLocator, FuncFormatter
from scipy.io import wavfile
from scipy.signal import spectrogram

def log_msg(status, message):
    print(f"[{status}] {message}")

def analyze_wav(file_path, freq_min=0, freq_max=None, fft_size=2048, overlap_frac=0.5, start_time=0.0, end_time=None, zoom_time=None, mark_freq=1200, space_freq=2200):
    if not os.path.exists(file_path):
        log_msg("ERR", f"File not found: {file_path}")
        sys.exit(1)

    # 1. Load the WAV file safely
    try:
        sample_rate, data = wavfile.read(file_path)
    except Exception as e:
        log_msg("ERR", f"Failed to read WAV file: {e}")
        sys.exit(1)

    # Determine bit depth and track characteristics
    dtype = data.dtype
    num_samples_total = data.shape[0]
    duration = num_samples_total / sample_rate
    channels = 1 if len(data.shape) == 1 else data.shape[1]

    log_msg("INFO", f"Filename: {os.path.basename(file_path)}")
    log_msg("INFO", f"Sample Rate: {sample_rate} Hz")
    log_msg("INFO", f"Channels: {channels}")
    log_msg("INFO", f"Duration: {duration:.3f} seconds ({num_samples_total} total samples)")
    log_msg("INFO", f"Data Type / Bit Depth Representation: {dtype}")

    # Validate and clamp time window parameters
    start_time = max(0.0, start_time)
    if end_time is None:
        end_time = duration
    else:
        end_time = min(duration, end_time)
    
    if start_time >= end_time:
        log_msg("ERR", f"Invalid time window: start_time ({start_time}s) >= end_time ({end_time}s)")
        sys.exit(1)

    # Convert time to sample indices
    start_idx = int(start_time * sample_rate)
    end_idx = int(end_time * sample_rate)

    log_msg("INFO", f"Display Window: {start_time:.3f}s to {end_time:.3f}s (duration: {end_time - start_time:.3f}s)")

    # Validate frequency range parameters
    nyquist = sample_rate / 2
    if freq_max is None:
        freq_max = nyquist
    
    freq_min = max(0, freq_min)
    freq_max = min(nyquist, freq_max)
    
    if freq_min >= freq_max:
        log_msg("ERR", f"Invalid frequency window: freq_min ({freq_min} Hz) >= freq_max ({freq_max} Hz)")
        sys.exit(1)
    
    log_msg("INFO", f"Frequency Range: {freq_min:.1f} Hz to {freq_max:.1f} Hz (Nyquist: {nyquist:.1f} Hz)")

    # Handle multi-channel audio by mixing down to mono for spectral analysis
    if channels > 1:
        log_msg("WARN", "Multi-channel audio detected. Converting to mono for DSP analysis.")
        audio_full = data.astype(np.float32).mean(axis=1)
    else:
        audio_full = data.astype(np.float32)

    # Slice to the requested time window
    audio_buffer = audio_full[start_idx:end_idx]
    num_samples = len(audio_buffer)
    display_duration = end_time - start_time

    # 2. Establish dynamic range boundaries based on integer ceilings vs float
    if np.issubdtype(dtype, np.integer):
        max_possible = np.iinfo(dtype).max
        # Normalize to standard [-1.0, 1.0] float range
        audio_buffer /= max_possible
    else:
        max_possible = 1.0

    # 3. Time-Domain Analytics (Clipping & Peak Power)
    peak_val = np.max(np.abs(audio_buffer))
    clipping_samples = np.sum(np.abs(audio_buffer) >= 0.999)
    clipping_percent = (clipping_samples / num_samples) * 100
    
    # Avoid log10(0) if the file is completely silent
    rms_val = np.sqrt(np.mean(audio_buffer**2))
    rms_db = 20 * np.log10(rms_val + 1e-9)
    peak_db = 20 * np.log10(peak_val + 1e-9)

    log_msg("ANALYSIS", f"Peak Amplitude: {peak_val:.4f} ({peak_db:.2f} dBFS)")
    log_msg("ANALYSIS", f"RMS Power Level: {rms_db:.2f} dBFS")
    if clipping_samples > 0:
        log_msg("ALERT", f"Clipping detected! {clipping_samples} samples breached ceiling ({clipping_percent:.4f}%)")
    else:
        log_msg("ANALYSIS", "No digital clipping detected.")

    # 4. Digital Signal Processing - Compute Spectrogram
    noverlap = int(fft_size * overlap_frac)
    frequencies, times, spec_density = spectrogram(
        audio_buffer, 
        fs=sample_rate, 
        window='hann', 
        nperseg=fft_size, 
        noverlap=noverlap
    )

    # Convert power spectral density to dB scale
    spec_db = 10 * np.log10(spec_density + 1e-12)

    # Identify dominant frequency component across the absolute spectrum
    mean_spectrum = np.mean(spec_density, axis=1)
    peak_freq_idx = np.argmax(mean_spectrum)
    dominant_freq = frequencies[peak_freq_idx]
    log_msg("DSP", f"Dominant Frequency Component: {dominant_freq:.2f} Hz")

    # 5. Visual Engineering (Matplotlib Layout)
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(11, 7), sharex=False)
    fig.suptitle(f"Technical Analysis: {os.path.basename(file_path)}", fontsize=14, fontweight='bold')

    # Time Domain Plot
    time_axis = np.linspace(start_time, end_time, num_samples)
    ax1.plot(time_axis, audio_buffer, color='royalblue', linewidth=0.5, alpha=0.8)
    ax1.axhline(1.0, color='crimson', linestyle='--', alpha=0.5, label="Clipping Bound")
    ax1.axhline(-1.0, color='crimson', linestyle='--')
    ax1.set_title("Time-Domain Waveform", fontsize=11, loc='left')
    ax1.set_ylabel("Normalized Amplitude")
    ax1.set_xlabel("Time (seconds)")
    ax1.grid(True, linestyle=':', alpha=0.6)
    ax1.set_xlim(start_time, end_time)
    ax1.set_ylim(-1.1, 1.1)
    
    # Format x-axis time labels with 2 decimal places
    def format_time(x, p):
        return f'{x:.2f}'
    ax1.xaxis.set_major_locator(MultipleLocator(0.05))
    ax1.xaxis.set_major_formatter(FuncFormatter(format_time))
    
    # Add zoomed inset showing 300ms window (optional, if zoom_time specified)
    if zoom_time is not None:
        from matplotlib.patches import Rectangle
        inset_duration = 0.1  # 100 ms
        inset_start = zoom_time
        inset_end = min(zoom_time + inset_duration, duration)
        
        if inset_end > inset_start:
            # Create inset axis
            axins = ax1.inset_axes([0.55, 0.55, 0.4, 0.35])
            inset_idx_start = int((inset_start - start_time) * sample_rate)
            inset_idx_end = int((inset_end - start_time) * sample_rate)
            
            # Clamp to available data
            inset_idx_start = max(0, inset_idx_start)
            inset_idx_end = min(len(audio_buffer), inset_idx_end)
            
            if inset_idx_end > inset_idx_start:
                inset_time = np.linspace(inset_start, inset_end, inset_idx_end - inset_idx_start)
                axins.plot(inset_time, audio_buffer[inset_idx_start:inset_idx_end], color='royalblue', linewidth=0.8)
                axins.set_xlim(inset_start, inset_end)
                axins.set_title(f"Zoom: {inset_duration*1000:.0f}ms @ {inset_start:.2f}s", fontsize=9)
                axins.grid(True, linestyle=':', alpha=0.4)
                axins.xaxis.set_major_locator(MultipleLocator(0.05))
                axins.xaxis.set_major_formatter(FuncFormatter(format_time))

    # Frequency Domain Spectrogram
    # Using 'inferno' or 'viridis' color mapping for maximum thermal separation of signals
    # Adjust time axis to absolute file time by adding start_time offset
    times_absolute = times + start_time
    mesh = ax2.pcolormesh(times_absolute, frequencies, spec_db, cmap='inferno', shading='gouraud', vmin=-100, vmax=0)
    ax2.set_title(f"Spectrogram Plot (FFT Window: {fft_size}, Window Function: Hann)", fontsize=11, loc='left')
    ax2.set_ylabel("Frequency (Hz)")
    ax2.set_xlabel("Time (seconds)")
    ax2.set_xlim(start_time, end_time)
    ax2.set_ylim(freq_min, freq_max)
    
    # Format x-axis time labels with 2 decimal places
    ax2.xaxis.set_major_locator(MultipleLocator(0.05))
    ax2.xaxis.set_major_formatter(FuncFormatter(format_time))
    
    # Add reference lines for AFSK tones (configurable mark/space frequencies)
    if freq_min <= mark_freq <= freq_max:
        ax2.axhline(mark_freq, color='lime', linestyle='--', alpha=0.6, linewidth=1.5, label=f'{mark_freq} Hz (Mark)')
    if freq_min <= space_freq <= freq_max:
        ax2.axhline(space_freq, color='yellow', linestyle='--', alpha=0.6, linewidth=1.5, label=f'{space_freq} Hz (Space)')
    
    # Only show legend if at least one tone is visible
    if (freq_min <= mark_freq <= freq_max) or (freq_min <= space_freq <= freq_max):
        ax2.legend(loc='upper right', fontsize=9)
    
    # Add a dedicated color bar for power intensity reference
    cbar = fig.colorbar(mesh, ax=ax2, orientation='vertical', pad=0.02)
    cbar.set_label("Relative Power Density (dBFS)", rotation=275, labelpad=12)

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python3 wav_analyzer.py <path_to_wav_file> [freq_min] [freq_max] [fft_size] [start_time] [end_time] [zoom_time] [mark_freq] [space_freq]")
        print()
        print("Arguments:")
        print("  path_to_wav_file  : Path to WAV file to analyze")
        print("  freq_min          : Minimum frequency in Hz (default: 0)")
        print("  freq_max          : Maximum frequency in Hz (default: Nyquist)")
        print("  fft_size          : FFT window size (default: 2048)")
        print("  start_time        : Start time in seconds (default: 0.0)")
        print("  end_time          : End time in seconds (default: end of file)")
        print("  zoom_time         : Start time for 300ms zoom inset (optional; if omitted, no zoom shown)")
        print("  mark_freq         : Mark tone frequency in Hz (default: 1200)")
        print("  space_freq        : Space tone frequency in Hz (default: 2200)")
        print()
        print("Examples:")
        print("  python3 wav_analyzer.py audio.wav")
        print("  python3 wav_analyzer.py audio.wav 1000 4000 2400 3.0 4.25")
        print("  python3 wav_analyzer.py audio.wav 1000 4000 2400 3.0 4.25 3.00")
        print("  python3 wav_analyzer.py audio.wav 1000 4000 2400 3.0 4.25 3.00 1200 2400")
        sys.exit(1)
        
    target_file = sys.argv[1]
    freq_min_param = int(sys.argv[2]) if len(sys.argv) > 2 else 0
    freq_max_param = int(sys.argv[3]) if len(sys.argv) > 3 else None
    fft_size_param = int(sys.argv[4]) if len(sys.argv) > 4 else 2048
    start_sec = float(sys.argv[5]) if len(sys.argv) > 5 else 0.0
    end_sec = float(sys.argv[6]) if len(sys.argv) > 6 else None
    zoom_sec = float(sys.argv[7]) if len(sys.argv) > 7 else None
    mark_freq_param = int(sys.argv[8]) if len(sys.argv) > 8 else 1200
    space_freq_param = int(sys.argv[9]) if len(sys.argv) > 9 else 2200
    
    analyze_wav(target_file, freq_min=freq_min_param, freq_max=freq_max_param, 
                fft_size=fft_size_param, start_time=start_sec, end_time=end_sec, 
                zoom_time=zoom_sec, mark_freq=mark_freq_param, space_freq=space_freq_param)
