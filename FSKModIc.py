#!/usr/bin/env python3
import os
import re
import time
import threading
import glob
import shutil
import subprocess
import tkinter as tk
from tkinter import ttk, messagebox
from datetime import datetime
import numpy as np
from scipy.io import wavfile as wav

try:
    import sounddevice as sd
except (ImportError, OSError):
    sd = None  # Headless system

try:
    import serial
except ImportError:
    serial = None

# --- User Adjustable Configuration Variables ---
MAX_PAYLOAD_CHARS = 256
SAMPLE_RATE = 48000
GAP_SECONDS = 3.0

# --- FCC Morse ID Configuration ---
MORSE_FREQ = 400.0             
MORSE_WPM = 20                 
POST_DATA_DELAY_SECS = 3.0     

# --- PTT & Audio Configuration ---
PTT_PORT = "/dev/ttyUSB1"       
PTT_DEVICE = None               
PTT_TXDELAY = 1.0              
PTT_TAIL = 0.1                 

# --- FSK Configuration ---
FSK_BAUD = 100                  
FSK_TONE_MARK = 1200            
FSK_TONE_SPACE = 2400           
FSK_PREAMBLE_BITS = 128         
FSK_SYNC_BITS = 16              

# Morse Code Dictionary
MORSE_DICT = {
    'A': '.-', 'B': '-...', 'C': '-.-.', 'D': '-..', 'E': '.', 'F': '..-.',
    'G': '--.', 'H': '....', 'I': '..', 'J': '.---', 'K': '-.-', 'L': '.-..',
    'M': '--', 'N': '-.', 'O': '---', 'P': '.--.', 'Q': '--.-', 'R': '.-.',
    'S': '...', 'T': '-', 'U': '..-', 'V': '...-', 'W': '.--', 'X': '-..-',
    'Y': '-.--', 'Z': '--..',
    '0': '-----', '1': '.----', '2': '..---', '3': '...--', '4': '....-',
    '5': '.....', '6': '-....', '7': '--...', '8': '---..', '9': '----.'
}

class FSKModulatorGUI:
    def __init__(self, root):
        self.root = root
        # Match the window title style of the demodulator[cite: 1, 2]
        self.root.title("PALMETTO TX")
        self.root.geometry("650x650")
        self.root.minsize(600, 520)

        # Pin the GUI to LIGHT mode regardless of system Dark Mode[cite: 1].
        self._force_light_theme()

        # --- UI State Variables ---
        self.callsign_var = tk.StringVar(value="")
        self.repeat_count = tk.StringVar(value="1")
        self.gap_secs = tk.StringVar(value=str(int(GAP_SECONDS)))
        self.status_msg = tk.StringVar(value="Ready to send.")

        self.build_ui()

    # =====================================================================
    #  Theme lock (light mode) - Imported from FSKDemodIc.py[cite: 1]
    # =====================================================================
    def _force_light_theme(self):
        try:
            self.root.tk.call("::tk::unsupported::MacWindowStyle",
                              "appearance", self.root, "aqua")
            return  
        except tk.TclError:
            pass

        LIGHT_BG = "#f0f0f0"
        try:
            self.root.configure(bg=LIGHT_BG)
            style = ttk.Style()
            style.theme_use("clam")
            style.configure(".", background=LIGHT_BG, foreground="#000000")
            style.configure("TLabel", background=LIGHT_BG, foreground="#000000")
            style.configure("TFrame", background=LIGHT_BG)
            style.configure("TLabelframe", background=LIGHT_BG)
            style.configure("TLabelframe.Label", background=LIGHT_BG, foreground="#2c3e50")
            style.configure("TButton", background="#e6e6e6", foreground="#000000")
            style.configure("TCheckbutton", background=LIGHT_BG, foreground="#000000")
            style.configure("TCombobox", fieldbackground="#ffffff", foreground="#000000")
            style.configure("TProgressbar", troughcolor="#dddddd")
        except Exception:
            pass

    def build_ui(self):
        # Enforce unified widget layout and color behaviors via TTK Style[cite: 1]
        try:
            style = ttk.Style()
            style.configure("TLabelFrame", font=("Arial", 10, "bold"), foreground="#2c3e50")
        except Exception:
            pass

        info_label = ttk.Label(self.root,
                               text=f"Baud: {FSK_BAUD}  |  Mark: {FSK_TONE_MARK}Hz  |  Space: {FSK_TONE_SPACE}Hz  |  Bell 202 Compatible",
                               font=("Arial", 10), foreground="#0066cc")
        info_label.pack(pady=(15, 10))

        # --- Top Compliance / Callsign Field ---
        callsign_frame = ttk.LabelFrame(self.root, text=" Regulatory Compliance ", padding=12)
        callsign_frame.pack(fill="x", padx=20, pady=5)
        
        ttk.Label(callsign_frame, text="Station Callsign (Morse ID):").grid(row=0, column=0, sticky="w", pady=5)
        
        vcmd = (self.root.register(self.validate_callsign), '%P')
        self.callsign_entry = ttk.Entry(callsign_frame, textvariable=self.callsign_var, width=10, 
                                        font=("Courier", 12, "bold"), validate="key", validatecommand=vcmd)
        self.callsign_entry.grid(row=0, column=1, sticky="w", padx=10, pady=5)
        self.callsign_var.trace_add("write", lambda *args: self.callsign_var.set(self.callsign_var.get().upper()))
        
        ttk.Label(callsign_frame, text="(Max 6 characters)", 
                  font=("Arial", 9, "italic"), foreground="#555").grid(row=0, column=2, sticky="w", pady=5)

        # --- Transmission Settings ---
        config_frame = ttk.LabelFrame(self.root, text=" Transmission Settings ", padding=12)
        config_frame.pack(fill="x", padx=20, pady=5)

        ttk.Label(config_frame, text="Repeats (Time Diversity):").grid(row=0, column=0, sticky="w", pady=5)
        repeat_spin = ttk.Spinbox(config_frame, from_=1, to=10, textvariable=self.repeat_count, width=5)
        repeat_spin.grid(row=0, column=1, sticky="w", padx=10, pady=5)
        ttk.Label(config_frame, text="(Repeats)", font=("Arial", 9, "italic"), foreground="#555").grid(row=0, column=2, sticky="w", padx=5, pady=5)

        ttk.Label(config_frame, text="Inter-frame gap (s):").grid(row=1, column=0, sticky="w", pady=5)
        gap_spin = ttk.Spinbox(config_frame, from_=0, to=30, increment=1, textvariable=self.gap_secs, width=5)
        gap_spin.grid(row=1, column=1, sticky="w", padx=10, pady=5)
        ttk.Label(config_frame, text="(Seconds)", font=("Arial", 9, "italic"), foreground="#555").grid(row=1, column=2, sticky="w", padx=5, pady=5)

        # --- Message Payload ---
        input_frame = ttk.LabelFrame(self.root, text=" Message Payload ", padding=12)
        input_frame.pack(fill="both", expand=True, padx=20, pady=10)

        ttk.Label(input_frame, text=f"Enter text to transmit (max {MAX_PAYLOAD_CHARS} chars):").pack(anchor="w", pady=(0, 2))

        self.char_count_var = tk.StringVar(value=f"0 / {MAX_PAYLOAD_CHARS}")
        char_count_label = ttk.Label(input_frame, textvariable=self.char_count_var, font=("Arial", 9), foreground="#666")
        char_count_label.pack(anchor="e", pady=(0, 2))

        # Light-mode palette matching FSKDemodIc's text view[cite: 1]
        self.payload_entry = tk.Text(input_frame, wrap="word", font=("Verdana", 11),
                                     background="#ffffff", foreground="#000000",
                                     insertbackground="black", relief="sunken", bd=1,
                                     height=4)
        self.payload_entry.pack(fill="both", expand=True, pady=5)
        self.payload_entry.bind("<KeyRelease>", self.update_char_count)

        # --- Execution Controls (Bottom Bar Alignment) ---
        btn_frame = ttk.Frame(self.root)
        btn_frame.pack(fill="x", padx=20, pady=(0, 10))

        self.clear_btn = ttk.Button(btn_frame, text="Clear", command=self.clear_display_pane)
        self.clear_btn.pack(side="left", ipadx=10, ipady=3)

        self.archive_btn = ttk.Button(btn_frame, text="Archive", command=self.archive_workspace_files)
        self.archive_btn.pack(side="left", ipadx=10, ipady=3, padx=(10, 0))

        # Primary action pinned right with ipadx=15[cite: 1]
        self.send_btn = ttk.Button(btn_frame, text="Transmit", command=self.process_and_send)
        self.send_btn.pack(side="right", ipadx=15, ipady=3)

        status_bar = ttk.Label(self.root, textvariable=self.status_msg, relief="sunken",
                               anchor="w", padding=(10, 8), font=("Arial", 11, "normal"))
        status_bar.pack(fill="x", side="bottom")

    def validate_callsign(self, text):
        if len(text) > 6:
            return False
        if text == "":
            return True
        return text.isalnum()

    def update_char_count(self, event=None):
        text = self.payload_entry.get("1.0", tk.END).strip()
        count = len(text)
        self.char_count_var.set(f"{count} / {MAX_PAYLOAD_CHARS}")
        if count > MAX_PAYLOAD_CHARS:
            self.char_count_var.set(f"{count} / {MAX_PAYLOAD_CHARS} \u26a0\ufe0f OVER")

    def clear_display_pane(self):
        self.payload_entry.delete("1.0", tk.END)
        self.char_count_var.set(f"0 / {MAX_PAYLOAD_CHARS}")
        self.status_msg.set("Display cleared.")

    def generate_morse_audio(self, callsign):
        dot_duration = 1.2 / MORSE_WPM 
        dash_duration = dot_duration * 3
        element_gap = dot_duration
        character_gap = dot_duration * 3
        
        morse_parts = []
        for idx, char in enumerate(callsign):
            if char not in MORSE_DICT:
                continue
            code = MORSE_DICT[char]
            for elem_idx, element in enumerate(code):
                duration = dot_duration if element == '.' else dash_duration
                num_samples = int(round(duration * SAMPLE_RATE))
                
                t = np.arange(num_samples) / SAMPLE_RATE
                tone = np.sin(2 * np.pi * MORSE_FREQ * t).astype(np.float32)
                
                ramp_samples = min(int(0.005 * SAMPLE_RATE), num_samples // 2)
                if ramp_samples > 0:
                    ramp = np.linspace(0, 1, ramp_samples, dtype=np.float32)
                    tone[:ramp_samples] *= ramp
                    tone[-ramp_samples:] *= ramp[::-1]
                    
                morse_parts.append(tone)
                if elem_idx < len(code) - 1:
                    morse_parts.append(np.zeros(int(round(element_gap * SAMPLE_RATE)), dtype=np.float32))
            
            if idx < len(callsign) - 1:
                morse_parts.append(np.zeros(int(round(character_gap * SAMPLE_RATE)), dtype=np.float32))
                
        return np.concatenate(morse_parts) if morse_parts else np.array([], dtype=np.float32)

    def archive_workspace_files(self):
        confirm = messagebox.askokcancel(title="Archive Confirmation",
                                         message="Move current fsk_*.wav / fsk_*.txt files to ./Archive?")
        if not confirm:
            self.status_msg.set("Archiving canceled.")
            return

        cwd = os.getcwd()
        archive_dir = os.path.join(cwd, "Archive")

        if not os.path.exists(archive_dir):
            try:
                os.makedirs(archive_dir)
            except Exception as e:
                self.status_msg.set(f"Archive error: {str(e)}")
                return

        wav_targets = glob.glob(os.path.join(cwd, "fsk_*.wav"))
        txt_targets = glob.glob(os.path.join(cwd, "fsk_*.txt"))
        all_targets = wav_targets + txt_targets

        if not all_targets:
            self.status_msg.set("Archive halted: no files to move.")
            return

        moved_count = errors = 0
        for file_path in all_targets:
            filename = os.path.basename(file_path)
            destination = os.path.join(archive_dir, filename)
            try:
                shutil.move(file_path, destination)
                moved_count += 1
            except Exception:
                errors += 1

        self.status_msg.set(f"Archived {moved_count} files. Errors: {errors}.")

    def reset_status_message(self):
        self.status_msg.set("Ready to send.")

    def _worker_thread(self, payload_bytes, repeats, gap_secs, callsign):
        def transmit_continuous(payload_audio, fs):
            total_duration = len(payload_audio) / fs
            if not PTT_PORT or not serial:
                self.root.after(0, lambda: self.status_msg.set("\U0001f50a Transmitting..."))
                sd.play(payload_audio, fs, device=PTT_DEVICE)
                time.sleep(total_duration + 0.1)
                self.root.after(0, lambda: self.status_msg.set("\u2705 Transmission complete."))
                self.root.after(5000, self.reset_status_message)
                return

            try:
                ser = serial.Serial()
                ser.port = PTT_PORT
                ser.timeout = 1
                ser.rts = False
                ser.dtr = False
                ser.open()
                ser.rts = True
                time.sleep(PTT_TXDELAY)

                self.root.after(0, lambda: self.status_msg.set("\U0001f50a PTT: Transmitting Payload & ID..."))
                sd.play(payload_audio, fs, device=PTT_DEVICE)
                time.sleep(total_duration)
                time.sleep(PTT_TAIL)
            except Exception as e:
                error_msg = str(e)
                self.root.after(0, lambda msg=error_msg: self.status_msg.set(f"\u26a0\ufe0f Serial Error: {msg}"))
            finally:
                sd.stop()
                try:
                    ser.rts = False
                    if ser.is_open:
                        ser.close()
                except:
                    pass
                self.root.after(0, lambda: self.status_msg.set("\u2705 Transmission complete."))
                self.root.after(5000, self.reset_status_message)

        try:
            self.root.after(0, lambda: self.status_msg.set("\u23f3 Generating compound wave..."))

            temp_txt = "temp_payload.txt"
            with open(temp_txt, "wb") as f:
                f.write(payload_bytes)

            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            output_wav = f"fsk_{timestamp}.wav"

            cmd = ["./fsk_mod", "-f", temp_txt, "-o", output_wav,
                   "-r", str(repeats), "-g", f"{gap_secs:.3f}"]
            result = subprocess.run(cmd, capture_output=True, text=True)

            if result.returncode != 0:
                raise Exception(f"C Engine Error: {result.stderr}")

            fs, frame = wav.read(output_wav)
            frame = frame.astype(np.float32) / 32767.0

            parts = [frame]
            
            if callsign:
                compliance_delay_len = int(round(POST_DATA_DELAY_SECS * SAMPLE_RATE))
                compliance_gap = np.zeros(compliance_delay_len, dtype=np.float32)
                parts.append(compliance_gap)
                
                morse_audio = self.generate_morse_audio(callsign)
                if morse_audio.size > 0:
                    parts.append(morse_audio)

            waveform_payload = np.concatenate(parts)
            total_secs = len(waveform_payload) / SAMPLE_RATE
            self.root.after(0, lambda d=total_secs, r=repeats:
                            self.status_msg.set(f"\u23f3 Waveform built ({r} copies + ID) -> {d:.1f}s total on air"))

            if os.path.exists(temp_txt):
                os.remove(temp_txt)

            time.sleep(0.5)
            transmit_continuous(waveform_payload, SAMPLE_RATE)

        except Exception as e:
            error_msg = str(e)
            self.root.after(0, lambda msg=error_msg: self.status_msg.set(f"\u26a0\ufe0f Error: {msg}"))
        finally:
            self.root.after(0, lambda: self.send_btn.config(state="normal"))
            self.root.after(0, lambda: self.clear_btn.config(state="normal"))
            self.root.after(0, lambda: self.archive_btn.config(state="normal"))

    def process_and_send(self):
        callsign = self.callsign_var.get().strip().upper()
        if not callsign:
            self.status_msg.set("\u26a0\ufe0f Compliance Error: Callsign field is required.")
            messagebox.showwarning("Missing Callsign", "Please enter a valid station callsign for Morse ID generation.")
            return

        raw_text = self.payload_entry.get("1.0", tk.END).strip()
        if not raw_text:
            self.status_msg.set("\u26a0\ufe0f Payload is empty.")
            return

        if len(raw_text) > MAX_PAYLOAD_CHARS:
            self.status_msg.set(f"\u26a0\ufe0f Payload too long ({len(raw_text)}/{MAX_PAYLOAD_CHARS} max).")
            return

        payload_bytes = raw_text.encode('utf-8')

        try:
            repeats = min(10, max(1, int(self.repeat_count.get())))
        except ValueError:
            repeats = 1

        try:
            gap_secs = max(0.0, float(self.gap_secs.get()))
        except ValueError:
            gap_secs = GAP_SECONDS

        self.send_btn.config(state="disabled")
        self.clear_btn.config(state="disabled")
        self.archive_btn.config(state="disabled")

        threading.Thread(
            target=self._worker_thread,
            args=(payload_bytes, repeats, gap_secs, callsign),
            daemon=True
        ).start()


if __name__ == "__main__":
    root = tk.Tk()
    app = FSKModulatorGUI(root)
    root.mainloop()
