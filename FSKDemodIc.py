#!/usr/bin/env python3
"""
FSKDemodIc.py - Viewer / decoder for the FSK receive chain.

    Capture and DSP now run unattended in the fsk_rxd daemon, which drops
    finished fsk_YYMMDD.hhmmss.wav files into the working directory (a
    "radio mailbox"). This GUI is just the front end for those WAVs:

        * Scan       -> list recent fsk_*.wav captures
        * Decode     -> run fsk_demod on the selected WAV (user-driven)
        * View       -> show the decode, and reload cached decodes on select
        * Archive    -> move fsk_*.wav / .txt / .log out to ./Archive

    fsk_demod runs on a worker thread so the UI stays live.

    LOOK & FEEL: styled to match FSKModIc.py -- same title/info header,
    LabelFrame sections with italic gray hints, green compatibility line,
    emoji status messages, and bottom button bar conventions.
"""

import os
import re
import glob
import queue
import shutil
import threading
import subprocess
import tkinter as tk
from tkinter import ttk, messagebox

# --- FSK Configuration (display-only; the C demodulator is authoritative) ---
FSK_BAUD = 100                  # symbols/sec (matches fsk_demod.c BAUD_RATE)
FSK_TONE_MARK = 1200            # mark tone (binary 1)
FSK_TONE_SPACE = 2400           # space tone (binary 0)
FSK_PREAMBLE_BITS = 128         # Longer preamble for better sync
FSK_SYNC_BITS = 16              # 16-bit sync

# Maximum number of recent captures shown in the dropdown
MAX_CAPTURES_SHOWN = 8

# Daemon capture filename: fsk_YYMMDD.hhmmss (UTC, dot separator).
TS_RE = re.compile(r"^fsk_(\d{6})\.(\d{6})$")
# Legacy pipeline filename: fsk_YYYYMMDD_HHMMSS (older captures).
TS_RE_LEGACY = re.compile(r"^fsk_(\d{8})_(\d{6})$")


class FSKViewerGUI:
    def __init__(self, root):
        self.root = root
        self.root.title(f"PALMETTO RX")
        self.root.geometry("650x550")
        self.root.minsize(600, 520)

        # Pin the GUI to LIGHT mode regardless of macOS system Dark Mode.
        self._force_light_theme()

        # ---- UI state ----
        self.status_msg      = tk.StringVar(value="Ready to view.")
        self.phase_var       = tk.StringVar(value="Idle.")
        self.verbose_enabled = tk.BooleanVar(value=False)
        self.selected_file   = tk.StringVar()
        self.line_count_var  = tk.StringVar(value="0 lines")

        self.display_to_filename = {}
        self.labels = {  # legacy test-harness impairment levels (old captures)
            "lvl0": "Clean (None)",
            "lvl1": "L1: In-band CW",
            "lvl2": "L2: AWGN Floor",
            "lvl3": "L3: Inter-bin CW",
            "lvl4": "L4: Drift + Clicks",
            "lvl5": "L5: Multipath Fade",
        }

        # ---- Thread / process state ----
        self._busy           = False
        self.current_proc    = None
        self._proc_lock      = threading.Lock()
        self.msg_queue       = queue.Queue()

        self.build_ui()
        self.refresh_file_list()
        self.root.after(100, self._poll_queue)

    # =====================================================================
    #  Theme lock (light mode)
    # =====================================================================
    def _force_light_theme(self):
        """Force a light appearance even when macOS is in system Dark Mode.

        Path 1 (preferred, macOS Tk 8.6.10+): ask Tk for the native light
        'aqua' appearance on this window. Widgets keep their native mac look.

        Path 2 (fallback, any platform / older Tk): switch to the 'clam'
        theme and paint an explicit light palette so nothing inherits dark
        system colors.
        """
        try:
            self.root.tk.call("::tk::unsupported::MacWindowStyle",
                              "appearance", self.root, "aqua")
            return  # Native light aqua achieved; nothing else needed.
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

    # =====================================================================
    #  UI construction
    # =====================================================================
    def build_ui(self):

        info_label = ttk.Label(self.root,
                               text=f"Baud: {FSK_BAUD}  |  Mark: {FSK_TONE_MARK}Hz  |  Space: {FSK_TONE_SPACE}Hz  |  Bell 202 Compatible",
                               font=("Arial", 10), foreground="#0066cc")
        info_label.pack(pady=(0, 10))

        # Enforce unified widget layout and color behaviors via TTK Style
        try:
            style = ttk.Style()
            style.configure("TLabelFrame", font=("Arial", 10, "bold"), foreground="#2c3e50")
        except Exception:
            pass

        # --- Capture selection row (no labeled box) ---
        mailbox_frame = ttk.Frame(self.root, padding=(0, 5))
        mailbox_frame.pack(fill="x", padx=20, pady=5)

        ttk.Label(mailbox_frame, text="Available Captures:",
                  font=("Arial", 10, "bold")).grid(row=0, column=0, sticky="w", pady=5)

        self.file_dropdown = ttk.Combobox(mailbox_frame, textvariable=self.selected_file,
                                          state="readonly", font=("Arial", 10), width=42)
        self.file_dropdown.grid(row=0, column=1, sticky="w", padx=10, pady=5)
        self.file_dropdown.bind("<<ComboboxSelected>>", lambda e: self.on_select())

        self.refresh_btn = ttk.Button(mailbox_frame, text="Scan", command=self.refresh_file_list)
        self.refresh_btn.grid(row=0, column=2, sticky="w", padx=5, pady=5)

        ttk.Label(mailbox_frame,
                  text=f"(Captures,newest {MAX_CAPTURES_SHOWN} shown)",
                  font=("Arial", 9, "italic"), foreground="#555").grid(
                      row=1, column=0, columnspan=3, sticky="w", pady=(0, 2))

        # --- Decoder Pipeline Configuration ---
        config_frame = ttk.LabelFrame(self.root, text=" Decoder Settings ", padding=12)
        config_frame.pack(fill="x", padx=20, pady=5)

        ttk.Checkbutton(config_frame, text="Verbose Diagnostics",
                        variable=self.verbose_enabled).grid(row=0, column=0, sticky="w", pady=5)
        ttk.Label(config_frame, text="(passes -v to fsk_demod for full pipeline trace)",
                  font=("Arial", 9, "italic"), foreground="#555").grid(
                      row=0, column=1, sticky="w", padx=5, pady=5)

        self.progress = ttk.Progressbar(config_frame, mode="determinate", maximum=100)
        self.progress.grid(row=1, column=0, columnspan=2, sticky="ew", pady=(10, 4))
        config_frame.columnconfigure(1, weight=1)

        ttk.Label(config_frame, textvariable=self.phase_var,
                  font=("Arial", 9, "italic"), foreground="#555").grid(
                      row=2, column=0, columnspan=2, sticky="w")

        line_count_label = ttk.Label(self.root, textvariable=self.line_count_var,
                                     font=("Arial", 9), foreground="#666")
        line_count_label.pack(anchor="e", padx=20, pady=(0, 2))

        # Light-mode palette matching FSKModIc's input view
        self.text_pane = tk.Text(self.root, wrap="word", font=("Verdana", 11),
                                 background="#ffffff", foreground="#000000",
                                 insertbackground="black", relief="sunken", bd=1,
                                 state="disabled", height=6)
        self.text_pane.pack(fill="both", expand=True, padx=20, pady=5)

        # --- Execution Controls (Bottom Bar Alignment) ---
        btn_frame = ttk.Frame(self.root)
        btn_frame.pack(fill="x", padx=20, pady=(0, 10))

        self.clear_btn = ttk.Button(btn_frame, text="Clear", command=self.clear_display_pane)
        self.clear_btn.pack(side="left", ipadx=10, ipady=3)

        self.archive_btn = ttk.Button(btn_frame, text="Archive", command=self.archive_workspace_files)
        self.archive_btn.pack(side="left", ipadx=10, ipady=3, padx=(10, 0))

        # Primary action pinned right with ipadx=15, matching Transmit in FSKModIc
        self.decode_btn = ttk.Button(btn_frame, text="Decode", command=self.start_decode)
        self.decode_btn.pack(side="right", ipadx=15, ipady=3)

        status_bar = ttk.Label(self.root, textvariable=self.status_msg, relief="sunken",
                               anchor="w", padding=(10, 8), font=("Arial", 11, "normal"))
        status_bar.pack(fill="x", side="bottom")

    # =====================================================================
    #  Text pane helpers
    # =====================================================================
    def _update_line_count(self):
        # end-1c excludes the trailing implicit newline Tk always keeps
        content = self.text_pane.get("1.0", "end-1c")
        count = len(content.splitlines()) if content else 0
        self.line_count_var.set(f"{count} lines")

    def _append_pane(self, text):
        self.text_pane.config(state="normal")
        self.text_pane.insert(tk.END, text + "\n")
        self.text_pane.see(tk.END)
        self.text_pane.config(state="disabled")
        self._update_line_count()

    def _set_pane(self, text):
        self.text_pane.config(state="normal")
        self.text_pane.delete("1.0", tk.END)
        if text:
            self.text_pane.insert(tk.END, text)
        self.text_pane.config(state="disabled")
        self._update_line_count()

    def clear_display_pane(self):
        self._set_pane("")
        self.status_msg.set("Display cleared.")

    # =====================================================================
    #  File list
    # =====================================================================
    def _friendly_name(self, basename):
        stem = os.path.splitext(basename)[0]

        # legacy impairment-labelled captures
        parts = stem.split("_")
        if len(parts) >= 4 and parts[3] in self.labels:
            return f"{parts[0]}_{parts[1]}_{parts[2]} ({self.labels[parts[3]]})"

        # daemon format: fsk_YYMMDD.HHMMSS (UTC)
        m = TS_RE.match(stem)
        if m:
            d, t = m.group(1), m.group(2)
            pretty = f"20{d[0:2]}-{d[2:4]}-{d[4:6]} {t[0:2]}:{t[2:4]}:{t[4:6]} UTC"
            return f"{stem}  ({pretty})"

        # legacy pipeline format: fsk_YYYYMMDD_HHMMSS
        m = TS_RE_LEGACY.match(stem)
        if m:
            d, t = m.group(1), m.group(2)
            pretty = f"{d[0:4]}-{d[4:6]}-{d[6:8]} {t[0:2]}:{t[2:4]}:{t[4:6]}"
            return f"{stem}  ({pretty})"

        return basename

    def refresh_file_list(self):
        if self._busy:
            return
        found = glob.glob(os.path.join(os.getcwd(), "fsk_*.wav"))
        if not found:
            self.file_dropdown.config(values=["No captures found (fsk_*.wav)"])
            self.file_dropdown.set("No captures found (fsk_*.wav)")
            self.status_msg.set("Scan complete: no fsk_*.wav in working directory.")
            return

        found.sort(key=os.path.getmtime, reverse=True)
        self.display_to_filename.clear()
        options = []
        for f in found[:MAX_CAPTURES_SHOWN]:
            base = os.path.basename(f)
            disp = self._friendly_name(base)
            options.append(disp)
            self.display_to_filename[disp] = base

        self.file_dropdown.config(values=options)
        self.file_dropdown.set(options[0])
        self.status_msg.set(f"Scan complete. {len(options)} recent capture(s).")

    def on_select(self):
        """Selecting a file shows a cached decode if one exists; never re-runs."""
        disp = self.file_dropdown.get()
        if not disp or "No captures" in disp:
            return
        wav = self.display_to_filename.get(disp, disp)
        txt = os.path.splitext(wav)[0] + ".txt"
        path = os.path.join(os.getcwd(), txt)
        if os.path.exists(path):
            try:
                with open(path, "r", encoding="utf-8") as fh:
                    self._set_pane(fh.read())
                self.status_msg.set(f"Loaded cached decode: {txt}")
                return
            except Exception:
                pass
        self._set_pane("")
        self.status_msg.set(f"Selected {wav}. Click Decode.")

    # =====================================================================
    #  Busy-state / progress
    # =====================================================================
    def _set_busy(self, busy):
        self._busy = busy
        en = "disabled" if busy else "normal"
        self.decode_btn.config(state=en)
        self.refresh_btn.config(state=en)
        self.archive_btn.config(state=en)
        self.clear_btn.config(state=en)
        self.file_dropdown.config(state="disabled" if busy else "readonly")

    def _render_phase(self):
        self.phase_var.set(self._phase_base if hasattr(self, "_phase_base") else "Idle.")

    def _start_progress_indeterminate(self):
        self._progress_running = True
        self.progress.config(mode="indeterminate")
        self.progress.start(15)

    def _stop_progress(self, complete):
        self._progress_running = False
        self.progress.stop()
        self.progress.config(mode="determinate")
        self.progress["value"] = 100 if complete else 0

    # =====================================================================
    #  Subprocess runner (worker-thread side)
    # =====================================================================
    def _run_proc(self, cmd, name, stream=True):
        """Launch cmd, capture combined stdout/stderr, and wait.
        If stream is True, lines are echoed live to the pane as they arrive;
        if False, output is captured silently for post-processing.
        Returns (returncode, captured_text)."""
        try:
            proc = subprocess.Popen(cmd, stdout=subprocess.PIPE,
                                    stderr=subprocess.STDOUT, text=True, bufsize=1)
        except FileNotFoundError:
            self.msg_queue.put(("error",
                f"{name} not found - is ./{name} compiled and in this directory?"))
            return (-1, "")
        except Exception as e:
            self.msg_queue.put(("error", f"Could not launch {name}: {e}"))
            return (-1, "")

        with self._proc_lock:
            self.current_proc = proc

        lines = []
        try:
            for line in proc.stdout:
                line = line.rstrip("\n")
                lines.append(line)
                if stream:
                    self.msg_queue.put(("log", line))
        except Exception:
            pass
        proc.wait()

        with self._proc_lock:
            self.current_proc = None

        return (proc.returncode, "\n".join(lines))

    # =====================================================================
    #  Decode (user-driven, threaded)
    # =====================================================================
    @staticmethod
    def _extract_payload(text):
        """Pull just the decoded payload out of fsk_demod's full report.

        The report contains a '--- Raw Payload ---' marker; the payload is
        everything from there until the next '---'/'====' section marker.
        Falls back to the full text if the marker isn't found (e.g. the
        demod aborted before printing a payload)."""
        lines = text.splitlines()
        try:
            start = next(i for i, l in enumerate(lines)
                         if l.strip() == "--- Raw Payload ---")
        except StopIteration:
            return text
        payload = []
        for l in lines[start + 1:]:
            s = l.strip()
            if s.startswith("---") or s.startswith("===="):
                break
            payload.append(l)
        result = "\n".join(payload).strip()
        return result if result else text

    def start_decode(self):
        if self._busy:
            return
        disp = self.file_dropdown.get()
        if not disp or "No captures" in disp:
            messagebox.showinfo("No file",
                "No capture selected. Click Scan first.")
            return
        wav = self.display_to_filename.get(disp, disp)
        if not os.path.exists(os.path.join(os.getcwd(), wav)):
            messagebox.showerror("Missing file", f"{wav} not found in working directory.")
            return

        self._set_busy(True)
        self._set_pane("")
        self._phase_base = "Decoding WAV..."
        self._render_phase()
        self._start_progress_indeterminate()
        self.status_msg.set(f"\u23f3 Decoding {wav}...")

        threading.Thread(target=self._decode_worker, args=(wav,), daemon=True).start()

    def _decode_worker(self, wav):
        try:
            verbose = self.verbose_enabled.get()
            cmd = ["./fsk_demod", wav]
            if verbose:
                cmd.append("-v")
                self.msg_queue.put(("log", "[*] " + " ".join(cmd)))

            # Verbose: stream the full report live. Normal: capture silently
            # and show only the extracted payload when the decode finishes.
            rc, text = self._run_proc(cmd, "fsk_demod", stream=verbose)
            # Exit code 2 = payload recovered but CRC mismatch. The payload is
            # still the display product; the CRC verdict belongs to the -v
            # report, not the payload pane. Anything else nonzero is a real
            # failure (no WAV, squelch unbroken, no sync) and stays an error.
            crc_failed = (rc == 2)
            if rc != 0 and not crc_failed:
                if not verbose and text:
                    self.msg_queue.put(("set", text))  # show report for debugging
                self.msg_queue.put(("error", f"fsk_demod exited with code {rc}")); return

            display = text if verbose else self._extract_payload(text)
            if not verbose:
                self.msg_queue.put(("set", display))

            # Cache exactly what was displayed, so re-selecting the WAV
            # reloads the same view.
            txt = os.path.splitext(wav)[0] + ".txt"
            try:
                with open(os.path.join(os.getcwd(), txt), "w", encoding="utf-8") as fh:
                    fh.write(display)
                self.msg_queue.put(("done_decode", txt))
            except Exception as e:
                self.msg_queue.put(("log", f"[!] Could not write decode cache: {e}"))
                self.msg_queue.put(("done_decode", None))
            # Status strip only -- the payload pane stays clean. Queued after
            # done_decode so it overrides that handler's status text; the 5 s
            # reset timer scheduled there still clears it.
            if crc_failed:
                self.msg_queue.put(("status",
                    "\u26a0\ufe0f Decoded, CRC mismatch (details via Verbose)."))
        except Exception as e:
            self.msg_queue.put(("error", f"Decode fault: {e}"))
        finally:
            with self._proc_lock:
                self.current_proc = None

    # =====================================================================
    #  Queue pump (main thread): all widget updates happen here
    # =====================================================================
    def _poll_queue(self):
        try:
            while True:
                tag, payload = self.msg_queue.get_nowait()
                if tag == "log":
                    self._append_pane(payload)
                elif tag == "set":
                    self._set_pane(payload)
                elif tag == "status":
                    self.status_msg.set(payload)
                elif tag == "phase":
                    self._phase_base = payload
                    self._render_phase()
                elif tag == "done_decode":
                    self._on_decode_done(payload)
                elif tag == "error":
                    self._on_error(payload)
        except queue.Empty:
            pass
        self.root.after(100, self._poll_queue)

    def reset_status_message(self):
        self.status_msg.set("Ready to view.")

    def _on_decode_done(self, txt):
        self._stop_progress(complete=True)
        self._phase_base = "Decode complete."
        self._render_phase()
        self._set_busy(False)
        if txt:
            self.status_msg.set(f"\u2705 Decode complete. Cached {txt}.")
        else:
            self.status_msg.set("\u2705 Decode complete (cache not written).")
        self.root.after(5000, self.reset_status_message)

    def _on_error(self, msg):
        self._stop_progress(complete=False)
        self._phase_base = "Error."
        self._render_phase()
        self._set_busy(False)
        self._append_pane(f"\n[!] {msg}")
        self.status_msg.set("\u26a0\ufe0f Error - see output.")

    # =====================================================================
    #  Archive
    # =====================================================================
    def archive_workspace_files(self):
        if self._busy:
            return
        if not messagebox.askokcancel("Archive Confirmation",
                "Move current fsk_*.wav / fsk_*.txt / fsk_*.log files to ./Archive?"):
            self.status_msg.set("Archiving canceled.")
            return

        archive_dir = os.path.join(os.getcwd(), "Archive")
        try:
            os.makedirs(archive_dir, exist_ok=True)
        except Exception as e:
            self.status_msg.set(f"Archive init error: {e}")
            return

        targets = (glob.glob(os.path.join(os.getcwd(), "fsk_*.wav")) +
                   glob.glob(os.path.join(os.getcwd(), "fsk_*.txt")) +
                   glob.glob(os.path.join(os.getcwd(), "fsk_*.log")))
        if not targets:
            self.status_msg.set("Archive halted: no files to move.")
            return

        moved = errors = 0
        for path in targets:
            try:
                shutil.move(path, os.path.join(archive_dir, os.path.basename(path)))
                moved += 1
            except Exception:
                errors += 1

        self.clear_display_pane()
        self.refresh_file_list()
        self.status_msg.set(f"Archived {moved} files. Errors: {errors}.")


if __name__ == "__main__":
    root = tk.Tk()
    app = FSKViewerGUI(root)
    root.mainloop()
