#!/usr/bin/env python
# -*- coding: utf-8 -*-

# *******************************************************************************
#
# Animatronic Control Panel GUI
#
# A simple Tkinter interface to manage and run the animatronic scripts.
#
# *******************************************************************************

import tkinter as tk
from tkinter import ttk, scrolledtext
import os
import threading
import queue

import scanner
import torque_control
import position_saver
import sequencer


class AnimatronicsGUI:
    def __init__(self, master):
        self.master = master
        master.title("Animatronic Control Panel")
        master.geometry("800x600")

        self.script_queue = queue.Queue()
        self.create_widgets()

        # Run the initial scan in a separate thread to not freeze the GUI
        self.run_script_in_thread(scanner.run_scan, "Initial system scan complete.")
        self.master.after(100, self.process_queue)

    def create_widgets(self):
        # --- Main Layout ---
        main_frame = ttk.Frame(self.master, padding="10")
        main_frame.pack(fill=tk.BOTH, expand=True)

        top_frame = ttk.Frame(main_frame)
        top_frame.pack(fill=tk.X, pady=5)

        run_frame = ttk.LabelFrame(main_frame, text="Run Commands")
        run_frame.pack(fill=tk.BOTH, expand=True, pady=10)

        status_frame = ttk.LabelFrame(main_frame, text="Script Output")
        status_frame.pack(fill=tk.X, pady=5)

        # --- Top Control Widgets ---
        torque_frame = ttk.LabelFrame(top_frame, text="Torque Control")
        torque_frame.pack(side=tk.LEFT, padx=5, fill=tk.Y)

        ttk.Button(torque_frame, text="Torque ON",
                   command=lambda: self.run_script_in_thread(torque_control.set_torque, "on")).pack(padx=5, pady=5)
        ttk.Button(torque_frame, text="Torque OFF",
                   command=lambda: self.run_script_in_thread(torque_control.set_torque, "off")).pack(padx=5, pady=5)

        save_pos_frame = ttk.LabelFrame(top_frame, text="Save Current Position")
        save_pos_frame.pack(side=tk.LEFT, padx=5, fill=tk.Y)

        self.save_pos_entry = ttk.Entry(save_pos_frame, width=30)
        self.save_pos_entry.pack(side=tk.LEFT, padx=5, pady=10)
        ttk.Button(save_pos_frame, text="Save", command=self.save_position).pack(side=tk.LEFT, padx=5, pady=10)

        # --- Run Command Widgets ---
        run_lists_frame = ttk.Frame(run_frame)
        run_lists_frame.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        # Positions Column
        pos_col = ttk.Frame(run_lists_frame)
        pos_col.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=5)
        ttk.Label(pos_col, text="Positions (.json)").pack()
        self.pos_listbox = tk.Listbox(pos_col)
        self.pos_listbox.pack(fill=tk.BOTH, expand=True)
        self.pos_listbox.bind("<Double-1>", self.run_from_list)

        # Animations Column
        anim_col = ttk.Frame(run_lists_frame)
        anim_col.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=5)
        ttk.Label(anim_col, text="Animations (.toml)").pack()
        self.anim_listbox = tk.Listbox(anim_col)
        self.anim_listbox.pack(fill=tk.BOTH, expand=True)
        self.anim_listbox.bind("<Double-1>", self.run_from_list)

        ttk.Button(run_frame, text="Refresh Lists", command=self.populate_file_lists).pack(pady=5)

        # --- Status Output Widget ---
        self.status_text = scrolledtext.ScrolledText(status_frame, height=10, wrap=tk.WORD, state=tk.DISABLED)
        self.status_text.pack(fill=tk.BOTH, expand=True)

        self.populate_file_lists()

    def populate_file_lists(self):
        """Scans the position and animation directories and fills the listboxes."""
        self.pos_listbox.delete(0, tk.END)
        self.anim_listbox.delete(0, tk.END)

        # Populate positions
        if os.path.isdir("position"):
            for item in sorted(os.listdir("position")):
                if item.lower().endswith('.json'):
                    self.pos_listbox.insert(tk.END, item)

        # Populate animations
        if os.path.isdir("animation"):
            for item in sorted(os.listdir("animation")):
                if item.lower().endswith('.toml'):
                    self.anim_listbox.insert(tk.END, item)

    def run_from_list(self, event):
        """Runs an animation or pose file when double-clicked."""
        widget = event.widget
        selection_index = widget.curselection()
        if not selection_index: return

        filename = widget.get(selection_index[0])
        self.log_status(f"--- Running '{filename}' ---")
        self.run_script_in_thread(sequencer.run_animation, filename)

    def save_position(self):
        """Handles the logic for saving a position file."""
        base_name = self.save_pos_entry.get()
        if not base_name:
            self.log_status("[ERROR] Please enter a filename for the position.")
            return

        # Ensure the name ends with .json
        if not base_name.lower().endswith('.json'):
            base_name += '.json'

        output_dir = "position"
        os.makedirs(output_dir, exist_ok=True)

        # Find a unique filename
        output_path = os.path.join(output_dir, base_name)
        counter = 1
        while os.path.exists(output_path):
            name, ext = os.path.splitext(base_name)
            output_path = os.path.join(output_dir, f"{name}_{counter}{ext}")
            counter += 1

        self.log_status(f"--- Saving positions to '{output_path}' ---")
        self.run_script_in_thread(position_saver.save_current_positions, output_path)

        # Give a moment for the file to be written, then refresh
        self.master.after(500, self.populate_file_lists)

    def log_status(self, message):
        """Appends a message to the status text box."""
        self.status_text.config(state=tk.NORMAL)
        self.status_text.insert(tk.END, message + "\n")
        self.status_text.see(tk.END)
        self.status_text.config(state=tk.DISABLED)

    def run_script_in_thread(self, target_func, *args):
        """Runs a function in a separate thread to keep the GUI responsive."""
        thread = threading.Thread(target=target_func, args=(self.script_queue, *args))
        thread.daemon = True
        thread.start()

    def process_queue(self):
        """Processes messages from the script threads to update the GUI."""
        try:
            message = self.script_queue.get_nowait()
            self.log_status(message)
        except queue.Empty:
            pass
        finally:
            self.master.after(100, self.process_queue)


if __name__ == '__main__':
    root = tk.Tk()
    app = AnimatronicsGUI(root)
    root.mainloop()