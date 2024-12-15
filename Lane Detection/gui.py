import tkinter as tk
from tkinter import Label, Frame
import cv2
from PIL import Image, ImageTk
import motor as mt
from processor import lane_following

class CameraApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Camera App")
        self.root.geometry("1000x800")
        self.started = False

        # Bind Escape key to close the app
        self.root.bind('<Escape>', lambda e: self.on_closing())

        # Configure root layout
        self.root.grid_rowconfigure(0, weight=1)
        self.root.grid_columnconfigure(1, weight=1)

        # Button frame
        button_frame = Frame(root, width=100)
        button_frame.grid(row=0, column=0, rowspan=2, padx=10, pady=10, sticky="ns")

        self.start_button = tk.Button(button_frame, text="Start", command=self.start)
        self.start_button.pack(pady=5)

        self.stop_button = tk.Button(button_frame, text="Stop", command=self.stop)
        self.stop_button.pack(pady=5)

        # Video frame grid
        video_frame = Frame(root)
        video_frame.grid(row=0, column=1, rowspan=2, padx=10, pady=10, sticky="nsew")

        # Configure grid for equal distribution
        for i in range(2):
            video_frame.grid_rowconfigure(i, weight=1)
            video_frame.grid_columnconfigure(i, weight=1)

        # Labels for video feeds
        self.video_labels = []
        self.cap = cv2.VideoCapture(0)
        self.running = True

        for i in range(4):
            label = Label(video_frame, bg="black")
            label.grid(row=i // 2, column=i % 2, padx=5, pady=5, sticky="nsew")
            self.video_labels.append(label)

        # Start video stream
        self.update_video()

    def update_video(self):
        if self.running:
            ret, frame = self.cap.read()
            if ret:
                # Process frames
                (direction, new_frame, canny, line_image, mask, combined_binary) = lane_following(frame, self.started)
                toShow = [new_frame, canny, line_image, combined_binary]

                for i, label in enumerate(self.video_labels):
                    if i < len(toShow):
                        # Use a fixed size if width/height not yet available
                        default_width = 480
                        default_height = 360

                        stretched_frame = cv2.resize(
                            toShow[i], (default_width, default_height), interpolation=cv2.INTER_LINEAR
                        )
                        stretched_frame = cv2.cvtColor(stretched_frame, cv2.COLOR_BGR2RGB)
                        img = Image.fromarray(stretched_frame)
                        imgtk = ImageTk.PhotoImage(image=img)
                        label.imgtk = imgtk
                        label.configure(image=imgtk)

        # Continue updating
        self.root.after(10, self.update_video)


    def start(self):
        self.started = True

    def stop(self):
        self.started = False

    def on_closing(self):
        mt.stop()
        self.running = False
        self.cap.release()
        self.root.destroy()

# Main
if __name__ == "__main__":
    root = tk.Tk()
    app = CameraApp(root)
    root.protocol("WM_DELETE_WINDOW", app.on_closing)
    root.mainloop()
