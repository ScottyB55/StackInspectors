import tkinter as tk
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

# Run from Terminal
# Prereq: pip install matplotlib

class LineDrawer(tk.Tk):
    def __init__(self):
        super().__init__()

        self.title('Line Drawer')
        self.geometry('400x300')

        self.create_widgets()

    def create_widgets(self):
        # Create a figure with a line
        fig = Figure(figsize=(4, 3), dpi=100)
        ax = fig.add_subplot(1, 1, 1)
        ax.plot([0, 1], [0, 1])

        # Create a canvas to display the figure
        canvas = FigureCanvasTkAgg(fig, master=self)
        canvas.draw()
        canvas.get_tk_widget().pack()

if __name__ == '__main__':
    app = LineDrawer()
    app.mainloop()