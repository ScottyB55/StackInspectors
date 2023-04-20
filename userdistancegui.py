import PySimpleGUI as sg

def distance_gui():
    # Add some color
    # to the window
    sg.theme('SandyBeach')

    # Very basic window.
    # Return values using
    # automatic-numbered keys
    layout = [
        [sg.Text('Please enter target distance')],
        [sg.Text('Target Distance', size=(15, 1)), sg.InputText()],
        [sg.Submit(), sg.Cancel()]
    ]

    window = sg.Window('Distance Setter', layout)

    while True:
        event, values = window.read()
        if event == sg.WIN_CLOSED or event == 'Cancel':
            break
        return values

    window.close()