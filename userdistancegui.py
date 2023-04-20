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
        elif event == 'Submit':
            # Process the submitted data
            target_distance = float(values[0])  # Typecast input value to float
            print(f'Target Distance: {target_distance}')
            window.close()  # Close the window after processing the data
            return target_distance
    
if __name__ == '__main__':
    distance_gui()