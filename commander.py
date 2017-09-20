# PIAPA Robot
# Grman Rodriguez
# Raspberry-Motors Connection
# Phase 2, TCP connection
# --------------------------------------------------
# Libraries
import socket  # library for TCP connection
import msvcrt as noEnter  # library for receiving user input without needing them to press Enter
# --------------------------------------------------
# --------------------------------------------------
# Main function


def main():
    IP = '52.14.138.255'  # Raspberry Pi's IP address
    PORT = 12345  # Communication will be done through port 12345
    # Initialize socket object
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    # Connect with Raspberry socket
    s.connect((IP, PORT))
    # Main loop, which receives input from the computer and sends it to the raspberry
    # The rover will be controlled with WASD, Q will make it stop. More commands to be added later
    while 1:
        print("Next command: ")
        m = noEnter.getch()
        if m == "w" or m == "a" or m == "s" or m == "d" or m == "q":
            s.send(m)
            print("Command registered.")
        else:
            print("Invalid command.")


main()
