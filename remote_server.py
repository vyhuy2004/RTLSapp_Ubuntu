import socket
import signal
import sys
from servo.PCA9685 import PCA9685
from TrackerModule.tracker import LocationTracker

PORT = 8802 # Use for both tracker thread and servo
TILT_SERVO      = 0
PAN_SERVO       = 1

s = socket.socket()

def signal_handler(sig, frame):
    s.close()
    sys.exit()
    
def main():
    signal.signal(signal.SIGINT, signal_handler)
    tracker = LocationTracker(remote=True, address=('192.168.70.1', 8801))
    pwm = PCA9685()
    try:
        pwm.setPWMFreq(50)
        pwm.setRotationAngle(TILT_SERVO, 179)
        pwm.setRotationAngle(PAN_SERVO, 90)
    except:
        pwm.exit_PCA9685()
        print ("[CAMERA] Servo Initialization Failed!")
        
    # Create a socket
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    print("Socket successfully created")
    
    # Bind to the port 
    s.bind(('', PORT))       
    print(f"Socket binded to {PORT}") 

    # put the socket into listening mode 
    s.listen()  
    print("Socket is listening")         


    # a forever loop untill clien close connection
    while True: 
        # Establish connection with client. 
        c, addr = s.accept()     
        print(f'Got connection from {addr}') 
        c.send(b'Raspberry Pi Server Ready!')
        
        while True:
            data = c.recv(1024).decode()
            if data == '':
                c.close()
                break
            print(f'From Client: {data}')
            data = data.split()
            if data[0] == 'run':
                tracker.run()
            elif data[0] == 'pan':
                pwm.setRotationAngle(PAN_SERVO, int(data[1]))
            elif data[0] == 'tilt':
                pwm.setRotationAngle(TILT_SERVO, int(data[1]))        
            # Close the connection with the client 
            

if __name__== "__main__":
    main()
