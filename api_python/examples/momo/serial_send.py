import serial

port_name = 'SerialPortForPython'

sp = serial.Serial(port_name, 9600)

message = 'Test message from pyserial 3 to 2\n'

sp.write(message.encode('ascii'))
sp.close()