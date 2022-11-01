import serial

port_name = 'SerialPortForPython'

sp = serial.Serial(port_name, 9600)

line = sp.read(6)
print(line)
sp.close()