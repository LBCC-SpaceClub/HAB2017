import serial.tools.list_ports
ports = list(serial.tools.list_ports.comports())

arduino = []
servos = []

print("All ports:")
for p in ports:
    print(p)
    if 'Pololu' in p[1]:
        servos += (p)
    elif 'Arduino' in p[1]:
        arduino += (p)

print("But your best bets are:")
for a in arduino:
    print(a)
print()

for s in servos:
    print(s)
