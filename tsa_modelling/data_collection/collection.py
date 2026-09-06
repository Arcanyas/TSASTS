from pasco import PASCOBLEDevice

cart = PASCOBLEDevice()
devices = cart.scan()
cart.connect(devices[0])

if devices:
    print("Devices found:")
    for i, device in enumerate(devices):
        print(f"  [{i}] {device.name}")   # prints e.g. "Smart Cart 123-456"
else:
    print("No devices found")

print(cart.get_measurement_list())

for i in range(100):
    sensor_readings = cart.read_data('Position')
    print(sensor_readings)

cart.disconnect()