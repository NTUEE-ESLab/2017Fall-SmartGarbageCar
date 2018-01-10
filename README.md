# 2017Fall-SmartGarbageCar

## Usage

set rpi address
```
DEVICE_ADDRESS = < rpi address >
```
writing value to GATT characteristic event instance
```
e = writeEvent()
```
connect arduino through HM-10 module
```
thread = myThread(e, DEVICE_ADDRESS)
thread.start()
```
control car:

go forward: F + 4 bits integer (e.g. F0050: go forward for 5000 ms)

go backward: B + 4 bits integer

turn left: L1

turn right: R1

pick up garbage: P

e.g. F0050B0020L0R1P == forward for 5000ms -> backward for 2000ms -> turn right -> pick garbage

```
str = "F0050B0020L0R1P"
e.set_str(str)
e.set()
```
