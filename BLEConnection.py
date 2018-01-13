import pygatt      
import binascii
import time
import threading



# Many devices, e.g. Fitbit, use random addressing - this is required to connect.
ADDRESS_TYPE = pygatt.BLEAddressType.public
DEVICE_ADDRESS = "34:15:13:87:DD:D8"


class writeEvent(threading.Event):
    def __init__(self):
        self.write_str="Empty Event"
        threading.Event.__init__(self)

    def set_str(self, str):
        self.write_str = str

#counter = 0

def write_pos(e, device, uuid):
    global counter
    while not e.isSet():
#        print("waiting for write to arduino events...")
        try:
            event_is_set = e.wait(2)
        except KeyboardInterrupt:
            break
        if event_is_set:
            str = e.write_str
#            print("Start writing " + str)
            in_buf = list(map(ord, str))
            device.char_write(uuid, in_buf[:20])
            device.char_write(uuid, in_buf[20:])
            e.clear()
            print("[INFO] Finish processing writing event, event cleared, event set: %s"%(event_is_set))
#            counter = 0
#        else:
#            counter += 1
#            if counter > 8:
#                break
        

class myThread (threading.Thread):
    def __init__(self, event, threadID=0, address="34:15:13:87:DD:D8", addr_type=pygatt.BLEAddressType.public):
        #threading.Thread.__init__(self)
        self.address = address
        self.type = addr_type
        self.write_uuid = "0000ffe2-0000-1000-8000-00805f9b34fb"
        try:
            self.adapter = pygatt.GATTToolBackend()
            self.adapter.start()
            print("===== adapter.connect() =====")
            self.device = self.adapter.connect(address, address_type=addr_type)
            print("[INFO] address: " + str(self.device._address))
        except:
            print("[Error] Cannot start adapter")
        print("[INFO] myThread Initialized successful!!")
        threading.Thread.__init__(self, target=write_pos, args=(event, self.device, self.write_uuid))
        self.threadID = threadID
        self.name = "device connection thread"

        #return device


def indication_callback(handle, value):
    print("indication, handle %d: %s " % (handle, value))

def handle_data(handle, value):
    """
    handle -- integer, characteristic read handle the data was received on
    value -- bytearray, the data returned in the notification
    """
    print("Received data: %s" % hexlify(value))


def main():
    #write_uuid = "0000ffe2-0000-1000-8000-00805f9b34fb"

    e = writeEvent()

    thread = myThread(e)
    thread.start()
    device = thread.device
    print("====== device.char_write_handle() =====")
    """
    F<forward cycle>B<backwardcycle>L<1:turn left>R<1:turn right>[p:pick up]
    """
    e.set_str("hello!F0050L1B0020R1!!!")
    
    if device != None:
        e.set()
        #write_pos(device, write_uuid, write_str)
    else:
        print("Device is None!!!")

    for i in range(5):
        time.sleep(12)
        #write_str = "smhX%d010Y%d525"%(i, i)
        e.set_str("F00%d0B00%d0L%dR%dP\n"%(i+2, i, i%2, (i+1)%2))
        #write_pos(device, write_uuid, write_str)
        e.set()

    thread.join()
    adapter.stop()


if __name__ == "__main__":
    main()

