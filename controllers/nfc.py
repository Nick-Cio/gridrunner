# Reads NFC tags at intersections. Background thread polls the reader.
import board
import busio
from adafruit_pn532.i2c import PN532_I2C

import threading, prctl
import time


class NFCSensor:

    def __init__(self):
        # Clear the last read data.
        self.last_read = None
        self.reading = False
        self.thread = None

        # TODO Initialize the hardware. If the reader is broken like it is right now  disable so NONE
        try:
            self.i2c = busio.I2C(board.SCL, board.SDA)
            self.pn532 = PN532_I2C(self.i2c, debug=False)
            self.pn532.SAM_configuration()
        except Exception as ex:
            print("NFC reader unavailable, disabling NFC: %s" % repr(ex))
            return

        # Start the worker thread.
        print("Starting NFC thread...")
        self.reading = True
        self.thread = threading.Thread(name="NFCThread", target=self.run)
        self.thread.start()

    def run(self):
        prctl.set_name(threading.current_thread().name)
        # Poll the reader every 50ms until told to stop
        while self.reading:
            # Returns None or a 7-byte UID
            try:
                uid = self.pn532.read_passive_target(timeout=0.2)
            except Exception:
                # skip this read and try again
                time.sleep(0.05)
                continue
            if uid is not None:
                self.last_read = '0x' + uid.hex()
            time.sleep(0.05)

    def read(self):
        # Grab the last seen tag and clear it so we don't reuse it next time
        data = self.last_read
        self.last_read = None
        return data

    def shutdown(self):
        # Stop the worker thread and wait for it to finish
        self.reading = False
        if self.thread is not None:
            print("Waiting for NFC thread to finish...")
            self.thread.join()
            print("NFC thread returned.")


if __name__ == "__main__":
    sensor = NFCSensor()

    try:
        while True:
            tag = sensor.read()
            if tag is not None:
                print(f"Tag: {tag}")
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass

    sensor.shutdown()
