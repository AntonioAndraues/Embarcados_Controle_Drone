import pyautogui
import serial
import argparse
import time
import logging

class MyControllerMap:
    def __init__(self):
        self.button = {'A': 'L', 'B': 'X', 'C': 'Z', 'D': 'J'} # Fast forward (10 seg) pro Youtube
    

class SerialControllerInterface:
    # Protocolo
    # byte 1 -> Botão 1 (estado - Apertado 1 ou não 0)
    # byte 2 -> EOP - End of Packet -> valor reservado 'X'

    def __init__(self, port, baudrate):
        self.ser = serial.Serial(port, baudrate=baudrate)
        self.mapping = MyControllerMap()
        self.incoming = '0'
        pyautogui.PAUSE = 0  ## remove delay
    
    def update(self):
        ## Sync protocol
        while self.incoming != b'X':
            self.incoming = self.ser.read()
            logging.debug("Received INCOMING: {}".format(self.incoming))

        data = self.ser.read()
        logging.debug("Received DATA: {}".format(data))
        if data == b'1':
            logging.info("KEYDOWN A")
            pyautogui.keyDown(self.mapping.button['A'])
        elif data == b'0':
            logging.info("KEYUP A")
            pyautogui.keyUp(self.mapping.button['A'])

        data = self.ser.read()

        logging.debug("Received DATA: {}".format(data))
        if data == b'2':
            logging.info("KEYDOWN B")
            pyautogui.keyDown(self.mapping.button['B'])
        elif data == b'3':
            logging.info("KEYUP B")
            pyautogui.keyUp(self.mapping.button['B'])

        data = self.ser.read()

        logging.debug("Received DATA: {}".format(data))
        if data == b'4':
            logging.info("KEYDOWN C")
            pyautogui.keyDown(self.mapping.button['C'])
        elif data == b'5':
            logging.info("KEYUP C")
            pyautogui.keyUp(self.mapping.button['C'])

        data = self.ser.read()

        logging.debug("Received DATA: {}".format(data))
        if data == b'6':
            logging.info("KEYDOWN D")
            pyautogui.keyDown(self.mapping.button['D'])
        elif data == b'7':
            logging.info("KEYUP D")
            pyautogui.keyUp(self.mapping.button['D'])

        self.incoming = self.ser.read()


class DummyControllerInterface:
    def __init__(self):
        self.mapping = MyControllerMap()

    def update(self):
        pyautogui.keyDown(self.mapping.button['A'])
        time.sleep(0.1)
        pyautogui.keyUp(self.mapping.button['A'])
        logging.info("[Dummy] Pressed A button")
        time.sleep(1)


if __name__ == '__main__':
    interfaces = ['dummy', 'serial']
    argparse = argparse.ArgumentParser()
    argparse.add_argument('serial_port', type=str)
    argparse.add_argument('-b', '--baudrate', type=int, default=9600)
    argparse.add_argument('-c', '--controller_interface', type=str, default='serial', choices=interfaces)
    argparse.add_argument('-d', '--debug', default=False, action='store_true')
    args = argparse.parse_args()
    if args.debug:
        logging.basicConfig(level=logging.DEBUG)

    print("Connection to {} using {} interface ({})".format(args.serial_port, args.controller_interface, args.baudrate))
    if args.controller_interface == 'dummy':
        controller = DummyControllerInterface()
    else:
        controller = SerialControllerInterface(port=args.serial_port, baudrate=args.baudrate)

    while True:
        controller.update()
