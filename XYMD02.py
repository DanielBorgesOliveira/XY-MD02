#!/usr/bin/env python
# -*- coding: utf-8 -*-
 
# pip install pymodbus

from __future__ import division
import pymodbus
import serial
from pymodbus.pdu import ModbusRequest
from pymodbus.client.sync import ModbusSerialClient as ModbusClient #initialize a serial RTU client instance
from pymodbus.transaction import ModbusRtuFramer
 
from pymodbus.constants import Endian              # Nodig voor 32-bit float getallen (2 registers / 4 bytes)
from pymodbus.payload import BinaryPayloadDecoder  # Nodig voor 32-bit float getallen (2 registers / 4 bytes)
from pymodbus.payload import BinaryPayloadBuilder  # Nodig om 32-bit floats te schrijven naar register

import time
import MySQLdb
import os
import subprocess

class XYMD02:
    def __init__(
        self,
        DeviceAddress,
        client,
    ):
        self.DeviceAddress = DeviceAddress
        self.client = client
    
    def ReadTemp(self):
        
        address=0x0001
        count=1
        
        data = self.client.read_input_registers(address=address, count=count, unit=self.DeviceAddress)
        if len(data.registers) > 1:
            return sum(data.registers)
        else:
            return data.registers[0]/10.
    
    def ReadHumidity(self):
        
        address=0x0002
        count=1
        
        data = self.client.read_input_registers(address=address, count=count, unit=self.DeviceAddress)
        if len(data.registers) > 1:
            return sum(data.registers)
        else:
            return data.registers[0]/10.
    
    def ReadDeviceAdress(self):
        
        address=0x0101
        count=1
        
        data = self.client.read_holding_registers(address=address, count=count, unit=self.DeviceAddress)
        if len(data.registers) > 1:
            return sum(data.registers)
        else:
            return data.registers[0]
    
    def ReadBaudRate(self):
        
        address=0x0102
        count=1
        
        data = self.client.read_holding_registers(address=address, count=count, unit=self.DeviceAddress)
        if len(data.registers) > 1:
            return sum(data.registers)
        else:
            return data.registers[0]
    
    def ReadTemperatureCorrection(self):
        
        address=0x0103
        count=1
        
        data = self.client.read_holding_registers(address=address, count=count, unit=self.DeviceAddress)
        if len(data.registers) > 1:
            return sum(data.registers)
        else:
            return data.registers[0]
        
    def ReadHumidityCorrection(self):
        
        address=0x0104
        count=1
        
        data = self.client.read_holding_registers(address=address, count=count, unit=self.DeviceAddress)
        if len(data.registers) > 1:
            return sum(data.registers)
        else:
            return data.registers[0]
        
    def WriteDeviceAdress(self):
        
        address=0x0101 # Adress of the register
        NewDeviceAddress=0x02 # New device address
        self.client.write_register(address = address, value = NewDeviceAddress, unit = self.DeviceAddress)

method = "rtu"
baudrate = 9600
stopbits = 1
bytesize = 8
parity = "N"
timeout = 1
retries = 2

port = ''
client = ModbusClient()

while True:
    time.sleep(3)
   
    if not os.path.exists(port):
        for i in range(10):
            if os.path.exists('/dev/ttyUSB'+str(i)):
                port = ('/dev/ttyUSB'+str(i))
                break
    
    if not client.is_socket_open():
        try:
            client = ModbusClient(
                method = method,
                port = port,
                stopbits = stopbits,
                bytesize = bytesize,
                parity = parity,
                baudrate = baudrate,
                timeout = timeout,
                retries = retries
            )
            connection = client.connect()
            device02 = XYMD02(DeviceAddress = 0x02, client = client)
        except:
            print "Modbus connectie error / DDS238-1 ZN"
            continue
    
    if client.is_socket_open():
        
        time.sleep(1)
        try:
            RoomTemperature = device02.ReadTemp()
            RoomHumidity = device02.ReadHumidity()
                        
            print(
                "****************"
                "\nTemperatura Ambiente [ºC]: "+str(RoomTemperature)+
                "\nHumidade Relativa [%]: "+str(RoomHumidity)+
                "\n****************"
            )
        except:
            print("Erro ao obter os dados no sensor XYMD02.")
            pass








