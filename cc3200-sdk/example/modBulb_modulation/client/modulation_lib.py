#
#  modulation_lib.py
#
#  This file contains some functions that are shared amoung the
#  implemented modulation tools.
#
#  Copyright (C) 2016 Uppsala Netowrked Objects.
#  
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions are
#  met:
#  
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following disclaimer
#    in the documentation and/or other materials provided with the
#    distribution.
#  * Neither the name of the  nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#  
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
#  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
#  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
#  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
#  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
#  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
#  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
#  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
#  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
#  Contributors: Abdalah Hilmia, Kasun Hewage and Ambuj Varshney.
#

import socket
import struct

# allowed modulation schemes
mod_schemes = ['OOK', 'BFSK', 'PPM']

# allowed device values
mod_devices = ['MCU', 'FPGA']

# UDP socket
cmdSocket = None

class ReturnCodes:
    """Return codes."""
    Success, DecodingError, SocketError, ArgumentError, FileReadError, FileWriteError = range(6)

class Addr(object):
    """IP:Port pair class."""
    ip = ''
    port = 0
    def __init__(self, addr):
        """Using `addr` value set the IP:Port pair.
        Args:
            addr (touple/str): The address to set the IP and Port to.
        """
        self.set(addr)

    def __str__(self):
        """Override string function."""
        return self.ip + ':' + str(self.port)

    def get(self):
        """Retun IP and Port as tuple.
        Returns:
            tuple: IP, Port tuple"""
        return (self.ip, self.port)

    def set(self, addr):
        """Set the IP and Port to the value provided in `addr`.
        Args:
            addr: tuple/str: IP, Port tuple or IP:Port string.
        
        Raises:
            AssertionError: When input value does not represent a valid address pair"""
        try:
            if isinstance(addr, basestring):
                addr = addr.split(':')
                ip = addr[0]
                port = addr[1]
            elif isinstance(addr, tuple):
                ip = addr[0]
                port = addr[1]
        except:
            raise AssertionError, 'Invalid address %s' % addr

        if ip == '':
            ip = '0'

        try:
            socket.inet_aton(addr[0])
        except:
            raise AssertionError, 'Invalid IP value %s' % ip

        try:
            port = int(port)
            if port < 1 or port > 0xFFFF:
                raise AssertionError
        except:
            raise AssertionError, 'Invalid port value %s' % port

        self.ip = ip
        self.port = port

    def valid(self):
        """Checks if the IP and Port of the class are valid.
        Returns:
            bool: True if address is valid. False otherwise."""
        try:
            socket.inet_aton(self.ip)
            port = int(self.port)
            if port < 1 or port > 0xFFFF:
                raise AssertionError
        except:
            return False

        return True

def sendInitCmd(addr, ind, scheme, device, f1, f2, dc, bs):
    """Send a UDP packet containing initalization command and parameters.

    Args:
        addr (tuple): The address to send the command to.
        ind (int): Initialization command indicator.
        scheme (str): Modulation scheme (value of mod_scheme).
        device (str): Modulation device (value of mod_device).
        f1 (int): BFSK frequency representing logical 0.
        f2 (int): BFSK frequency representing logical 1.
        dc (int): BFSK signal duty cycle (0 <= `dc` <= 100).
        bs (int): PPM bit/symbol.

    Returns:
        int: ReturnCodes.Success on success. ReturnCodes.ArgumentError when passed arguments are
            invalid. ReturnCodes.SocketError if socket fails to open or send."""
    try:
        msg  = struct.pack('!B', ind & 0xFF)
        msg += struct.pack('!B', mod_schemes.index(scheme) + 1)
        msg += struct.pack('!B', mod_devices.index(device) + 1)
        msg += struct.pack('!I', f1)
        msg += struct.pack('!I', f2)
        msg += struct.pack('!B', dc)
        msg += struct.pack('!B', bs)
    except:
        return ReturnCodes.ArgumentError

    try:
        global cmdSocket
        if cmdSocket is None:
            cmdSocket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            cmdSocket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        
        if isinstance(addr, Addr):
            cmdSocket.sendto(msg, addr.get())
        else:
            cmdSocket.sendto(msg, addr)
    except:
        return ReturnCodes.SocketError

    return ReturnCodes.Success

def sendModCmd(addr, ind, cnt, delay, br, data):
    """Send a UDP packet containing modulation command and parameters.

    Args:
        addr (tuple): The address to send the command to.
        ind (int): Modulation command indicator.
        cnt (int): The number of times to send the data.
        delay (int): The interpacket delay in ms.
        br (int): Modulation bitrate.
        data (str): data to send for moulation."""
    try:
        msg  = struct.pack('!B', ind & 0xFF)
        msg += struct.pack('!H', cnt & 0xFFFF)
        msg += struct.pack('!H', delay & 0xFFFF)
        msg += struct.pack('!I', br)
        msg += struct.pack('!H', len(data))
        msg += data
    except:
        return ReturnCodes.ArgumentError

    try:
        global cmdSocket
        if cmdSocket is None:
            cmdSocket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            cmdSocket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        
        if isinstance(addr, Addr):
            cmdSocket.sendto(msg, addr.get())
        else:
            cmdSocket.sendto(msg, addr)
    except:
        return ReturnCodes.SocketError

    return ReturnCodes.Success

