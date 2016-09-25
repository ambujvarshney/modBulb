#!/usr/bin/env python
#
#  initialize_modulation.py
#  
#  A script to initialize the modulation directly though cli.
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

import argparse
from modulation_lib import ReturnCodes, Addr, sendInitCmd, mod_schemes, mod_devices

def main(argv=None):
    if argv is None:
        parser = argparse.ArgumentParser(description='Init modBulb modulator')

        parser.add_argument('addr', type=Addr, default=Addr(('192.168.0.255',5001)), nargs='?',
            help='the UDP broadcast address to send init command to')
        parser.add_argument('scheme', type=str.upper, choices=mod_schemes, default='OOK',
            nargs='?', help='the modulation scheme to use')
        parser.add_argument('device', type=str.upper, choices=mod_devices, default='MCU',
            nargs='?', help='the device to use to modulate the data')
        parser.add_argument('-f', '--freq', type=int, default=[1e7, 8e6], nargs=2,
            help='BFSK frequencies')
        parser.add_argument('-d', '--duty', type=int, default=50, help='The BFSK duty cycle')
        parser.add_argument('-n', '--bit-symbol', type=int, default=2, help='PPM bit per symbol')
        parser.add_argument('indicator', type=int, default=0xe7, nargs='?',
            help='the command indicator to send (0 <= val <= 255')

        args = parser.parse_args()
        argv = (args.addr, args.indicator, args.scheme, args.device, args.freq, args.duty,
                args.bit_symbol)

    if len(argv) < 8:
        return ReturnCodes.ArgumentError

    try:
        return sendInitCmd(argv[0], argv[1], argv[2], argv[3], argv[4][0], argv[4][1], argv[5],
                            argv[6])
    except:
        return ReturnCodes.ArgumentError


if __name__ == '__main__':
    exit(main())
