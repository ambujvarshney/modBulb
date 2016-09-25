#!/usr/bin/env python
#
#  modulate.py
#  
#  A script to send a modulation command directly though cli.
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
from modulation_lib import ReturnCodes, Addr, sendModCmd

def main(argv=None):
    if argv is None:
        parser = argparse.ArgumentParser(description='Modulate data using modBulb')

        parser.add_argument('addr', type=Addr, 
            default=Addr(('192.168.0.255',5001)), nargs='?',
            help='the UDP broadcast address to send init command to')
        parser.add_argument('data', type=str, default='Hello! World.',
            nargs='?', help='the data to send to be modulated')
        parser.add_argument('encoding', type=str.upper, choices=['HEX', 'PLAIN'],
            default='PLAIN', nargs='?', help='the encoding of the provided data')
        parser.add_argument('bitrate', type=int, default=1e5, nargs='?',
            help='the modulation bitrate')
        parser.add_argument('indicator', type=int, default=0xea, nargs='?',
            help='the command indicator to send (0 <= val <= 255')

        args = parser.parse_args()
        argv = (args.addr, args.indicator,  args.bitrate, args.data, args.encoding)

    if len(argv) < 5:
        return ReturnCodes.ArgumentError

    data = None
    if argv[4] == 'HEX':
        try:
            data = argv[3].decode('hex')
        except:
            return ReturnCodes.DecodingError
    else:
        data = argv[3]
        
    try:
        return sendModCmd(argv[0], argv[1], argv[2], data)
    except:
        return ReturnCodes.ArgumentError


if __name__ == '__main__':
    exit(main())
