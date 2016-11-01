#!/usr/bin/env python

from time import sleep
import argparse
from reedsolo import RSCodec

import logging
logging.getLogger("scapy.runtime").setLevel(logging.ERROR)
from scapy.all import *

from modulation_lib import ReturnCodes, Addr, sendModCmd, sendInitCmd

def hexdump_pretty(src, length=16):
    FILTER = ''.join([(len(repr(chr(x))) == 3) and chr(x) or '.' for x in range(256)])
    lines = []
    for c in xrange(0, len(src), length):
        chars = src[c:c+length]
        hex_str = ' '.join(["%02x" % x for x in chars])
        printable = ''.join(["%s" % ((x <= 127 and FILTER[x]) or '.') for x in chars])
        lines.append("%04x  %-*s  %s\n" % (c, length*3, hex_str, printable))
    return ''.join(lines)

def hexdump(src):
    return ''.join(["%02x" % x for x in src])
        
        

def manchester_encode(data):
    
    N = lambda x:170-(x & 1 | x * 2 & 4| x * 4 & 16| x * 8 & 64)
    M = lambda A:sum( [[N(a), N(a >> 4)] for a in A], [] )
    
    encoded_buffer = []
    for c in data:
        encoded_buffer.extend((M([c]))[::-1])

    return encoded_buffer


def construct_vlc_frame(data, src_id, dst_id):
    frame = []
    ecc_phy_header = []
    non_ecc_phy_header = []
    ecc_data = []
    
    # Setting non error corrected part of the PHY header
    non_ecc_phy_header.append(0xa3)
    non_ecc_phy_header.append((len(data) >> 8) & 0xff)
    non_ecc_phy_header.append(len(data) & 0xff)
    
    # Setting error corrected part of the PHY header
    ecc_phy_header.append((dst_id >> 8) & 0xff)
    ecc_phy_header.append(dst_id & 0xff)
    
    ecc_phy_header.append((src_id >> 8) & 0xff)
    ecc_phy_header.append(src_id & 0xff)
    
    # Setting protocol. Not sure why this specific value is used
    ecc_phy_header.append(0x08)
    ecc_phy_header.append(0x00)
    
    rs = RSCodec(nsym=16, prim=0x11d, c_exp=8, nsize=216, generator=2, fcr=0)
    
    # Only the bytes after the legnth field are subjected to error correction 
    ecc_data = rs.encode(ecc_phy_header + data)    

    # Constructing the frame
    # Setting preamble    
    frame.extend([0xaa, 0xaa, 0xaa])
    
    # All the bytes after the preamble are manchester encoded
    frame.extend(manchester_encode(non_ecc_phy_header))

    frame.extend(manchester_encode(ecc_data))
    
    return frame

def construct_udp_packet(dst_addr, dst_port, payload):
    packet = IP(dst=dst_addr)/UDP(dport=dst_port)/str(payload)
    # Convert the packet to a byte array
    return map(ord, str(packet))

def main(argv=None):

    if argv is None:
        parser = argparse.ArgumentParser(description='Modulate data using modBulb for OpenVLC')

        parser.add_argument('-m', '--maddr', 
                            type=Addr, 
                            default=Addr(('192.168.0.255',5001)), 
                            nargs='?', 
                            help='IP:Port of modBulb')
        
        parser.add_argument('-o', '--oaddr', 
                            type=Addr, 
                            default=Addr(('192.168.0.255',5001)), 
                            nargs='?',
                            help='IP:Port of UDP server on OpenVLC')
        
        args = parser.parse_args()
        argv = (args.maddr, args.oaddr)

    if len(argv) < 2:
        return ReturnCodes.ArgumentError
    
    # Send initilize command for modBulb MCU
        
    ret = sendInitCmd(args.maddr, 0xe7, "OOK", "MCU", 1e7, 8e6, 50, 2)
    
    if ret != ReturnCodes.Success:
        print "INIT ERROR %d" % ret
        return ret
    
    # Sleep 1 second
    sleep(1)

    global cliRunning
    cliRunning = True
    while cliRunning:
        try:
            line_input = raw_input('')
        except:
            print ''
            return 0
        
        line_input += '\n'

        # Construct UDP packet with the line_input as the payload
        packet_bytes = construct_udp_packet(args.oaddr.ip, args.oaddr.port, line_input)
        
        # Create VLC frame
        vlc_frame = construct_vlc_frame(packet_bytes, 3, 2)
        # Send the VLc frame to modBulb for modulation
        frame_str = ''.join(["%02x" % x for x in vlc_frame])
        
        #print frame_str

        ret = sendModCmd(args.maddr, 0xea, 50000, frame_str.decode('hex'))
        
        if ret != ReturnCodes.Success:
            print "MOD ERROR %d" % ret
            return ret
    
    return 0

if __name__ == '__main__':
    exit(main())
