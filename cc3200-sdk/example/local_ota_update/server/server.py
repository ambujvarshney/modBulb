#!/usr/bin/env python
#
#  server.py
#  
#  A script update a CC3200 device running ota_client application.
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
import sys
import time
import struct
import os
import hashlib
import threading
import argparse
from decimal import *

class Updater:
    """The updater class.
    This class handles the OTA update functionality.
    """

    class ServerStatus:
        """The OTA server status enumerable."""
        STOPPED, STOPPING, RUNNING, BUFFERING_CLIENTS = range(4)

    class ClientStatus:
        """The OTA client status enumerable."""
        PENDING = 'PENDING'
        UPDATING = 'UPDATING'
        UPDATED = 'UPDATED'
        FAILED = 'FAILED'

    update_mcu = False
    update_fpga = False
    uf_mcu_name = ''
    uf_mcu_len = -1
    uf_mcu_md5 = -1
    uf_fpga_name = ''
    uf_fpga_len = -1
    uf_fpga_md5 = -1
    verbose = False
    use_sig = False
    uf_chunksize = 0
    ur_msg = ''
    ur_n = 0
    bc_addr = ()
    srv_addr = ()
    max_clients = 0
    srv_running = False
    srv_st = ServerStatus.STOPPED
    active_clients_cnt = 0
    active_clients_lock = None
    print_lock = None
    srv_soc = 0
    connected_clients = {}
    clients_time = {}
    listener_timeout = 0
    client_timeout = 0
    

    def __init__(self, bc_addr, srv_addr, update_mcu, uf_mcu_name, update_fpga, uf_fpga_name, \
            use_sig, verbose, uf_chunksize, ur_msg, ur_n, max_clients, listener_timeout, \
            client_timeout):
        """Initialize the `Updater` class.
        Args:
            bc_addr (tuple): The UDP broadcast address to send the update requests to.
            srv_addr (tuple): The address to bind the OTA update server to.
            update_mcu (bool): Update the MCU image of connected OTA clients.
            uf_mcu_name (str): The path to the updated MCU image.
            update_fpga (bool): Update the FPGA array configuration of connected OTA clients.
            uf_fpga_name (str): The path to the updated FPGA array configuration file.
            use_sig (bool): Compute and send MD5 signature of update files.
            verbose (boo): Display status messages of this class operation.
            uf_chunksize (int): The number of update data to store in the memory per client.
            ur_msg (str): The message to send as the update request.
            ur_n (int): The number of times to broadcast the update request.
            max_clients (int): The maximum number of active clients.
            listener_timeout (int): The number of seconds to wait for client connections.
            client_timeout (int): The number of seconds to wait for client to respond
                after sending all update data.
        Raises:
            IOError: when either `uf_mcu_name` or `uf_fpga_name` is an incorrect path.
        """
        # Initiate variables
        self.update_mcu = update_mcu
        self.update_fpga = update_fpga
        self.verbose = verbose
        self.use_sig = use_sig
        self.uf_chunksize = uf_chunksize
        self.ur_msg = ur_msg
        self.ur_n = ur_n
        self.bc_addr = bc_addr
        self.srv_addr = srv_addr
        self.max_clients = max_clients
        self.srv_running = False
        self.active_clients_cnt = 0
        self.active_clients_lock = threading.Lock()
        self.print_lock = threading.Lock()
        self.srv_st = self.ServerStatus.STOPPED
        self.connected_clients = {}
        self.clients_time = {}
        self.listener_timeout = listener_timeout
        self.client_timeout = client_timeout
        # create OTA server socket
        self.srv_soc = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.srv_soc.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.srv_soc.bind(self.srv_addr)
        # Initiate update files variables
        self.GetUpdateFileInfo(uf_mcu_name, uf_fpga_name)

    def ServerRunning(self):
        """Check if the OTA server is running.
        Returns:
            bool: True if server not stopped or stopping.
                False otherwise.
        """
        return self.srv_st != self.ServerStatus.STOPPED and \
                self.srv_st != self.ServerStatus.STOPPING

    def ServerStopped(self):
        """Check if the OTA server is stopped.
        Returns:
            bool: True if server is stoppped. False otherwise.
        """
        return self.srv_st == self.ServerStatus.STOPPED

    def StopServer(self):
        """Stop the OTA server."""
        self.srv_running = False
        # connect to OTA server to unblock execution
        try:
            socket.socket(socket.AF_INET, 
                  socket.SOCK_STREAM).connect(self.srv_addr)
        except:
            return

    def _cprint(self, strn):
        """Print `strn` if `verboes` is True.
        Args:
            strn (str): The string to print.
        """
        if (self.verbose):
            self.print_lock.acquire()
            print strn
            self.print_lock.release()

    def PrintStatus(self):
        """Print the client status.
        This function prints the MAC, IP, update status and activity time.
        """
        i = 1
        self.print_lock.acquire()
        for key in self.connected_clients.keys():
            print '%d. MAC: %s, IP: %s, Status: %s, Time: %.3fs' % \
                (i, self.connected_clients[key][0], key, self.connected_clients[key][1],
                self.clients_time[key][1] - self.clients_time[key][0])
            i += 1
        print 'Number of connected clients %d' % len(self.connected_clients)
        print 'Number of updated clients %d' % \
            len([x for x in self.connected_clients.values() if x[1] == self.ClientStatus.UPDATED])
        print 'Number of failed clients %d' % \
            len([x for x in self.connected_clients.values() if x[1] != self.ClientStatus.UPDATED])
        self.print_lock.release()

    def GetUpdateFileInfo(self, uf_mcu_name, uf_fpga_name):
        """Set the update files data.
        Based on `update_mcu`, `update_fpga` and `use_sig` flags, get the file 
        length and MD5 signature.
        Args:
            uf_mcu_name (str): The path to the updated MCU image.
            uf_fpga_name (str): The path to the update FPGA array configuration.
        """
        if self.update_mcu:
            # get the MCU image length and MD5 signature.
            self.uf_mcu_name = uf_mcu_name
            self.uf_mcu_len = os.path.getsize(uf_mcu_name)
            if self.use_sig:
                self.uf_mcu_md5 = hashlib.md5();
                with open(uf_mcu_name, 'rb') as uf:
                    d = uf.read(self.uf_chunksize)
                    while(d):
                        self.uf_mcu_md5.update(d)
                        d = uf.read(self.uf_chunksize)
            else:
                self.uf_mcu_md5 = 0
            self._cprint('MCU update file info updated\nupdate info:\n' \
                'file name: %s\nfile length: %s' \
                % (self.uf_mcu_name, self.uf_mcu_len))
            if self.use_sig:
                self._cprint('file MD5: %s\n' % self.uf_mcu_md5.hexdigest())
            else:
                self._cprint('\n')

        if self.update_fpga:
            # get the FPGA array configuration file length and MD5 signature
            self.uf_fpga_name = uf_fpga_name
            self.uf_fpga_len = os.path.getsize(uf_fpga_name)
            if self.use_sig:
                self.uf_fpga_md5 = hashlib.md5();
                with open(uf_fpga_name, 'rb') as uf:
                    d = uf.read(self.uf_chunksize)
                    while(d):
                        self.uf_fpga_md5.update(d)
                        d = uf.read(self.uf_chunksize)
            else:
                self.uf_fpga_md5 = 0
            self._cprint('FPGA update file info updated\nupdate info:\n' \
                'file name: %s\nfile length: %s' \
                % (self.uf_fpga_name, self.uf_fpga_len))
            if self.use_sig:
                self._cprint('file MD5: %s\n' % self.uf_fpga_md5.hexdigest())
            else:
                self._cprint('\n')

    def HandleClient(self, conn, addr):
        """Handle a connected OTA Client.
        Start a thread that sends the required updates to the connected client.
        Args:
            conn (socket): The opened client-server socket.
            addr (str): The address of the connected client.
        """
        def _modifyClients(n, mac, state):
            """Local function that manages write operations to client status."""
            self.active_clients_lock.acquire()
            self.active_clients_cnt += n
            if addr not in self.connected_clients:
                self.clients_time[addr] = [time.time(), 0]
            self.connected_clients[addr] = (mac, state)
            self.clients_time[addr][1] = time.time()
            self.active_clients_lock.release()

        def _handler():
            """Local function to be used by client handler thread."""
            _modifyClients(1, None, self.ClientStatus.PENDING)

            conn.settimeout(self.client_timeout)
            try:
                mac = conn.recv(6)
                mac = '%02x:%02x:%02x:%02x:%02x:%02x' % struct.unpack('BBBBBB', mac)
            except:
                self._cprint('[%s] failed to recieve MAC' % addr)
                _modifyClients(-1, None, self.ClientStatus.FAILED)
                conn.close()
                self._cprint('[%s] connection closed' % addr)
                return
            self._cprint('[%s] is (%s)' % (mac, addr))
            self._cprint('[%s] starting update...' % mac)
            _modifyClients(0, mac, self.ClientStatus.UPDATING)
            if self.update_mcu:
                self._cprint('[%(mac)s] sending MCU update file...\n' \
                    '[%(mac)s] sending update file length...' % {'mac' : mac})
                # send MCU image file length
                try:
                    conn.send(struct.pack('!I', self.uf_mcu_len))
                except:
                    self._cprint('[%s] failed to send file length' % mac)
                    _modifyClients(-1, mac, self.ClientStatus.FAILED)
                    conn.close()
                    self._cprint('[%s] connection closed' % mac)
                    return
                self._cprint('[%s] update file length sent' % mac)
                time.sleep(0.1)
                # send update file data
                self._cprint('[%s] sending update file...' % mac)
                try:
                    with open(self.uf_mcu_name, 'rb') as uf:
                        d = uf.read(self.uf_chunksize)
                        while(d):
                            conn.send(d)
                            d = uf.read(self.uf_chunksize)
                except IOError:
                    self._cprint('[%s] failed to read from MCU update file' % mac)
                    _modifyClients(-1, mac, self.ClientStatus.FAILED)
                    conn.close()
                    self._cprint('[%s] connection closed' % mac)
                    return
                except:
                    self._cprint('[%s] failed to send file data' % mac)
                    _modifyClients(-1, mac, self.ClientStatus.FAILED)
                    conn.close()
                    self._cprint('[%s] connection closed' % mac)
                    return

                self._cprint('[%s] update file sent' % mac)
                time.sleep(0.1)
                # send signature if required
                if self.use_sig:
                    self._cprint('[%s] sending MD5...' % mac)
                    try:
                        conn.send(self.uf_mcu_md5.digest())
                    except:
                        self._cprint('[%s] failed to send MD5 signature' % mac)
                        _modifyClients(-1, mac, self.ClientStatus.FAILED)
                        conn.close()
                        self._cprint('[%s] connection closed' % mac)
                        return 
                    self._cprint('[%s] MD5 sent' % mac)
                    time.sleep(0.1)
                else:
                    try:
                        conn.send(struct.pack('!I', 0))
                    except:
                        self._cprint('[%s] failed to send no signature indication' % mac)
                        _modifyClients(-1, mac, self.ClientStatus.FAILED)
                        conn.close()
                        self._cprint('[%s] connection closed' % mac)
                        return 
            else:
                try:
                    conn.send(struct.pack('!I', 0))
                except:
                    self._cprint('[%s] failed to send no MCU update indication' % mac)
                    _modifyClients(-1, mac, self.ClientStatus.FAILED)
                    conn.close()
                    self._cprint('[%s] connection closed' % mac)
                    return

            if self.update_fpga:
                self._cprint('[%(mac)s] sending FPGA update file...\n' \
                    '[%(mac)s] sending update file length...' % {'mac' : mac})
                try:
                    conn.send(struct.pack('!I', self.uf_fpga_len))
                except:
                    self._cprint('[%s] failed to send file length' % mac)
                    _modifyClients(-1, mac, self.ClientStatus.FAILED)
                    conn.close()
                    self._cprint('[%s] connection closed' % mac)
                    return
                self._cprint('[%s] update file length sent' % mac)
                time.sleep(0.1)
                self._cprint('[%s] sending update file...' % mac)
                try:
                    with open(self.uf_fpga_name, 'rb') as uf:
                        d = uf.read(self.uf_chunksize)
                        while(d):
                            conn.send(d)
                            d = uf.read(self.uf_chunksize)
                except IOError:
                    self._cprint('[%s] failed to read from FPGA update file' % mac)
                    _modifyClients(-1, mac, self.ClientStatus.FAILED)
                    conn.close()
                    self._cprint('[%s] connection closed' % mac)
                    return
                except:
                    self._cprint('[%s] failed to send file data' % mac)
                    _modifyClients(-1, mac, self.ClientStatus.FAILED)
                    conn.close()
                    self._cprint('[%s] connection closed' % mac)
                    return
                self._cprint('[%s] update file sent' % mac)
                time.sleep(0.1)
                if self.use_sig:
                    self._cprint('[%s] sending MD5...' % mac)
                    try:
                        conn.send(self.uf_fpga_md5.digest())
                    except:
                        self._cprint('[%s] failed to send MD5 signature' % mac)
                        _modifyClients(-1, mac, self.ClientStatus.FAILED)
                        conn.close()
                        self._cprint('[%s] connection closed' % mac)
                        return
                    self._cprint('[%s] MD5 sent' % mac)
                    time.sleep(0.1)
                else:
                    try:
                        conn.send(struct.pack('!I', 0))
                    except:
                        self._cprint('[%s] failed to send no signature indication' % mac)
                        _modifyClients(-1, mac, self.ClientStatus.FAILED)
                        conn.close()
                        self._cprint('[%s] connection closed' % mac)
                        return
            else:
                try:
                    conn.send(struct.pack('!I', 0))
                except:
                    self._cprint('[%s] failed to send no FPGA update indication' % mac)
                    _modifyClients(-1, mac, self.ClientStatus.FAILED)
                    conn.close()
                    self._cprint('[%s] connection closed' % mac)
                    return

            self._cprint('[%(mac)s] updates sent\n' \
                        '[%(mac)s] awaiting client response...' % {'mac' : mac})
            # wait for client confirmation
            st = -1
            try:
                st = conn.recv(4)
                st = struct.unpack('!i', st)[0]
            except socket.timeout:
                self._cprint('[%s] client timed out' % mac)
                _modifyClients(-1, mac, self.ClientStatus.FAILED)
                conn.close()
                self._cprint('[%s] connection closed' % mac)
                return
            except:
                self._cprint('[%s] reception error' % mac)
                _modifyClients(-1, mac, self.ClientStatus.FAILED)
                conn.close()
                self._cprint('[%s] connection closed' % mac)
                return
            if st != 0:
                self._cprint('[%s] update failed' % mac)
                _modifyClients(-1, mac, self.ClientStatus.FAILED)
            else:
                self._cprint('[%s] update was successful' % mac)
                _modifyClients(-1, mac, self.ClientStatus.UPDATED)
            
            conn.close()
            self._cprint('[%s] connection closed' % mac)


        th = threading.Thread(target=_handler, name='[ct] Client '+addr+' handler thread')
        th.daemon = True
        th.start()

    def InitiateUpdate(self):
        """Initiate the client update.
        Start a thread that broadcats `ur_n` update requests (i.e. `ur_msg` messages) 
        over the provided broadcast address.
        """
        def _initiator():
            """Local function used by the thread."""
            self._cprint('[initiator] broadcasting update request messages...')
            soc = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            # create a UDP socket
            soc.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
            # broadcast UDP packets with 50ms interval
            for i in range(self.ur_n):
                soc.sendto(self.ur_msg + struct.pack('!H', self.srv_addr[1]), self.bc_addr)
                time.sleep(0.05)
            self._cprint('[initiator] %d update request messages broadcasted' % self.ur_n)


        th = threading.Thread(target=_initiator, name='[bt] Update request broadcast thread')
        th.daemon = True
        th.start()

    def AccecptClients(self):
        """Await and handle client connections.
        Start a thread that awaits client connections. When a client connects,
        call `HandleClient` to handle the connection.
        """
        def _listenerTimeout():
            """Local function used by the listener timeout thread."""
            time.sleep(self.listener_timeout)
            self.StopServer()

        def _listener():
            """Local function used by the listener thread."""
            self.srv_soc.listen(self.max_clients)
            self.active_clients_cnt = 0
            self.srv_running = True
            self.srv_st = self.ServerStatus.RUNNING
            threading.Thread(target=_listenerTimeout, 
                name='[lt] Socket listener timeout thread').start()
            self._cprint('[server] awaiting new connection...')
            # await client connections
            while self.srv_running:
                while self.active_clients_cnt >= self.max_clients:
                    self.srv_st = self.ServerStatus.BUFFERING_CLIENTS
                self.srv_st = self.ServerStatus.RUNNING
                conn, addr = self.srv_soc.accept()
                if not self.srv_running:
                    break
                self._cprint('[server] connected to client (%s)' % addr[0])
                # handle connected client
                self.HandleClient(conn, addr[0])
                
            # wait for connections to terminate 
            self.srv_st = self.ServerStatus.STOPPING
            self._cprint('[server] server is stopping\n' \
                '[server] awaiting running connections to close...')
            while self.active_clients_cnt != 0:
                pass
            # close the OTA server socket
            self.srv_soc.close()
            self._cprint('[server] server is stopped')
            self.srv_st = self.ServerStatus.STOPPED


        th = threading.Thread(target=_listener, name='[lt] Socket listener thread')
        th.daemon = True
        th.start()




def strAddrToTuple(strn):
    try:
        strn = strn.split(':')
        socket.inet_aton(strn[0])
        return strn[0], int(strn[1])
    except:
        raise argparse.ArgumentTypeError('%s is not of format: xx.xx.xx.xx:x' % strn)

def verifyPath(strn):
    if os.path.isfile(strn) == True:
        return strn
    else:
        raise argparse.ArgumentTypeError('%s is not a file' % strn)

def verifyPNZInt(strn):
    try:
        if int(strn) > 0:
            return int(strn)
        else:
            raise TypeError
    except:
        raise argparse.ArgumentTypeError('%s is not a positive non-zero integer' % strn)

def verifyPInt(strn):
    try:
        if int(strn) >= 0:
            return int(strn)
        else:
            raise TypeError
    except:
        raise argparse.ArgumentTypeError('%s is not a positive integer' % strn)

def verifyPNZDecimal(strn):
    try:
        if Decimal(strn) > 0:
            return float(strn)
        else:
            raise TypeError
    except:
        raise argparse.ArgumentTypeError('%s is not a positive non-zero decimal' % strn)

def verifyPDecimal(strn):
    try:
        if Decimal(strn) >= 0:
            return float(strn)
        else:
            raise TypeError
    except:
        raise argparse.ArgumentTypeError('%s is not a positive non-zero decimal' % strn)


def main(argv=None):
    """The main OTA update routine.
    Args:
        argv (:obj:`list`, optional): The argument vector to pass to Updater class.
    """

    if argv is None:
        # create argument vector from sys.argv
        parser = argparse.ArgumentParser(description='Update modBulbs firmware')

        parser.add_argument('-ba', '--bc-addr', type=strAddrToTuple, 
            default=('192.168.0.255',5001), 
            help='the UDP broadcast address to send update request to')
        parser.add_argument('-sa', '--srv-addr', type=strAddrToTuple, default=('0.0.0.0',5001), 
            help='the address to bind the update server to')

        parser.add_argument('-mf', '--uf-mcu-path', type=verifyPath, default='update.bin', 
            help='the path to the MCU update file')
        parser.add_argument('-ff', '--uf-fpga-path', type=verifyPath, default='update.dat', 
            help='the path to the FPGA array update file')

        parser.add_argument('-um', '--update-mcu', action='store_true', 
            help='update the MCU image')
        parser.add_argument('-uf', '--update-fpga', action='store_true', 
            help='update the FPGA array')
        parser.add_argument('-vb', '--verbose', action='store_true', 
            help='display unnecessary messages')
        parser.add_argument('-us', '--use-sig', action='store_true', 
            help='compute and send MD5 signature of update files')

        parser.add_argument('-cs', '--chunksize', type=verifyPNZInt, default=4096, 
            help='the number of byte to handle at once')
        parser.add_argument('-rm', '--ur-msg', type=str, default='update', 
            help='the message to broadcast to initiate the update')
        parser.add_argument('-rn', '--ur-n', type=verifyPInt, default=2, 
            help='the number of update request messages to broadcast')
        parser.add_argument('-mc', '--max-clients', type=verifyPNZInt, default=20, 
            help='the number of clients to handle at once')
        parser.add_argument('-lt', '--listener-timeout', type=verifyPNZDecimal, default=5, 
            help='the number of seconds to wait for incoming connections')
        parser.add_argument('-ct', '--client-timeout', type=verifyPNZDecimal, default=40, 
            help='the number of seconds to wait for client response')
        


        args = parser.parse_args()
        argv = (args.bc_addr, args.srv_addr, args.update_mcu, args.uf_mcu_path, 
            args.update_fpga, args.uf_fpga_path, args.use_sig, args.verbose, 
            args.chunksize, args.ur_msg, args.ur_n, args.max_clients, 
            args.listener_timeout, args.client_timeout)

    # clear display
    os.system('cls' if os.name == 'nt' else 'clear')

    # init updater
    updater = Updater(*argv)

    # start the update server
    print '[main] starting server...'
    updater.AccecptClients()
    while not updater.ServerRunning():
        pass
    time.sleep(0.01)
    print '[main] server started'
    
    # broadcat update requests
    print '[main] broadcasting update request...'
    updater.InitiateUpdate()
    while len([t for t in threading.enumerate() if '[bt]' in t.name]) > 0:
        pass
    print '[main] update request broadcasted'
    print '[mian] server is running (hit Ctrl+C to stop)'

    # wait for keyboard interrupt
    try:
        while not updater.ServerStopped():
            pass
        print '[mian] server stopped'
    except KeyboardInterrupt:
        # stop the OTA sever and close the application
        print '[mian] stopping server... (hit Ctrl+C to force)'
        updater.StopServer()
        try:
            while not updater.ServerStopped():
                pass
            print '[mian] server stopped'
        except KeyboardInterrupt:
            pass
    print '[main] update status:'
    updater.PrintStatus()
    print '[main] exiting application...'
    sys.exit()

if __name__ == "__main__":
    main()
