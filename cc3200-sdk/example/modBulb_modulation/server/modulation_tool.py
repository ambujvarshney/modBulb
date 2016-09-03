#
#  modulation_tool.py
#  
#  A script to send initialization and modulation commands.
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

#!/usr/bin/env python

import os
import argparse
import pickle
import copy
from collections import OrderedDict
import readline
import textwrap
from modulation_lib import ReturnCodes, Addr, sendInitCmd, sendModCmd, mod_schemes, mod_devices

paramAddr = ['tAddr']
paramPInt = ['cmdInd', 'iniInd', 'modInd', 'offset', 'freq1', 'freq2', 'dutyCycle']
paramPNZInt = ['length', 'count', 'bitrate', 'symLen']
paramChoice = OrderedDict([('scheme', mod_schemes), ('device', mod_devices)])
paramObj = ['data']

cmdP0 = ['init', 'modulate', 'exit', 'help']
cmdP1 = ['print', 'save', 'load', 'help']
cmdP2 = ['set', 'save', 'load']

cmdHelp = OrderedDict([
    (cmdP0[3]   , ['help\t\t[CMD[,CMD...]]', 'Show help for command(s) provided in `CMD`. If no '
                '`CMD` is provided, help will be shown for all commands.']),
    (cmdP1[0]   , ['print\t\t{all,PAR[,PAR...]}', 'Print the value(s) of provided parameters in '
                '`PAR` (or all parameters, if `all` is specified)']),
    (cmdP2[0]   , ['set\t\tPAR[,PAR...] VAL[,VAL...]', 'assign each `VAL` to its corresponding '
                '`PAR`.']),
    (cmdP2[1]   , ['save\t\t{all,PAR[,PAR...]} [PATH]', 'Save the parameters provided in `PAR` '
                ' (or all parameters, if `all` is chosen) to `PATH`. If no `PATH` is specified, '
                'parameters are saved to default path (provided by `--config-file` option)']),
    (cmdP2[2]   , ['load\t\t{default,data,all,PAR[,PAR...] [PATH]', 'Load default parameters if '
                '`default`, is specified (no `PATH` is required). Load `DATA` from `PATH`, if '
                '`data` is specified. Load all parameters from `PATH` if `all is specified. Load '
                'parameters specified in `PAR` from `PATH` otherwise. If no `PATH` is provided '
                '`PATH` is assumed to be the default path (provided by `--config-file` option).']),
    (cmdP0[0]   ,['init', 'Initialize the modulate state of modBulb.']),
    (cmdP0[1]   ,['modulate', 'Send modulation command to modBulb.'])
    ])

defaultParams = OrderedDict([
        (paramAddr[0]           , Addr('192.168.0.255:5001')),
        (paramPInt[0]           , 0xE0),
        (paramPInt[1]           , 0x07),
        (paramPInt[2]           , 0x0a), 
        (paramChoice.keys()[0]  , 'OOK'),
        (paramChoice.keys()[1]  , 'MCU'),
        (paramPInt[4]           , 10000000),
        (paramPInt[5]           , 8000000),
        (paramPInt[6]           , 50),
        (paramPNZInt[3]         , 4),
        (paramPInt[3]           , 0),
        (paramPNZInt[0]         , 13),
        (paramPNZInt[1]         , 1),
        (paramPNZInt[2]         , 100000),
        (paramObj[0]            , 'Hello! World.')])

cmdParams = None

cliRunning = False

def _checkCmd(cmd):

    # return True if cmd and the number of its parameters are valid
    return ((cmd[0] in cmdP0 and len(cmd) == 1) or (cmd[0] in cmdP1 and len(cmd) == 2) or
            (cmd[0] in cmdP2 and len(cmd) == 3))

def _checkPars(pars):

    # return True if every par is pars is a valid
    for el in pars:
        if el not in defaultParams.keys():
            return False
    return True

def _setPar(par, val, dic):

    # check if par is valid
    if par not in defaultParams.keys():
        return False

    # assign val to par if val matches the category of par
    if par in paramAddr:
        try:
            if isinstance(val, Addr):
                dic[par] = copy.deepcopy(val)
            else:
                dic[par] = Addr(val)
        except:
            return False

    elif par in paramPInt:
        try:
            if int(val) >= 0:
                dic[par] = int(val)
            else:
                return False
        except:
            return False

    elif par in paramPNZInt:
        try:
            if int(val) > 0:
                dic[par] = int(val)
            else:
                return False
        except:
            return False

    elif par in paramChoice:
        if val.upper() in paramChoice[par]:
            dic[par] = val.upper()
        else:
            return False
    else:
        dic[par] = val

    return True

def _getParStr(par, dic):
    # return an empty string if par is not valid
    if not par in defaultParams.keys():
        return ''

    # return value of par as a string
    return str(dic[par])

def _checkDict(dic, pars=[]):
    
    if not pars:
        pars = defaultParams.keys()
    else:
        # check if pars are valid
        for key in pars:
            if key not in defaultParams.keys():
                return False

    # check if all keys are available
    for key in pars:
        if key not in dic.keys():
            return False

    for key in pars:
        # check if val is valid
        if ((key in paramAddr and (not isinstance(dic[key], Addr) or not dic[key].valid())) or 
            (key in paramPInt and (not isinstance(dic[key], int) or dic[key] < 0)) or
            (key in paramPNZInt and (not isinstance(dic[key], int) or dic[key] <= 0)) or
            (key in paramChoice and dic[key] not in paramChoice[key])):
            return False

    return True

def _loadDict(path):
    tmp = None
    
    try:
        # load data from file
        with open(path, 'rb') as f:
            tmp = pickle.load(f)
    except:
        return False
    
    # check if loaded data is valid
    if not _checkDict(tmp):
        return False

    # return valid dictionary
    return tmp

def _loadData(path):

    # retun data from file
    try:
        with open(path, 'rb') as f:
            return f.read()
    except:
        return False

def _saveDict(path, dic):

    # save dic to file
    try:
        with open(path, 'wb') as f:
            pickle.dump(dic, f)
    except:
        return False

    return True

def _printPars(pars, dic):
    # print (par, val) pairs as par : val
    for par in pars:
        print '{0:12}: {1}'.format(par, _getParStr(par, dic)[:100])

def assign(pars, vals):
    
    global cmdParams
    tmp = copy.deepcopy(cmdParams)

    # check if pars and vals have the same length
    if len(pars) != len(vals):
        return ReturnCodes.ArgumentError

    # check if pars are valid
    if not _checkPars(pars):
        return ReturnCodes.ArgumentError

    # set each par to its val
    for par, val in zip(pars, vals):
        if not _setPar(par, val, tmp):
            return ReturnCodes.ArgumentError

    # commit changes 
    cmdParams = tmp

    # print changes
    _printPars(pars, cmdParams)

    return ReturnCodes.Success

def show(pars):
    # check if pars are valid
    if not _checkPars(pars):
        return ReturnCodes.ArgumentError

    # print requested pars
    _printPars(pars, cmdParams)
    
    return ReturnCodes.Success

def save(path, pars=[]):
    if pars:
        # check if pars are valid 
        if not _checkPars(pars):
            return ReturnCodes.ArgumentError

        # get values from file
        tmp = _loadDict(path)
        if not tmp:
            # get default values
            tmp = copy.deepcopy(defaultParams)

        # set new values
        for par in pars:
            tmp[par] = cmdParams[par]
    else:
        tmp = cmdParams

    if not _saveDict(path, cmdParams):
        return ReturnCodes.FileWriteError
    
    return ReturnCodes.Success

def load(sel, path='', pars=[]):
    
    global cmdParams
    
    if sel == 0:
        # load default
        cmdParams = copy.deepcopy(defaultParams)
        _printPars(cmdParams.keys(), cmdParams)

    elif sel == 1:
        # load data
        data = _loadData(path)
        if not data:
            return ReturnCodes.FileReadError
        cmdParams['data'] = data

        # print changes
        _printPars(['data'], cmdParams)

    elif sel == 2:
        # load pars
        tmp = _loadDict(path)
        if not tmp:
            return ReturnCodes.FileReadError

        if pars:
            # check if pars are valid
            if not _checkPars(pars):
                return ReturnCodes.ArgumentError

            # assign par values
            for par in pars:
                _setPar(par, tmp[par], cmdParams)

            # print changes
            _printPars(pars, cmdParams)
        else:
            cmdParams = tmp
            # print changes
            _printPars(cmdParams.keys(), cmdParams)
    else:
        # invalid sel
        return -1

    return ReturnCodes.Success

def showHelp(cmds=[]):
    # check if cmds are valid
    if cmds:
        for cmd in cmds:
            if cmd not in cmdP0 and cmd not in cmdP1 and cmd not in cmdP2:
                return ReturnCodes.ArgumentError
    else:
        cmds = cmdHelp.keys()

    wraper = textwrap.TextWrapper(initial_indent='\t\t', width=60)
    for cmd in cmds:
        text = wraper.wrap(cmdHelp[cmd][1])
        print cmdHelp[cmd][0]
        print '\n\t\t'.join(text)

def cmdParser(cmd):

    global cmdParams

    # return False if cmd is empty
    if not cmd:
        return False

    # check if cmd is valid
    if not _checkCmd(cmd):
        print 'Invalid command %s.' % cmd[0]
        return False
    
    # parse cmd
    if cmd[0] == 'exit':
        global cliRunning
        cliRunning = False

    elif cmd[0] == 'help':
        showHelp()

    elif cmd[0] == 'set':
        # get pars and vals from cmd string
        pars = [el.strip() for el in cmd[1].split(',')]
        vals = [el.strip() for el in cmd[2].split(',')]

        # assign vals to pars
        if assign(pars, vals) != ReturnCodes.Success:
            print 'set: Invalid arguments.'
            return False

    elif cmd[0] == 'print':
        
        pars = None

        # get pars
        if cmd[1].lower() == 'all':
            pars = defaultParams.keys()
        else:
            pars = [el.strip() for el in cmd[1].split(',')]

        # print pars
        if show(pars) != ReturnCodes.Success:
            print 'print: Invalid arguments.'
            return False

    elif cmd[0] == 'save':
        
        # get path
        path = ''
        if len(cmd) == 3:
            path = cmd[2]
            if not os.path.isfile(path):
                print 'save: Invalid path %s.' % path
                return False
        else:
            path = cmdConfigFile

        if cmd[1].lower() == 'all':
            # save all pars
            if save(path) != ReturnCodes.Success:
                print 'save: Could not write to file %s.' % path
                return False

        else:
            # get pars from cmd string
            pars = [el.strip() for el in cmd[1].split(',')]
            
            # save pars
            ret = save(path, pars)
            if ret != ReturnCodes.Success:
                if ret == ReturnCodes.ArgumentError:
                    print 'save: Invalid arguments.'
                else:
                    print 'save: Could not write to file %s.' % path
                return False

    elif cmd[0] == 'load':

        if cmd[1].lower() == 'default':
            # load default values
            if len(cmd) == 3:
                print 'load: Invalid number of arguments.'
                return False
            load(0)

        else:

            # get path
            path = ''
            if len(cmd) == 3:
                path = cmd[2]
                if not os.path.isfile(path):
                    print 'load: Invalid path %s.' % path
                    return False
            else:
                path = cmdConfigFile

            
            if cmd[1].lower() == 'data':
                # load data
                if load(1, path) != ReturnCodes.Success:
                    print 'load: Could not load data from file %s.' % path
                    return False


            elif cmd[1].lower() == 'all':
                # load all pars
                if load(2, path) != ReturnCodes.Success:
                    print 'load: Invalid file content %s.' % path
                    return False
            else:
                # load pars
                # get pars from cmd string
                pars = [el.strip() for el in cmd[1].split(',')]
                ret = load(2, path, pars)
                if ret != 0:
                    if ret == ReturnCodes.ArgumentError:
                        print 'load: Invalid arguments.'
                    else:
                        print 'load: Invalid file content %s.' % path
                    return False

    elif cmd[0] == 'init':
        rc = sendInitCmd(cmdParams['tAddr'], cmdParams['cmdInd'] | cmdParams['iniInd'], 
                    cmdParams['scheme'], cmdParams['device'], cmdParams['freq1'],
                    cmdParams['freq2'], cmdParams['dutyCycle'], cmdParams['symLen'])
        if rc != ReturnCodes.Success:
            print '%s: Invalid parameter values. Error: %d.' % (cmd[0], rc)
            return False

    elif cmd[0] == 'modulate':
        args = [cmdParams['tAddr'], cmdParams['cmdInd'] | cmdParams['modInd'],
                cmdParams['bitrate'], None]
        dLen = cmdParams['length']
        dOff = cmdParams['offset']
        for i in range(cmdParams['count']):
            args[3] = cmdParams['data'][dOff : dOff + dLen]
            dOff += dLen
            
            rc = sendModCmd(*args)
            if rc != ReturnCodes.Success:
                print '%s: Invalid parameter values. Error: %d.' % (cmd[0], rc)
                return False

    return True

def main(argv=None):
    if argv is None:
        parser = argparse.ArgumentParser(description='modBulb VLC modulation tool')

        parser.add_argument('command', type=str, default='cli', nargs='?', 
                            help='the command to run, enter help for list of allowed commands')
        parser.add_argument('arg', type=str, default=[], nargs='*', 
                            help='the arguments to pass to the provided command')
        parser.add_argument('-c', '--config-file', type=str, default='modulation_tool.conf', 
                            help='path to the configuration file')
        parser.add_argument('-s', '--always-save', action='store_true',
                            help='write configuration to file after every operation')

        args = parser.parse_args()
        argv = (args.command, args.arg, args.config_file, args.always_save)
    
    global cmdConfigFile
    global cmdParams

    cmdConfigFile = argv[2]

    cmdParams = copy.deepcopy(defaultParams)

    if os.path.isfile(cmdConfigFile):
        tmp = _loadDict(cmdConfigFile)
        if tmp:
            cmdParams = tmp
    
    if argv[0].lower() == 'cli':
        global cliRunning
        cliRunning = True
        while cliRunning:
            try:
                cmd = raw_input('>> ').strip().split()
            except:
                return 0
            if cmdParser(cmd) and argv[3]:
                _saveDict(cmdConfigFile)
        return 0
            
    else:
        ret = cmdParser([argv[0]] + list(argv[1]))
        if ret and argv[3]:
            _saveDict(cmdConfigFile)
        if ret:
            return 0
        else:
            return 1

if __name__ == '__main__':
    exit(main())