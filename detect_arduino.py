#!/usr/bin/env python

from __future__ import print_function

import re
import os

launch_file_path = 'launch/l2bot.launch'

# Get the new id
try:
    global ids
    ids = os.listdir('/dev/serial/by-id')
except:
    print('No board detected. Please make sure your board is plugged in.')
    exit(1)

if len(ids) < 1:
    print('No board detected. Please make sure your board is plugged in.')
    exit(1)
elif len(ids) > 1:
    print('Warining: multiple boards detected, using the first one.')

board = ids[0]

# Verify that it is actually a board
if re.search('usb-Arduino__www.arduino.cc__.*', board) is None:
    print('Non-arduino board detected: %s' % board)
    print('Would you like to use this id anyway? [y/n]: ', end='')
    choice = raw_input().lower()
    if not choice == 'y' and not choice == 'yes':
        print('Script aborted')
        exit(1)
else:
    print('Board detected: %s' % board)

# Open the launch file
launch = open(launch_file_path)
contents = launch.read()
launch.close()

# Find the start and end index of the board id
startidx = contents.find('usb-Arduino__')
endidx = contents.find('"', startidx)
oldboard = contents[startidx:endidx]

# Split the string at the old board
splits = contents.split(oldboard)

# Insert the new board
splits.insert(1, board)
new_contents = ''.join(splits)

# Write to the launch file
launch = open(launch_file_path, 'r+')
launch.write(new_contents)
launch.truncate()
launch.close()

print('complete')
