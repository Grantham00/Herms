#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Mar 18 04:49:40 2020

@author: ant
"""

import subprocess

subprocess.Popen('python2 /home/ant/Documents/Herms/cmd_serial.py'.split())
print("Launched cmd_serial.py")
subprocess.Popen('python2 /home/ant/Documents/Herms/listen_odom_imu.py'.split())
print("Launched listen_odom_imu.py")
