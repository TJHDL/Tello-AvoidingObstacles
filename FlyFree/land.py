# -*- coding: utf-8 -*-
"""
Created on Tue Aug 25 16:12:40 2020

@author: 12830
"""


import socket

tello_ip=('192.168.10.1',8889)

with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as s:
    s.connect(tello_ip)
    s.sendall(b'land')
    
#%%
import socket

with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
    ip=('',8890)
    s.bind(ip)
    data = s.recvfrom(1024)
    print(data)
 
#%%
    
import socket

tello_ip=('192.168.10.1',8889)

def f(x:str):
    with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as s:
        s.connect(tello_ip)
        s.sendall(x.encode())


f('command')
f('streamon')