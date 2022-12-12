#!/usr/bin/env python3.8
#coding: utf-8
"""
================================================
A radio app for the AVerMedia AVerTV USB2.0 Plus
================================================
Version:    0.1
Author:     Sinan Güngör
"""

import libusb_package
import usb.core
import usb.util
import usb.backend.libusb1
import os
import sys
import platform


#-----------------------
VENDOR_ID = 0x07ca
PRODUCT_ID = 0x0036
#-----------------------

# For non-root linux users, a udev rule is necessary to access the device.
# See tools/udev-rule-07ca-0036.sh
#
# To use libusb on Windows, a driver must be installed for the Interface 0
# See https://github.com/libusb/libusb/wiki/Windows#Driver_Installation
#
# the following packages are to be installed:
# pyusb
# libusb-package 
# python-vlc
# pillow


#-----------------------
# Find device
#-----------------------

print("libusb path:\n\t{}".format(libusb_package.get_library_path())) 

was_kernel_driver_active = False
dev = None

if platform.system() == 'Windows':
    backend = usb.backend.libusb1.get_backend(find_library=libusb_package.find_library)    
    dev= usb.core.find(backend=backend, idVendor=VENDOR_ID, idProduct=PRODUCT_ID)
elif platform.system() == 'Linux':
    dev = usb.core.find(idVendor=VENDOR_ID, idProduct=PRODUCT_ID)
    # if the OS kernel already claimed the device
    if dev.is_kernel_driver_active(0) is True:
        # tell the kernel to detach
        dev.detach_kernel_driver(0)
        was_kernel_driver_active = True
        try:
            dev.detach_kernel_driver(0)
        except usb.core.USBError as e:
            sys.exit("Could not detatch kernel driver from interface({0}): {1}".format(0, str(e)))
else:
    dev = usb.core.find(idVendor=VENDOR_ID, idProduct=PRODUCT_ID)
    

 
if dev is None:
    raise ValueError('Device not found. Please ensure it is connected to the computer.')
    sys.exit(1)

#--------------------------------------------------------------------
#  Configuration (capture device for vlc)
#  If needed, fix the capture device setting in the config.xml file.
#--------------------------------------------------------------------

import xml.etree.ElementTree as ET
    
def read_configuration(file):
    tree=ET.parse(file)
    root=tree.getroot()
    global captureDevice
    captureDevice=root.find('captureDevice').text    
    
def write_configuration(file):
    root = ET.Element("configuration")
    CaptureDevice=ET.SubElement(root,"captureDevice").text="{}".format(captureDevice)
    tree = ET.ElementTree(root)
    tree.write(file,encoding="utf-8", xml_declaration=True)
    if platform.system() == 'Linux':
        cmd="xml format {file} > {file}.1".format(file=fileConfiguration)
        print(cmd)
        os.system(cmd)
        cmd="mv {file}.1 {file} ".format(file=fileConfiguration)
        print(cmd)
        os.system(cmd)  


captureDevice=""
fileConfiguration="config.xml"

# 
# to find out audio source in Linux:
# pactl list short sources 
#

isFile = os.path.isfile(fileConfiguration)
if isFile:
    read_configuration(fileConfiguration)
else:
    if platform.system() == 'Linux':
        captureDevice='pulse://alsa_input.usb-AVerMedia_AVerTV_USB_2.0_Plus-01.analog-stereo'
    if platform.system() == 'Windows':
        captureDevice= 'dshow:// :dshow-vdev=none :dshow-adev="Digital Audio Interface (AVerTV USB 2.0 Plus)" :live-caching=300'
    write_configuration(fileConfiguration)
        
#-------------------------------
#  VLC 
#-------------------------------
import vlc 

if platform.system() == 'Linux':    
    vlcOptions="-I 'dummy'"    
    vlcInstance = vlc.Instance(vlcOptions)
    print("")

if platform.system() == 'Windows':
    vlcOptions="--no-video"
    vlcInstance = vlc.Instance(vlcOptions)
    
 
audioPlayer = vlcInstance.media_player_new() 
capture_media = vlcInstance.media_new(captureDevice) 
audioPlayer.set_media(capture_media)
audioPlayer.play() 
 
 
#-------------------------------
#  libusb control transfers
#-------------------------------  
 
def ctrl_tx(index,value):
   ret = dev.ctrl_transfer(0x40,1,value,index,0,timeout=1000)
   
def ctrl_rx(index):
    ret = dev.ctrl_transfer(0xc0, 0, 0x0000, index,1,timeout=1000) 
    return ret[0]

def print_creg(index):
    value=ctrl_rx(index)
    #print("Index 0x{i:04x} : {v:04d} : 0x{v:02x} : {b}".format(i=index,v=value,b=byte2bits(value)))
    return value
 
#-----------------------
# Radio playlist
#----------------------- 

class radioStation():
    def __init__(self):
        self.frequency_MHz = None
        self.name  = None

def write_playlist(file):
    root = ET.Element("playlist")
    Playing=ET.SubElement(root,"playing").text="{}".format(station)
    for i in range (nStation):
        Station=ET.SubElement(root,"station")
        ET.SubElement(Station,"name").text=stations[i].name
        ET.SubElement(Station,"frequency_MHz").text="{freq:.2f}".format(freq=stations[i].frequency_MHz)
    tree = ET.ElementTree(root)
    tree.write(file,encoding="utf-8", xml_declaration=True)
    if platform.system() == 'Linux':
        cmd="xml format {file} > {file}.1".format(file=filePlaylist)
        print(cmd)
        os.system(cmd)
        cmd="mv {file}.1 {file} ".format(file=filePlaylist)
        print(cmd)
        os.system(cmd)

def read_playlist(file):
    tree=ET.parse(file)
    root=tree.getroot()
    global station
    global nStation
    global stations
    station=int(root[0].text)
    stations = []
    for s in root.findall('station'):
        stations.append(radioStation())  
        nStation=len(stations)
        stations[nStation-1].name=s[0].text
        stations[nStation-1].frequency_MHz=float(s[1].text)
    
def print_playlist():
    print('Station: {}'.format(station))
    print('# of Station: {}'.format(nStation))
    for i in range (nStation):
        print("{} : {}".format(stations[i].frequency_MHz, stations[i].name))


#----------------------- 

nStation = 0
station = 0
stations = []
filePlaylist="playlist.xml"

isFile = os.path.isfile(filePlaylist)
if isFile:
    read_playlist(filePlaylist)
else:
    newStation=radioStation()
    newStation.frequency_MHz=87.50
    newStation.name="Unknown Station"
    stations.insert(station,newStation)
    nStation+=1

#print_playlist()
  
#-----------------------
# Some bit operation
#----------------------- 

def set_bit(value, bit):
    return value | (1<<bit)

def clear_bit(value, bit):
    return value & ~(1<<bit)

def get_bit(value,bit):
    return ((value >> bit) & 1)   


#-----------------------
# Utils
#----------------------- 

def word2bits(x):
    return "{:04b}".format(x>>12)+" "+"{:04b}".format((x>>8)&0x0F)+" "+"{:04b}".format((x>>4)&0x0F)+" "+"{:04b}".format(x&0x0F)

def byte2bits(x):
    return "{:04b}".format(x>>4)+" "+"{:04b}".format(x&0x0F)



#===============================================================================    
# I2C stuff
#===============================================================================


def i2c_start():
    ctrl_tx(0x0003,0x0083)
    ctrl_tx(0x0001,0x0003)
    ctrl_tx(0x0000,0x00fc)
    ctrl_tx(0x0001,0x0002)
    ctrl_tx(0x0000,0x00ec)
    ctrl_tx(0x0000,0x00ec)
    ctrl_tx(0x0003,0x0083)

def i2c_bit_tx(b):
    if b :
        ctrl_tx(0x0001,0x0003) # 1
    else:
        ctrl_tx(0x0001,0x0002) # 0   
    ctrl_tx(0x0000,0x00fc)
    ctrl_tx(0x0000,0x00ec)

def i2c_byte_tx(byte):
    for x in range(7,-1,-1):
        b=get_bit(byte,x)
        i2c_bit_tx(b)

def i2c_ack_tx():   
    ctrl_tx(0x0003,0x0082)
    ctrl_tx(0x0001,0x0003)
    ctrl_tx(0x0000,0x00fc)
    ctrl_tx(0x0000,0x00ec)
    ctrl_tx(0x0000,0x00ec)
    ctrl_tx(0x0003,0x0083)   

def i2c_stop_tx():  
    ctrl_tx(0x0003,0x0082)
    ctrl_tx(0x0001,0x0003)
    ctrl_tx(0x0000,0x00fc)
    ctrl_tx(0x0000,0x00ec)
    ctrl_tx(0x0003,0x0083)
    ctrl_tx(0x0001,0x0002)
    ctrl_tx(0x0000,0x00fc)
    ctrl_tx(0x0001,0x0003) 


def i2c_bit_rx():
    ctrl_tx(0x0000,0x00fc)
    i1=ctrl_rx(0x0001)
    ctrl_tx(0x0000,0x00ec)    
    if(i1 == 0x03):
        b=1
    else:
        b=0
    return b

def i2c_byte_rx():
    byte=0x00
    for x in range(7,-1,-1):
        b=i2c_bit_rx()
        if b == 1 :
            byte=set_bit(byte,x)
    return byte 

def i2c_ack_rx():
    ctrl_tx(0x0003,0x0083)
    ctrl_tx(0x0001,0x0002)
    ctrl_tx(0x0000,0x00fc)
    ctrl_tx(0x0000,0x00ec)
    ctrl_tx(0x0001,0x0003)
    ctrl_tx(0x0003,0x0082)    

    
def i2c_stop_rx():   
    ctrl_tx(0x0003,0x0083)
    ctrl_tx(0x0001,0x0003)
    ctrl_tx(0x0000,0x00fc)
    ctrl_tx(0x0000,0x00ec)
    ctrl_tx(0x0003,0x0083)
    ctrl_tx(0x0001,0x0002)
    ctrl_tx(0x0000,0x00fc)
    ctrl_tx(0x0001,0x0003)


#===============================================================================
# TEA5767 stuff
#===============================================================================

#-------------------------------
# Write mode register values 
#-------------------------------

# First register
TEA5767_MUTE = 0x80         # Mutes output 
TEA5767_SM = 0x40           # Activates station search  

# Third register

TEA5767_SUD = 0x80          # Station search from bottom to up
TEA5767_SL_MASK = 0x60
TEA5767_SL_H = 0x60         # Searches with ADC output = 10
TEA5767_SL_M = 0x40         # Searches with ADC output = 7
TEA5767_SL_L = 0x20         # Searches with ADC output = 5
TEA5767_HLSI = 0x10         # If on, div=4*(Frf+Fif)/Fref otherwise, div=4*(Frf-Fif)/Freq)
TEA5767_MS= 0x08            # Disable stereo 
TEA5767_MR = 0x04           # Disable right channel and turns to mono 
TEA5767_ML = 0x02           # Disable left channel and turns to mono
TEA5767_SWP1 = 0x01         # 

# Fourth register 

TEA5767_SWP2 = 0x80         #
TEA5767_STDBY = 0x40        # Chips stops working. Only I2C bus remains on
TEA5767_BL = 0x20           # Japan freq 76-108 MHz . If disabled, 87.5-108 MHz
TEA5767_XTAL = 0x10         # Selected means 32.768 KHz freq as reference. Otherwise Xtal at 13 MHz 
TEA5767_SMUTE = 0x08        # Cuts weak signals
TEA5767_HCC = 0x04          # Activates high cut control
TEA5767_SNC = 0x02          # Activates stereo noise control
TEA5767_SI = 0x01           # If activate PORT 1 indicates SEARCH or else it is used as PORT1 */    

# Fifth register

TEA5767_PLLREF =  0x80      # By activating, it will use Xtal at 13 MHz as reference for divider
TEA5767_DTC = 0X40          # By activating, deemphasis=75us, or else, deemphasis of 50us 

#-------------------------------
# Read mode register values 
#-------------------------------

# First register 
TEA5767_READY_FLAG_MASK = 0x80  # Tuning completed or BL reached / Busy
TEA5767_BAND_LIMIT_MASK = 0X40  # Band limit reached / not reached
TEA5767_PLL_H_MASK = 0X3F       # Bits 0-5 for divider MSB after search or preset 

# Second register 
TEA5767_PLL_L_MASK = 0XFF       # Bits 0-7 for divider LSB after search or preset 

# Third register
TEA5767_STEREO_MASK = 0x80      # Stereo reception / Mono reception
TEA5767_IF_CNTR_MASK = 0x7f     # IF counter result 

# Fourth register 
TEA5767_ADC_LEVEL_MASK = 0xf0   # Level ADC output

TEA5767_CHIP_ID_MASK = 0x0f     # Should be 0 

# Fifth register 
TEA5767_RESERVED_MASK = 0xff    # Reserved 

#------------------------------------------------------------------------------- 
tea5767_registers_r=[0,0,0,0,0]
tea5767_registers_w=[0,0,0,0,0]
#-------------------------------------------------------------------------------
    
def tea5767_read():
    global tea5767_registers_r
    i2c_start()
    i2c_byte_tx(0xc1)
    i2c_ack_rx()
    tea5767_registers_r[0]=i2c_byte_rx()
    i2c_ack_rx()
    tea5767_registers_r[1]=i2c_byte_rx()
    i2c_ack_rx()
    tea5767_registers_r[2]=i2c_byte_rx()
    i2c_ack_rx()
    tea5767_registers_r[3]=i2c_byte_rx()
    i2c_ack_rx()
    tea5767_registers_r[4]=i2c_byte_rx()
    i2c_stop_rx()   
  
def tea5767_write():  
    i2c_start()
    i2c_byte_tx(0xc0)
    i2c_ack_tx()
    i2c_byte_tx(tea5767_registers_w[0])
    i2c_ack_tx()
    i2c_byte_tx(tea5767_registers_w[1])
    i2c_ack_tx()
    i2c_byte_tx(tea5767_registers_w[2])
    i2c_ack_tx()
    i2c_byte_tx(tea5767_registers_w[3])
    i2c_ack_tx()
    i2c_byte_tx(tea5767_registers_w[4])
    i2c_stop_tx()    
    
def radio_status():
    tea5767_read()
    print_radio_status()    
    
def print_radio_status():
    TEA5767_Ready=(tea5767_registers_r[0]&TEA5767_READY_FLAG_MASK)>>7
    TEA5767_Band_Limits=(tea5767_registers_r[0]&TEA5767_BAND_LIMIT_MASK)>>6  
    TEA5767_PLL=((tea5767_registers_r[0]&TEA5767_PLL_H_MASK)<<8)|tea5767_registers_r[1]&TEA5767_PLL_L_MASK
    TEA5767_Stereo=(tea5767_registers_r[2]&TEA5767_STEREO_MASK)>>7
    TEA5767_IF_Counter=tea5767_registers_r[2]&TEA5767_IF_CNTR_MASK
    TEA5767_ADC_Level=(tea5767_registers_r[3]&TEA5767_ADC_LEVEL_MASK)>>4
    print("\nReady flag: {rf}".format(rf=TEA5767_Ready))
    print("Band limit flag: {blf}".format(blf=TEA5767_Band_Limits))
    frequencyMHz = ((TEA5767_PLL*50.0)/4.0+225.0)/1000
    print("Frequency: {f:.2f} MHz".format(f=frequencyMHz))
    print("Stereo flag: {sf}".format(sf=TEA5767_Stereo))
    print("ADC level: {adc}".format(adc=TEA5767_ADC_Level))   

def print_radio_control():  
    TEA5767_Mute=(tea5767_registers_w[0]&TEA5767_MUTE)>>7
    TEA5767_SearchMode=(tea5767_registers_w[0]&TEA5767_SM)>>6
    TEA5767_PLL=((tea5767_registers_w[0]&TEA5767_PLL_H_MASK)<<8)|tea5767_registers_w[1]&TEA5767_PLL_L_MASK
    ctrl_frequencyMHz = ((TEA5767_PLL*50.0)/4.0+225.0)/1000
    TEA5767_Search_UpDown = (tea5767_registers_w[2]&TEA5767_SUD)>>7
    TEA5767_Search_Level = (tea5767_registers_w[2]&TEA5767_SL_MASK)>>5
    TEA5767_High_Low_Injection=(tea5767_registers_w[2]&TEA5767_HLSI)>>4
    TEA5767_Mono_Stereo=(tea5767_registers_w[2]&TEA5767_MS)>>3
    TEA5767_Mute_Right=(tea5767_registers_w[2]&TEA5767_MR)>>2
    TEA5767_Mute_Left=(tea5767_registers_w[2]&TEA5767_ML)>>1
    TEA5767_SoftwareProgrammable_P1=(tea5767_registers_w[2]&TEA5767_SWP1)
    TEA5767_SoftwareProgrammable_P2=(tea5767_registers_w[3]&TEA5767_SWP2)>>7
    TEA5767_Standby=(tea5767_registers_w[3]&TEA5767_STDBY)>>6
    TEA5767_Band_Limits=(tea5767_registers_w[3]&TEA5767_BL)>>5
    TEA5767_Clock_Frequency=(tea5767_registers_w[3]&TEA5767_XTAL)>>4
    TEA5767_Soft_Mute=(tea5767_registers_w[3]&TEA5767_SMUTE)>>3
    TEA5767_High_Cut_Control=(tea5767_registers_w[3]&TEA5767_HCC)>>2
    TEA5767_Stereo_Noise_Cancelling=(tea5767_registers_w[3]&TEA5767_SNC)>>1
    TEA5767_Search_Indicator=(tea5767_registers_w[3]&TEA5767_SI)
    TEA5767_PLL_Reference=(tea5767_registers_w[4]&TEA5767_PLLREF)>>7
    TEA5767_Deemphasis_Time_Constant=(tea5767_registers_w[4]&TEA5767_DTC)>>6
    
    print("\n")
    print("Mute: {m}".format(m=TEA5767_Mute))
    print("Search Mode: {s}".format(s=TEA5767_SearchMode))
    print("PLL: {pll} ({f:.2f} MHz)".format(pll=TEA5767_PLL, f=ctrl_frequencyMHz))
    print("Search Up: {sud}".format(sud=TEA5767_Search_UpDown))
    print("Search Level: {sl}".format(sl=TEA5767_Search_Level))
    print("High/Low Side Injection: {hlsi}".format(hlsi=TEA5767_High_Low_Injection))
    print("Mute Right: {mr}".format(mr=TEA5767_Mute_Right))
    print("Mute Left: {ml}".format(ml=TEA5767_Mute_Left))
    print("Software Programmable Port 1: {p1}".format(p1=TEA5767_SoftwareProgrammable_P1))
    print("Software Programmable Port 2: {p2}".format(p2=TEA5767_SoftwareProgrammable_P2))
    print("Standby: {stdby}".format(stdby=TEA5767_Standby))
    print("Band Limits: {bl}".format(bl=TEA5767_Band_Limits))
    print("Clock frenquency: {xtal}".format(xtal=TEA5767_Clock_Frequency))
    print("Soft Mute: {sm}".format(sm=TEA5767_Soft_Mute))
    print("High Cut Control: {hcc}".format(hcc=TEA5767_High_Cut_Control))
    print("Stereo Noise Cancelling: {snc}".format(snc=TEA5767_Stereo_Noise_Cancelling))
    print("Search Indicator: {si}".format(si=TEA5767_Search_Indicator))
    print("PLL Reference: {pllref}".format(pllref=TEA5767_PLL_Reference))
    print("Deemphasis Time Constant: {dtc}".format(dtc=TEA5767_Deemphasis_Time_Constant))
    print("\n")

#===============================================================================

frequencyMHz_min=87.5
frequencyMHz_max=108

def radio_frequency_limits(bandLimits):
    global requencyMHz_min
    global requencyMHz_max
    if bandLimits == 0 :
        frequencyMHz_min=87.5
        frequencyMHz_max=108
    else:
        frequencyMHz_min=76
        frequencyMHz_max=91

def radio_signal_level():
    tea5767_read()
    TEA5767_ADC_Level=(tea5767_registers_r[3]&TEA5767_ADC_LEVEL_MASK)>>4
    print("ADC level: {adc}".format(adc=TEA5767_ADC_Level))
    return TEA5767_ADC_Level

def radio_set_frequency(frequencyMHz):
    global tea5767_registers_w
    # print("Frequency %.2f MHz" % frequencyMHz)
    wPLL = round((frequencyMHz*1000.0-225.0)*4.0/50.0)
    PLL_L = wPLL & 0x00FF
    PLL_H = (wPLL >> 8) & 0x003F   
    tea5767_registers_w[0]=(tea5767_registers_w[0]&~TEA5767_PLL_H_MASK)|PLL_H
    tea5767_registers_w[1]=PLL_L
    tea5767_write()

def radio_play_station (station):
    global  frequencyMHz 
    frequencyMHz = stations[station].frequency_MHz
    radio_set_frequency(frequencyMHz)
    print(  "\n%0.2f" % stations[station].frequency_MHz,"MHz (%s) is playing" % stations[station].name)     
    
def radio_mute():
    global tea5767_registers_w
    tea5767_registers_w[0]=set_bit(tea5767_registers_w[0],7)
    tea5767_write()    
    
def radio_unmute():
    global tea5767_registers_w
    tea5767_registers_w[0]=clear_bit(tea5767_registers_w[0],7)
    tea5767_write()
    
def radio_set_mono():
    global tea5767_registers_w
    tea5767_registers_w[2]=set_bit(tea5767_registers_w[2],3)
    tea5767_write()    
    
def radio_set_stereo():
    global tea5767_registers_w
    tea5767_registers_w[2]=clear_bit(tea5767_registers_w[2],3)
    tea5767_write()  
    
def radio_mute_left():
    global tea5767_registers_w
    tea5767_registers_w[2]=set_bit(tea5767_registers_w[2],1)
    tea5767_write()    
    
def radio_unmute_left():
    global tea5767_registers_w
    tea5767_registers_w[2]=clear_bit(tea5767_registers_w[2],1)
    tea5767_write() 
    
def radio_mute_right():
    global tea5767_registers_w
    tea5767_registers_w[2]=set_bit(tea5767_registers_w[2],2)
    tea5767_write()    
    
def radio_unmute_right():
    global tea5767_registers_w
    tea5767_registers_w[2]=clear_bit(tea5767_registers_w[2],2)
    tea5767_write()     

def radio_set_Band_Limits_Japan():
    global tea5767_registers_w
    tea5767_registers_w[3]=set_bit(tea5767_registers_w[3],5)
    tea5767_write()    
    
def radio_Band_Limits_EU_USA():
    global tea5767_registers_w
    tea5767_registers_w[3]=clear_bit(tea5767_registers_w[3],5)
    tea5767_write() 

def radio_set_search_mode():
    global tea5767_registers_w
    tea5767_registers_w[0]=set_bit(tea5767_registers_w[0],6)
    tea5767_write() 
def radio_clear_search_mode():
    global tea5767_registers_w
    tea5767_registers_w[0]=clear_bit(tea5767_registers_w[0],6)
    tea5767_write() 
    
def radio_search_up():
    global tea5767_registers_w
    tea5767_registers_w[2]=set_bit(tea5767_registers_w[2],7)
    tea5767_write()    
    
def radio_search_down():
    global tea5767_registers_w
    tea5767_registers_w[2]=clear_bit(tea5767_registers_w[2],7)
    tea5767_write()     
    
def radio_set_SSL(ssl):
    global tea5767_registers_w
    tea5767_registers_w[0]=(tea5767_registers_w[0]&~TEA5767_SL_MASK)|ssl


def radio_SNC_on():
    global tea5767_registers_w
    tea5767_registers_w[3]=set_bit(tea5767_registers_w[3],1)
    tea5767_write()    
    
def radio_SNC_off():
    global tea5767_registers_w
    tea5767_registers_w[3]=clear_bit(tea5767_registers_w[3],1)
    tea5767_write() 

def radio_HCC_on():
    global tea5767_registers_w
    tea5767_registers_w[3]=set_bit(tea5767_registers_w[3],2)
    tea5767_write()    
    
def radio_HCC_off():
    global tea5767_registers_w
    tea5767_registers_w[3]=clear_bit(tea5767_registers_w[3],2)
    tea5767_write() 


def radio_SoftMute_on():
    global tea5767_registers_w
    tea5767_registers_w[3]=set_bit(tea5767_registers_w[3],3)
    tea5767_write()    
    
def radio_SoftMute_off():
    global tea5767_registers_w
    tea5767_registers_w[3]=clear_bit(tea5767_registers_w[3],3)
    tea5767_write() 


#===============================================================================
# Realtek ACL655 Audio Codec stuff
#===============================================================================

vol=0

#-------------------------------
# Mixer registers
#-------------------------------
MASTER_VOL=0x02         # MX02 (Front) Master volume
MIC_VOL=0x0e            # MX0E MIC Volume
LINE_IN_VOL=0x10        # MX10 LINE_IN Volume
CD_VOL=0x12             # MX12 CD Volume
AUX_VOL=0x16            # MX16 AUX Volume
RECORD_SELECT=0x1a      # MX1A Record Select
RECORD_GAIN=0x1c        # MX1C Record Gain    


def get_MX(mx):
    ctrl_tx(0x0504,mx)
    ctrl_tx(0x0500,0x008b)
    MX_L=ctrl_rx(0x0502)
    MX_H=ctrl_rx(0x0503)           
    MX=(MX_H<<8)|MX_L
    return MX

def set_MX(mx,value):
    ctrl_tx(0x0504,mx)
    MX_L=value&0x00FF
    MX_H=(value>>8)&0x00FF
    ctrl_tx(0x0502,MX_L)
    ctrl_tx(0x0503,MX_H)
    ctrl_tx(0x0500,0x008c)


def set_volume(vol):
    if vol > 15 :
        vol=15
    if vol < 0 :
        vol=0
    LRG=vol
    RRG=vol
    RG=(LRG<<8)|RRG
    set_MX(RECORD_GAIN,RG)
    print("Volume: {}".format(vol))
    
def set_volume_LR(volL,volR):
    if volL > 15 :
        volL=15
    if volL < 0 :
        volL=0
    if volR > 15 :
        volR=15
    if volR < 0 :
        volR=0  
 
    LRG=volL
    RRG=volR
    RG=(LRG<<8)|RRG
    set_MX(RECORD_GAIN,RG)
    print("Volume: {l} L {r} R".format(l=volL,r=volR))    
    
def mute():
    RG=get_MX(RECORD_GAIN)
    RG=set_bit(RG,15)
    set_MX(RECORD_GAIN,RG)
    
def unmute():
    RG=get_MX(RECORD_GAIN)
    RG=clear_bit(RG,15)
    set_MX(RECORD_GAIN,RG)


#===============================================================================
# Others
#===============================================================================

def LED_on():
    reg0x00=ctrl_rx(0x0000)
    reg0x00=set_bit(reg0x00,6)
    ctrl_tx(0x0000,reg0x00)
    ctrl_tx(0x0002,0x00e8)

def LED_off():
    reg0x00=ctrl_rx(0x0000)
    reg0x00=clear_bit(reg0x00,6)
    ctrl_tx(0x0000,reg0x00)
    ctrl_tx(0x0002,0x00e8)

#===============================================================================

def init_driver():
    if dev.get_active_configuration().bConfigurationValue != 1 :
        dev.set_configuration(1)
    dev.set_interface_altsetting(interface = 0, alternate_setting = 0) 
    
    ctrl_tx(0x0000,0x0060)
    ctrl_tx(0x0002,0x00e8)
    ctrl_tx(0x0003,0x0083)
    ctrl_tx(0x0002,0x00ef)
    
    ctrl_tx(0x0000,0x0064)
    ctrl_tx(0x0000,0x0064)
    ctrl_tx(0x0000,0x0066)
    ctrl_tx(0x0000,0x0067)
    ctrl_tx(0x0000,0x0066)
    ctrl_tx(0x0000,0x0064)
    ctrl_tx(0x0000,0x0065)
    ctrl_tx(0x0000,0x0064)
    ctrl_tx(0x0000,0x0064)
    ctrl_tx(0x0000,0x0065)
    ctrl_tx(0x0000,0x0064)
    ctrl_tx(0x0000,0x0064)
    ctrl_tx(0x0000,0x0065)
    ctrl_tx(0x0000,0x0064)
    ctrl_tx(0x0000,0x0064)
    ctrl_tx(0x0000,0x0065)
    ctrl_tx(0x0000,0x0064)
    ctrl_tx(0x0000,0x0064)
    ctrl_tx(0x0000,0x0065)
    ctrl_tx(0x0000,0x0064)
    ctrl_tx(0x0000,0x0064)
    ctrl_tx(0x0000,0x0065)
    ctrl_tx(0x0000,0x0064)
    ctrl_tx(0x0000,0x0064)
    ctrl_tx(0x0000,0x0065)
    ctrl_tx(0x0000,0x0064)
    ctrl_tx(0x0000,0x0064)
    ctrl_tx(0x0000,0x0065)
    ctrl_tx(0x0000,0x0061)
    
    ctrl_tx(0x0002,0x00ef)
    ctrl_tx(0x0000,0x0061)
    ctrl_tx(0x0203,0x00ba)
    ctrl_tx(0x0002,0x00e8)
    ctrl_tx(0x0003,0x0083)
    ctrl_tx(0x0000,0x0060)
    ctrl_tx(0x0001,0x0002)
    ctrl_tx(0x000d,0x0000)
    ctrl_tx(0x000f,0x0002)
    ctrl_tx(0x0103,0x0000)
    
    ctrl_tx(0x0300,0x0012)
    ctrl_tx(0x0350,0x002d)
    ctrl_tx(0x0351,0x0001)
    ctrl_tx(0x0352,0x0000)
    ctrl_tx(0x0353,0x0000)
    ctrl_tx(0x0300,0x0080)
    
    ctrl_tx(0x0018,0x0010)
    ctrl_tx(0x0019,0x0000)
    ctrl_tx(0x0202,0x001e)
    ctrl_tx(0x0110,0x0050)
    ctrl_tx(0x0111,0x0000)
    ctrl_tx(0x0112,0x0019)
    ctrl_tx(0x0113,0x0000)
    ctrl_tx(0x0114,0x0050)
    ctrl_tx(0x0115,0x0005)
    ctrl_tx(0x0116,0x0009)
    ctrl_tx(0x0117,0x0001)
    ctrl_tx(0x0100,0x0033)
    
    ctrl_tx(0x0203,0x00ba)
    
    ctrl_tx(0x0204,0x007f)
    ctrl_tx(0x0205,0x0000)
    ctrl_tx(0x0200,0x0001)
    
    ctrl_tx(0x0208,0x0080)
    ctrl_tx(0x0200,0x0020)
    
    ctrl_tx(0x0208,0x0081)
    ctrl_tx(0x0200,0x0020)
    
    ctrl_tx(0x0203,0x00a0)
    
    ctrl_tx(0x0208,0x003c)
    ctrl_tx(0x0200,0x0020)
    
    ctrl_tx(0x0203,0x00ba)
    
    ctrl_tx(0x0002,0x00e8)
    ctrl_tx(0x0003,0x0083)
    ctrl_tx(0x0000,0x0060)
    ctrl_tx(0x0001,0x0002)
    ctrl_tx(0x000d,0x0000)
    ctrl_tx(0x000f,0x0002)
    ctrl_tx(0x0103,0x0000)
    ctrl_tx(0x0300,0x0012)
    ctrl_tx(0x0350,0x002d)
    ctrl_tx(0x0351,0x0001)
    ctrl_tx(0x0352,0x0000)
    ctrl_tx(0x0353,0x0000)
    ctrl_tx(0x0300,0x0080)
    ctrl_tx(0x0018,0x0010)
    ctrl_tx(0x0019,0x0000)
    
    ctrl_tx(0x0202,0x001e)
    
    ctrl_tx(0x0110,0x0050)
    ctrl_tx(0x0111,0x0000)
    ctrl_tx(0x0112,0x0019)
    ctrl_tx(0x0113,0x0000)
    ctrl_tx(0x0114,0x0050)
    ctrl_tx(0x0115,0x0005)
    ctrl_tx(0x0116,0x0009)
    ctrl_tx(0x0117,0x0001)
    ctrl_tx(0x0100,0x0033)
    
    ctrl_tx(0x0204,0x0008)
    ctrl_tx(0x0205,0x0008)
    ctrl_tx(0x0200,0x0005)
    
    ctrl_tx(0x0204,0x0028)
    ctrl_tx(0x0205,0x000c)
    ctrl_tx(0x0200,0x0005)
    
    ctrl_tx(0x0204,0x0030)
    ctrl_tx(0x0205,0x0000)
    ctrl_tx(0x0200,0x0005)
    
    ctrl_tx(0x0204,0x000f)
    ctrl_tx(0x0205,0x000a)
    ctrl_tx(0x0200,0x0005)
    
    ctrl_tx(0x0500,0x0094)
    ctrl_tx(0x0500,0x008c)
    ctrl_tx(0x0506,0x0001)
    ctrl_tx(0x0507,0x0000)
    
    ctrl_tx(0x0100,0x0033)
    ctrl_tx(0x0000,0x006c)
    
    ctrl_tx(0x0204,0x0000)
    ctrl_tx(0x0205,0x0002)
    ctrl_tx(0x0200,0x0005)
    
    ctrl_tx(0x0204,0x0003)
    ctrl_tx(0x0205,0x006f)
    ctrl_tx(0x0200,0x0005)
    
    ctrl_tx(0x0100,0x0033)
    
    ctrl_tx(0x0504,0x0010)
    ctrl_tx(0x0500,0x008b)
    
    ctrl_tx(0x0504,0x0010)
    ctrl_tx(0x0502,0x0008)
    ctrl_tx(0x0503,0x0088)
    ctrl_tx(0x0500,0x008c)
    
    ctrl_tx(0x0504,0x0012)
    ctrl_tx(0x0500,0x008b)
    
    ctrl_tx(0x0504,0x0012)
    ctrl_tx(0x0502,0x0008)
    ctrl_tx(0x0503,0x0008)
    ctrl_tx(0x0500,0x008c)
    
    ctrl_tx(0x0504,0x000e)
    ctrl_tx(0x0500,0x008b)
    
    ctrl_tx(0x0504,0x000e)
    ctrl_tx(0x0502,0x0008)
    ctrl_tx(0x0503,0x0000)
    ctrl_tx(0x0500,0x008c)
    
    ctrl_tx(0x0504,0x0016)
    ctrl_tx(0x0500,0x008b)
    
    ctrl_tx(0x0504,0x0016)
    ctrl_tx(0x0502,0x0008)
    ctrl_tx(0x0503,0x0008)
    ctrl_tx(0x0500,0x008c)
    
    ctrl_tx(0x0504,0x001a)
    ctrl_tx(0x0502,0x0001)
    ctrl_tx(0x0503,0x0001)
    ctrl_tx(0x0500,0x008c)
    
    ctrl_tx(0x0504,0x001c)
    ctrl_tx(0x0502,0x0000)
    ctrl_tx(0x0503,0x0000)
    ctrl_tx(0x0500,0x008c)


####################################################################################

def init_radio():
    dev.set_interface_altsetting(interface = 0, alternate_setting = 5) 
    ctrl_tx(0x0203,0x00ba)
    ctrl_tx(0x0002,0x00e8)
    ctrl_tx(0x0003,0x0083)
    ctrl_tx(0x0000,0x0060)
    ctrl_tx(0x0001,0x0002)
    ctrl_tx(0x000d,0x0000)
    ctrl_tx(0x000f,0x0002)
    ctrl_tx(0x0103,0x0000)
    ctrl_tx(0x0300,0x0012)
    ctrl_tx(0x0350,0x002d)
    ctrl_tx(0x0351,0x0001)
    ctrl_tx(0x0352,0x0000)
    ctrl_tx(0x0353,0x0000)
    ctrl_tx(0x0300,0x0080)
    ctrl_tx(0x0018,0x0010)
    ctrl_tx(0x0019,0x0000)
    ctrl_tx(0x0202,0x001e)
    ctrl_tx(0x0110,0x0050)
    ctrl_tx(0x0111,0x0000)
    ctrl_tx(0x0112,0x0019)
    ctrl_tx(0x0113,0x0000)
    ctrl_tx(0x0114,0x0050)
    ctrl_tx(0x0115,0x0005)
    ctrl_tx(0x0116,0x0009)
    ctrl_tx(0x0117,0x0001)
    ctrl_tx(0x0100,0x00b3)
    
    ctrl_tx(0x0204,0x0008)
    ctrl_tx(0x0205,0x0008)
    ctrl_tx(0x0200,0x0005)
    
    ctrl_tx(0x0204,0x0028)
    ctrl_tx(0x0205,0x000c)
    ctrl_tx(0x0200,0x0005)
    
    ctrl_tx(0x0204,0x0030)
    ctrl_tx(0x0205,0x0000)
    ctrl_tx(0x0200,0x0005)
    
    ctrl_tx(0x0204,0x000f)
    ctrl_tx(0x0205,0x000a)
    ctrl_tx(0x0200,0x0005)
    
    ctrl_tx(0x0100,0x0033)
    ctrl_tx(0x0103,0x0000)
    ctrl_tx(0x0100,0x00b3)
    ctrl_tx(0x0106,0x0000)
    ctrl_tx(0x0100,0x0033)
    ctrl_tx(0x0000,0x006c)
    
    ctrl_tx(0x0204,0x0000)
    ctrl_tx(0x0205,0x0002)
    ctrl_tx(0x0200,0x0005)
    
    ctrl_tx(0x0204,0x0003)
    ctrl_tx(0x0205,0x006f)
    ctrl_tx(0x0200,0x0005)
    
    ctrl_tx(0x0100,0x00b3)
    ctrl_tx(0x0203,0x00ba)
    ctrl_tx(0x0002,0x00e8)
    ctrl_tx(0x0003,0x0083)
    ctrl_tx(0x0000,0x0060)
    ctrl_tx(0x0001,0x0002)
    ctrl_tx(0x000d,0x0000)
    ctrl_tx(0x000f,0x0002)
    ctrl_tx(0x0103,0x0000)
    ctrl_tx(0x0300,0x0012)
    ctrl_tx(0x0350,0x002d)
    ctrl_tx(0x0351,0x0001)
    ctrl_tx(0x0352,0x0000)
    ctrl_tx(0x0353,0x0000)
    ctrl_tx(0x0300,0x0080)
    ctrl_tx(0x0018,0x0010)
    ctrl_tx(0x0019,0x0000)
    ctrl_tx(0x0202,0x001e)
    ctrl_tx(0x0110,0x0050)
    ctrl_tx(0x0111,0x0000)
    ctrl_tx(0x0112,0x0019)
    ctrl_tx(0x0113,0x0000)
    ctrl_tx(0x0114,0x0050)
    ctrl_tx(0x0115,0x0005)
    ctrl_tx(0x0116,0x0009)
    ctrl_tx(0x0117,0x0001)
    ctrl_tx(0x0100,0x00b3)
    
    ctrl_tx(0x0204,0x0008)
    ctrl_tx(0x0205,0x0008)
    ctrl_tx(0x0200,0x0005)
    
    ctrl_tx(0x0204,0x0028)
    ctrl_tx(0x0205,0x000c)
    ctrl_tx(0x0200,0x0005)
    
    ctrl_tx(0x0204,0x0030)
    ctrl_tx(0x0205,0x0000)
    ctrl_tx(0x0200,0x0005)
    
    ctrl_tx(0x0204,0x000f)
    ctrl_tx(0x0205,0x000a)
    ctrl_tx(0x0200,0x0005)
    
    ctrl_tx(0x0100,0x0033)
    ctrl_tx(0x0103,0x0000)
    ctrl_tx(0x0100,0x00b3)
    ctrl_tx(0x0100,0x0033)
    ctrl_tx(0x0000,0x006c)
    
    ctrl_tx(0x0204,0x0000)
    ctrl_tx(0x0205,0x0002)
    ctrl_tx(0x0200,0x0005)
    
    ctrl_tx(0x0204,0x0003)
    ctrl_tx(0x0205,0x006f)
    ctrl_tx(0x0200,0x0005)
    
    ctrl_tx(0x0100,0x00b3)
    ctrl_tx(0x0110,0x0050)
    ctrl_tx(0x0111,0x0000)
    ctrl_tx(0x0112,0x0019)
    ctrl_tx(0x0113,0x0000)
    ctrl_tx(0x0114,0x0550)
    ctrl_tx(0x0115,0x0005)
    ctrl_tx(0x0116,0x0109)
    ctrl_tx(0x0117,0x0001)
    ctrl_tx(0x0002,0x0078)
    ctrl_tx(0x0100,0x0033)
    ctrl_tx(0x0100,0x00b3)
    
    ctrl_tx(0x0204,0x0009)
    ctrl_tx(0x0205,0x007e)
    ctrl_tx(0x0200,0x0005)
    
    ctrl_tx(0x0208,0x0009)
    ctrl_tx(0x0200,0x0020)
    
    ctrl_tx(0x0204,0x000c)
    ctrl_tx(0x0205,0x0080)
    ctrl_tx(0x0200,0x0005)
    
    ctrl_tx(0x0208,0x000c)
    ctrl_tx(0x0200,0x0020)
    
    ctrl_tx(0x0204,0x000b)
    ctrl_tx(0x0205,0x0000)
    ctrl_tx(0x0200,0x0005)
    
    ctrl_tx(0x0208,0x000b)
    ctrl_tx(0x0200,0x0020)
    
    ctrl_tx(0x0204,0x000a)
    ctrl_tx(0x0205,0x009b)
    ctrl_tx(0x0200,0x0005)
    
    ctrl_tx(0x0208,0x000a)
    ctrl_tx(0x0200,0x0020)
    
    ctrl_tx(0x0204,0x0008)
    ctrl_tx(0x0205,0x0008)
    ctrl_tx(0x0200,0x0005)
    
    ctrl_tx(0x0208,0x0008)
    ctrl_tx(0x0200,0x0020)
    
    ctrl_tx(0x0002,0x0078)
    ctrl_tx(0x0100,0x0033)
    
    ctrl_tx(0x0100,0x00b3)
    ctrl_tx(0x0116,0x0000)
    ctrl_tx(0x0117,0x0000)
    
    ctrl_tx(0x0100,0x00b3)
    ctrl_tx(0x0116,0x0009)
    ctrl_tx(0x0117,0x0001)

####################################################################################

def init_freq():
    ctrl_tx(0x0003,0x0082)
    ctrl_tx(0x0001,0x0003)
    ctrl_tx(0x0000,0x00fc)
    ctrl_tx(0x0000,0x00ec)
    ctrl_tx(0x0000,0x00ec)
    ctrl_tx(0x0003,0x0083)
    # --- Send Byte xxxx ------------------
    # --- 0x00
    ctrl_tx(0x0001,0x0002)
    ctrl_tx(0x0000,0x00fc)
    ctrl_tx(0x0000,0x00ec)
    
    ctrl_tx(0x0001,0x0002)
    ctrl_tx(0x0000,0x00fc)
    ctrl_tx(0x0000,0x00ec)
    
    ctrl_tx(0x0001,0x0002)
    ctrl_tx(0x0000,0x00fc)
    ctrl_tx(0x0000,0x00ec)
    
    ctrl_tx(0x0001,0x0002)
    ctrl_tx(0x0000,0x00fc)
    ctrl_tx(0x0000,0x00ec)
    
    ctrl_tx(0x0001,0x0002)
    ctrl_tx(0x0000,0x00fc)
    ctrl_tx(0x0000,0x00ec)
    
    ctrl_tx(0x0001,0x0002)
    ctrl_tx(0x0000,0x00fc)
    ctrl_tx(0x0000,0x00ec)
    
    ctrl_tx(0x0001,0x0002)
    ctrl_tx(0x0000,0x00fc)
    ctrl_tx(0x0000,0x00ec)
    
    ctrl_tx(0x0001,0x0002)
    ctrl_tx(0x0000,0x00fc)
    ctrl_tx(0x0000,0x00ec)
    
    #-------------------------------
    
    ctrl_tx(0x0003,0x0082)
    
    ctrl_tx(0x0001,0x0003)
    ctrl_tx(0x0000,0x00fc)
    ctrl_tx(0x0000,0x00ec)
    
    ctrl_tx(0x0000,0x00ec)
    ctrl_tx(0x0003,0x0083)
    
    # --- Send Byte xxxx ------------------
    # --- 0x00
    ctrl_tx(0x0001,0x0002)
    ctrl_tx(0x0000,0x00fc)
    ctrl_tx(0x0000,0x00ec)
    
    ctrl_tx(0x0001,0x0002)
    ctrl_tx(0x0000,0x00fc)
    ctrl_tx(0x0000,0x00ec)
    
    ctrl_tx(0x0001,0x0002)
    ctrl_tx(0x0000,0x00fc)
    ctrl_tx(0x0000,0x00ec)
    
    ctrl_tx(0x0001,0x0002)
    ctrl_tx(0x0000,0x00fc)
    ctrl_tx(0x0000,0x00ec)
    
    ctrl_tx(0x0001,0x0002)
    ctrl_tx(0x0000,0x00fc)
    ctrl_tx(0x0000,0x00ec)
    
    ctrl_tx(0x0001,0x0002)
    ctrl_tx(0x0000,0x00fc)
    ctrl_tx(0x0000,0x00ec)
    
    ctrl_tx(0x0001,0x0002)
    ctrl_tx(0x0000,0x00fc)
    ctrl_tx(0x0000,0x00ec)
    
    ctrl_tx(0x0001,0x0002)
    ctrl_tx(0x0000,0x00fc)
    ctrl_tx(0x0000,0x00ec)
    
    #-------------------------------
    
    ctrl_tx(0x0003,0x0082)
    ctrl_tx(0x0001,0x0003)
    ctrl_tx(0x0000,0x00fc)
    ctrl_tx(0x0000,0x00ec)
    ctrl_tx(0x0000,0x00ec)
    ctrl_tx(0x0003,0x0083)
    
    # --- Send Byte xxxx ------------------
    # --- 0x00
    
    ctrl_tx(0x0001,0x0002)
    ctrl_tx(0x0000,0x00fc)
    ctrl_tx(0x0000,0x00ec)
    
    ctrl_tx(0x0001,0x0002)
    ctrl_tx(0x0000,0x00fc)
    ctrl_tx(0x0000,0x00ec)
    
    ctrl_tx(0x0001,0x0002)
    ctrl_tx(0x0000,0x00fc)
    ctrl_tx(0x0000,0x00ec)
    
    ctrl_tx(0x0001,0x0002)
    ctrl_tx(0x0000,0x00fc)
    ctrl_tx(0x0000,0x00ec)
    
    ctrl_tx(0x0001,0x0002)
    ctrl_tx(0x0000,0x00fc)
    ctrl_tx(0x0000,0x00ec)
    
    ctrl_tx(0x0001,0x0002)
    ctrl_tx(0x0000,0x00fc)
    ctrl_tx(0x0000,0x00ec)
    
    ctrl_tx(0x0001,0x0002)
    ctrl_tx(0x0000,0x00fc)
    ctrl_tx(0x0000,0x00ec)
    
    ctrl_tx(0x0001,0x0002)
    ctrl_tx(0x0000,0x00fc)
    ctrl_tx(0x0000,0x00ec)
    
    #-------------------------------
    ctrl_tx(0x0003,0x0082)
    
    ctrl_tx(0x0001,0x0003)
    ctrl_tx(0x0000,0x00fc)
    ctrl_tx(0x0000,0x00ec)
    
    ctrl_tx(0x0003,0x0083)
    
    ctrl_tx(0x0001,0x0002)
    ctrl_tx(0x0000,0x00fc)
    ctrl_tx(0x0001,0x0003)
    #-------------------------------
    
    
    ctrl_tx(0x0504,0x0012)
    ctrl_tx(0x0500,0x008b)
    ctrl_tx(0x0504,0x0012)
    ctrl_tx(0x0502,0x0008)
    ctrl_tx(0x0503,0x0088)
    ctrl_tx(0x0500,0x008c)
    ctrl_tx(0x0504,0x000e)
    ctrl_tx(0x0500,0x008b)
    ctrl_tx(0x0504,0x000e)
    ctrl_tx(0x0502,0x0008)
    ctrl_tx(0x0503,0x0080)
    ctrl_tx(0x0500,0x008c)
    ctrl_tx(0x0504,0x0016)
    ctrl_tx(0x0500,0x008b)
    ctrl_tx(0x0504,0x0016)
    ctrl_tx(0x0502,0x0008)
    ctrl_tx(0x0503,0x0008)
    ctrl_tx(0x0500,0x008c)
    ctrl_tx(0x0504,0x0010)
    ctrl_tx(0x0500,0x008b)
    ctrl_tx(0x0504,0x0010)
    ctrl_tx(0x0502,0x0008)
    ctrl_tx(0x0503,0x0088)
    ctrl_tx(0x0500,0x008c)
    ctrl_tx(0x0504,0x001a)
    ctrl_tx(0x0502,0x0003)
    ctrl_tx(0x0503,0x0003)
    ctrl_tx(0x0500,0x008c)
    ctrl_tx(0x0504,0x001c)

#===============================================================================


init_driver()
init_radio()
init_freq()


frequencyMHz=0.0
frequencyMHz=stations[station].frequency_MHz

vol=7
set_volume(vol)

radio_play_station(station)
#radio_set_frequency(stations[station].frequency_MHz)
#print(  "\n%0.2f" % stations[station].frequency_MHz,"MHz (%s) is playing" % stations[station].name)

bandLimits=0
radio_frequency_limits(bandLimits)



#===============================================================================
# GUI
#===============================================================================

def RBGAImage(img):
    return Image.open(img).convert("RGBA")

def set_opacity(mask,size,rgb,opacity_step):
    img = RBGAImage(mask)
    img = img.resize(size, Image.Resampling.LANCZOS)
    data = img.getdata()
    newData = []
    for item in data:
        opacity_scale=0.2+0.8*opacity_step/15
        newData.append((*rgb,int(opacity_scale*item[3]/1)) )
    img.putdata(newData)
    return img

# - GUI callbacks ----------------------   

def volume_up(event):
    global vol
    vol+=1
    if vol > 15 :
        vol=15
    set_volume(vol)
    img=set_opacity("icons/audio-volume.png",(16,16),(255,255,255),vol)
    radioM036.imgAudioVolume=ImageTk.PhotoImage(img)    
    radioM036.displayFrame.audioVolumeLabel['image']=radioM036.imgAudioVolume    


def volume_down(event):  
    global vol
    vol-=1
    if vol < 0 :
        vol=0
    set_volume(vol)
    img=set_opacity("icons/audio-volume.png",(16,16),(255,255,255),vol)
    radioM036.imgAudioVolume=ImageTk.PhotoImage(img)    
    radioM036.displayFrame.audioVolumeLabel['image']=radioM036.imgAudioVolume  
    
    
def station_next(event):
    global station
    station+=1
    if station > nStation-1 :
        station=0
    radioM036.displayFrame.MHzLabel['text']="%0.2f MHz" % stations[station].frequency_MHz
    radioM036.displayFrame.stationLabel['text']=stations[station].name
    radioM036.controlFrame.stationCombobox.set(valuesPlaylist[station])
    radio_play_station(station)
    img=set_opacity("icons/antenna.png",(16,16),(255,255,255),radio_signal_level())
    radioM036.imgStationSignal=ImageTk.PhotoImage(img)    
    radioM036.displayFrame.stationSignalLabel['image']=radioM036.imgStationSignal


def station_prev(event):  
    global station
    station-=1
    if station < 0 :
        station=nStation-1
    radioM036.displayFrame.MHzLabel['text']="%0.2f MHz" % stations[station].frequency_MHz
    radioM036.displayFrame.stationLabel['text']=stations[station].name   
    radioM036.controlFrame.stationCombobox.set(valuesPlaylist[station])
    radio_play_station(station)  
    img=set_opacity("icons/antenna.png",(16,16),(255,255,255),radio_signal_level())
    radioM036.imgStationSignal=ImageTk.PhotoImage(img)    
    radioM036.displayFrame.stationSignalLabel['image']=radioM036.imgStationSignal


    
def controlPanel_upDown(event):
    global ctrlPanelUp
    if ctrlPanelUp:
        radioM036.master.geometry(geometry1)
        radioM036.buttonFrame.controlUpDownLabel['image']=radioM036.imgGoTop
        ctrlPanelUp=False
    else:
        radioM036.master.geometry(geometry0)
        radioM036.buttonFrame.controlUpDownLabel['image']=radioM036.imgGoBottom
        ctrlPanelUp=True

def muteUnmute(event):
    global paused
    if paused:
        unmute()
        radioM036.buttonFrame.muteUnmuteLabel['image']=radioM036.imgPause
        img=set_opacity("icons/audio-volume.png",(16,16),(255,255,255),vol)
        radioM036.imgAudioVolume=ImageTk.PhotoImage(img)    
        radioM036.displayFrame.audioVolumeLabel['image']=radioM036.imgAudioVolume 
        paused=False
    else:
        mute()
        radioM036.buttonFrame.muteUnmuteLabel['image']=radioM036.imgStart
        img=set_opacity("icons/audio-volume.png",(16,16),(255,255,255),0)
        radioM036.imgAudioVolume=ImageTk.PhotoImage(img)    
        radioM036.displayFrame.audioVolumeLabel['image']=radioM036.imgAudioVolume 
        paused=True
   
def freq_down (event):
    global frequencyMHz
    print ("Frequency down")
    frequencyMHz -= deltaFrequencyMHz
    if frequencyMHz < frequencyMHz_min:
        frequencyMHz=frequencyMHz_min
    radioM036.displayFrame.MHzLabel['text']="%0.2f MHz" %frequencyMHz
    radioM036.displayFrame.stationLabel['text']="..."      
    radio_set_frequency(frequencyMHz)
    img=set_opacity("icons/antenna.png",(16,16),(255,255,255),radio_signal_level())
    radioM036.imgStationSignal=ImageTk.PhotoImage(img)    
    radioM036.displayFrame.stationSignalLabel['image']=radioM036.imgStationSignal

def freq_up(event):
    global frequencyMHz
    print ("Frequency up") 
    frequencyMHz += deltaFrequencyMHz
    if frequencyMHz > frequencyMHz_max:
        frequencyMHz=frequencyMHz_max
    radioM036.displayFrame.MHzLabel['text']="%0.2f MHz" %frequencyMHz
    radioM036.displayFrame.stationLabel['text']="..."        
    radio_set_frequency(frequencyMHz)
    img=set_opacity("icons/antenna.png",(16,16),(255,255,255),radio_signal_level())
    radioM036.imgStationSignal=ImageTk.PhotoImage(img)    
    radioM036.displayFrame.stationSignalLabel['image']=radioM036.imgStationSignal

def quit(event):
    mute()
    write_playlist(filePlaylist)
    radioM036.quit()
    if platform.system() == 'Windows':
        os.system('taskkill /IM vlc.exe > nul 2> nul')

def app_delete():
    mute()
    write_playlist(filePlaylist)
    radioM036.quit()
    if platform.system() == 'Windows':
        os.system('taskkill /IM vlc.exe > nul 2> nul')


def station_changed(event):
    global station
    print("Station changed!")
    print("Current station {}".format(radioM036.controlFrame.stationCombobox.get()))
    station=radioM036.controlFrame.stationCombobox.current()
    radioM036.displayFrame.MHzLabel['text']="%0.2f MHz" % stations[station].frequency_MHz
    radioM036.displayFrame.stationLabel['text']=stations[station].name  
    radio_play_station(station)
    img=set_opacity("icons/antenna.png",(16,16),(255,255,255),radio_signal_level())
    radioM036.imgStationSignal=ImageTk.PhotoImage(img)    
    radioM036.displayFrame.stationSignalLabel['image']=radioM036.imgStationSignal

# - Other GUI functions ----------------   

def stations2valuesPlaylist():
    global valuesPlaylist
    valuesPlaylist = ["" for i in range(len(stations))] 
    for i in range(len(stations)):
        item="{freq:6.2f} MHz : {name}"
        valuesPlaylist[i]=item.format(freq=stations[i].frequency_MHz,name=stations[i].name)

# - GUI settings -----------------------

geometry0='340x176'
geometry1='340x348'
geometryDialogSR='312x127'
geometryDialogSA='206x127'

if platform.system() == 'Windows':
    geometry0='340x158'
    geometry1='340x324'
    geometryDialogSR='312x119'
    geometryDialogSA='204x122'

ctrlPanelUp=True
paused=False

displayColor = '#1e94f1'

valuesPlaylist = []  
stations2valuesPlaylist() 

deltaFrequencyMHz=0.05



#---------------------------------------  
    
from tkinter import *
try:
    import Tkinter as tk
    import ttk
    import tkFont
except ImportError:
    import tkinter as tk
    from tkinter import ttk
    from tkinter import font as tkFont
    
from PIL import Image, ImageTk

def RBGAImage(path):
    return Image.open(path).convert("RGBA")

#==================================================================================
class dialogStationAdd(object):
    def __init__(self, parent):
        # The "return value" of the dialog,
        self.stationName = None

        self.root=Toplevel(parent)
          
        self.root.bind('<Key>', self.handle_key)  
          
        p1 = PhotoImage(file='icons/radioM036.png')
        self.root.iconphoto(False,p1)  
        self.root.title("Add station")  
        self.root.geometry(geometryDialogSA) 
            
        self.style = ttk.Style(self.root)
        self.style.theme_use('clam')
        helv10 = tkFont.Font(family='Helvetica',size=10, weight='normal')
        helv10b = tkFont.Font(family='Helvetica',size=10, weight='bold')
        
        self.frameStationAdd=ttk.Frame(self.root)
        self.frameStationAdd['relief'] = 'sunken'
        self.frameStationAdd['padding']=(10,5,10,10) # ipad
        self.frameStationAdd.grid(padx=5,pady=5)
        #print(self.frameStationAdd.config()) 
        
        self.labelStatitonName = ttk.Label(self.frameStationAdd, text="Station name:")
        self.labelStatitonName['font']=helv10
        self.labelStatitonName.grid(row=0,pady=0,sticky=tk.W)
        
        self.entryStationName = ttk.Entry(self.frameStationAdd)
        self.entryStationName['width']=24
        self.entryStationName['font']=helv10
        self.entryStationName['justify']='center'

        self.entryStationName.grid(row=1,pady=12,sticky=tk.EW)
        print(self.entryStationName.config())
        
        self.addButton = ttk.Button(self.frameStationAdd, text="Add", command=self.Add)
        self.addButton.grid(row=2,pady=0, sticky=tk.E+tk.S)

        # Modal window.
        # Wait for visibility or grab_set doesn't seem to work.
        self.root.wait_visibility()   # <<< NOTE
        self.root.grab_set()          # <<< NOTE
        self.root.transient(parent)   # <<< NOTE
        
        self.parent = parent

    def handle_key(self, event):
        k = event.keysym
        print("got k: {k}".format(k))
        if k == 'Escape':
            self.root.grab_release()
            self.root.destroy()  

    def Add(self):
        self.stationName = self.entryStationName.get()
        self.root.grab_release()
        self.root.destroy()

#==================================================================================

class dialogStationRemove(object):
    def __init__(self, parent):
        # The "return value" of the dialog,
        self.yesNo = 'No'

        self.root=Toplevel(parent)
          
        self.root.bind('<Key>', self.handle_key)  
          
        p1 = PhotoImage(file='icons/radioM036.png')
        self.root.iconphoto(False,p1)  
        self.root.title("Remove station")    
        self.root.geometry(geometryDialogSR)   
        #print(self.root.config())
            
        self.style = ttk.Style(self.root)
        self.style.theme_use('clam')
        
        helv10b = tkFont.Font(family='Helvetica',size=10, weight='bold')
        
        self.frameStationRemove=ttk.Frame(self.root)
        self.frameStationRemove['padding']=(10,10,10,8)
        self.frameStationRemove['relief'] = 'sunken'
        self.frameStationRemove['width'] = 302
        self.frameStationRemove.grid(padx=5, pady=5,sticky=tk.W+tk.N+tk.E+tk.S)
        
        self.frameStationRemove.columnconfigure(0, minsize=141)
        self.frameStationRemove.columnconfigure(1, minsize=141)
        
        msg="Are you sure to remove the station"
        self.labelQuestion1 = ttk.Label(self.frameStationRemove, text=msg)
        self.labelQuestion1['padding']=(5,5,5,0)
        self.labelQuestion1['font']=helv10b
        self.labelQuestion1.grid(row=0,columnspan=2)

        msg="{freq:6.2f} MHz : {name} ?".format(freq=stations[station].frequency_MHz,name=stations[station].name)
        self.labelQuestion2 = ttk.Label(self.frameStationRemove, text=msg)
        self.labelQuestion2['padding']=(5,0,5,5)
        self.labelQuestion2['font']=helv10b
        self.labelQuestion2.grid(row=1,columnspan=2)                
               
        self.buttonYes = ttk.Button(self.frameStationRemove, text="Yes", command=self.Yes)
        self.buttonYes.focus()
        self.buttonYes.grid(row=2,column=0, padx=10, pady=5, sticky=tk.E)
        
        self.buttonNo = ttk.Button(self.frameStationRemove, text="No", command=self.No)
        self.buttonNo.grid(row=2,column=1, padx=10, sticky=tk.W)

        # Modal window.
        # Wait for visibility or grab_set doesn't seem to work.
        self.root.wait_visibility()   # <<< NOTE
        self.root.grab_set()          # <<< NOTE
        self.root.transient(parent)   # <<< NOTE
        
        self.parent = parent

    def handle_key(self, event):
        k = event.keysym
        print("got k: {k}".format(k))
        if k == 'Escape':
            self.root.grab_release()
            self.root.destroy()  

    def Yes(self):
        self.yesNo = "Yes"
        self.root.grab_release()
        self.root.destroy()

    def No(self):
        self.yesNo = "No"
        self.root.grab_release()
        self.root.destroy()


#==================================================================================
class Application(ttk.Frame):

    def __init__(self, master=None):
        ttk.Frame.__init__(self, master)
        
        self.style = ttk.Style(self)
        self.style.theme_use('clam')
        self['padding']=(5,5,5,5)
        self.grid(sticky=tk.EW)
        self.master.bind('<Key>', self.handle_key)
        
        self.CreateWidgets()
        
        
    def handle_key(self, event):
        k = event.keysym
        print("got k: {k}".format(k))
        if k == 'q' or k == 'Q':
            mute()
            write_playlist(filePlaylist)
            radioM036.quit()
    

    def CreateWidgets(self):
        
        s = ttk.Style()
        s.configure('My.TFrame', background=displayColor)
        s.configure('My.TLabel', background=displayColor,foreground="white")

        self.displayFrame = ttk.Frame(self)
        self.displayFrame['padding']=(10,15,10,5)
        self.displayFrame['relief'] = 'sunken'
        self.displayFrame['style'] = 'My.TFrame'
        self.displayFrame.grid(row=0,sticky=tk.EW)
        self.displayFrame.columnconfigure(0, minsize=290)
        #print(self.displayFrame.config())

        helv36b = tkFont.Font(family='Helvetica',size=36, weight='bold')
        helv12b = tkFont.Font(family='Helvetica',size=12, weight='bold')
        self.displayFrame.MHzLabel=ttk.Label(self.displayFrame,text="%0.2f MHz" % stations[station].frequency_MHz)
        self.displayFrame.MHzLabel['font']=helv36b
        self.displayFrame.MHzLabel['style'] = 'My.TLabel'
        self.displayFrame.MHzLabel.grid(row=0, column=0, columnspan=3)
        
        
        self.displayFrame.stationLabel=ttk.Label(self.displayFrame,text=stations[station].name)
        self.displayFrame.stationLabel['font']=helv12b
        self.displayFrame.stationLabel['style']='My.TLabel'
        self.displayFrame.stationLabel.grid(row=1, column=0, columnspan=3)
        
        img=set_opacity("icons/antenna.png",(16,16),(255,255,255),radio_signal_level())
        self.imgStationSignal=ImageTk.PhotoImage(img)    
        self.displayFrame.stationSignalLabel=ttk.Label(self.displayFrame,background=displayColor)
        self.displayFrame.stationSignalLabel['image']=self.imgStationSignal
        self.displayFrame.stationSignalLabel.grid(row=2,column=0,sticky=tk.W)
            
        
        img=set_opacity("icons/audio-volume.png",(16,16),(255,255,255),vol)
        self.imgAudioVolume=ImageTk.PhotoImage(img) 
        self.displayFrame.audioVolumeLabel=ttk.Label(self.displayFrame,background=displayColor)
        self.displayFrame.audioVolumeLabel['image']=self.imgAudioVolume
        self.displayFrame.audioVolumeLabel.grid(row=2,column=2,sticky=tk.E)    
            

            
        #-----------------------------------------------------------------------------------------        
        self.buttonFrame = ttk.Frame(self)
        #self.buttonFrame['relief'] = 'sunken'
        self.buttonFrame['padding']=(0,5,0,5)
        self.buttonFrame.grid(row=1,column=0,sticky=tk.EW)

        self.imgStationPrev=PhotoImage(file='icons/media-skip-backward-symbolic.symbolic.png')
        self.buttonFrame.stationPrevLabel=ttk.Label(self.buttonFrame)
        self.buttonFrame.stationPrevLabel['image']=self.imgStationPrev
        self.buttonFrame.stationPrevLabel['padding']=(40,0,5,0)
        self.buttonFrame.stationPrevLabel.bind("<Button-1>", station_prev)        
        self.buttonFrame.stationPrevLabel.grid(row=0,column=0)

        self.imgStationNext=PhotoImage(file='icons/media-skip-forward-symbolic.symbolic.png')
        self.buttonFrame.stationNextLabel=ttk.Label(self.buttonFrame)
        self.buttonFrame.stationNextLabel['image']=self.imgStationNext
        self.buttonFrame.stationNextLabel['padding']=(5,0,20,0)
        self.buttonFrame.stationNextLabel.bind("<Button-1>", station_next)
        self.buttonFrame.stationNextLabel.grid(row=0,column=1,sticky=tk.E)

        self.imgVolDown=PhotoImage(file='icons/go-down-symbolic.symbolic.png')
        self.buttonFrame.volDownLabel=ttk.Label(self.buttonFrame)
        self.buttonFrame.volDownLabel['image']=self.imgVolDown
        self.buttonFrame.volDownLabel['padding']=(20,0,5,0)
        self.buttonFrame.volDownLabel.bind("<Button-1>", volume_down)
        self.buttonFrame.volDownLabel.grid(row=0,column=2)

        self.imgVolUp=PhotoImage(file='icons/go-up-symbolic.symbolic.png')
        self.buttonFrame.volUpLabel=ttk.Label(self.buttonFrame)
        self.buttonFrame.volUpLabel['image']=self.imgVolUp
        self.buttonFrame.volUpLabel['padding']=(5,0,5,0)
        self.buttonFrame.volUpLabel.bind("<Button-1>", volume_up)
        self.buttonFrame.volUpLabel.grid(row=0,column=3)

        self.imgPause=PhotoImage(file='icons/media-playback-pause-symbolic.symbolic.png')
        self.imgStart=PhotoImage(file='icons/media-playback-start-symbolic.symbolic.png')
        
        self.buttonFrame.muteUnmuteLabel=ttk.Label(self.buttonFrame)
        self.buttonFrame.muteUnmuteLabel['image']=self.imgPause
        self.buttonFrame.muteUnmuteLabel['padding']=(5,0,20,0)
        self.buttonFrame.muteUnmuteLabel.bind("<Button-1>", muteUnmute)
        self.buttonFrame.muteUnmuteLabel.grid(row=0,column=4)

        self.imgGoBottom=PhotoImage(file='icons/go-bottom-symbolic.symbolic.png')
        self.imgGoTop=PhotoImage(file='icons/go-top-symbolic.symbolic.png')
        
        self.buttonFrame.controlUpDownLabel=ttk.Label(self.buttonFrame)
        self.buttonFrame.controlUpDownLabel['image']=self.imgGoBottom
        self.buttonFrame.controlUpDownLabel['padding']=(20,0,5,0)
        self.buttonFrame.controlUpDownLabel.bind("<Button-1>", controlPanel_upDown)
        self.buttonFrame.controlUpDownLabel.grid(row=0,column=5)
        
        self.imgShutdown=PhotoImage(file='icons/system-shutdown-symbolic.symbolic.png')
        self.buttonFrame.quitLabel=ttk.Label(self.buttonFrame)
        self.buttonFrame.quitLabel['image']=self.imgShutdown
        self.buttonFrame.quitLabel['padding']=(5,0,15,0)
        self.buttonFrame.quitLabel.bind("<Button-1>", quit)
        self.buttonFrame.quitLabel.grid(row=0,column=6)
        
        #-----------------------------------------------------------------------------------------   
        self.controlFrame = ttk.Frame(self)
        self.controlFrame['relief'] = 'sunken'
        self.controlFrame['padding']=(20,5,20,5)
        self.controlFrame.grid(row=2,sticky=tk.N+tk.E+tk.S+tk.W)
        
        self.controlFrame.columnconfigure(0, weight=1)
        self.controlFrame.columnconfigure(1, weight=1)
        self.controlFrame.columnconfigure(2, weight=1)
                
        helv10 = tkFont.Font(family='Helvetica',size=10, weight='normal')
        
        self.controlFrame.stationCombobox = ttk.Combobox(self.controlFrame)
        self.controlFrame.stationCombobox.grid(column=0, row=0, columnspan=3, padx=2, pady=4, ipady=6, sticky=tk.NS+tk.EW)
        self.controlFrame.stationCombobox['values']=valuesPlaylist
        self.controlFrame.stationCombobox['state'] = 'readonly'
        self.controlFrame.stationCombobox['justify'] = 'center'
        self.controlFrame.stationCombobox['font'] = helv10
        
        self.controlFrame.stationCombobox.set(valuesPlaylist[station])
        self.controlFrame.stationCombobox.bind('<<ComboboxSelected>>', station_changed)
        #print(self.controlFrame.stationCombobox.config())
        
        self.controlFrame.stationPrevLabel=ttk.Label(self.controlFrame)
        self.controlFrame.stationPrevLabel['image']=self.imgStationPrev
        self.controlFrame.stationPrevLabel.bind("<Button-1>", station_prev)
        self.controlFrame.stationPrevLabel.grid(row=1,column=0,sticky=tk.E)
        self.controlFrame.stationPrevLabel['padding']=(0,10,0,0)

        self.controlFrame.stationNextLabel=ttk.Label(self.controlFrame)
        self.controlFrame.stationNextLabel['image']=self.imgStationNext
        self.controlFrame.stationNextLabel.bind("<Button-1>", station_next)
        self.controlFrame.stationNextLabel.grid(row=1,column=2,sticky=tk.W)
        self.controlFrame.stationNextLabel['padding']=(0,10,0,0)
                
        self.imgStationRemove=PhotoImage(file='icons/list-remove-symbolic.symbolic.png')
        self.controlFrame.stationRemoveLabel=ttk.Label(self.controlFrame)
        self.controlFrame.stationRemoveLabel['image']=self.imgStationRemove
        self.controlFrame.stationRemoveLabel.bind("<Button-1>", self.popupDialogStationRemove)
        self.controlFrame.stationRemoveLabel.grid(row=2,column=1)
        self.controlFrame.stationRemoveLabel['padding']=(0,0,0,10)
                
        self.imgFreqDown=PhotoImage(file='icons/media-seek-backward-symbolic.symbolic.png')
        self.controlFrame.freqDownLabel=ttk.Label(self.controlFrame)
        self.controlFrame.freqDownLabel['image']=self.imgFreqDown
        self.controlFrame.freqDownLabel.bind("<Button-1>", freq_down)
        self.controlFrame.freqDownLabel.grid(row=3,column=0,sticky=tk.E)

        self.imgFreqUp=PhotoImage(file='icons/media-seek-forward-symbolic.symbolic.png')
        self.controlFrame.freqUpLabel=ttk.Label(self.controlFrame)
        self.controlFrame.freqUpLabel['image']=self.imgFreqUp
        self.controlFrame.freqUpLabel.bind("<Button-1>", freq_up)
        self.controlFrame.freqUpLabel.grid(row=3,column=2,sticky=tk.W)

        self.imgStationAdd=PhotoImage(file='icons/list-add-symbolic.symbolic.png')
        self.controlFrame.stationAddLabel=ttk.Label(self.controlFrame)
        self.controlFrame.stationAddLabel['image']=self.imgStationAdd
        self.controlFrame.stationAddLabel['padding']=(0,0,0,10)
        self.controlFrame.stationAddLabel.bind("<Button-1>", self.popupDialogStationAdd)
        self.controlFrame.stationAddLabel.grid(row=4,column=1)      
        #print(self.controlFrame.stationAddLabel.config())
                
    def popupDialogStationAdd(self,event):
        d = dialogStationAdd(self.master)
        print("'Station Add' dialog is opened, waiting to respond")
        self.master.wait_window(d.root)
        print('End of wait_window, back in MainWindow code')
        print('got data: {sn}'.format(sn=d.stationName)) 
        global station
        global nStation
        if d.stationName :
            print("Station will be added")
            newStation=radioStation()
            newStation.frequency_MHz=frequencyMHz
            newStation.name=d.stationName
            stations.insert(station+1,newStation)
            nStation+=1
            station+=1
            stations2valuesPlaylist()
            self.controlFrame.stationCombobox['values']=valuesPlaylist
            self.controlFrame.stationCombobox.set(valuesPlaylist[station]) 
            self.displayFrame.MHzLabel['text']="%0.2f MHz" % stations[station].frequency_MHz
            self.displayFrame.stationLabel['text']=stations[station].name 
            radio_play_station(station)
            write_playlist(filePlaylist)

    def popupDialogStationRemove(self,event):
        d = dialogStationRemove(self.master)
        print("'Station Remove' dialog is opened, waiting to respond")
        self.master.wait_window(d.root)
        print('End of wait_window, back in MainWindow code')
        print('got data: {yn}'.format(yn=d.yesNo)) 
        global station
        global nStation
        if d.yesNo == 'Yes' :
            if nStation > 1 :
                del stations[station]
                if station == nStation-1:
                    station = station -1
                nStation=len(stations)
            stations2valuesPlaylist()
            self.controlFrame.stationCombobox['values']=valuesPlaylist
            self.controlFrame.stationCombobox.set(valuesPlaylist[station]) 
            self.displayFrame.MHzLabel['text']="%0.2f MHz" % stations[station].frequency_MHz
            self.displayFrame.stationLabel['text']=stations[station].name 
            radio_play_station(station)
            write_playlist(filePlaylist)



radioM036 = Application()
radioM036.master.title('AVerMedia Radio M036')
radioM036.master.geometry(geometry0)
radioM036.master.wait_visibility()
radioM036.master.resizable(False,False)
radioM036.master.attributes('-topmost',1)
#radioM036.master.protocol("WM_DELETE_WINDOW", app_delete)
#radioM036.master.attributes('-alpha',0.0)
#radioM036.master.attributes("-fullscreen", 1)
p1 = PhotoImage(file='icons/radioM036.png')
radioM036.master.iconphoto(False,p1)
radioM036.mainloop()
