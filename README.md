# radioM036
**A radio app for the AVerMedia AVerTV USB2.0 Plus**

---

![ScreenShot](http://burabizim.org/radioM036/Screenshot-radioM036.png) 

The AVerMedia AVerTV USB2.0 Plus is a composite device with three interfaces. The device has the following functions: Analog TV, FM radio, image capture and video recorder. It also has a remote control. Interface 0 is vendor specific and a kernel driver is available only for Windows XP. Interface 1 and 2 are for audio control and audio streaming. Various OS provide an audio driver. Device hardware contains a DC1120-E controller chip. Unfortunately, no documentation is available for this controller chip. To write the radioM036 app USB traffic was sniffed and hacked by using USBPcap and Wireshark for Windows XP.

## Requirements

- Device access
    
    *Linux*: 
    
    For non-root users, a udev rule is necessary to access the device.

    `SUBSYSTEM=="usb", ATTR{idVendor}=="07ca", ATTR{idProduct}=="0036", MODE="0666"`
    
    *Windows*:
    
    To use libusb on Windows, a driver must be installed for the Interface 0.
    
    See *[https://github.com/libusb/libusb/wiki/Windows#how-to-use-libusb-on-windows](https://github.com/libusb/libusb/wiki/Windows#how-to-use-libusb-on-windows)*
    
    
    
- VLC media player
    
    It is used to capture audio steam. The capture device location is like as:
    
    *For Linux*:
    
    `pulse://alsa_input.usb-AVerMedia_AVerTV_USB_2.0_Plus-01.analog-stereo`

    *For Windows*:
    
    `dshow:// :dshow-vdev=none :dshow-adev="Digital Audio Interface (AVerTV USB 2.0 Plus)" :live-caching=300`
    
    If needed, please fix the capture device setting in `config.xml`  file.

- Python >= 3.7

    The following python modules are to be installed.

    - libusb-package
    - pyusb
    - python-vlc
    - pillow
    - pyinstaller (optional, if you want to build an executable)
