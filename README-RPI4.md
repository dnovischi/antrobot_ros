# TODO: 

RPI4 Antrobot Configuration

The following outline the details of setting up rpi4 for Antrobot.

1. Flash ubuntu server distribution using rassberry Imager where:
    - the hostname should be defined as rpi-[name on buttom robot sticker], e.g. rpi-antrobot1
    - ssh should be enabled
    - default user is ubuntu with passwd: antrobot
    - SSID should be antrobot with passwd: antrobot
2. Bootup and make sure you can connect via ssh
3. Install i2c-tools libi2c-dev python3-smbus
4. Install docker engine following the [official docker documentation](https://docs.docker.com/engine/install/ubuntu/#install-using-the-repository)
