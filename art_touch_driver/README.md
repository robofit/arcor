Pycopia core and aid modules needed (https://github.com/kdart/pycopia.git)

./setup.py install core
./setup.py install aid

add udev rule (i.e. /etc/udev/rules.d/99-input.rules): SUBSYSTEM=="input", MODE="660", GROUP="touch_foil"
add user to touch_foil group

How to disable touch screen as XOrg input device:
```
$ xinput 
⎡ Virtual core pointer                    	id=2	[master pointer  (3)]
⎜   ↳ Virtual core XTEST pointer              	id=4	[slave  pointer  (2)]
⎜   ↳ Genius                                  	id=8	[slave  pointer  (2)]
⎜   ↳ Genius USB Optical Mouse                	id=11	[slave  pointer  (2)]
⎜   ↳ USBest Technology SiS HID Touch Controller	id=10	[slave  pointer  (2)]
⎣ Virtual core keyboard                   	id=3	[master keyboard (2)]
    ↳ Virtual core XTEST keyboard             	id=5	[slave  keyboard (3)]
    ↳ Power Button                            	id=6	[slave  keyboard (3)]
    ↳ Power Button                            	id=7	[slave  keyboard (3)]
    ↳ CHICONY USB Keyboard                    	id=9	[slave  keyboard (3)]
$ xinput --disable 10
```
