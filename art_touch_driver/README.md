Pycopia core and aid modules needed (https://github.com/kdart/pycopia.git)

./setup.py install core
./setup.py install aid

add udev rule (i.e. /etc/udev/rules.d/99-input.rules): SUBSYSTEM=="input", MODE="660", GROUP="touch_foil"
add user to touch_foil group
