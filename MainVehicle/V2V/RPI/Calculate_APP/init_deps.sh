sudo apt install pip
sudo apt install git 
sudo apt install cmake
python3 -m venv Control_PY
source Control_PY/bin/activate
pip install sysv_ipc
pip install RPi.GPIO
sudo apt-get update
sudo apt-get install mosquitto mosquitto-clients
sudo systemctl start mosquitto
sudo systemctl enable mosquitto
sudo systemctl status mosquitto
sudo apt-get install libc-ares-dev libssl-dev
cd ~
git clone https://github.com/eclipse/paho.mqtt.c.git
cd paho.mqtt.c
cmake CMakeLists.txt
make
sudo make install


