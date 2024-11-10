sudo apt update
sudo apt full-upgrade -y
sudo apt install libhdf5-dev libcap-dev libcamera-dev python3-pip python3-libcamera python3-picamera2 -y
python3 -m venv /home/pi/env
/home/pi/env/bin/python3 -m pip install argparse numpy opencv-python onnxruntime
sudo mv -n /home/pi/env/lib/python3.11/site-packages/* /usr/lib/python3/dist-packages/
sudo rm -r /home/pi/env
python3 -c 'import picamera2'
libcamera-hello
