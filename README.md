# RPLIDAR A3

Code supposed to be used for the France robotics cup 2019:

Code works with https://github.com/gobgob/chariot-elevateur.

It works on Raspberry PI's GPIO.

### Sources:

- [the official RPLidar's SDK](https://github.com/Slamtec/rplidar_sdk)
- [a project made by INTech club members](https://github.com/Club-INTech/rplidar_a2)

### How to use this?
```bash
sudo apt-get install libnet1-dev
git clone git@github.com:gobgob/rplidar_a3.git
cd rplidar_a3/src
make
./output/Linux/Release/cdr2019 &
./../test_client.py
```
