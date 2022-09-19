
./waf configure --board=TSDH743-FCU

./waf --targets bin/arducopter --upload

cd ardupilot/ArduCopter

../Tools/autotest/sim_vehicle.py -f gazebo-iris --console --map