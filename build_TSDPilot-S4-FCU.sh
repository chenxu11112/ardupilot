#!/bin/bash

./waf configure --board=TSDPilot-S4-FCU

./waf copter 

./waf --target bin/arducopter --upload