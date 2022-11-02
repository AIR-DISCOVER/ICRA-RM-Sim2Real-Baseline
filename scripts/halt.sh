#!/bin/bash
docker stop client -t 0
docker stop sim-server -t 0
docker stop ros-master -t 0
docker network rm net-sim
