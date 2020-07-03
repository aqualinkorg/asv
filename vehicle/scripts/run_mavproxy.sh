#!/bin/bash
set -e

mavproxy.py \
    --master=/dev/ttyACM0,115200 \
    --load-module='GPSInput' \
    --source-system=200 \
    --cmd="set heartbeat 0" \
    --out udpin:localhost:9000 \
    --out udpout:localhost:9002 \
    --out udpin:0.0.0.0:14660 \
    --out udpbcast:10.10.10.255:14550 \
    --mav20 \
    --aircraft telemetry \
    --streamrate 10 \
    --daemon