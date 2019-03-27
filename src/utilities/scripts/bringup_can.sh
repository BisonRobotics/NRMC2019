#!/bin/bash

sudo ip link set vesc_can type can bitrate 500000
sudo ip link set vesc_can up

sudo ip link set tracker_can type can bitrate 500000
sudo ip link set tracker_can up

