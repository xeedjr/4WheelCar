# 4WheelCar
The Firmware of ros car


version: '3'
services:
  carmen:
    image: andriyp/carmen-dev-web:latest
    container_name: carmen
    restart: always
    user: root
    ports:
      - "8080:8080"
      - "8181:8181"
      - "8282:8282"
      - "8090:8090"
      - "9090:9090"
	  - "11411:11411"
    privileged: true
    devices:
        - "/dev/ttyUSB0:/dev/ttyUSB0"