# GPS-IMU Docker

## About


<details open="open">
<summary>Table of Contents</summary>

- [About](#about)
- [Requirements](#requirements)
- [Installation](#installation)
- [Configure the files](#configure-the-files)
- [Running the workspace](#running-the-workspace)
</details>

-------------------------------------------------------------------------------------
## Requirements
1. arm64 (Raspery Pi 4b)
2. GPS (Ublox-GPS ZED-F9P-02B) with base-rover-Config  
3. Dual-Anntene (ANN-MB-00)
4. IMU (xsens MTi 30)

-------------------------------------------------------------------------------------

## INSTALLATION

### installation in a new catkin workspace 

installation using with docker build - this will take a while

```sh
docker build  --network=host . -t nav-system:1.0
```


run the docker 
```sh
docker run -it nav-system
```

switch into that workspace
```sh
cd /opt/ros/overlay_ws/
```

source the workspace (install space should have been created during build)
```sh
source install/setup.bash 
```

## Configure the files

1. change the configuration from IMU
    ```sh
    nano /opt/ros/overlay_ws/src/atb_ublox_gps/config/xsens_mti_node.yaml
    ```
    ```yaml
    /**:
    ros__parameters:
    
        scan_for_devices: false
        port: "/dev/ttyUSB0" # port name, e.g. '/dev/ttyUSB0'
        baudrate: 115200 
        device_id: ""
    
        enable_logging: false
        log_file: log.mtb
    
        publisher_queue_size: 100
    
        frame_id: "imu_link" #douple check the frame_id. it should be "imu_link" 
    
        # Message publishers
        pub_imu: true
        pub_quaternion: true
        pub_mag: true
        pub_angular_velocity: true
        pub_acceleration: true
        pub_free_acceleration: true
        pub_dq: true
        pub_dv: true
        pub_sampletime: true
        pub_temperature: true
        pub_pressure: true
        pub_gnss: true
        pub_twist: true
        pub_transform: false #this value should "false"
        pub_positionLLA: true
        pub_velocity: true
    
        linear_acceleration_stddev: [0.0, 0.0, 0.0] # [m/s^2]
        angular_velocity_stddev: [0.0, 0.0, 0.0] # [rad/s]
        orientation_stddev: [0.0, 0.0, 0.0] # [rad]
        magnetic_field_stddev: [0.0, 0.0, 0.0] # [Tesla]
    ```

2.  Build xspublic from your ament workspace and Build Xsens MTi driver package::

    ```sh
    pushd src/bluespace_ai_xsens_ros_mti_driver/lib/xspublic && make && popd
    colcon build --symlink-install --packages-skip-build-finished 
    ```
	```sh
	source install/setup.bash
    ```

3. change the configuration from the gps-base and rover

    ```sh
    nano /opt/ros/overlay_ws/src/atb_ublox_gps/config/zed_f9p_base.yaml
    ```
    ```yaml
    base/gps_node:
    ros__parameters:
        device: /dev/ttyACM0 # port name, e.g. '/dev/ttyUSB0'
        frame_id: base
        uart1:
            baudrate: 460800
            in: 0
            out: 32
        usb:
            in: 32
            out: 2
        config_on_startup: False
        nav:
            rate: 8
    
        tmode3: 0 
    
        publish:
        all: False 
    ```
    ```sh
    nano /opt/ros/overlay_ws/src/atb_ublox_gps/config/zed_f9p_rover.yaml
    ```
    ```yaml
    rover/gps_node:
        ros__parameters:
        device: /dev/ttyACM1 # port name, e.g. '/dev/ttyUSB0'
        frame_id: rover
        config_on_startup: False
        # TMODE3 Config
        # tmode3: 0                   # Survey-In Mode
        uart1:
            baudrate: 460800
            in: 32
            out: 0
        usb:
            in: 32
            out: 32
             config_on_startup: False
            nav:
            rate: 8
    
            publish:
            all: True
            nmea: True
    ```

4. change the configuration from the ntrip-client

  ```sh
  nano /opt/ros/overlay_ws/src/atb_ublox_gps/config/params.yaml
  ```

   ```yaml
   ntrip_client:
     ntrip_client_node:
       ros__parameters:
         host: sapos-bb-ntrip.de #change the host 
         port: 2101
         mountpoint: VRS_3_4G_BB #change the mountpoint
  
         ntrip_version: None
  
         authenticate: True
  
         username: user #change the username
         password: pass #change the password
  
         ssl: False
  
         cert: None
         key: None
  
         ca_cert: None
         rtcm_frame_id: base
   
         nmea_max_length: 82
         nmea_min_length: 3
   
         rtcm_message_package: rtcm_msgs
   
         reconnect_attempt_max: 10
         reconnect_attempt_wait_seconds: 5
   
         rtcm_timeout_seconds: 4
  
   ```


## RUNNING THE workspace

### Running
inside the docker run
```sh
ros2 launch atb_ublox_gps docker_final_launch.py
```
