# Camera tracking

## Server setup (Ubuntu)

1. Go to the server folder

    ```bash
    sudo apt update
    sudo apt upgrade -y
    chmod +x install_server.sh
    ./install_server.sh
    ```

2. You should have 2 cameras streams.

    ```bash
    rtsp://192.168.2.150:8554/cam1

    rtsp://192.168.2.150:8554/cam2
    ```

3. Test the streaming using the python code client.py (if you haven't done the calibration step below the code issues a warning)

If things do not work as expected, modify the mediamtx.yml config file (scroll to the bottom)

```bash
nano mediamtx.yml
sudo cp mediamtx.yml /usr/local/etc/mediamtx.yml
sudo systemctl daemon-reload
sudo systemctl restart mediamtx
```

## Server setup (macOS)

1. Go to the server folder

    ```bash
    chmod +x install_server_mac.sh
    ./install_server_mac.sh
    ```

## Camera calibration

1. On your local computer, run the calibrate.py python script (run with the command "$ python3 calibrate.py 192.168.2.150")

2. Capture 20 frames with changing orientations of the aruco board (press 'c' to capture)

3. test the aruco localisation using the python code client.py (there should be no warning any more and an arudo marker should show data overlay)

## Tracking test

1. Put an aruco marker on the robot

2. update rtsp_url, ID (marker ID) and MY_IP variables in the tracking code

3. run the code. The robot should circle around the camera origin on a 20cm radius.



