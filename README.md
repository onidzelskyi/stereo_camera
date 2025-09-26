# Stereo camera


<img src='https://www.waveshare.com/w/upload/a/a7/Ws-watermark-en.svg' style='width: 60px'/>

[IMX219-83 Stereo Camera](https://www.waveshare.com/wiki/IMX219-83_Stereo_Camera)


<img src='https://www.waveshare.com/w/upload/thumb/1/11/IMX219-83-Stereo-Camera-1.jpg/600px-IMX219-83-Stereo-Camera-1.jpg' style='width:400px'/>

# Dependencies

<hr>

[https://libcamera.org](https://libcamera.org)

# Build

<hr>

```bash
cd src
g++ image.cpp file_sink.cpp udp_cam_libcamera_gst.cpp -o udp_cam_libcamera_gst -g  $(pkg-config --cflags --libs libcamera gstreamer-1.0 gstreamer-app-1.0)  -pthread -I./
```

# Run

<hr>

```bash
./udp_cam_libcamera_gst 192.168.1.83 5000
```