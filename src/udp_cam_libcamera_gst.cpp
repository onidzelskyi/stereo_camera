// udp_cam_libcamera_gst.cpp
//
// Capture from libcamera (XRGB8888) and push frames into GStreamer appsrc.
// Pipeline converts to I420, x264 encodes and sends RTP/H264 to UDP port.
//
// Build:
// g++ udp_cam_libcamera_gst.cpp -o udp_cam_libcamera_gst \
//   $(pkg-config --cflags --libs libcamera gstreamer-1.0 gstreamer-app-1.0) -pthread
//
// Run:
// ./udp_cam_libcamera_gst <destination-ip> <port>
// Example: ./udp_cam_libcamera_gst 192.168.1.50 5000
//

#include <iostream>
#include <memory>
#include <vector>
#include <cstring>
#include <atomic>
#include <csignal>
#include <thread>
#include <chrono>
#include <cstdint>

// libcamera
#include <libcamera/libcamera.h>
// #include <apps/common/image.h>
#include <libcamera/camera_manager.h>
#include <libcamera/framebuffer.h>
#include <libcamera/request.h>

#include "image.h"

// mmap & sockets (we use only mmap here)
#include <sys/mman.h>
#include <unistd.h>

// GStreamer
#include <gst/gst.h>
#include <gst/app/gstappsrc.h>

using namespace libcamera;

// Globals
static std::shared_ptr<Camera> g_camera;
static std::unique_ptr<CameraManager> g_camManager;
static GstElement *g_appsrc = nullptr;
static std::atomic<bool> g_running{true};
static GstElement *pipeline;
#define FPS 30
const uint8_t *data_ptr = nullptr;
uint8_t *frame = nullptr;
size_t bytes_used = 0;
uint32_t width = 800;
uint32_t height = 600;
std::vector<uint8_t> rgbBuffer(width * height * 3);


// ************ Encoder ************************************************
// Convert XRGB8888 → RGB (drop X channel)
void XRGB8888toRGB(const uint8_t* src, uint8_t* dst, int width, int height)
{
    int srcStride = width * 4;  // 4 bytes per pixel (XRGB)
    int dstStride = width * 3;  // 3 bytes per pixel (RGB)

    for (int y = 0; y < height; y++) {
        const uint8_t* s = src + y * srcStride;
        uint8_t* d = dst + y * dstStride;
        for (int x = 0; x < width; x++) {
            d[0] = s[1]; // R
            d[1] = s[2]; // G
            d[2] = s[3]; // B
            s += 4;
            d += 3;
        }
    }
}
// ************ Encoder ************************************************

// ************ Gstreamer ************************************************
static gboolean push_frame(gpointer data) {
    static guint64 timestamp = 0;

    GstBuffer *buffer;
    GstFlowReturn ret;
    GstMapInfo map;

    if (!frame) return TRUE;
    
    // XRGB8888toRGB(frame, rgbBuffer.data(), width, height);
    // buffer = gst_buffer_new_allocate(NULL, rgbBuffer.size(), NULL);
    // gst_buffer_map(buffer, &map, GST_MAP_WRITE);
    // memcpy(map.data, rgbBuffer.data(), rgbBuffer.size());
    
    buffer = gst_buffer_new_allocate(NULL, bytes_used, NULL);
    gst_buffer_map(buffer, &map, GST_MAP_WRITE);
    memcpy(map.data, frame, bytes_used);
    
    gst_buffer_unmap(buffer, &map);
    GST_BUFFER_PTS(buffer) = timestamp;
    GST_BUFFER_DURATION(buffer) = gst_util_uint64_scale_int(1, GST_SECOND, FPS);
    timestamp += GST_BUFFER_DURATION(buffer);

    g_signal_emit_by_name(g_appsrc, "push-buffer", buffer, &ret);
    gst_buffer_unref(buffer);

    if (ret != GST_FLOW_OK) {
        g_print("Failed to push buffer: %d\n", ret);
        return FALSE;
    }

    return TRUE;
}
// ************ Gstreamer ************************************************

// requestCompleted callback: push frame to appsrc
static void requestComplete(Request *request)
{    
    if (request->status() != Request::RequestComplete)
        return;

    for (auto [stream, buffer] : request->buffers()) {
        std::unique_ptr<Image> image = Image::fromFrameBuffer(buffer, Image::MapMode::ReadOnly);
    	for (unsigned int i = 0; i < buffer->planes().size(); ++i) {
    		const unsigned int bytesused = buffer->metadata().planes()[i].bytesused;
    
    		Span<uint8_t> data = image->data(i);
    		bytes_used = std::min<unsigned int>(bytesused, data.size());
    
    		if (bytesused > data.size())
    			std::cerr << "payload size " << bytesused
    				  << " larger than plane size " << data.size()
    				  << std::endl;
    
            if (!frame) {
                frame = (uint8_t*)malloc(bytes_used);
            }
            memcpy(frame, data.data(), bytes_used);
        }
    }
    // Reuse and requeue request for next capture
    request->reuse(Request::ReuseBuffers);
    g_camera->queueRequest(request);
}

// SIGINT handler to stop gracefully
static void sigint_handler(int)
{
    g_running = false;
    if (g_appsrc)
        gst_app_src_end_of_stream(GST_APP_SRC(g_appsrc));
}

int main(int argc, char *argv[])
{
    // ***************** Arguments ********************************************
    if (argc < 3) {
        std::cerr << "Usage: " << argv[0] << " <destination-ip> <port>\n";
        return EXIT_FAILURE;
    }
    const char *dest_ip = argv[1];
    int dest_port = atoi(argv[2]);
    // ***************** Arguments ********************************************


    // Setup SIGINT
    // std::signal(SIGINT, sigint_handler);

    // ***************** Camera ***********************************************
    g_camManager = std::make_unique<CameraManager>();
    if (g_camManager->start() != 0) {
        std::cerr << "Failed to start CameraManager\n";
        gst_object_unref(g_appsrc);
        gst_object_unref(pipeline);
        return EXIT_FAILURE;
    }

    auto cameras = g_camManager->cameras();
    if (cameras.empty()) {
        std::cerr << "No cameras available\n";
        g_camManager->stop();
        gst_object_unref(g_appsrc);
        gst_object_unref(pipeline);
        return EXIT_FAILURE;
    }

    // choose first camera (CSI on Pi is usually index 0)
    g_camera = g_camManager->get(cameras[0]->id());
    if (!g_camera) {
        std::cerr << "Failed to acquire camera\n";
        g_camManager->stop();
        gst_object_unref(g_appsrc);
        gst_object_unref(pipeline);
        return EXIT_FAILURE;
    }

    if (g_camera->acquire() != 0) {
        std::cerr << "Failed to acquire camera\n";
        g_camManager->stop();
        gst_object_unref(g_appsrc);
        gst_object_unref(pipeline);
        return EXIT_FAILURE;
    }

    // Generate configuration and select XRGB8888
    // std::unique_ptr<CameraConfiguration> config = g_camera->generateConfiguration({ StreamRole::VideoRecording, StreamRole::Viewfinder });
    // std::unique_ptr<CameraConfiguration> config = g_camera->generateConfiguration({ StreamRole::StillCapture });
    // std::unique_ptr<CameraConfiguration> config = g_camera->generateConfiguration({ StreamRole::VideoRecording });
    std::unique_ptr<CameraConfiguration> config = g_camera->generateConfiguration({ StreamRole::Viewfinder });
    if (!config) {
        std::cerr << "Failed to generate camera configuration\n";
        g_camera->release();
        g_camManager->stop();
        gst_object_unref(g_appsrc);
        gst_object_unref(pipeline);
        return EXIT_FAILURE;
    }

    StreamConfiguration &streamCfg = config->at(0);
    // Pick a sensible resolution & format
    // streamCfg.size.width = 640;
    // streamCfg.size.height = 480;
    // streamCfg.size.width = 1640;
    // streamCfg.size.height = 1232;

    // Preferred pixel format for Raspberry Pi (if available)
    // streamCfg.pixelFormat = libcamera::formats::XRGB8888;
    // streamCfg.pixelFormat = libcamera::formats::YUV420;
    // streamCfg.pixelFormat = libcamera::formats::SBGGR16;
    
    // Validate & configure
    CameraConfiguration::Status status = config->validate();
    if (status == CameraConfiguration::Status::Invalid) {
        std::cerr << "Camera configuration invalid\n";
        g_camera->release();
        g_camManager->stop();
        gst_object_unref(g_appsrc);
        gst_object_unref(pipeline);
        return EXIT_FAILURE;
    }
    g_camera->configure(config.get());

    // Save negotiated size (in case driver chose different)
    width = streamCfg.size.width;
    height = streamCfg.size.height;
    // std::cout << "Using camera size: " << width << "x" << height << " format XRGB8888\n";
    std::cout << "Default viewfinder configuration is: " << streamCfg.toString() << std::endl;

    // (Optional) Update caps on appsrc if camera changed resolution/format
    // GstCaps *caps = gst_caps_from_string(
    //     ("video/x-raw,format=XRGB,width=" + std::to_string(width) + ",height=" + std::to_string(height) + ",framerate=30/1").c_str());
    // gst_app_src_set_caps(GST_APP_SRC(g_appsrc), caps);
    // gst_caps_unref(caps);

    // Allocate buffers
    FrameBufferAllocator allocator(g_camera);
    for (auto &cfg : *config) {
        if (allocator.allocate(cfg.stream()) < 0) {
            std::cerr << "Failed to allocate buffers\n";
            g_camera->release();
            g_camManager->stop();
            gst_object_unref(g_appsrc);
            gst_object_unref(pipeline);
            return EXIT_FAILURE;
        }
    }

    // Create requests
    std::vector<std::unique_ptr<Request>> requests;
    Stream *stream = streamCfg.stream();
    const auto &buffers = allocator.buffers(stream);
    for (auto &buf : buffers) {
        std::unique_ptr<Request> req = g_camera->createRequest();
        if (!req) {
            std::cerr << "Failed to create request\n";
            continue;
        }
        if (req->addBuffer(stream, buf.get()) < 0) {
            std::cerr << "Failed to add buffer\n";
            continue;
        }
        requests.push_back(std::move(req));
    }

    // Connect callback
    g_camera->requestCompleted.connect(requestComplete);

    // Start camera & queue requests
    if (g_camera->start() != 0) {
        std::cerr << "Failed to start camera\n";
        g_camera->release();
        g_camManager->stop();
        gst_object_unref(g_appsrc);
        gst_object_unref(pipeline);
        return EXIT_FAILURE;
    }

    for (auto &r : requests)
        g_camera->queueRequest(r.get());

    std::cout << "Streaming to " << dest_ip << ":" << dest_port << " — press Ctrl+C to stop\n";

    // Main loop: keep running until SIGINT
        std::cout << "Streaming MJPEG RTP to " << dest_ip << ":" << dest_port << " ...\n";
    std::cout << "Press Ctrl-C to stop\n";
    // ***************** Camera ***********************************************

    // ***************** GStreamer ********************************************
    gst_init(&argc, &argv);

    char pipeline_desc[1024];
    std::snprintf(pipeline_desc, sizeof(pipeline_desc),
        "appsrc name=mysrc is-live=true block=true format=TIME "
        "caps=video/x-raw,format=BGRx,width=800,height=600,framerate=30/1 "
        "! videoconvert "
        "! video/x-raw,format=I420 "
        "! x264enc tune=zerolatency speed-preset=ultrafast "
        "! rtph264pay config-interval=1 pt=96 "
        "! udpsink host=%s port=%d auto-multicast=false",
        dest_ip, dest_port);
    
    std::cout << "GStreamer pipeline: " << pipeline_desc << std::endl;

    GError *err = nullptr;
    pipeline = gst_parse_launch(pipeline_desc, &err);
    if (!pipeline) {
        std::cerr << "Failed to create pipeline: " << (err ? err->message : "(unknown)") << "\n";
        if (err) g_error_free(err);
        return EXIT_FAILURE;
    }

    // Get appsrc element
    g_appsrc = gst_bin_get_by_name(GST_BIN(pipeline), "mysrc");
    if (!g_appsrc) {
        std::cerr << "Failed to get appsrc element from pipeline\n";
        gst_object_unref(pipeline);
        return EXIT_FAILURE;
    }

    // Start pipeline playing
    gst_element_set_state(pipeline, GST_STATE_PLAYING);

    // Push one frame every 1/FPS second
    g_timeout_add(1000 / FPS, push_frame, NULL);
    
    // Main loop
    GMainLoop *loop = g_main_loop_new(NULL, FALSE);
    g_main_loop_run(loop);
    // ***************** GStreamer ********************************************

    // Clean up
    std::cout << "Stopping...\n";
    g_camera->stop();
    // free allocator buffers
    for (auto &cfg : *config)
        allocator.free(cfg.stream());

    g_camera->release();
    g_camManager->stop();

    // End-of-stream for appsrc and stop pipeline
    gst_app_src_end_of_stream(GST_APP_SRC(g_appsrc));
    gst_element_set_state(pipeline, GST_STATE_NULL);
    gst_object_unref(g_appsrc);
    gst_object_unref(pipeline);

    return 0;
}
