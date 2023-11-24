#https://gist.github.com/liviaerxin/bb34725037fd04afa76ef9252c2ee875#send-a-webcam-with-h264-rtp-stream-while-record-jetson-nano

rm playlist.m3u8
rm segment*.ts

# This one plays to a HLS stream
gst-launch-1.0 v4l2src device=/dev/video1 ! image/jpeg,format=MJPG,width=1920,height=1080,framerate=30/1 ! nvv4l2decoder mjpeg=1 ! nvvidconv ! nvv4l2h264enc ! h264parse ! hlssink3 target-duration=1 max-files=10


# Test HLS Sink
#gst-launch-1.0 videotestsrc is-live=true ! x264enc ! h264parse ! hlssink2 max-files=5