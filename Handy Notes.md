# Handy notes to keep around for debugging

## Starting sudo nodejs
```bash
sudo "$(which node)" ilename.js
```

## LibGOMP errors
```bash
export LD_PRELOAD=/usr/lib/aarch64-linux-gnu/libgomp.so.1 
```

## Add rust stuff to GSTREAMER LIBS
```bash
export GST_PLUGIN_PATH="/home/jetson/gst-plugins-rs/target/aarch64-unknown-linux-gnu/debug:$GST_PLUGIN_PATH"
```

## Hotswap GST ver.
```
cd ~/gstreamer
gst-env.py
```


## List device capabilities
```

v4l2-ctl --list-formats-ext -d /dev/video0
```

## Scan wifi
```bash
sudo nmcli device wifi rescan && sudo nmcli device wifi list
```