import cv from '@u4/opencv4nodejs';
let {VideoCapture} = cv;
//let cap = new VideoCapture(0);

// #1: works but slow (mpeg encode and decode = very bad, high latency)
//const cap = new VideoCapture("v4l2src device=/dev/video1 ! image/jpeg,format=MJPG,width=640,height=400,framerate=20/1 ! nvv4l2decoder mjpeg=1 ! nvvidconv ! video/x-raw,format=BGRx ! appsink")
//#2 : works rather optimally, may consdier getting rid of blur. faster but only has one camera stream

//const cap = new VideoCapture(`v4l2src device=/dev/video1 ! video/x-raw,format=YUY2,width=640,height=400! videoconvert ! gaussianblur sigma=2 ! videoconvert !appsink`)

//#3: two videos merge now
const cap = new VideoCapture(`v4l2src device=/dev/video1 ! video/x-raw,format=YUY2,width=320,height=240! videobox border-alpha=0 alpha=0.5 left=-320 ! videomixer name=mix ! videoconvert ! videoconvert ! appsink v4l2src device=/dev/video0 ! video/x-raw,format=YUY2,width=320,height=240! videobox border-alpha=0 alpha=0.5 ! mix.`)

export async function cameraDataURL(){
    let frame = await cap.readAsync();
    let buf = await cv.imencodeAsync(".jpg", frame);
    return `data:image/jpg;base64,` + buf.toString("base64");
}

let loggedResolution = false;
export async function cameraDataBuffer(){
    let frame = await cap.readAsync();
    if(!loggedResolution){
        console.log(frame.sizes)
        loggedResolution = true
    }
    return await cv.imencodeAsync(".jpg", frame);
}

let isPrepolling = false;
let cameraPrepollCache = Buffer.alloc(0);

export function startCameraPrepoll(){
    if(isPrepolling) return;
    isPrepolling = true;
    updateCameraPrepoll();
}

async function updateCameraPrepoll(){
    cameraPrepollCache = await cameraDataBuffer();
    if(isPrepolling){
        setTimeout(updateCameraPrepoll, 1000/30);
    }
}

export function stopCameraPrepoll(){
    isPrepolling = false;
}

export function getCameraPrepollBuffer(){
    return cameraPrepollCache;
}