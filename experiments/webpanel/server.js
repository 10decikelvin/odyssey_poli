import { createServer } from 'http';
import { createReadStream, readFileSync } from 'fs';
import { WebSocketServer } from 'ws';
import {move} from "../driver/driver.js";
import { cameraDataBuffer, cameraDataURL, getCameraPrepollBuffer, startCameraPrepoll } from '../opencvjs/camera.js';

const directory = "experiments/webpanel";

const mimeTypes = {
  ".html": "text/html",
  ".json": "application/json",
  ".js": "text/javascript",
  ".png": "image/png"
}


startCameraPrepoll();

const server = createServer(async (req, res) => {
  if(req.url === "/"){
    res.setHeader("Content-Type", "text/html");
    createReadStream(directory + "/static/index.html").pipe(res);
  }else if(req.url.startsWith("/static")){
    let url = req.url.replace(/[^A-Za-z0-9/.-]/g, "").replace(/\.\./g, "");
    for(let ext of Object.keys(mimeTypes)){
      if(url.endsWith(ext)){
        res.setHeader("Content-Type", mimeTypes[ext])
      };
    }
    let str = createReadStream(directory + url)
    str.on("error", () => {
      console.log(404, directory + url)
      res.end("404 Not Found")
    })
    str.pipe(res);
  }else if(req.url.startsWith("/camera")){
    res.end(getCameraPrepollBuffer())
  }else{
    res.writeHead(404, "Not Found");
    res.end();
  }
})


const wss = new WebSocketServer({ server });
wss.on('connection', function connection(ws) {
  ws.on('message', function message(_data) {
    let data = JSON.parse(_data);
    console.log(data)
    move(parseFloat(data.left), parseFloat(data.right))
  });
  ws.send("open");
});

server.listen(8080);