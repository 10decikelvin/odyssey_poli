import { createReadStream } from "fs";
import {createServer} from "http";

createServer((req, res) => {
    let str = createReadStream("." + req.url)
    str.on("error", () => {})
    str.pipe(res);
}).listen(8000)