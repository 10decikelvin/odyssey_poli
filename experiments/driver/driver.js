import { SerialPort }from 'serialport';
import { DelimiterParser }from '@serialport/parser-delimiter';
// Create a port
let port = new SerialPort({
  path: '/dev/ttyUSB0',
  baudRate: 115200
});

const parser = port.pipe(new DelimiterParser({ delimiter: '\n' }));
let initialised = false;
parser.on('data', d => {
  //dont use tostring as i have no clue what type it is
  if((d + "").includes("Command format: ") && !initialised){
    initialised = true;
    console.log("Intialised.")
  }
  //console.log(`Serial > ${d}`)
});

export function move(left, right){
  port.write(`<move,${left},${right}>`);
}
