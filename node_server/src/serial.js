const { SerialPort, ReadlineParser } = require("serialport");

class Serial {
  constructor() {
    this.port = null;
    this.parser = null;
    this.onDataCallback = null;
  }

  async findAndConnectCh340() {
    const ports = await SerialPort.list();
    // console.log(ports);
    const port = ports.find((port) => port.friendlyName.includes("CH340"));
    if (port) {
      this.port = new SerialPort({
        path: port.path,
        baudRate: 9600,
      });

      this.parser = this.port.pipe(new ReadlineParser({ delimiter: "\n" }));
      this.parser.on("data", (data) => {
        // console.log("got word from arduino:", data);
        if (this.onDataCallback) {
          this.onDataCallback(data);
        }
      });

      return this.port;
    } else {
      throw new Error("Cannot find CH340 device");
    }
  }

  async getOrConnect() {
    if (this.port) {
      return this.port;
    } else {
      return this.findAndConnectCh340();
    }
  }

  onData(callback) {
    this.getOrConnect();
    this.onDataCallback = callback;
  }

  async write(data) {
    await this.getOrConnect();
    this.port.write(data);
  }
}

module.exports = { Serial: new Serial() };
