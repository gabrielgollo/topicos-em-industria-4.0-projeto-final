const express = require("express");
const { Serial } = require("./src/serial");
const app = express();
const port = 5000;

// serve public files
app.use(express.static("public"));

// parse json
app.use(express.json());

// variable to store last data
let lastData = null;
function onData(data) {
  timeStamp = new Date().getTime();
  lastData = `${timeStamp} - ${data}`;
}

Serial.onData(onData);

app.get("/api/read", async (req, res) => {
  try {
    const data = lastData;
    return res.status(200).send({ status: "ok", data });
  } catch (error) {
    return res.status(500).send({ status: "error", error });
  }
});

app.post("/api/write", (req, res) => {
  try {
    const { data = "rpm" } = req.body;
    console.log("got data:", data);

    const dataString = `${data}\n`;
    Serial.write(dataString);

    return res.status(200).send({ status: "ok" });
  } catch (error) {
    return res.status(500).send({ status: "error", error });
  }
});

app.listen(port, () => {
  console.log(`Example app listening on http://localhost:${port}`);
});
