const express = require("express");
const cors = require('cors');
const path = require('path');
const PORT = process.env.PORT || 3001; 
const app = express();
const WebSocket = require('ws');
const wss = new WebSocket.Server({ port: 3003 });
var mysql = require('mysql');
var con = mysql.createConnection( {
    host: "3.8.159.120",
    user: "test1", 
    password: "123456", 
    database: "Testdb"
});
con.connect(function(err) { if (err) throw err;
    console.log("Successfully connected to the database...\n"); });
 

app.use(cors({ origin: '*'
}));

app.use(express.json());


app.use(express.static(path.resolve(__dirname, './client/build')));


app.use(cors({
    methods: ['GET','POST','DELETE','UPDATE','PUT','PATCH']
    }));

app.get("/pollServer", (req, res) => { var d = new Date();
    const json_res = {
        "time" : d.toTimeString() };
    res.send(json_res); });


app.get("/pointQuery", (req,res)=>{
    con.query("SELECT * FROM Points", function (err,result,fields){
        if(err) throw err;
        res.json(result)
    });
});

app.post('/api/data', (req,res) => {
    // req.body contains the incoming JSON data
    console.log(req.body);

    let data = req.body; // assuming this is an array of records

    let sql = `INSERT INTO Points (timestamp, BeaconType, xcordinate, ycordinate) VALUES ?`;

    let values = data.map(d => {
        let date = new Date(d.timestamp);
        let formattedTimestamp = date.toISOString().slice(0, 19).replace('T', ' ');
        return [formattedTimestamp, d.BeaconType, d.xcordinate, d.ycordinate];
    });

    con.query(sql, [values], (err, result) => {
        if (err) {
            console.error(err);
            res.status(500).json({ error: 'An error occurred while inserting data.' });
            return;
        }

        // If no error, send a success message
        res.json({ message: 'Data inserted successfully' });
    });
});

wss.on('connection', (ws) => {
    console.log('Client connected');
  
    ws.on('message', (message) => {
      console.log('Received: %s', message);
  
      // Assuming the message is a JSON string
      let data = JSON.parse(message); 
  
      let sql = `INSERT INTO Points (timestamp, BeaconType, xcordinate, ycordinate) VALUES ?`;
  
      let values = data.map(d => {
        let date = new Date(d.timestamp);
        let formattedTimestamp = date.toISOString().slice(0, 19).replace('T', ' ');
        return [formattedTimestamp, d.BeaconType, d.xcordinate, d.ycordinate];
      });
  
      con.query(sql, [values], (err, result) => {
        if (err) {
          console.error(err);
          ws.send(JSON.stringify({ error: 'An error occurred while inserting data.' }));
          return;
        }
  
        // If no error, send a success message
        ws.send(JSON.stringify({ message: 'Data inserted successfully' }));
      });
  
    });
  
    ws.on('close', () => {
      console.log('Client disconnected');
    });
  });

app.delete('/api/data', (req, res) => {
    let sql = `TRUNCATE TABLE Points`;

    con.query(sql, (err, result) => {
        if (err) {
            console.error(err);
            res.status(500).json({ error: 'An error occurred while deleting data.' });
            return;
        }

        // If no error, send a success message
        res.json({ message: 'Data deleted successfully' });
    });
});




app.get('*', (req, res) => {
    res.sendFile(path.resolve(__dirname, './client/build', 'index.html'));
});

app.listen(PORT, () => {
    console.log(`Server listening on ${PORT}`);
});