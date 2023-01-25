// const http = require('http')
// const fs = require('fs')
// const port = 3000

// const server = http.createServer(function(req,res) {
//     res.writeHead(200, {'Content-type': 'text/html'})
//     fs.readFile('index.html', function(error, data){
//         if (error) {
//             res.writeHead(404)
//             res.write('Error: File Not Found')
//         }
//         else {
//             res.write(data)
//         }
//     })
// })

// server.listen(port, function(error) {
//     if (error) {
//         console.log('Something went wrong', error)
//     }
//     else {
//         console.log('Server is listening on port ' + port)
//     }
// })

const express = require('express');
const path = require('path');

const app = express();
const port = 3000;

// sendFile will go here
app.get('/', function(req, res) {
  res.sendFile(path.join(__dirname, '/index.html'));
});

app.use(express.static(path.join(__dirname, 'public')));
app.listen(port);
console.log('Server is listening on port ' + port);