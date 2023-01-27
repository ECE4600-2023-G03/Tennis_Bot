const express = require('express');
const path = require('path');

const app = express();
const port = 3000;

// index
app.get('/', function(req, res) {
  res.sendFile(path.join(__dirname, '/index.html'));
});

// devpage
app.get('/devpage', function(req, res) {
  res.sendFile(path.join(__dirname, '/devpage.html'));
});

app.use(express.static(path.join(__dirname, 'public')));
app.listen(port);
console.log('Server is listening on port ' + port);