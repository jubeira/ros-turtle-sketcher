// Connecting to ROS
  // -----------------
var ros = new ROSLIB.Ros({
	url : 'ws://localhost:9090'
});

ros.on('connection', function() {
	console.log('Connected to websocket server.');
});

ros.on('error', function(error) {
	console.log('Error connecting to websocket server: ', error);
});

ros.on('close', function() {
    console.log('Connection to websocket server closed.');
});

// Subscribing to a Topic
// ----------------------

var listener = new ROSLIB.Topic({
    ros : ros,
    name : '/turtle1/pose',
    messageType : 'turtlesim/Pose',
	throttle_rate : 200
});

var x1 = null;
var y1 = null;
var x2 = null;
var y2 = null;



listener.subscribe(reportAndDraw);

function correctYCoord(y) {
    return 11 - y;
}

function reportAndDraw(message) {
console.log('Received message on ' + listener.name + ': ' + message.x + ", " + message.y);

	x2 = x1;
	y2 = y1;
	x1 = message.x;	
	y1 = message.y;
	sketch(x1, correctYCoord(y1), x2, correctYCoord(y2), 60);
}


function updateListener(reportAndDraw) {
    var turtleName = document.getElementById("turtleName").value;
    console.log("Old listener name: " + listener.name + "\n")

    listener.unsubscribe();
    listener.name = '/' + turtleName + '/pose';
    listener.subscribe(reportAndDraw);

    console.log("New listener name: " + listener.name + "\n")
}
