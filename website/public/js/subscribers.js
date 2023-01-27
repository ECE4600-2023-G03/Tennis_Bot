// Create ros object to communicate over your Rosbridge connection
const ros = new ROSLIB.Ros({ url: "ws://192.168.21.158:9090" });

// When the Rosbridge server connects, fill the span with id "status" with "successful"
ros.on("connection", () => {
    document.getElementById("status").innerHTML = "successful";
});

// When the Rosbridge server experiences an error, fill the "status" span with the returned error
ros.on("error", (error) => {
    document.getElementById("status").innerHTML = `errored out (${error})`;
});

// When the Rosbridge server shuts down, fill the "status" span with "closed"
ros.on("close", () => {
    document.getElementById("status").innerHTML = "closed";
});


// Create a listener for /string_topic
const string_topic = new ROSLIB.Topic({
    ros,
    name: "/string_topic",
    messageType: "std_msgs/String"
});

// When we receive a message on /string_topic, add its data as a list item to the “string_messages" ul
string_topic.subscribe((message) => {
    const p = document.getElementById("string_messages");
    p.innerHTML = message.data;
});


// Create a listener for /twist_topic
const twist_topic = new ROSLIB.Topic({
    ros,
    name: "/turtle1/cmd_vel",
    messageType: "geometry_msgs/msg/Twist"
});

// When we receive a message on /twist_topic, add its data as a list item to the “twist_messages" ul
twist_topic.subscribe((message) => {
    document.getElementById("linear_x").innerHTML = message.linear.x;
    document.getElementById("linear_y").innerHTML = message.linear.y;
    document.getElementById("linear_z").innerHTML = message.linear.z;
    document.getElementById("angular_x").innerHTML = message.angular.x;
    document.getElementById("angular_y").innerHTML = message.angular.y;
    document.getElementById("angular_z").innerHTML = message.angular.z;
});


// Create a listener for /twist_topic
const int16_topic = new ROSLIB.Topic({
    ros,
    name: "/int16_topic",
    messageType: "std_msgs/msg/Int16"
});

// When we receive a message on /twist_topic, add its data as a list item to the “twist_messages" ul
int16_topic.subscribe((message) => {
    document.getElementById("int16_messages1").innerHTML = message.data;
    document.getElementById("int16_messages2").innerHTML = message.data;
});


// Create a listener for /twist_topic
const bool_topic = new ROSLIB.Topic({
    ros,
    name: "/bool_topic",
    messageType: "std_msgs/msg/Bool"
});

// When we receive a message on /twist_topic, add its data as a list item to the “twist_messages" ul
bool_topic.subscribe((message) => {
    document.getElementById("bool_messages").innerHTML = message.data;
});


const terminalMessageTopic = new ROSLIB.Topic({
    ros,
    name: '/rosout',
    messageType: 'rcl_interfaces/msg/Log'
});

// terminalMessageTopic.subscribe((message) => {
//     this.terminalMessages.push({ line: message.msg });
// });

terminalMessageTopic.subscribe((message) => {
    const ul = document.getElementById("terminal_log");
    const newMessage = document.createElement("li");
    newMessage.appendChild(document.createTextNode(message.msg));
    ul.appendChild(newMessage);
});

document.getElementById("terminal_log").scrollTop = document.getElementById("terminal_log").scrollHeight;

// terminalMessageTopic.subscribe((message) => {
//     this.terminalMessages.push({ line: message.msg });
// });

// var textarea = document.getElementById('terminal_log');
// textarea.scrollTop = textarea.scrollHeight;
