// When we receive a message on /my_topic, add its data as a list item to the "messages" ul
function publish() {
    // Create a listener for /my_topic
    const my_topic_listener = new ROSLIB.Topic({
        ros,
        name: "/my_topic",
        messageType: "std_msgs/String",
    });

    var str = new ROSLIB.Message({
        data: 'hello'
    });

    my_topic_listener.publish(str);

    my_topic_listener.subscribe((message) => {
        const ul = document.getElementById("messages");
        const newMessage = document.createElement("p");
        newMessage.appendChild(document.createTextNode(str.data));
        ul.appendChild(newMessage);
    });

}


