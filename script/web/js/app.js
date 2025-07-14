//webkitURL is deprecated but nevertheless
URL = window.URL || window.webkitURL;

var gumStream; 						//stream from getUserMedia()
var rec; 							//Recorder.js object
var input; 							//MediaStreamAudioSourceNode we'll be recording

// shim for AudioContext when it's not avb. 
var AudioContext = window.AudioContext || window.webkitAudioContext;
var audioContext //audio context to help us record

var recordButton = document.getElementById("recordButton");
var pauseButton = document.getElementById("pauseButton");
var stopButton = document.getElementById("stopButton");

// remembr remote
var rmbstopButton = document.getElementById("rmbstopButton");

//add events to buttons
recordButton.addEventListener("click", startRecording);
pauseButton.addEventListener("click", pauseRecording);
stopButton.addEventListener("click", stopRecording);
rmbstopButton.addEventListener("click", publishRmbStop);

//ROS init
const ros = new ROSLIB.Ros({
	url: "wss://172.17.4.107:9090"
});

ros.on("connection", () => console.log("Connected to ROS 2!"));
ros.on("error", (error) => console.error("Connection error:", error));
ros.on("close", () => console.log("Disconnected from ROS 2"));

// publish to asr to query
const audio_topic = new ROSLIB.Topic({
	ros: ros,
	name: '/audio_path',
	messageType: 'std_msgs/ByteMultiArray'
});
// publish to agent to stop current query
const stop_topic = new ROSLIB.Topic({
	ros: ros,
	name: '/terminate_remembr',
	messageType: 'std_msgs/Empty'
});
// subscribe from asr to get current query
const speech_topic = new ROSLIB.Topic({
	ros: ros,
	name: "/speech",
	messageType: "std_msgs/String"
});
// subscribe from agent to get current remembr status
const status_topic = new ROSLIB.Topic({
	ros: ros,
	name: "/remembr_status",
	messageType: "std_msgs/String"
});
// subscribe from agent to get remembr response
const response_topic = new ROSLIB.Topic({
	ros: ros,
	name: "/goal_response",
	messageType: "std_msgs/String"
});

// Publishers
function publishRmbQuery() {
	const message = new ROSLIB.Message({
		data: "/remembr/simple-recorderjs-demo/query.wav"
	});
	audio_topic.publish(message);
}
function publishRmbStop() {
	const message = new ROSLIB.Message({});
	stop_topic.publish(message);
}
// Subscribers
status_topic.subscribe((message) => {
  	document.getElementById("status_output").innerText = `ReMembR status: ${message.data}`;
	  if (message.data === "idle") {
		rmbstopButton.disabled = true;
	} else {
		rmbstopButton.disabled = false;
	}
});
speech_topic.subscribe((message) => {
	document.getElementById("query_input").innerText = `Query: ${message.data}`;
});
response_topic.subscribe((message) => {
	document.getElementById("response_output").innerText = `Response: ${message.data}`;
});

function startRecording() {
	// console.log("recordButton clicked");

	/*
		Simple constraints object, for more advanced audio features see
		https://addpipe.com/blog/audio-constraints-getusermedia/
	*/
    
    var constraints = { audio: true, video:false }

 	/*
    	Disable the record button until we get a success or fail from getUserMedia() 
	*/

	recordButton.disabled = true;
	stopButton.disabled = false;
	pauseButton.disabled = false

	/*
    	We're using the standard promise based getUserMedia() 
    	https://developer.mozilla.org/en-US/docs/Web/API/MediaDevices/getUserMedia
	*/

	navigator.mediaDevices.getUserMedia(constraints).then(function(stream) {
		// console.log("getUserMedia() success, stream created, initializing Recorder.js ...");

		/*
			create an audio context after getUserMedia is called
			sampleRate might change after getUserMedia is called, like it does on macOS when recording through AirPods
			the sampleRate defaults to the one set in your OS for your playback device

		*/
		audioContext = new AudioContext();

		//update the format 
		document.getElementById("formats").innerHTML="Format: 1 channel pcm @ "+audioContext.sampleRate/1000+"kHz"

		/*  assign to gumStream for later use  */
		gumStream = stream;
		
		/* use the stream */
		input = audioContext.createMediaStreamSource(stream);

		/* 
			Create the Recorder object and configure to record mono sound (1 channel)
			Recording 2 channels  will double the file size
		*/
		rec = new Recorder(input,{numChannels:1})

		//start the recording process
		rec.record()

		// console.log("Recording started");

	}).catch(function(err) {
	  	//enable the record button if getUserMedia() fails
    	recordButton.disabled = false;
    	stopButton.disabled = true;
    	pauseButton.disabled = true
	});
}

function pauseRecording(){
	// console.log("pauseButton clicked rec.recording=",rec.recording );
	if (rec.recording){
		//pause
		rec.stop();
		pauseButton.innerHTML="Resume";
	}else{
		//resume
		rec.record()
		pauseButton.innerHTML="Pause";

	}
}

function stopRecording() {
	// console.log("stopButton clicked");

	//disable the stop button, enable the record too allow for new recordings
	stopButton.disabled = true;
	recordButton.disabled = false;
	pauseButton.disabled = true;

	//reset button just in case the recording is stopped while paused
	pauseButton.innerHTML="Pause";
	
	//tell the recorder to stop the recording
	rec.stop();

	//stop microphone access
	gumStream.getAudioTracks()[0].stop();

	//create the wav blob and pass it on to createDownloadLink
	rec.exportWAV(createDownloadLink);
}

function blobToArrayBuffer(blob) {
	const reader = new FileReader();
	reader.readAsArrayBuffer(blob);

	return new Promise((resolve) => {
		reader.onloadend = () => resolve(reader.result);
	});
}

function createDownloadLink(blob) {
	
	var url = URL.createObjectURL(blob);
	var au = document.createElement('audio');
	var li = document.createElement('li');
	var link = document.createElement('a');

	//name of .wav file to use during upload and download (without extendion)
	var filename = new Date().toISOString();

	//add controls to the <audio> element
	au.controls = true;
	au.src = url;

	//save to disk link
	link.href = url;
	link.download = filename+".wav"; //download forces the browser to donwload the file using the  filename
	link.innerHTML = "Save to disk";

	//add the new audio element to li
	li.appendChild(au);
	
	//add the filename to the li
	li.appendChild(document.createTextNode(filename+".wav "))

	//add the save to disk link to li
	li.appendChild(link);
	
	//upload link
	var upload = document.createElement('a');
	upload.href="#";
	upload.innerHTML = "Query";
	upload.addEventListener("click", function(event){
		  async function blobToByteArray(blob) {
			const arrayBuffer = await blob.arrayBuffer();  // Convert Blob to ArrayBuffer
			return new Uint8Array(arrayBuffer);            // Convert ArrayBuffer to Uint8Array
		  }
		  blobToByteArray(blob).then(byteArray => {
			const message = new ROSLIB.Message({
				data: Array.from(byteArray) 
			  });
			  audio_topic.publish(message);
		  });
		  
	})
	li.appendChild(document.createTextNode (" "))//add a space in between
	li.appendChild(upload)//add the upload link to li

	//add the li element to the ol
	recordingsList.appendChild(li);
	if (recordingsList.children.length > 3) {
		recordingsList.removeChild(recordingsList.children[0]);
	}
}
