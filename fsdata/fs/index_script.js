var ws;

window.onload = function() {
	wsOpen();
}

function wsOpen() {
	if (typeof ws === "undefined" || ws.readyState != WebSocket.CONNECTING) {
		if(location.host.length < 8) {
			return;
		}
		
		ws = new WebSocket("ws://" + location.host);
		
		ws.onopen = function(evt) {
			requestConfigValues();
		};
		
		ws.onerror = function(evt) {
			console.error("WebSocket error:", evt);
			ws.close();
		};
		
		ws.onclose = function(evt) {
			document.getElementsByTagName("body")[0].style.display = "none";
			
			alert("Conexão perdida. Recarregue a página para tentar conectar novamente.")
		};
		
		ws.onmessage = function(evt) {
			var receivedData = evt.data.split("\t");
			
			if(receivedData[0] === "RTC" && typeof receivedData[1] === "string") {
				updateRTCText(receivedData[1]);
			} else if(receivedData[0] === "CFGV" && typeof receivedData[1] === "string" && typeof receivedData[2] === "string") {
				updateConfigurationValue(receivedData[1], receivedData[2]);
			}
		};
	}
}

function updateRTCText(value) {
	var localDate = new Date();
	var receivedTime = new Date(parseInt(value, 10) * 1000);
	var timeElement = document.getElementById('rtc-time-text');
	
	if(receivedTime.getTime === 0) {
		timeElement.innerText = "Data/Hora inválida!";
		timeElement.style.color = "darkred";
	} else {
		let hoursText = receivedTime.getHours().toString().padStart(2, "0");
		let minutesText = receivedTime.getMinutes().toString().padStart(2, "0");
		let secondsText = receivedTime.getSeconds().toString().padStart(2, "0");
		let yearText = receivedTime.getFullYear();
		let monthText = (receivedTime.getMonth() + 1).toString().padStart(2, "0");
		let mdayText = receivedTime.getDate().toString().padStart(2, "0");
		
		timeElement.innerText = hoursText + ":" + minutesText + ":" + secondsText + " " + mdayText + "/" + monthText + "/" + yearText;
		
		timeElement.style.color = (Math.abs(receivedTime.getTime() - localDate.getTime()) <= 3000) ? "black" : "darkorange";
	}
}

function updateConfigurationValue(configName, configValue) {
	var inputFieldArray = document.getElementsByClassName("config-inputs");
	var disabledCount = 0;
	
	for(let input_i = 0; input_i < inputFieldArray.length; input_i++) {
		let inputField = inputFieldArray[input_i];
		
		if(inputField.dataset.config_name === configName) {
			inputField.value = configValue;
			inputField.dataset.config_value = configValue;
			inputField.disabled = false;
		}
		
		if(inputField.disabled === true)
			disabledCount++;
	}
	
	if(disabledCount === 0)
		document.getElementById("save-config-button").classList.remove("pure-button-disabled");
}

function sendRTCTime() {
	if(confirm("Você vai ajustar o relógio do medidor para o horário atual do seu dispositivo. Continuar?") == false)
		return;
	
	ws.send("RTCU\t" + Math.trunc((new Date()).getTime() / 1000));
}

function saveConfig() {
	var inputFieldArray = document.getElementsByClassName("config-inputs");
	var modifiedCount = 0;
	
	for(let input_i = 0; input_i < inputFieldArray.length; input_i++) {
		let inputField = inputFieldArray[input_i];
		
		if(inputField.value === inputField.dataset.config_value)
			continue;
		
		inputField.disabled = true;
		
		if(new Blob([inputField.value]).size > 31)
			continue;
		
		modifiedCount++;
		
		ws.send("CFGWR\t" + inputField.dataset.config_name + "\t" + inputField.value);
	}
	
	if(modifiedCount > 0)
		document.getElementById("save-config-button").classList.add("pure-button-disabled");
}

function requestConfigValues() {
	var configInputElements = document.getElementsByClassName("config-inputs");
	
	for(let input_i = 0; input_i < configInputElements.length; input_i++) {
		let configName = configInputElements[input_i].dataset.config_name;
		
		if(typeof configName !== "string")
			continue;
		
		setTimeout(function() { ws.send("CFGRD\t" + configName); }, input_i * 150);
		
	}
}

function restartDevice() {
	if(confirm("Tem certeza que deseja reiniciar o medidor?") == false)
		return;
	
	ws.send("RST\t");
}
