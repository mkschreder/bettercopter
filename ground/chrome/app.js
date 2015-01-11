
var _acc_chart = null, _gyr_chart = null; 
var _imu_data_acc_x = [], _imu_data_acc_y = [], _imu_data_acc_z = []; 
var _imu_data_acc_x_val = 0; 
var _imu_data_gyr_x = [], _imu_data_gyr_y = [], _imu_data_gyr_z = []; 
var _imu_data_gyr_x_val = 0; 
var _imu_data_pres = []; 
var _connection_id = 0; 

function ab2str(buf) {
	return String.fromCharCode.apply(null, new Uint8Array(buf));
}

function str2ab(str) {
  var buf = new ArrayBuffer(str.length); // 2 bytes for each char
  var bufView = new Uint8Array(buf);
  for (var i=0, strLen=str.length; i<strLen; i++) {
    bufView[i] = str.charCodeAt(i);
  }
  return buf;
}

function processPacket(packet){
	if("frame_log" in packet){
		var log = packet["frame_log"]; 
		if("raw_acc_x" in log){
			if(_imu_data_acc_x.length > 100)
				_imu_data_acc_x.shift();
			if(_imu_data_acc_y.length > 100)
				_imu_data_acc_y.shift();
			if(_imu_data_acc_z.length > 100)
				_imu_data_acc_z.shift();
			_imu_data_acc_x.push({x: _imu_data_acc_x_val, y: log["raw_acc_x"]}); 
			_imu_data_acc_y.push({x: _imu_data_acc_x_val, y: log["raw_acc_y"]}); 
			_imu_data_acc_z.push({x: _imu_data_acc_x_val, y: log["raw_acc_z"]}); 
			_imu_data_acc_x_val++; 
			_acc_chart.render(); 
		}
		if("raw_gyr_x" in log){
			if(_imu_data_gyr_x.length > 100)
				_imu_data_gyr_x.shift();
			if(_imu_data_gyr_y.length > 100)
				_imu_data_gyr_y.shift();
			if(_imu_data_gyr_z.length > 100)
				_imu_data_gyr_z.shift();
			_imu_data_gyr_x.push({x: _imu_data_gyr_x_val, y: log["raw_gyr_x"]}); 
			_imu_data_gyr_y.push({x: _imu_data_gyr_x_val, y: log["raw_gyr_y"]}); 
			_imu_data_gyr_z.push({x: _imu_data_gyr_x_val, y: log["raw_gyr_z"]}); 
			_imu_data_gyr_x_val++; 
			_gyr_chart.render(); 
		}
	}
}

function writeToDrone(obj){
	chrome.serial.write(_connection_id, str2ab(JSON.stringify(obj))); 
}



$(document).ready(function(){
	$("#demo").sidr(); 
	$("#sidr a").click(function(){
		$("#sidr").sidr("close"); 
		window.location.href = $(this).attr('href');
	}); 
	var embeds = document.querySelectorAll("embed");
	for(i = 0; e = embeds[i]; i++){
		e.addEventListener("load",function(){
			var svgRoot = this.getSVGDocument().documentElement; //get the inner DOM of alpha.svg
			var needle = $("#min", svgRoot); 
			var angle = 0; 
			setInterval(function(){
				needle.attr("transform", "rotate("+angle+", 50, 50)");
				angle++; 
			}, 50); 
			var arrow = $("#arrow", svgRoot); 
			var arrowangle = 0; 
			setInterval(function(){
				arrow.attr("transform", "rotate("+arrowangle+", 50, 50)");
				arrowangle++; 
			}, 50); 
		},false);
  }
	
	_acc_chart = new CanvasJS.Chart("acc_chart",{
		title :{
			text: "Accelerometer"
		},
		axisX: {	
			title: "Time"
		},
		axisY: {	
			title: "Value"
		},
		data: [{
			type: "line",
			showInLegend: true, 
			legendText: "Acc X",
			name: "acc_x", 
			dataPoints : _imu_data_acc_x
		}, {
			type: "line",
			showInLegend: true, 
			legendText: "Acc Y",
			name: "acc_y", 
			dataPoints : _imu_data_acc_y
		}, {
			type: "line",
			showInLegend: true, 
			legendText: "Acc Z",
			dataPoints : _imu_data_acc_z
		}]
	});
	_gyr_chart = new CanvasJS.Chart("gyr_chart",{
		title :{
			text: "Gyroscope"
		},
		axisX: {	
			title: "Time"
		},
		axisY: {	
			title: "Value"
		},
		data: [{
			type: "line",
			showInLegend: true, 
			legendText: "Gyr X",
			dataPoints : _imu_data_gyr_x
		}, {
			type: "line",
			showInLegend: true, 
			legendText: "Gyr Y",
			dataPoints : _imu_data_gyr_y
		}, {
			type: "line",
			showInLegend: true, 
			legendText: "Gyr Z",
			dataPoints : _imu_data_gyr_z
		}]
	});
	for(c = 0; c < 100; c++) {
		_imu_data_acc_x.push({x: _imu_data_acc_x_val, y: 0}); 
		_imu_data_acc_y.push({x: _imu_data_acc_x_val, y: 0});
		_imu_data_acc_z.push({x: _imu_data_acc_x_val, y: 0});  
		_imu_data_gyr_x.push({x: _imu_data_gyr_x_val, y: 0});  
		_imu_data_gyr_y.push({x: _imu_data_gyr_x_val, y: 0});  
		_imu_data_gyr_z.push({x: _imu_data_gyr_x_val, y: 0});  
		_imu_data_acc_x_val++;
		_imu_data_gyr_x_val++; 
	}
	_acc_chart.render();
	_gyr_chart.render();

	//$("h1").first().html("Ready!");
}); 

var Application = angular.module('Application', []);

Application.controller('IndexController', function ($scope) {
	$scope.devices = []; 
	function updateDevices(){
		chrome.serial.getDevices(function(ports){
			$scope.devices = []; 
			$.each(ports, function(id, port){
				$scope.devices.push({
					value: port.path,
					text: port.displayName + " ("+ port.path +")"
				});
			});
			
			$scope.$apply(); 
		}); 
	}
	$scope.status_success = function(str){
		$scope.status = str; 
		$scope.status_style={color: "green"}; 
		$scope.$apply(); 
	}
	$scope.status_error = function(str){
		$scope.status = str.slice(0, 32); 
		$scope.status_style={color: "red"}; 
		$scope.$apply(); 
	}
	$scope.updateDevices = function(){updateDevices();}
	$scope.log = ""; 
	$scope.status=""; 
	$scope.status_style={}; 
	$scope.status_success("Ready"); 
	// set defaults
	$scope.config = {
		pid_stab_yaw: {p: 5, i: 0, d: 0},
		pid_stab_pitch: {p: 5, i: 0, d: 0}, 
		pid_stab_roll: {p: 5, i: 0, d: 0}, 
		pid_rate_yaw: {p: 5, i: 0, d: 0}, 
		pid_rate_pitch: {p: 5, i: 0, d: 0}, 
		pid_rate_roll: {p: 5, i: 0, d: 0}
	}; 
	
	$scope.doConnect = function(){
		var port = $scope.currentDevice; 
		if(port == undefined || port == "") return; 
		
		$scope.log += "Connecting to "+port+"... "
		
		chrome.serial.connect(port, {
			bitrate: 38400, 
		}, function(info){
			if(!info) {
				$scope.log += "failed!\n"; 
				$scope.$apply(); 
				$scope.status_error("Error connecting!"); 
				return; 
			} 
			
			$scope.log += "success!\n"; 
			$scope.$apply(); 
			
			$scope.status_success("Connected!"); 
			
			_connection_id = info.connectionId; 
			
			var str_buf = ""; 
			chrome.serial.onReceive.addListener(function(obj){
				if(obj.connectionId != info.connectionId) return; 
				
				var buf = ab2str(obj.data); 
				var idx = buf.indexOf("\n"); 

				$.each(buf.split("\n"), function(k, v){
					if(k == 0){ // first value 
						str_buf += v; 
					}
					
					if(idx != -1){
						// try convert to object
						var packet = null; 
						try {
							packet = JSON.parse(str_buf); 
							str_buf = ""; 
						} catch(e) {
							str_log=""; 
						}
						if(packet) 
							processPacket(packet); 
					}
					
					// if not first token then place it in str buf and exit
					if(k != 0)
						str_buf = v; 
				});
			}); 
		}); 
	}
	updateDevices(); 
});
