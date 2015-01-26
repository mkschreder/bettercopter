var Application = angular.module('Application', []);

Application.controller('IndexController', function ($scope) {
	$scope.calculate = function(){
		//this.max_stab_i_rate = this.max_stab_i_resp / this.desired_stab_integral_time; 
		this.total_response = parseInt(this.desired_rate_response) + 
			parseInt(this.desired_rate_acceleration_response) + parseInt(this.desired_stab_response); 
		
		this.max_stab_i_resp = this.desired_stab_response * 4; 
		this.max_stab_i_rate = $scope.stab.pitch.p * $scope.max_angle * $scope.rate.pitch.p; 
	
		this.rate.pitch.p = this.desired_rate_response / this.max_rate; 
		this.rate.pitch.i = 0; 
		this.rate.pitch.max_i = 0; 
		this.rate.pitch.d = this.desired_rate_acceleration_response / this.max_rate_acc; 
		this.stab.pitch.p = this.desired_stab_response / this.rate.pitch.p / this.max_angle; 
		this.stab.pitch.max_i = this.desired_stab_integral_response / this.rate.pitch.p; //this.max_stab_i_resp / this.rate.pitch.p; 
		this.stab.pitch.i = this.stab.pitch.max_i / this.desired_stab_integral_time / this.max_angle;
		this.stab.pitch.d = 0; 
		
		this.total_response += this.stab.pitch.max_i * this.rate.pitch.p; 
	}
	$scope.changed_top_rate = function(){
		this.calculate(); 
	}
	$scope.total_response = 0; 
	$scope.desired_rate_response = 25; //12; 
	$scope.desired_rate_acceleration_response = 10; //72; 
	$scope.desired_stab_response = 20; //30; 
	$scope.desired_stab_integral_response = 70; 
	$scope.desired_stab_integral_time = 11; 
	$scope.max_angle = 45; 
	$scope.max_rate_acc = 360; 
	$scope.max_rate = 180; 
	$scope.rate = {
		yaw: 		{p: 0, i: 0, max_i: 0, d: 0},
		pitch: 	{p: 0, i: 0, max_i: 0, d: 0},
		roll: 	{p: 0, i: 0, max_i: 0, d: 0}
	}; 
	$scope.stab = {
		yaw: 		{p: 0, i: 0, max_i: 0, d: 0},
		pitch: 	{p: 0, i: 0, max_i: 0, d: 0},
		roll: 	{p: 0, i: 0, max_i: 0, d: 0}
	}; 
});
