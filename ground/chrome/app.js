$(document).ready(function(){
	var c=document.getElementById("main_canvas");
	var ctx=c.getContext("2d");
	ctx.beginPath();
	ctx.moveTo(0,0);
	ctx.lineTo(300,150);
	ctx.stroke();
	
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
			$("h1").first().html("Ready!");
		},false);
    }
	$("button").click(function(){
		chrome.serial.getDevices(function(ports){
			$("#altimeter").svg(function(svg){
				var g = svg.group("needle"); 
				$(g).attr('transform', 'rotate(60)');
				svg.line(g, 0, 0, 20, 0, {stroke: 'red', strokeWidth: 3});
				svg.polygon(g, [[25, 0],[20, 3],[20, -3]], {fill: 'red', stroke: 'red', strokeWidth: 1});
				svg.circle(g, 0, 0, 3, {fill: 'white', stroke: 'red', strokeWidth: 2});
				$(g).animate({svgTransform: 'translate(150 150) rotate(45)'}, 5000).
				animate({svgTransform: 'translate(150 150) rotate(90)'}, 5000);
			}); 
			$("h1").first().html(JSON.stringify(ports)); 
		}); 
	}); 
}); 
