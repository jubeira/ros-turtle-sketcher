var canvas = document.getElementById("canvas");
var ctx = canvas.getContext("2d");

function drawStar(cx, cy, spikes, outerRadius, innerRadius) {
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    var rot = Math.PI / 2 * 3;
    var x = cx;
    var y = cy;
    var step = Math.PI / spikes;

    ctx.strokeSyle = "#000";
    ctx.beginPath();
    ctx.moveTo(cx, cy - outerRadius)
    console.log("starting star")
    for (i = 0; i < spikes; i++) {
        x = cx + Math.cos(rot) * outerRadius;
        y = cy + Math.sin(rot) * outerRadius;
        console.log((x /100.0) + " " + ( (1100 -y) / 100.0) + "\n")
        ctx.lineTo(x, y)
        rot += step

        x = cx + Math.cos(rot) * innerRadius;
        y = cy + Math.sin(rot) * innerRadius;
        ctx.lineTo(x, y)
        console.log((x /100.0) + " " + ( (1100 - y) / 100.0) + "\n")
                rot += step
    }
    ctx.lineTo(cx, cy - outerRadius)
    ctx.closePath();
    ctx.lineWidth=5;
    ctx.strokeStyle='blue';
    ctx.stroke();
    ctx.fillStyle='skyblue';
    ctx.fill();

}

//drawStar(75, 100, 5, 30, 15);
//drawStar(175, 100, 12, 30, 10);
//drawStar(75, 200, 6, 30, 15);
//drawStar(175, 200, 20, 30, 25);

//drawStar(550, 550, 5, 500, 250)
//drawStar(550, 550, 12, 500, 250)
//drawStar(550, 550, 6, 500, 287.5)
drawStar(550, 550, 4, 500, 175)
