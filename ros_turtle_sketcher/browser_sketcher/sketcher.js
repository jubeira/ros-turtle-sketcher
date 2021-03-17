var canvas = document.getElementById("canvas");
var ctx = canvas.getContext("2d");

function sketch(x1, y1, x2, y2, amplify) {
    
    if (x1 == null || x2 == null) {
        return;
    }

    console.log("x1: " + x1 + "y1: " + y1 + "x2: " + x2 + "y2: " +y2)
    ctx.strokeSyle = "#000";
    ctx.beginPath();
    ctx.moveTo(x1 * amplify, y1 * amplify)
    ctx.lineTo(x2 * amplify, y2 * amplify)

//    ctx.closePath();
    ctx.lineWidth=3;
    ctx.strokeStyle='blue';
    ctx.stroke();
    ctx.fillStyle='skyblue';
    ctx.fill();
}
