'use strict'
const first = document.querySelector('#js_first')
const ttllogo = document.querySelector('#js_logo');
const second = document.querySelector('#js_second');
second.style.transform = "translate(120%)";



first.addEventListener('click', function (event) {//クリックをすると…
    first.style.transform = "translate(120%)";
    second.style.transform = "translate(0)";
    console.log('logochange');
    // event.stopPropagation();
})

let fstr1 = 1;
let fsttimer1 = null;

//左上の波紋
setTimeout(() => {
    const canvas1 = document.querySelector('#fstcanvas1');
    fsttimer1 = setInterval(function () {
        fstr1 += 1;
        let ctx = canvas1.getContext('2d');//紙みたいなのを取得
        draw1(ctx, 110, 110);
        console.log('drawCanvas1');

        if (fstr1 >= 120) {
            console.log('timerup1');
            clearInterval(fsttimer1);
            fstr1 = 1;
        }
    }, 15);
}, 5 * 1000)

function draw1(ctx, x, y) {
    let gradient = ctx.createRadialGradient(x, y, 1, x, y, fstr1);
    gradient.addColorStop(0.0, 'rgba(250, 130, 0, 1)'); //開始色
    gradient.addColorStop(0.1, 'rgba(250, 130, 0, 0.1)'); //開始色
    gradient.addColorStop(1.0, 'rgba(250, 130, 0, 0)'); //終了色
    ctx.fillStyle = gradient;
    ctx.beginPath();
    ctx.arc(x, y, fstr1, 0, Math.PI * 2, true);
    ctx.fill();
}


//右下の波紋

let fstr2 = 1;
let fsttimer2 = null;

setTimeout(() => {
    const canvas2 = document.querySelector('#fstcanvas2');
    fsttimer2 = setInterval(function () {
        fstr2 += 1;
        let ctx = canvas2.getContext('2d');//紙みたいなのを取得
        draw2(ctx, 75, 75);
        console.log('drawCanvas2')

        if (fstr2 >= 75) {
            console.log('timerup1');
            clearInterval(fsttimer2);
            fstr2 = 50
        }
    }, 15);
}, 7 * 1000)

function draw2(ctx, x, y) {
    let gradient = ctx.createRadialGradient(x, y, 1, x, y, fstr2);
    gradient.addColorStop(0.0, 'rgba(5, 106, 180, 1)'); //開始色
    gradient.addColorStop(0.1, 'rgba(5, 106, 180, 0.1)'); //開始色
    gradient.addColorStop(1.0, 'rgba(5, 106, 180, 0)'); //終了色
    ctx.fillStyle = gradient;
    ctx.beginPath();
    ctx.arc(x, y, fstr2, 0, Math.PI * 2, true);
    ctx.fill();
}






//-----------------------------------ここからスポイトと連動し、動くところ-------------------------------------------//
'use strict'

const canvas = document.querySelector('#canvas');//取得
// canvas.width = 1000;
// canvas.height = 800;
// let R=221
// let G=87
// let B=21
let r = 50;
let timer = null;//タイマー用変数
let temp_e;
let pointerup_flag = false;
// drawCircle(ctx, 100, 100);

//イベント設定
canvas.addEventListener('pointerup', function (e) {
    pointerup_flag = true;
})

canvas.addEventListener('pointerdown', function (e) {
    //タイマー起動setInterval(20msec?)
    console.log('pointerdown');
    leafony.sendCommand('CSND');
    temp_e = e;
});
/**
 *xy
 */
function drawCircle(ctx, x, y) {
    let gradient = ctx.createRadialGradient(x, y, 1, x, y, r);
    // let color=document.querySelector('#color');
    // console.log(color.value);
    // R = document.getElementById('r');
    // G = document.getElementById('g');
    // B = document.getElementById('b');



    console.log(R, G, B);
    // gradient.addColorStop(0.0, 'rgba('+R+','+G+','+B+',1)'); //開始色
    // gradient.addColorStop(1.0, 'rgba('+R+','+G+','+B+',0)'); //終了色
    gradient.addColorStop(0.0, `rgba(${R}, ${G}, ${B},1)`); //開始色
    gradient.addColorStop(0.1, `rgba(${R}, ${G}, ${B},0.1)`); //終了色
    gradient.addColorStop(1.0, `rgba(${R}, ${G}, ${B},0)`); //終了色
    ctx.fillStyle = gradient;
    ctx.beginPath();
    ctx.arc(x, y, r, 0, Math.PI * 2, true);
    ctx.fill();
    // ctx.fillRect(x,y,50,50);
}

function colordown(e) {
    //タイマー起動setInterval(20msec?)
    console.log('colordown');
    // console.log(e);
    timer = setInterval(function () {
        r += 3;
        let ctx = canvas.getContext('2d');//紙みたいなのを取得
        drawCircle(ctx, e.clientX, e.clientY);
        // console.log(e.clientX);
        // console.log(e.clientY);
        if (pointerup_flag) {
            //タイマー停止
            clearInterval(timer);
            const data = {
                'leafony_name': leafony_name,
                'x': e.clientX,
                'y': e.clientY,
                'red': R,
                'green': G,
                'blue': B,
                'radius': r
            };

            const opt = {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json',
                    'Accept': 'application/json'
                },
                body: JSON.stringify(data)
            };
            fetch('https://www.jz.jec.ac.jp/mcpc2022/manage/public/api/colors', opt)
                .then(res => {
                    const result = (res.status == 201) ? 'データ登録成功' : 'データ登録失敗';
                    console.log(result);
                });
            console.log('pointerup');
            r = 50
            pointerup_flag = false;
        }
    }, 20);

}