var SN=['IDLE','FREEFALL','IMPACT','WAIT_LIE_DOWN','SOS'];
var SC=['#4ecca3','#ffc107','#ff9800','#ff9800','#e94560'];
var SBG=['rgba(78,204,163,0.12)','rgba(255,193,7,0.12)','rgba(255,152,0,0.12)','rgba(255,152,0,0.12)','rgba(233,69,96,0.15)'];

function updateData(){
  fetch('/api/data').then(function(r){return r.json()}).then(function(d){
    document.getElementById('accel_g').textContent=d.accel_g.toFixed(2);
    document.getElementById('gyro').textContent=d.gyro.toFixed(0);
    document.getElementById('roll').textContent=d.roll.toFixed(1);
    document.getElementById('pitch').textContent=d.pitch.toFixed(1);

    var w=document.getElementById('wifi'),wd=document.getElementById('wifiDot');
    if(d.wifi_connected){w.textContent='Connected';wd.className='wifi-dot on'}
    else{w.textContent='Disconnected';wd.className='wifi-dot off'}

    var s=d.fall_state||0,e=document.getElementById('fall_state');
    e.textContent=SN[s]||'UNKNOWN';
    e.style.color=SC[s]||'#fff';
    e.style.background=SBG[s]||'transparent';
    e.style.borderColor=SC[s]||'#fff';
    if(s===4){e.classList.add('sos')}else{e.classList.remove('sos')}

    var c=document.getElementById('container');
    if(d.alert_active){c.classList.add('alert')}else{c.classList.remove('alert')}
  });
}

setInterval(updateData,100);
