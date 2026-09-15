"""Export an offline replay of a free-base MuJoCo stand-up experiment.

Laptop-only optional dependency: python -m pip install mujoco==3.3.7
The exported HTML needs neither Python nor a connection to the robot.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
from urllib.request import urlopen

from src.balance import extend_arms_forward
from src.config import Config, STANDING, STAND_ANG
from src.getup import GetupEngine
from src.walking_engine import angle_to_pwm
from .standup_physics import simulate


HTML = r'''<!doctype html>
<html lang="vi"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>H17 | Stand up physics</title>
<style>
*{box-sizing:border-box;letter-spacing:0}body{margin:0;background:#111515;color:#e7ece9;font:14px system-ui,sans-serif}
header{height:64px;display:flex;align-items:center;justify-content:space-between;padding:0 24px;border-bottom:1px solid #37413d}h1{font-size:20px;margin:0}header span{color:#d6b875;font-size:12px}
main{display:grid;grid-template-columns:minmax(0,1fr) 320px;height:calc(100dvh - 64px)}#scene{position:relative;min-width:0;min-height:340px;overflow:hidden}canvas{display:block;width:100%;height:100%}
#label{position:absolute;top:20px;left:24px;pointer-events:none;font:16px monospace;color:#9ce2b0}
aside{border-left:1px solid #37413d;padding:20px;overflow:auto;background:#191e1c}h2{font-size:15px;margin:0 0 14px}label{display:block;margin:16px 0 7px;color:#c0cdc6}input[type=range]{width:100%;accent-color:#9ce2b0}select,button{background:#26312d;color:#eef6f1;border:1px solid #52665b;border-radius:4px;padding:8px;cursor:pointer}button{width:38px;height:36px;font-size:17px}button:hover{background:#3e5448}button:focus-visible,input:focus-visible,select:focus-visible{outline:2px solid #9ce2b0}
.bar{display:flex;gap:8px;align-items:center;margin:12px 0}.bar select{margin-left:auto}output{font-family:monospace;color:#f6c96c}.time{display:flex;justify-content:space-between;font:12px monospace;color:#adbfb4}table{width:100%;border-collapse:collapse;margin-top:16px;font:12px monospace}td,th{padding:7px 4px;border-bottom:1px solid #343d38;text-align:right}td:first-child,th:first-child{text-align:left}.ankle{color:#f6c96c}#error{position:absolute;inset:80px 24px auto;color:#ff9d91}
dl{display:grid;grid-template-columns:1fr 1fr;gap:8px;margin:16px 0;font-size:12px}dt{color:#a7b5ac}dd{margin:0;text-align:right;font-family:monospace;overflow-wrap:anywhere}#verdict{color:#ffb595;font-size:14px;line-height:1.5}details{border-top:1px solid #37413d;padding-top:12px}summary{cursor:pointer}
@media(max-width:700px){header{padding:0 14px}header span{max-width:135px;text-align:right}main{display:flex;flex-direction:column;height:auto}#scene{height:52dvh;min-height:320px}aside{border-left:0;border-top:1px solid #37413d;overflow:visible;padding:16px}table{font-size:13px}}
</style>
<header><h1>H17 / Stand up</h1><span>MuJoCo · Chưa hiệu chuẩn</span></header>
<main><section id="scene"><div id="label"></div><div id="error" role="alert"></div></section>
<aside><h2>Playback</h2><div class="bar"><button id="back" title="Previous phase" aria-label="Previous phase">&#9198;</button><button id="play" title="Play / pause" aria-label="Play / pause">&#9654;</button><button id="next" title="Next phase" aria-label="Next phase">&#9197;</button><button id="reset" title="Restart" aria-label="Restart">&#8634;</button><select id="speed" aria-label="Playback speed"><option value=".25">0.25x</option><option value=".5">0.5x</option><option selected value="1">1x</option><option value="2">2x</option></select></div>
<input id="seek" aria-label="Timeline" type="range" min="0" step="1" value="0"><div class="time"><span id="time"></span><span id="total"></span></div>
<label for="case">Mô hình / quỹ đạo</label><select id="case" style="width:100%"></select>
<label for="phase">Phase</label><select id="phase" style="width:100%"></select>
<label for="view">View</label><select id="view" style="width:100%"><option value="side">Side</option><option value="front">Front</option><option value="iso" selected>Perspective</option></select>
<dl><dt>Độ nghiêng thân</dt><dd id="tilt"></dd><dt>Cao độ hông</dt><dd id="height"></dd><dt>Tiếp xúc sàn</dt><dd id="contacts"></dd><dt>Đứng ổn định</dt><dd id="stable"></dd></dl>
<p id="verdict" role="status"></p>
<details><summary>Thông số giả định</summary><dl id="assumptions"></dl></details>
<table><thead><tr><th>Servo</th><th>Lệnh (us)</th><th>Khớp mô phỏng (°)</th></tr></thead><tbody id="values"></tbody></table>
</aside></main>
<script>__THREE__</script><script>
const dataset=__DATA__;
let data=dataset.cases[0];
const $=id=>document.getElementById(id);
window.addEventListener('error',e=>{$('error').textContent=e.message});
let index=0,playing=false,clock=0,last=0;
let frames=data.frames, duration=frames.at(-1).t;
$('seek').max=frames.length-1;$('total').textContent=duration.toFixed(2)+' s';
let bounds=frames.reduce((a,f,i)=>{if(!i||f.phase!==frames[i-1].phase)a.push(i);return a},[]);
dataset.cases.forEach((c,i)=>{const o=document.createElement('option');o.value=i;o.textContent=c.label;$('case').append(o)});
for(const i of bounds){const o=document.createElement('option');o.value=i;o.textContent=frames[i].phase;$('phase').append(o)}
for(const id of Object.keys(frames[0].pose)){
 const tr=document.createElement('tr');if(id==='15'||id==='18')tr.className='ankle';
 tr.innerHTML=`<td>${id}</td><td id="p${id}"></td><td id="a${id}"></td>`;$('values').append(tr);
}
function showAssumptions(){
$('assumptions').replaceChildren();
for(const [label,value] of [['Vai - khuỷu',data.assumptions.upper_arm_mm+' mm'],['Khuỷu - tay chống',data.assumptions.forearm_mm+' mm'],['Khối lượng',data.assumptions.mass_kg+' kg'],['Mô-men giới hạn',data.assumptions.torque_nm+' Nm'],['Ma sát',data.assumptions.friction],['Khoảng cách hông',data.assumptions.hip_spacing_mm+' mm'],['Khoảng hông source',data.assumptions.source_hip_spacing_mm+' mm'],['Tốc độ đặt góc',data.assumptions.servo_speed_rad_s+' rad/s']]){
 const dt=document.createElement('dt'),dd=document.createElement('dd');dt.textContent=label;dd.textContent=value;$('assumptions').append(dt,dd);
}}
showAssumptions();
const scene=new THREE.Scene();scene.background=new THREE.Color('#111515');
const renderer=new THREE.WebGLRenderer({antialias:true,preserveDrawingBuffer:true});
renderer.setPixelRatio(Math.min(devicePixelRatio,2));$('scene').append(renderer.domElement);
const camera=new THREE.PerspectiveCamera(40,1,1,4000);
scene.add(new THREE.HemisphereLight(0xffffff,0x64786a,2.5));
const light=new THREE.DirectionalLight(0xffffff,2);light.position.set(200,400,200);scene.add(light);
const grid=new THREE.GridHelper(1200,48,0x57695d,0x27352d);grid.position.y=-.5;scene.add(grid);
function createMeshes(){return data.geometry.map(g=>{
 const m=new THREE.Mesh(new THREE.BoxGeometry(...g.size),new THREE.MeshStandardMaterial({color:new THREE.Color(...g.color),roughness:.65,metalness:.2}));scene.add(m);return m;
})}
let meshes=createMeshes();
// Frame the recorded path without changing the robot's simulated transform.
const center=new THREE.Vector3();let distance;
function framePath(){const extents=new THREE.Box3();
for(const f of frames)for(const t of f.transforms)extents.expandByPoint(new THREE.Vector3(...t.slice(0,3)));
extents.expandByScalar(30);extents.getCenter(center);
distance=Math.max(500,extents.getSize(new THREE.Vector3()).length()*1.6);}
framePath();
function draw(){
 const f=frames[index],contact=new Set(f.contacts);
 f.transforms.forEach((t,i)=>{meshes[i].position.set(...t.slice(0,3));meshes[i].quaternion.set(...t.slice(3));meshes[i].material.emissive.set(contact.has(data.geometry[i].name)?0x382509:0x000000)});
 $('label').textContent=f.phase;$('time').textContent=f.t.toFixed(2)+' s';$('seek').value=index;
 $('phase').value=bounds.filter(b=>b<=index).at(-1);$('tilt').textContent=f.tilt_deg.toFixed(1)+'°';$('height').textContent=f.height_mm.toFixed(1)+' mm';$('stable').textContent=f.stable_s.toFixed(2)+' s';
 $('contacts').textContent=f.contacts.length?f.contacts.join(', '):'Không';
 $('verdict').textContent=index===frames.length-1?(data.result==='STANDING IN MODEL'?'ĐỨNG ĐƯỢC TRONG MÔ HÌNH':data.release_blocked?'CHƯA ĐỨNG ĐƯỢC - GIỮ TAY CHỐNG':'CHƯA ĐỨNG ĐƯỢC TRONG MÔ HÌNH'):'Chưa kết thúc lượt kiểm tra';
 for(const id of Object.keys(f.pose)){$('p'+id).textContent=f.pose[id];$('a'+id).textContent=f.actual_deg[id].toFixed(1)}
 $('play').innerHTML=playing?'&#10074;&#10074;':'&#9654;';renderer.render(scene,camera);
 window.simState={index,phase:f.phase,pwm:f.pose,playing,frames:frames.length,tilt:f.tilt_deg,height:f.height_mm,contacts:f.contacts,stable:f.stable_s,result:data.result,root:f.root_mm,caseLabel:data.label,releaseBlocked:data.release_blocked};
}
function view(){
 const v=$('view').value,offset=new THREE.Vector3(...(v==='front'?[0,.2,1]:v==='side'?[1,.2,0]:[.8,.5,1])).normalize().multiplyScalar(distance);
 camera.position.copy(center).add(offset);camera.lookAt(center);draw();
}
function seek(i){index=Math.max(0,Math.min(frames.length-1,i));clock=frames[index].t;draw()}
$('seek').oninput=()=>{playing=false;seek(Number($('seek').value))};$('phase').onchange=()=>{playing=false;seek(Number($('phase').value))};
$('play').onclick=()=>{if(index===frames.length-1)seek(0);playing=!playing;draw()};$('reset').onclick=()=>{playing=false;seek(0)};
$('next').onclick=()=>{playing=false;seek(bounds.find(b=>b>index)??frames.length-1)};$('back').onclick=()=>{playing=false;seek(bounds.filter(b=>b<index).at(-1)??0)};
$('view').onchange=view;
$('case').onchange=()=>{
 playing=false;index=0;clock=0;data=dataset.cases[Number($('case').value)];frames=data.frames;duration=frames.at(-1).t;
 $('seek').max=frames.length-1;$('total').textContent=duration.toFixed(2)+' s';
 bounds=frames.reduce((a,f,i)=>{if(!i||f.phase!==frames[i-1].phase)a.push(i);return a},[]);
 $('phase').replaceChildren();for(const i of bounds){const o=document.createElement('option');o.value=i;o.textContent=frames[i].phase;$('phase').append(o)}
 for(const m of meshes){scene.remove(m);m.geometry.dispose();m.material.dispose()}meshes=createMeshes();
 showAssumptions();framePath();view();
};
new ResizeObserver(()=>{const w=$('scene').clientWidth,h=$('scene').clientHeight;renderer.setSize(w,h);camera.aspect=w/h;camera.updateProjectionMatrix();draw()}).observe($('scene'));
let dragging=false,startX=0;renderer.domElement.style.touchAction='none';
renderer.domElement.onpointerdown=e=>{dragging=true;startX=e.clientX;renderer.domElement.setPointerCapture(e.pointerId)};
renderer.domElement.onpointerup=()=>dragging=false;
renderer.domElement.onpointermove=e=>{if(!dragging)return;const a=(e.clientX-startX)*.008;startX=e.clientX;camera.position.sub(center).applyAxisAngle(new THREE.Vector3(0,1,0),-a).add(center);camera.lookAt(center);draw()};
function tick(t){if(playing){clock+=Math.min((t-last)/1000,.1)*Number($('speed').value);while(index<frames.length-1&&frames[index+1].t<=clock)index++;if(clock>=duration){index=frames.length-1;playing=false}draw()}last=t;requestAnimationFrame(tick)}
view();requestAnimationFrame(tick);
</script></html>'''


def command_frames(config: Config, initial: dict[int, int]):
    engine = GetupEngine(dt=config.update_ms / 1000, speed=config.getup_speed)
    engine.start(initial)
    frames = [{'t': 0.0, 'phase': engine.label, 'pose': initial}]
    while engine.running:
        phase = engine.label
        pose = engine.update()
        frames.append({'t': round(len(frames) * engine.dt, 6), 'phase': phase, 'pose': pose})
        if len(frames) > 10000:
            raise RuntimeError('Getup sequence did not finish')
    return frames


def recovery_command_frames(config: Config, initial: dict[int, int]):
    """Simulation-only candidate; keep arms until measured support allows release."""
    stages = [
        ('plant-feet', 0.9, 30, 55, 90),
        ('tuck-knees', 1.6, 108, 110, 62),
        ('shift-over-feet', 1.4, 100, 126, 61),
        ('upright-crouch', 1.6, 70, 120, 50),
        ('extend-legs', 1.6, 18, 36, 18),
        ('hold-standing', 0.6, 18, 36, 18),
        ('release-arms', 1.0, 18, 36, 18),
    ]
    dt = config.update_ms / 1000
    pose = dict(initial)
    frames = [dict(t=0.0, phase='support', pose=pose)]
    for label, duration, hip, knee, ankle in stages:
        target = dict(pose)
        for side, ids in (('L', (13, 14, 15)), ('R', (20, 19, 18))):
            bases = [STAND_ANG[side + '_' + name] for name in ('hip_pitch', 'knee', 'ankle')]
            for sid, base, value in zip(ids, bases, (hip, knee, ankle)):
                target[sid] = angle_to_pwm(sid, base, value, STANDING[sid])
        if label == 'release-arms':
            target = dict(STANDING)
        ticks = max(1, round(duration / dt))
        for tick in range(1, ticks + 1):
            progress = tick / ticks
            progress = progress * progress * (3 - 2 * progress)
            blended = {sid: round(pose[sid] + progress * (target[sid] - pose[sid])) for sid in pose}
            frames.append(dict(t=round(len(frames) * dt, 6), phase=label, pose=blended))
        pose = target
    return frames


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=Path('out/standup_simulation.html'))
    parser.add_argument('--pose-json', type=Path, help='Initial servo PWM map, or dashboard /api/latest JSON')
    parser.add_argument('--three-js', type=Path, help='Local three.min.js (r160), instead of downloading')
    parser.add_argument('--torque-nm', type=float, default=2.0, help='Assumed actuator torque limit, not measured')
    parser.add_argument('--friction', type=float, default=0.8, help='Assumed contact friction, not measured')
    parser.add_argument('--hip-spacing-mm', type=float, default=90.0, help='Assumed hip spacing; source is 56 mm')
    parser.add_argument('--upper-arm-mm', type=float, default=75.0, help='Reference model shoulder-to-elbow length, NOT measured')
    parser.add_argument('--forearm-mm', type=float, default=75.0, help='Reference model elbow-to-palm length, NOT measured')
    parser.add_argument('--report', type=Path, help='Optional physics result JSON')
    args = parser.parse_args()
    if (not 0 < args.torque_nm <= 10 or not 0 < args.friction <= 3 or not 50 <= args.hip_spacing_mm <= 160
            or not 40 <= args.upper_arm_mm <= 150 or not 40 <= args.forearm_mm <= 150):
        parser.error('Invalid experimental torque, friction, hip spacing, or arm length')
    config = Config()
    initial = extend_arms_forward(STANDING, config.fall_arm_forward_pwm)
    if args.pose_json:
        raw = json.loads(args.pose_json.read_text(encoding='utf-8-sig'))
        initial = {**STANDING, **{int(k): int(v) for k, v in raw.get('pose_pwm', raw).items()}}
        if set(initial) != set(STANDING) or any(not 500 <= v <= 2500 for v in initial.values()):
            raise ValueError('Initial pose must contain valid servo IDs and PWM 500..2500')
    physics = dict(torque_nm=args.torque_nm, friction=args.friction, hip_spacing_mm=args.hip_spacing_mm,
                   upper_arm_mm=args.upper_arm_mm, forearm_mm=args.forearm_mm, hold_s=3.0)
    proposed = recovery_command_frames(config, initial)
    cases = [
        dict(label=f'Quy dao moi | tay {args.upper_arm_mm:g}/{args.forearm_mm:g} mm (gia dinh)',
             **simulate(proposed, **physics, gate_arm_release=True)),
        dict(label='Code hien tai | cung kich thuoc tham chieu',
             **simulate(command_frames(config, initial), **physics)),
        dict(label='Quy dao moi | tay 61/66 mm (mo hinh cu)',
             **simulate(proposed, **{**physics, 'upper_arm_mm': 61.0, 'forearm_mm': 66.0}, gate_arm_release=True)),
    ]
    data = {'cases': cases}
    if args.three_js:
        library = args.three_js.read_text(encoding='utf-8')
    else:
        with urlopen('https://cdn.jsdelivr.net/npm/three@0.160.1/build/three.min.js', timeout=30) as response:
            library = response.read().decode('utf-8')
    page = HTML.replace('__THREE__', library.replace('</script', '<\\/script')).replace('__DATA__', json.dumps(data, allow_nan=False))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(page, encoding='utf-8')
    if args.report:
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(json.dumps(data, allow_nan=False), encoding='utf-8')
    print(f'Offline physics replay: {args.output.resolve()}')
    for case in cases:
        print(case['label'], case['result'], 'release_blocked=', case['release_blocked'])
    print('Uncalibrated geometry, mass, friction and actuators. Not a hardware validation.')


if __name__ == '__main__':
    main()
