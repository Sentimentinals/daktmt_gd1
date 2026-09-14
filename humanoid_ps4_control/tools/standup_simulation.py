"""Export an offline, non-actuating Three.js view of GetupEngine commands."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
from urllib.request import urlopen

from src.balance import extend_arms_forward
from src.config import Config, DIR, PWM_PER_DEG, ROBOT, STANDING, STAND_ANG
from src.getup import GetupEngine


HTML = r'''<!doctype html>
<html lang="vi"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>H17 | Stand up</title>
<style>
*{box-sizing:border-box;letter-spacing:0}body{margin:0;background:#111515;color:#e7ece9;font:14px system-ui,sans-serif}
header{height:64px;display:flex;align-items:center;justify-content:space-between;padding:0 24px;border-bottom:1px solid #37413d}h1{font-size:20px;margin:0}header span{color:#9cacaa;font-size:12px}
main{display:grid;grid-template-columns:minmax(0,1fr) 300px;height:calc(100dvh - 64px)}#scene{position:relative;min-width:0;min-height:340px;overflow:hidden}canvas{display:block;width:100%;height:100%}
#label{position:absolute;top:20px;left:24px;pointer-events:none;font:16px monospace;color:#9ce2b0}
aside{border-left:1px solid #37413d;padding:20px;overflow:auto;background:#191e1c}h2{font-size:15px;margin:0 0 14px}label{display:block;margin:16px 0 7px;color:#c0cdc6}input[type=range]{width:100%;accent-color:#9ce2b0}select,button{background:#26312d;color:#eef6f1;border:1px solid #52665b;border-radius:4px;padding:8px;cursor:pointer}button{width:38px;height:36px;font-size:17px}button:hover{background:#3e5448}button:focus-visible,input:focus-visible,select:focus-visible{outline:2px solid #9ce2b0}button:disabled{opacity:.4}
.bar{display:flex;gap:8px;align-items:center;margin:12px 0}.bar select{margin-left:auto}output{font-family:monospace;color:#f6c96c}.time{display:flex;justify-content:space-between;font:12px monospace;color:#adbfb4}table{width:100%;border-collapse:collapse;margin-top:16px;font:12px monospace}td,th{padding:7px 4px;border-bottom:1px solid #343d38;text-align:right}td:first-child,th:first-child{text-align:left}.ankle{color:#f6c96c}#error{position:absolute;inset:80px 24px auto;color:#ff9d91}
@media(max-width:700px){header{padding:0 14px}header span{max-width:135px;text-align:right}main{display:flex;flex-direction:column;height:auto}#scene{height:52dvh;min-height:320px}aside{border-left:0;border-top:1px solid #37413d;overflow:visible;padding:16px}table{font-size:13px}}
</style>
<header><h1>H17 / Stand up</h1><span>17 DOF · PWM trajectory</span></header>
<main><section id="scene"><div id="label"></div><div id="error" role="alert"></div></section>
<aside><h2>Playback</h2><div class="bar"><button id="back" title="Previous phase" aria-label="Previous phase">&#9198;</button><button id="play" title="Play / pause" aria-label="Play / pause">&#9654;</button><button id="next" title="Next phase" aria-label="Next phase">&#9197;</button><button id="reset" title="Restart" aria-label="Restart">&#8634;</button><select id="speed" aria-label="Playback speed"><option value=".25">0.25x</option><option value=".5">0.5x</option><option selected value="1">1x</option><option value="2">2x</option></select></div>
<input id="seek" aria-label="Timeline" type="range" min="0" step="1" value="0"><div class="time"><span id="time"></span><span id="total"></span></div>
<label for="phase">Phase</label><select id="phase" style="width:100%"></select>
<label for="view">View</label><select id="view" style="width:100%"><option value="side">Side</option><option value="front">Front</option><option value="iso" selected>Perspective</option></select>
<label for="tilt">Goc than gia dinh: <output id="tiltValue">90°</output></label><input id="tilt" type="range" min="0" max="90" value="90" aria-label="Assumed body pitch">
<label for="spacing">Khoang cach hong hien thi: <output id="spacingValue">90 mm</output></label><input id="spacing" type="range" min="56" max="130" value="90" aria-label="Visual hip spacing">
<table><thead><tr><th>Servo</th><th>PWM (us)</th><th>Delta (deg)</th></tr></thead><tbody id="values"></tbody></table>
</aside></main>
<script>__THREE__</script><script>
const data=__DATA__;
const $=id=>document.getElementById(id),rad=Math.PI/180;
window.addEventListener('error',e=>{$('error').textContent=e.message});
let index=0,playing=false,clock=0,last=0;
const frames=data.frames, duration=frames.at(-1).t;
$('seek').max=frames.length-1;$('total').textContent=duration.toFixed(2)+' s';
const bounds=frames.reduce((a,f,i)=>{if(!i||f.phase!==frames[i-1].phase)a.push(i);return a},[]);
bounds.forEach(i=>{let o=document.createElement('option');o.value=i;o.textContent=frames[i].phase;$('phase').append(o)});
for(const id of Object.keys(data.standing)){let tr=document.createElement('tr');if(id==='15'||id==='18')tr.className='ankle';tr.innerHTML=`<td>${id}</td><td id="p${id}"></td><td id="a${id}"></td>`;$('values').append(tr)}
const scene=new THREE.Scene();scene.background=new THREE.Color('#111515');
const renderer=new THREE.WebGLRenderer({antialias:true,preserveDrawingBuffer:true});renderer.setPixelRatio(Math.min(devicePixelRatio,2));$('scene').append(renderer.domElement);
const camera=new THREE.PerspectiveCamera(40,1,1,2500);
scene.add(new THREE.HemisphereLight(0xffffff,0x5b6660,2.5));let light=new THREE.DirectionalLight(0xffffff,2);light.position.set(200,400,200);scene.add(light);
let grid=new THREE.GridHelper(600,24,0x57695d,0x27352d);grid.position.y=-225;scene.add(grid);
const root=new THREE.Group();scene.add(root);
const material=color=>new THREE.MeshStandardMaterial({color,roughness:.6,metalness:.35});
const metal=material(0x7b8883),black=material(0x272d2b),left=material(0x7eca99),right=material(0xe0b165),sole=material(0x131817);
function box(parent,w,h,d,x,y,z,mat){let m=new THREE.Mesh(new THREE.BoxGeometry(w,h,d),mat);m.position.set(x,y,z);parent.add(m);return m}
function joint(parent,x,y,z,mat){let g=new THREE.Group();g.position.set(x,y,z);parent.add(g);box(g,24,22,27,0,0,0,black);let disk=new THREE.Mesh(new THREE.CylinderGeometry(9,9,4,20),mat);disk.rotation.z=Math.PI/2;g.add(disk);return g}
box(root,95,82,40,0,56,0,metal);box(root,64,14,36,0,9,0,black);
const head=joint(root,0,116,0,metal);box(head,35,37,32,0,23,0,black);box(head,25,10,2,0,25,17,left);
const legs=[];
for(const side of ['L','R']){const s=side==='L'?-1:1, mat=s<0?left:right;
 let hipRoll=joint(root,s*Number($('spacing').value)/2,0,0,mat),hip=joint(hipRoll,0,0,0,mat);
 box(hip,21,data.robot.upper_leg,20,0,-data.robot.upper_leg/2,0,metal);
 let knee=joint(hip,0,-data.robot.upper_leg,0,mat);box(knee,19,data.robot.lower_leg,18,0,-data.robot.lower_leg/2,0,metal);
 let ankle=joint(knee,0,-data.robot.lower_leg,0,mat),roll=joint(ankle,0,0,0,mat);
 box(roll,43,7,77,0,-16,15,mat);box(roll,43,3,77,0,-21,15,sole);
 legs.push({side,hipRoll,hip,knee,ankle,roll,ids:s<0?[12,13,14,15,16]:[21,20,19,18,17]});
}
const arms=[];
for(const s of [-1,1]){let shoulder=joint(root,s*64,86,0,s<0?left:right),upper=joint(shoulder,0,0,0,metal);box(upper,15,61,16,0,-30,0,metal);let elbow=joint(upper,0,-61,0,metal);box(elbow,13,63,14,0,-31,0,metal);box(elbow,24,6,28,0,-66,7,black);arms.push({s,shoulder,upper,elbow,ids:s<0?[11,10,9]:[22,23,24]})}
function delta(p,id){return (p[id]-data.standing[id])/(data.direction[id]||1)/data.pwm_per_degree}
function angle(p,id,key){return data.angles[key]+delta(p,id)}
function draw(){let f=frames[index],p=f.pose;root.rotation.x=Number($('tilt').value)*rad;
 for(const l of legs)l.hipRoll.position.x=(l.side==='L'?-1:1)*Number($('spacing').value)/2;
 for(const l of legs){let [hr,h,k,a,r]=l.ids;l.hipRoll.rotation.z=(l.side==='L'?1:-1)*angle(p,hr,l.side+'_hip_abduct')*rad;l.hip.rotation.x=angle(p,h,l.side+'_hip_pitch')*rad;l.knee.rotation.x=-angle(p,k,l.side+'_knee')*rad;l.ankle.rotation.x=angle(p,a,l.side+'_ankle')*rad;l.roll.rotation.z=-delta(p,r)*rad}
 for(const a of arms){let [s,u,e]=a.ids;a.shoulder.rotation.x=-(p[s]-data.standing[s])/data.pwm_per_degree*a.s*rad;a.upper.rotation.z=(p[u]-data.standing[u])/data.pwm_per_degree*a.s*rad;a.elbow.rotation.x=-(p[e]-data.standing[e])/data.pwm_per_degree*a.s*rad}
 head.rotation.y=delta(p,25)*rad;
 root.position.y=0;root.updateMatrixWorld(true);
 root.position.y=grid.position.y-new THREE.Box3().setFromObject(root).min.y;
 $('spacingValue').textContent=$('spacing').value+' mm';
 $('label').textContent=f.phase;$('time').textContent=f.t.toFixed(2)+' s';$('seek').value=index;$('phase').value=bounds.filter(b=>b<=index).at(-1);$('tiltValue').textContent=$('tilt').value+'°';
 for(const id of Object.keys(data.standing)){$('p'+id).textContent=p[id];$('a'+id).textContent=delta(p,id).toFixed(1)}
 $('play').innerHTML=playing?'&#10074;&#10074;':'&#9654;';renderer.render(scene,camera);
 window.simState={index,phase:f.phase,pwm:p,playing,frames:frames.length};
}
function view(){let v=$('view').value;camera.position.set(...(v==='front'?[0,50,700]:v==='side'?[700,30,0]:[430,260,530]));camera.lookAt(0,-35,0);draw()}
function seek(i){index=Math.max(0,Math.min(frames.length-1,i));clock=frames[index].t;draw()}
$('seek').oninput=()=>{playing=false;seek(Number($('seek').value))};$('phase').onchange=()=>{playing=false;seek(Number($('phase').value))};
$('play').onclick=()=>{if(index===frames.length-1)seek(0);playing=!playing;draw()};$('reset').onclick=()=>{playing=false;seek(0)};
$('next').onclick=()=>{playing=false;seek(bounds.find(b=>b>index)??frames.length-1)};$('back').onclick=()=>{playing=false;seek(bounds.filter(b=>b<index).at(-1)??0)};
$('tilt').oninput=draw;$('spacing').oninput=draw;$('view').onchange=view;
new ResizeObserver(()=>{let w=$('scene').clientWidth,h=$('scene').clientHeight;renderer.setSize(w,h);camera.aspect=w/h;camera.updateProjectionMatrix();draw()}).observe($('scene'));
let dragging=false,startX=0;renderer.domElement.style.touchAction='none';renderer.domElement.onpointerdown=e=>{dragging=true;startX=e.clientX;renderer.domElement.setPointerCapture(e.pointerId)};renderer.domElement.onpointerup=()=>dragging=false;
renderer.domElement.onpointermove=e=>{if(!dragging)return;let a=(e.clientX-startX)*.008;startX=e.clientX;camera.position.applyAxisAngle(new THREE.Vector3(0,1,0),-a);camera.lookAt(0,-35,0);draw()};
function tick(t){if(playing){clock+=Math.min((t-last)/1000,.1)*Number($('speed').value);while(index<frames.length-1&&frames[index+1].t<=clock)index++;if(clock>=duration){index=frames.length-1;playing=false}draw()}last=t;requestAnimationFrame(tick)}
view();requestAnimationFrame(tick);
</script></html>'''


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=Path('out/standup_simulation.html'))
    parser.add_argument('--pose-json', type=Path, help='Initial servo PWM map, or dashboard /api/latest JSON')
    parser.add_argument('--three-js', type=Path, help='Local three.min.js (r160), instead of downloading')
    args = parser.parse_args()
    config = Config()
    initial = extend_arms_forward(STANDING, config.fall_arm_forward_pwm)
    if args.pose_json:
        raw = json.loads(args.pose_json.read_text(encoding='utf-8-sig'))
        initial = {**STANDING, **{int(k): int(v) for k, v in raw.get('pose_pwm', raw).items()}}
        if set(initial) != set(STANDING) or any(not 500 <= v <= 2500 for v in initial.values()):
            raise ValueError('Initial pose must contain valid servo IDs and PWM 500..2500')
    engine = GetupEngine(dt=config.update_ms / 1000, speed=config.getup_speed)
    engine.start(initial)
    frames = [{'t': 0.0, 'phase': engine.label, 'pose': initial}]
    while engine.running:
        phase = engine.label
        pose = engine.update()
        frames.append({'t': round(len(frames) * engine.dt, 6), 'phase': phase, 'pose': pose})
        if len(frames) > 10000:
            raise RuntimeError('Getup sequence did not finish')
    data = dict(frames=frames, standing=STANDING, angles=STAND_ANG, direction=DIR,
                pwm_per_degree=PWM_PER_DEG, robot=ROBOT)
    if args.three_js:
        library = args.three_js.read_text(encoding='utf-8')
    else:
        with urlopen('https://cdn.jsdelivr.net/npm/three@0.160.1/build/three.min.js', timeout=30) as response:
            library = response.read().decode('utf-8')
    page = HTML.replace('__THREE__', library.replace('</script', '<\\/script')).replace('__DATA__', json.dumps(data))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(page, encoding='utf-8')
    print(f'Offline simulation: {args.output.resolve()} ({len(frames)} frames)')
    print('Command visualization only. No hardware, contact forces, or physical balance simulation.')


if __name__ == '__main__':
    main()
