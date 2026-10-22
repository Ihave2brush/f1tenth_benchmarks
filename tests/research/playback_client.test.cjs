// Run the actual client controller with delayed HTTP responses and a minimal
// DOM/canvas harness. These are async-ordering tests, not browser automation.
const test = require('node:test');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');

const source = fs.readFileSync(path.join(__dirname,
  '../../f1tenth_benchmarks/research/overtaking/playback.js'), 'utf8');
const tick = () => new Promise(resolve => setImmediate(resolve));

function sample(step) {
  return {control_step:step, sim_time_s:7.3+step*.1, relative_time_s:step*.1,
    valid:true, pose:[0,0,0], pixel:[0,0], body:[], heading_pixel:null,
    speed_mps:1.2, frenet:{s_wrapped:step,s_unwrapped:step,d:0,e_psi:0},
    trail:[], plan:null, candidate_age_s:null,
    at_end:step===2, end_reason:step===2?'requested_laps_reached':null};
}

async function harness() {
  const elements = new Map(), requests = [], timers = [];
  function element(id) {
    if (!elements.has(id)) elements.set(id, {
      value:id==='rate'?'1':'0', checked:true, disabled:false, style:{}, textContent:'',
      width:600,height:400,clientWidth:600,clientHeight:400,
      replaceChildren(){}, append(){}, addEventListener(){},
      getContext:() => new Proxy({}, {get:(target,key) => target[key] || (()=>{})}),
    });
    return elements.get(id);
  }
  const meta = {state_count:3,plan_count:0,timeline:[0,.1,.2],duration_s:.2,
    origin_sim_time_s:7.3,width:100,height:100,center:[],left:[],right:[],
    completed_full_laps:1,session_name:'fixture',index_status:'verified',
    lookahead_m:40,plan_period_s:.1,control_dt_s:.1,recording_complete:true};
  const context = vm.createContext({
    document:{getElementById:element, createElement:() => element(Symbol()), addEventListener(){}},
    window:{addEventListener(){}}, performance:{now:()=>0}, AbortController,
    Image:class {async decode(){}},
    setTimeout:(fn,delay) => {timers.push({fn,delay}); return timers.length;}, clearTimeout(){},
    fetch:(url) => url==='/api/session' ? Promise.resolve({ok:true,json:async()=>meta}) :
      new Promise(resolve => requests.push({url,respond:(frame)=>resolve({ok:true,json:async()=>frame})})),
  });
  vm.runInContext(source, context);
  await tick();
  requests.shift().respond(sample(0)); await tick();
  return {context, requests, timers, elements,
    run:code=>vm.runInContext(code,context), el:element};
}

test('late future response cannot overwrite a newer backward seek', async () => {
  const h = await harness();
  assert.match(h.el('planningSettings').textContent,/前視 40.0 m/);
  assert.match(h.el('planningSettings').textContent,/每 0.1 s 更新/);
  const future = h.run("loadFrame('step=2')");
  const backward = h.run("loadFrame('step=1')");
  h.requests[1].respond(sample(1)); await backward;
  h.requests[0].respond(sample(2)); await future;
  assert.equal(h.run('cursor'),1);
  assert.match(h.el('position').textContent,/1 \/ 2/);
});

test('pause invalidates an in-flight automatic frame', async () => {
  const h = await harness();
  h.run('play()');
  const request = h.run("loadFrame('step=1',false,playEpoch)");
  h.run('pause()');
  h.requests[0].respond(sample(1)); await request;
  assert.equal(h.run('cursor'),0);
  assert.equal(h.run('playing'),false);
});

test('play cannot start from an old cursor while manual seek is pending', async () => {
  const h = await harness();
  const request = h.run("loadFrame('step=1')");
  h.run('play()');
  assert.equal(h.run('playing'),false);
  assert.equal(h.el('play').disabled,true);
  h.requests[0].respond(sample(1)); await request;
  assert.equal(h.run('cursor'),1);
  assert.equal(h.el('play').disabled,false);
});

test('all playback rates use recorded intervals; end does not restart', async () => {
  const h = await harness();
  for (const rate of [.25,.5,1,2,4]) {
    h.el('rate').value=String(rate); h.run('play()');
    assert.equal(h.timers.at(-1).delay,100/rate); h.run('pause()');
  }
  const request = h.run("loadFrame('step=2')");
  h.requests[0].respond(sample(2)); await request;
  h.run('play()');
  assert.equal(h.run('cursor'),2);
  assert.equal(h.run('playing'),false);
  assert.equal(h.el('play').disabled,true);
  assert.match(h.el('playbackStatus').textContent,/已到結尾/);
});
