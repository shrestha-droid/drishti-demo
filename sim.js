/*
 * Drishti traffic simulation engine.
 *
 * A small microscopic simulator for Indian mixed traffic on a 2 x 2 grid of
 * signalised junctions. Left-hand traffic, two lanes per direction, two-wheelers,
 * autos, cars, buses, trucks, cycles, pedestrians and stray cattle.
 *
 * Two signal controllers:
 *   - "fixed":    a fixed-time plan split by average demand (what most Indian
 *                 junctions run today).
 *   - "adaptive": camera-detected, PCU-weighted actuated / pressure control with
 *                 gap-out, spillback awareness, pedestrian demand and an
 *                 emergency-vehicle green corridor.
 *
 * Runs in the browser (window.DrishtiSim) and in Node (require('./sim.js')).
 * Units: 1 world unit = 0.25 m, time in seconds.
 */
(function (root, factory) {
  if (typeof module === 'object' && module.exports) module.exports = factory();
  else root.DrishtiSim = factory();
})(typeof self !== 'undefined' ? self : this, function () {
  'use strict';

  const U = 4;                              // world units per metre
  const DT = 0.05;                          // fixed step, seconds
  const WORLD = { w: 1500, h: 1000 };
  const LANE_W = 14;                        // 3.5 m
  const MEDIAN = 4;
  const HALF_ROAD = MEDIAN / 2 + 2 * LANE_W;
  const CROSSWALK = 12;
  const SIDEWALK = 16;
  const PED_OFF = HALF_ROAD + SIDEWALK / 2;
  const B_COMF = 2.5 * U;                   // comfortable deceleration
  const B_MAX = 8 * U;
  const YELLOW = 3;
  const ALL_RED = 2;
  const DETECT = 240;                       // camera detection zone, 60 m upstream
  const PREEMPT = 1000;                     // green-corridor horizon ahead of an ambulance, 250 m
  const CO2_PER_L = 2.4;                    // kg CO2 per litre, petrol/diesel blend

  // Adaptive tuning: switch early when the waiting side outweighs the green side.
  const TUNE = { gain: 3, bias: 10, minSwitch: 15, gapNear: 0.5, gapHold: 0, spill: 6, maxG: 45, minG: 8 };

  const ROADS = [
    { id: 'mg', name: 'MG Road', axis: 'h', c: 310, demand: 0.42 },
    { id: 'stn', name: 'Station Road', axis: 'h', c: 690, demand: 0.24 },
    { id: 'nehru', name: 'Nehru Marg', axis: 'v', c: 470, demand: 0.27 },
    { id: 'tilak', name: 'Tilak Road', axis: 'v', c: 1030, demand: 0.2 },
  ];

  // Approximate PCU factors and idle fuel burn (litres/hour) per class.
  const TYPES = {
    two:   { label: '2-wheeler', tag: '2W',        len: 8,  wid: 3.4, vmax: 12.5 * U, acc: 2.6 * U, s0: 1.0 * U, T: 0.8, pcu: 0.5, share: 0.44, idle: 0.12, kerb: 0.5,  vio: 0.10 },
    auto:  { label: 'Auto-rickshaw', tag: 'AUTO',  len: 11, wid: 5.6, vmax: 9.5 * U,  acc: 1.6 * U, s0: 1.5 * U, T: 1.0, pcu: 1.0, share: 0.15, idle: 0.35, kerb: 0.8,  vio: 0.06 },
    car:   { label: 'Car', tag: 'CAR',             len: 17, wid: 7,   vmax: 13.5 * U, acc: 2.0 * U, s0: 2.0 * U, T: 1.2, pcu: 1.0, share: 0.29, idle: 0.7,  kerb: 0.3,  vio: 0.03 },
    bus:   { label: 'City bus', tag: 'BUS',        len: 44, wid: 10,  vmax: 11 * U,   acc: 1.0 * U, s0: 2.5 * U, T: 1.5, pcu: 3.0, share: 0.06, idle: 2.4,  kerb: 0.85, vio: 0.02 },
    truck: { label: 'Truck', tag: 'TRUCK',         len: 32, wid: 9.6, vmax: 10.5 * U, acc: 0.9 * U, s0: 2.5 * U, T: 1.5, pcu: 3.0, share: 0.03, idle: 2.0,  kerb: 0.85, vio: 0.02 },
    cycle: { label: 'Bicycle', tag: 'CYCLE',       len: 7,  wid: 2.6, vmax: 4.5 * U,  acc: 0.8 * U, s0: 1.0 * U, T: 1.0, pcu: 0.5, share: 0.03, idle: 0,    kerb: 1.0,  vio: 0.15 },
    amb:   { label: 'Ambulance', tag: 'AMB',       len: 20, wid: 8,   vmax: 16 * U,   acc: 2.5 * U, s0: 2.0 * U, T: 0.9, pcu: 1.0, share: 0,    idle: 0.8,  kerb: 0,    vio: 0 },
  };
  const MIX = Object.keys(TYPES).filter((k) => TYPES[k].share > 0);

  const STATES = ['MH', 'KA', 'DL', 'TN', 'TS', 'GJ', 'UP', 'WB', 'RJ', 'KL'];
  const PED_COLORS = ['#e07a2f', '#c2367a', '#2f8f8a', '#6b4fb8', '#d4a017', '#3a6fc4', '#b8452f', '#4d7a3a'];
  const AXIS = ['E–W', 'N–S'];

  function mulberry32(a) {
    return function () {
      a |= 0; a = (a + 0x6d2b79f5) | 0;
      let t = Math.imul(a ^ (a >>> 15), 1 | a);
      t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
      return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
    };
  }

  const laneLat = (lane) => MEDIAN / 2 + LANE_W * (lane + 0.5);

  function plate(r) {
    const st = STATES[Math.floor(r() * STATES.length)];
    const d = String(1 + Math.floor(r() * 40)).padStart(2, '0');
    const l = String.fromCharCode(65 + Math.floor(r() * 26), 65 + Math.floor(r() * 26));
    const n = String(1000 + Math.floor(r() * 9000));
    return `${st} ${d} ${l} ${n}`;
  }

  function signalOf(j, phase) {
    if (j.state === 'AR') return 'R';
    return j.phase === phase ? j.state : 'R';
  }

  const mkApp = () => ({ q: 0, near: 0, maxWait: 0, peds: 0, spill: 0 });

  function buildNetwork(rng) {
    const hRoads = ROADS.filter((r) => r.axis === 'h');
    const vRoads = ROADS.filter((r) => r.axis === 'v');
    const junctions = [];
    let n = 1;
    for (const h of hRoads) {
      for (const v of vRoads) {
        const dh = h.demand, dv = v.demand;
        const g0 = Math.max(20, Math.min(60, Math.round(80 * dh / (dh + dv))));
        junctions.push({
          id: `J${n++}`, name: `${h.name} × ${v.name}`, x: v.c, y: h.c, h, v,
          phase: 0, state: 'G', t: rng() * 20, nextPhase: 1,
          fixedG: [g0, 80 - g0], lastGreenEnd: [-99, -99], preempt: null,
          app: [mkApp(), mkApp()], qByStream: {}, decision: '', pressure: [0, 0],
          switches: 0,
        });
      }
    }

    const streams = [];
    for (const r of ROADS) {
      for (const dir of [1, -1]) {
        const hx = r.axis === 'h' ? dir : 0;
        const hy = r.axis === 'v' ? dir : 0;
        const L = r.axis === 'h' ? WORLD.w : WORLD.h;
        const start = r.axis === 'h'
          ? { x: dir > 0 ? 0 : WORLD.w, y: r.c }
          : { x: r.c, y: dir > 0 ? 0 : WORLD.h };
        const compass = r.axis === 'h' ? (dir > 0 ? 'EB' : 'WB') : (dir > 0 ? 'SB' : 'NB');
        // Left-hand traffic: lateral offsets are measured along the driver's left normal.
        const st = {
          id: `${r.id}-${compass}`, road: r, dir, compass, hx, hy, lx: hy, ly: -hx,
          start, L, lanes: [[], []], pending: [], hazards: [], phi: rng() * Math.PI * 2,
          phase: r.axis === 'h' ? 0 : 1, junctions: [],
        };
        for (const j of junctions) {
          if (j.h !== r && j.v !== r) continue;
          const coord = r.axis === 'h' ? j.x : j.y;
          const sC = dir > 0 ? coord : L - coord;
          st.junctions.push({ j, sC, stopS: sC - HALF_ROAD - CROSSWALK - 2, exitS: sC + HALF_ROAD, phase: st.phase });
        }
        st.junctions.sort((a, b) => a.stopS - b.stopS);
        streams.push(st);
      }
    }

    // Pedestrian sidewalk lines; each crosses the perpendicular roads at zebra crossings.
    const pedLines = [];
    for (const r of ROADS) {
      for (const side of [-1, 1]) {
        const perp = ROADS.filter((p) => p.axis !== r.axis);
        pedLines.push({
          road: r, axis: r.axis, c: r.c + side * PED_OFF,
          L: r.axis === 'h' ? WORLD.w : WORLD.h,
          zones: perp.map((p) => ({
            a: p.c - HALF_ROAD - 2, b: p.c + HALF_ROAD + 2,
            j: junctions.find((j) => (j.h === r && j.v === p) || (j.v === r && j.h === p)),
            phase: r.axis === 'h' ? 0 : 1,
          })),
        });
      }
    }
    return { junctions, streams, pedLines };
  }

  function createSim(opts = {}) {
    const rng = mulberry32(opts.seed || 7);
    const net = buildNetwork(rng);
    const sim = {
      // Arrivals use their own random stream so every controller sees identical demand.
      t: 0, rng, arr: mulberry32((opts.seed || 7) * 7919 + 1), controller: opts.controller || 'adaptive',
      demand: opts.demand ?? 1, surge: !!opts.surge,
      autoIncidents: !!opts.autoIncidents, warmupT: opts.warmupT || 0,
      junctions: net.junctions, streams: net.streams, pedLines: net.pedLines,
      vehicles: new Map(), peds: [], hazards: [], events: [], eventSeq: 0,
      nextId: 1, nextAmb: 101, nextIncidentT: 90,
      stats: {
        done: [], samples: [], co2: 0, stoppedVS: 0, exited: 0,
        violations: { red: 0, helmet: 0, triple: 0 },
        amb: [], pedWait: 0, pedN: 0,
        acc: { n: 0, delay: 0, stops: 0, co2: 0, pedWait: 0, pedN: 0 },
      },
    };
    const nPeds = opts.peds ?? 46;
    for (let i = 0; i < nPeds; i++) spawnPed(sim, true);
    return sim;
  }

  function log(sim, kind, sev, text, sub, ref) {
    sim.events.push({ seq: ++sim.eventSeq, t: sim.t, kind, sev, text, sub: sub || '', ref: ref || null });
    if (sim.events.length > 200) sim.events.splice(0, sim.events.length - 200);
  }

  // ---------------------------------------------------------------- spawning

  function pickType(r) {
    const x = r();
    let acc = 0;
    for (const k of MIX) { acc += TYPES[k].share; if (x < acc) return k; }
    return 'car';
  }

  function makeVehicle(sim, st, type, lane, t0, seed) {
    const T = TYPES[type], r = mulberry32(seed);
    const v = {
      id: sim.nextId++, kind: 'veh', type, p: T, stream: st, lane, s: 0, v: 0, acc: 0,
      len: T.len, wid: T.wid, vmax: T.vmax * (type === 'amb' ? 1 : 0.85 + 0.3 * r()),
      t0, stops: 0, stopped: 0, moving: true, lineWait: 0,
      jit: type === 'two' || type === 'cycle' ? (r() - 0.5) * 6 : (r() - 0.5) * 1.5,
      latVis: laneLat(lane), plate: plate(r), commit: {}, nextJ: 0, lcTimer: r() * 0.5,
      violator: r() < T.vio, noHelmet: type === 'two' && r() < 0.12, triple: type === 'two' && r() < 0.04,
      flags: [], conf: 0.9 + 0.09 * r(), shade: r(),
    };
    if (type === 'amb') v.callsign = `108-${sim.nextAmb++}`;
    return v;
  }

  function trySpawn(sim, st) {
    const p = st.pending[0];
    if (!p) return;
    const T = TYPES[p.type];
    if (p.lane == null) p.lane = p.type === 'amb' ? 0 : (p.r < T.kerb ? 1 : 0);
    for (const lane of [p.lane, 1 - p.lane]) {
      const arr = st.lanes[lane];
      const last = arr[arr.length - 1];
      if (last && last.s - last.len < T.s0 + 2) continue;
      if (st.hazards.some((h) => h.lane === lane && h.s < 40)) continue;
      const v = makeVehicle(sim, st, p.type, lane, p.t0, p.seed);
      v.v = Math.min(v.vmax, last ? last.v + 2 * U : v.vmax) * 0.8;
      arr.push(v);
      sim.vehicles.set(v.id, v);
      st.pending.shift();
      if (v.type === 'amb') {
        log(sim, 'emergency', 'warn', `${v.callsign} dispatched on ${st.road.name} (${st.compass})`,
          sim.controller === 'adaptive' ? 'Green corridor requested from junctions ahead' : 'Fixed-time signals: no pre-emption', v.id);
      }
      return;
    }
  }

  function spawnVehicles(sim) {
    const k = sim.demand * (sim.surge ? 1.4 : 1);
    for (const st of sim.streams) {
      const rate = st.road.demand * k * (1 + 0.35 * Math.sin((2 * Math.PI * sim.t) / 360 + st.phi));
      const a = sim.arr;
      const hit = a() < rate * DT;
      const type = pickType(a), r = a(), seed = Math.floor(a() * 4294967296);
      if (hit && st.pending.length < 80) st.pending.push({ type, t0: sim.t, r, seed });
      trySpawn(sim, st);
    }
  }

  function dispatchAmbulance(sim, streamId) {
    const st = sim.streams.find((s) => s.id === streamId) || sim.streams[0];
    st.pending.unshift({ type: 'amb', t0: sim.t, lane: 0, seed: sim.nextAmb });
    return st;
  }

  function addCattle(sim, streamId) {
    const r = sim.rng;
    const st = sim.streams.find((s) => s.id === streamId) || sim.streams[Math.floor(r() * sim.streams.length)];
    // Pick a mid-block point between two junctions (or before the first one).
    const segs = [];
    let prev = 0;
    for (const sj of st.junctions) { segs.push([prev + 40, sj.stopS - 60]); prev = sj.exitS; }
    segs.push([prev + 40, st.L - 60]);
    const ok = segs.filter(([a, b]) => b - a > 80);
    const [a, b] = ok[Math.floor(r() * ok.length)];
    const hz = {
      id: sim.nextId++, kind: 'cow', stream: st, lane: 1, s: a + (b - a) * (0.3 + 0.4 * r()), half: 9,
      t0: sim.t, until: sim.t + 45 + 20 * r(), detected: false, heading: r() * Math.PI * 2,
    };
    st.hazards.push(hz);
    sim.hazards.push(hz);
    return hz;
  }

  // ---------------------------------------------------------------- pedestrians

  function spawnPed(sim, anywhere) {
    const r = sim.rng;
    const line = sim.pedLines[Math.floor(r() * sim.pedLines.length)];
    const dir = r() < 0.5 ? 1 : -1;
    let s = dir > 0 ? -4 : line.L + 4;
    if (anywhere) {
      for (let k = 0; k < 10; k++) {
        s = r() * line.L;
        if (!line.zones.some((z) => s > z.a - 3 && s < z.b + 3)) break;
      }
    }
    sim.peds.push({
      id: sim.nextId++, kind: 'ped', line, dir, s, speed: (1.05 + 0.5 * r()) * U,
      wait: 0, waiting: null, color: PED_COLORS[Math.floor(r() * PED_COLORS.length)],
      off: (r() - 0.5) * 6, crossing: null,
    });
  }

  function updatePeds(sim) {
    for (let i = sim.peds.length - 1; i >= 0; i--) {
      const p = sim.peds[i];
      const ns = p.s + p.dir * p.speed * DT;
      let blocked = false;
      p.waiting = null;
      p.crossing = null;
      for (const z of p.line.zones) {
        const edge = p.dir > 0 ? z.a : z.b;
        if (p.s > z.a && p.s < z.b) { p.crossing = z; continue; }
        const reaching = p.dir > 0 ? p.s <= edge && ns > edge - 1 : p.s >= edge && ns < edge + 1;
        if (!reaching) continue;
        const go = z.j.phase === z.phase && z.j.state === 'G' &&
          (sim.controller === 'adaptive' || z.j.t < z.j.fixedG[z.phase] - 6);
        if (go) {
          sim.stats.pedWait += p.wait; sim.stats.pedN++;
          if (sim.t >= sim.warmupT) { sim.stats.acc.pedWait += p.wait; sim.stats.acc.pedN++; }
          p.wait = 0;
        } else {
          blocked = true; p.wait += DT; p.waiting = z;
          z.j.app[z.phase].peds += 1;
        }
      }
      if (!blocked) p.s = ns;
      if (p.s < -10 || p.s > p.line.L + 10) { sim.peds.splice(i, 1); spawnPed(sim, false); }
    }
  }

  // ---------------------------------------------------------------- vehicles

  function idmAcc(v, gap, dv) {
    const p = v.p;
    const sStar = p.s0 + Math.max(0, v.v * p.T + (v.v * dv) / (2 * Math.sqrt(p.acc * B_COMF)));
    const g = Math.max(gap, 0.01);
    return p.acc * (1 - Math.pow(v.v / v.vmax, 4) - (sStar / g) * (sStar / g));
  }

  function followAcc(v, leader, st, lane) {
    let a = leader ? idmAcc(v, leader.s - leader.len - v.s, v.v - leader.v)
                   : v.p.acc * (1 - Math.pow(v.v / v.vmax, 4));
    for (const h of st.hazards) {
      if (h.lane !== lane || h.s < v.s - 2) continue;
      a = Math.min(a, idmAcc(v, h.s - h.half - v.s, v.v));
    }
    return a;
  }

  function signalAcc(sim, st, v) {
    const sj = st.junctions[v.nextJ];
    if (!sj) return Infinity;
    const gap = sj.stopS - v.s;
    if (gap > 320) return Infinity;
    const J = sj.j;
    const sig = signalOf(J, sj.phase);
    if (sig === 'G' || v.commit[J.id]) return Infinity;
    if (sig === 'Y') {
      const need = (v.v * v.v) / (2 * B_COMF);
      if (v.violator || gap < need * 0.9 || gap < 3) { v.commit[J.id] = true; return Infinity; }
    } else {
      if (v.violator && sim.t - J.lastGreenEnd[sj.phase] < YELLOW + 2.5 && gap < 40 && v.v > 2 * U) {
        v.commit[J.id] = true; return Infinity;
      }
      if (v.type === 'amb' && v.v < 0.5 && gap < 12) {
        v.lineWait += DT;
        if (v.lineWait > 3) { v.commit[J.id] = true; v.lineWait = 0; return Infinity; }
      }
    }
    return idmAcc(v, gap, v.v);
  }

  function inBox(st, v) {
    for (const sj of st.junctions) if (v.s > sj.stopS - 4 && v.s - v.len < sj.exitS) return true;
    return false;
  }

  function tryLaneChange(sim, st, v, idx) {
    if (inBox(st, v)) return;
    const cur = st.lanes[v.lane], other = 1 - v.lane, tgt = st.lanes[other];
    let lead = null, fol = null, insertAt = tgt.length;
    for (let k = 0; k < tgt.length; k++) {
      if (tgt[k].s >= v.s) lead = tgt[k];
      else { fol = tgt[k]; insertAt = k; break; }
    }
    if (lead && lead.s - lead.len - v.s < v.p.s0 * 0.7) return;
    if (fol && v.s - v.len - fol.s < fol.p.s0 + fol.v * 0.5) return;
    if (st.hazards.some((h) => h.lane === other && Math.abs(h.s - v.s) < h.half + v.len)) return;

    const aCur = followAcc(v, idx > 0 ? cur[idx - 1] : null, st, v.lane);
    const aNew = followAcc(v, lead, st, other);
    let gain = aNew - aCur;
    if (v.p.kerb > 0.5) gain += other === 1 ? 0.25 * U : -0.25 * U;
    if (v.type === 'amb') gain += other === 0 ? 0.3 * U : -0.3 * U;
    // Make way for an ambulance approaching from behind.
    for (let k = idx + 1; k < cur.length && cur[k].s > v.s - 140; k++) if (cur[k].type === 'amb') gain += 3 * U;
    for (let k = insertAt; k < tgt.length && tgt[k].s > v.s - 140; k++) if (tgt[k].type === 'amb') gain -= 3 * U;
    if (fol) {
      const folAcc = idmAcc(fol, v.s - v.len - fol.s, fol.v - v.v);
      if (folAcc < -B_COMF * 1.2) return;
    }
    if (gain < 0.4 * U) return;
    cur.splice(idx, 1);
    tgt.splice(insertAt, 0, v);
    v.lane = other;
  }

  function onCrossStop(sim, v, sj) {
    const sig = signalOf(sj.j, sj.phase);
    if (sig === 'R' && v.type !== 'amb') {
      sim.stats.violations.red++;
      if (!v.flags.includes('Red-light jump')) v.flags.push('Red-light jump');
      log(sim, 'violation', 'bad', `Red-light jump · ${v.p.label} ${v.plate}`, `${sj.j.id} ${sj.j.name} · e-challan draft created`, v.id);
    }
    if (v.noHelmet && !v.flags.includes('No helmet')) {
      v.flags.push('No helmet');
      sim.stats.violations.helmet++;
      log(sim, 'violation', 'warn', `Rider without helmet · ${v.plate}`, `${sj.j.id} camera · confidence ${v.conf.toFixed(2)}`, v.id);
    }
    if (v.triple && !v.flags.includes('Triple riding')) {
      v.flags.push('Triple riding');
      sim.stats.violations.triple++;
      log(sim, 'violation', 'warn', `Triple riding · ${v.plate}`, `${sj.j.id} camera`, v.id);
    }
  }

  function exitVehicle(sim, v) {
    const travel = sim.t - v.t0;
    const ff = v.stream.L / v.vmax;
    const delay = Math.max(0, travel - ff);
    sim.stats.exited++;
    sim.stats.done.push({ t: sim.t, delay, stops: v.stops, type: v.type });
    if (sim.stats.done.length > 4000) sim.stats.done.splice(0, 1000);
    if (v.t0 >= sim.warmupT && v.type !== 'amb') {
      sim.stats.acc.n++; sim.stats.acc.delay += delay; sim.stats.acc.stops += v.stops;
    }
    if (v.type === 'amb') {
      sim.stats.amb.push({ travel, ff, delay, controller: sim.controller });
      log(sim, 'emergency', delay < 10 ? 'ok' : 'warn',
        `${v.callsign} cleared ${v.stream.road.name} in ${travel.toFixed(0)} s`,
        `Free-flow time ${ff.toFixed(0)} s · ${delay.toFixed(0)} s lost in traffic`, v.id);
    }
    sim.vehicles.delete(v.id);
  }

  function updateVehicles(sim) {
    // Reset junction approach sensors.
    for (const j of sim.junctions) {
      for (const a of j.app) { a.q = 0; a.near = 0; a.maxWait = 0; a.spill = 0; }
      j.qByStream = {};
    }

    for (const st of sim.streams) {
      for (let l = 0; l < 2; l++) {
        const lane = st.lanes[l];
        for (let i = 0; i < lane.length; i++) {
          const v = lane[i];
          while (v.nextJ < st.junctions.length && v.s > st.junctions[v.nextJ].stopS + 0.5) v.nextJ++;
          const a = Math.min(followAcc(v, i > 0 ? lane[i - 1] : null, st, l), signalAcc(sim, st, v));
          v.acc = Math.max(-B_MAX, a);
        }
      }
    }

    for (const st of sim.streams) {
      for (let l = 0; l < 2; l++) {
        const lane = st.lanes[l];
        for (let i = 0; i < lane.length; i++) {
          const v = lane[i];
          const prevS = v.s;
          v.v = Math.max(0, v.v + v.acc * DT);
          v.s += v.v * DT;
          const leader = i > 0 ? lane[i - 1] : null;
          if (leader) {
            const maxS = leader.s - leader.len - 0.3;
            if (v.s > maxS) { v.s = Math.max(prevS, maxS); v.v = Math.min(v.v, leader.v); }
          }
          const sj = st.junctions[v.nextJ];
          if (sj && prevS <= sj.stopS && v.s > sj.stopS) onCrossStop(sim, v, sj);

          // Stop / idle bookkeeping.
          if (v.v < 0.25 * U) {
            if (v.moving) { v.stops++; v.moving = false; }
            v.stopped += DT;
            sim.stats.stoppedVS += DT;
            const kg = (v.p.idle / 3600) * DT * CO2_PER_L;
            sim.stats.co2 += kg;
            if (sim.t >= sim.warmupT) sim.stats.acc.co2 += kg;
          } else if (v.v > 2 * U) {
            v.moving = true;
          }
          v.latVis += (laneLat(v.lane) - v.latVis) * Math.min(1, DT * 3);

          // Feed junction sensors: what a camera at the stop line would see.
          if (sj) {
            const dist = sj.stopS - v.s;
            if (dist < DETECT && dist > -2) {
              const app = sj.j.app[sj.phase];
              app.q += v.p.pcu;
              if (dist < 60 || (dist < 160 && v.v > 1 * U)) app.near += v.p.pcu;
              if (v.v < 0.5 * U) app.maxWait = Math.max(app.maxWait, v.stopped);
              if (v.v < 1 * U) sj.j.qByStream[st.id] = (sj.j.qByStream[st.id] || 0) + v.p.pcu;
            }
          }
          const back = st.junctions[v.nextJ - 1];
          if (back && v.s - back.exitS < 140 && v.v < 1 * U) back.j.app[back.phase].spill += v.p.pcu;
        }
      }
    }

    // Lane changes, staggered.
    for (const st of sim.streams) {
      for (let l = 0; l < 2; l++) {
        const lane = st.lanes[l];
        for (let i = lane.length - 1; i >= 0; i--) {
          const v = lane[i];
          if (!v || v.lane !== l) continue;
          v.lcTimer -= DT;
          if (v.lcTimer > 0) continue;
          v.lcTimer = 0.5;
          tryLaneChange(sim, st, v, i);
        }
      }
    }

    // Exits.
    for (const st of sim.streams) {
      for (const lane of st.lanes) {
        while (lane.length && lane[0].s - lane[0].len > st.L) exitVehicle(sim, lane.shift());
      }
    }
  }

  // ---------------------------------------------------------------- signals

  function pressure(j, ph) {
    const a = j.app[ph];
    return a.q + 0.08 * a.maxWait + 0.25 * a.peds - 0.6 * a.spill;
  }

  function corridor(sim) {
    for (const j of sim.junctions) j.preempt = null;
    if (sim.controller !== 'adaptive') return;
    for (const v of sim.vehicles.values()) {
      if (v.type !== 'amb') continue;
      // Clear every junction ahead within reach, so queues discharge before the ambulance arrives.
      for (let k = v.nextJ; k < v.stream.junctions.length; k++) {
        const sj = v.stream.junctions[k];
        if (sj.stopS - v.s > PREEMPT) break;
        if (sj.j.preempt != null && sj.j.preempt !== sj.phase) continue;
        sj.j.preempt = sj.phase;
        if (!v.commit['pre' + sj.j.id]) {
          v.commit['pre' + sj.j.id] = true;
          log(sim, 'emergency', 'ok', `Green corridor · ${sj.j.id} gives ${AXIS[sj.phase]} green to ${v.callsign}`,
            `${sj.j.name} · queue flushed ahead of arrival`, v.id);
        }
      }
    }
  }

  function adaptiveDecision(j) {
    const cur = j.phase, oth = 1 - cur;
    const pc = j.pressure[cur], po = j.pressure[oth];
    if (j.preempt != null) {
      const go = j.preempt !== cur && j.t >= 3;
      j.decision = go ? 'Emergency pre-emption · switching' : `Emergency pre-emption · holding ${AXIS[cur]}`;
      return go;
    }
    if (j.t < TUNE.minG) { j.decision = `Minimum green · ${AXIS[cur]}`; return false; }
    if (j.t >= TUNE.maxG && j.app[oth].q > 0) { j.decision = 'Maximum green reached'; return true; }
    if (j.app[cur].spill > TUNE.spill && j.app[oth].q > 1) { j.decision = `Exit blocked downstream · releasing ${AXIS[oth]}`; return true; }
    j.gapT = j.app[cur].near < TUNE.gapNear ? (j.gapT || 0) + DT : 0;
    if (j.gapT >= TUNE.gapHold && j.app[cur].near < TUNE.gapNear && j.app[oth].q > 0.5) { j.decision = `Gap-out · ${AXIS[cur]} approach empty`; return true; }
    if (j.t >= TUNE.minSwitch && po > pc * TUNE.gain + TUNE.bias) {
      j.decision = `Switching · demand ${po.toFixed(1)} vs ${pc.toFixed(1)} PCU`;
      return true;
    }
    j.decision = `Holding ${AXIS[cur]} · demand ${pc.toFixed(1)} vs ${po.toFixed(1)} PCU`;
    return false;
  }

  function updateSignals(sim) {
    corridor(sim);
    for (const j of sim.junctions) {
      j.t += DT;
      j.pressure[0] = pressure(j, 0);
      j.pressure[1] = pressure(j, 1);
      if (j.state === 'Y') { if (j.t >= YELLOW) { j.state = 'AR'; j.t = 0; } continue; }
      if (j.state === 'AR') {
        if (j.t >= ALL_RED) { j.phase = j.nextPhase; j.state = 'G'; j.t = 0; j.switches++; }
        continue;
      }
      let go;
      if (sim.controller === 'fixed') {
        go = j.t >= j.fixedG[j.phase];
        j.decision = `Fixed plan · ${j.fixedG[0]} s / ${j.fixedG[1]} s, no sensing`;
      } else {
        go = adaptiveDecision(j);
      }
      if (go) {
        j.lastGreenEnd[j.phase] = sim.t;
        j.state = 'Y'; j.t = 0; j.nextPhase = 1 - j.phase;
      }
    }
  }

  // ---------------------------------------------------------------- incidents

  function updateHazards(sim) {
    for (let i = sim.hazards.length - 1; i >= 0; i--) {
      const h = sim.hazards[i];
      if (!h.detected && sim.t - h.t0 > 1.4) {
        h.detected = true;
        log(sim, 'incident', 'warn', `Stray cattle on ${h.stream.road.name} (${h.stream.compass}, kerb lane)`,
          'Detected in 1.4 s · alert sent to municipal cattle squad', h.id);
      }
      if (sim.t > h.until) {
        h.stream.hazards.splice(h.stream.hazards.indexOf(h), 1);
        sim.hazards.splice(i, 1);
        log(sim, 'incident', 'ok', `Carriageway clear on ${h.stream.road.name}`, `Cattle moved off after ${(h.until - h.t0).toFixed(0)} s`, h.id);
      }
    }
    if (sim.autoIncidents && sim.t > sim.nextIncidentT) {
      sim.nextIncidentT = sim.t + 180 + sim.rng() * 120;
      if (sim.hazards.length < 2) addCattle(sim);
    }
  }

  // ---------------------------------------------------------------- stepping & metrics

  function step(sim) {
    sim.t += DT;
    spawnVehicles(sim);
    updateSignals(sim);
    for (const j of sim.junctions) { j.app[0].peds = 0; j.app[1].peds = 0; }
    updatePeds(sim);
    updateVehicles(sim);
    updateHazards(sim);
    const samples = sim.stats.samples;
    if (!samples.length || sim.t - samples[samples.length - 1].t >= 1) {
      samples.push({ t: sim.t, co2: sim.stats.co2, exited: sim.stats.exited });
      if (samples.length > 1300) samples.splice(0, 300);
    }
  }

  function advance(sim, seconds) {
    const n = Math.round(seconds / DT);
    for (let i = 0; i < n; i++) step(sim);
  }

  function metrics(sim, win = 300) {
    const t0 = sim.t - win;
    let n = 0, delay = 0, stops = 0;
    for (const d of sim.stats.done) if (d.t >= t0 && d.type !== 'amb') { n++; delay += d.delay; stops += d.stops; }
    let queued = 0, moving = 0;
    for (const v of sim.vehicles.values()) { if (v.v < 1 * U) queued += v.p.pcu; else moving++; }
    const samples = sim.stats.samples;
    let co2Rate = 0;
    if (samples.length > 2) {
      const last = samples[samples.length - 1];
      const first = samples.find((s) => s.t >= t0) || samples[0];
      if (last.t > first.t) co2Rate = ((last.co2 - first.co2) / (last.t - first.t)) * 3600;
    }
    const span = Math.min(win, Math.max(1, sim.t));
    return {
      avgDelay: n ? delay / n : 0, throughput: (n / span) * 60, avgStops: n ? stops / n : 0,
      queued, moving, onRoad: sim.vehicles.size, co2Rate, violations: sim.stats.violations,
      pedWait: sim.stats.pedN ? sim.stats.pedWait / sim.stats.pedN : 0,
    };
  }

  /** Headless run used by the benchmark: same seed, same demand, one controller. */
  function runScenario(opts) {
    const warm = opts.warmup ?? 180;
    const dur = opts.duration ?? 900;
    const sim = createSim({ ...opts, warmupT: warm });
    const ambs = opts.ambulances ?? [
      [warm + 60, 'mg-EB'], [warm + 200, 'nehru-NB'], [warm + 340, 'stn-WB'],
      [warm + 480, 'tilak-SB'], [warm + 620, 'mg-WB'], [warm + 760, 'nehru-SB'],
    ];
    let k = 0;
    while (sim.t < warm + dur) {
      while (k < ambs.length && sim.t >= ambs[k][0]) dispatchAmbulance(sim, ambs[k++][1]);
      step(sim);
    }
    // Let ambulances still on the road finish (traffic keeps flowing).
    let guard = 0;
    while ([...sim.vehicles.values()].some((v) => v.type === 'amb') && guard++ < 4000) step(sim);
    const a = sim.stats.acc;
    const amb = sim.stats.amb;
    return {
      controller: sim.controller, seed: opts.seed,
      vehicles: a.n,
      avgDelay: a.n ? a.delay / a.n : 0,
      avgStops: a.n ? a.stops / a.n : 0,
      throughput: a.n / (dur / 60),
      co2PerHour: (a.co2 / dur) * 3600,
      pedWait: a.pedN ? a.pedWait / a.pedN : 0,
      ambDelay: amb.length ? amb.reduce((s, x) => s + x.delay, 0) / amb.length : 0,
      ambTravel: amb.length ? amb.reduce((s, x) => s + x.travel, 0) / amb.length : 0,
      redLight: sim.stats.violations.red,
      helmet: sim.stats.violations.helmet,
      left: sim.streams.reduce((s, st) => s + st.pending.length, 0),
    };
  }

  function vehiclePose(v) {
    const st = v.stream;
    const sc = v.s - v.len / 2;
    const lat = v.latVis + v.jit;
    return {
      x: st.start.x + st.hx * sc + st.lx * lat,
      y: st.start.y + st.hy * sc + st.ly * lat,
      ang: Math.atan2(st.hy, st.hx),
    };
  }

  function pedPose(p) {
    return p.line.axis === 'h' ? { x: p.s, y: p.line.c + p.off } : { x: p.line.c + p.off, y: p.s };
  }

  function hazardPose(h, t) {
    const st = h.stream;
    const k = Math.min(1, (t - h.t0) / 3);
    const ease = k * k * (3 - 2 * k);
    const lat = PED_OFF + (laneLat(1) - PED_OFF) * ease;
    return { x: st.start.x + st.hx * h.s + st.lx * lat, y: st.start.y + st.hy * h.s + st.ly * lat };
  }

  return {
    U, DT, WORLD, LANE_W, MEDIAN, HALF_ROAD, CROSSWALK, SIDEWALK, PED_OFF, DETECT, YELLOW, ALL_RED, TUNE,
    ROADS, TYPES, createSim, step, advance, metrics, runScenario, dispatchAmbulance, addCattle,
    vehiclePose, pedPose, hazardPose, laneLat, signalOf, mulberry32,
  };
});
