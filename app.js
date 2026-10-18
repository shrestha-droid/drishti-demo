/*
 * Drishti "Machine sight" demo: renders the simulation as a clay-model city and
 * a machine-vision lens (sensor points, semantic segments, flow), and drives
 * the control panel, live twin comparison and benchmark.
 */
(() => {
  'use strict';

  const S = window.DrishtiSim;
  const { WORLD, HALF_ROAD, SIDEWALK, CROSSWALK, LANE_W, MEDIAN, U, ROADS, TYPES, DETECT } = S;
  const $ = (q) => document.querySelector(q);

  const stage = $('#stage');
  const canvas = $('#view');
  const ctx = canvas.getContext('2d');
  const mcan = document.createElement('canvas');
  const mctx = mcan.getContext('2d');
  const reduceMotion = matchMedia('(prefers-reduced-motion: reduce)').matches;

  const OX = 0.22, OY = -0.42;          // oblique projection per unit of height
  const SEED = 2026;
  const START = 8 * 3600 + 30 * 60;     // 08:30 IST
  const FONT_MONO = '"IBM Plex Mono", ui-monospace, Menlo, monospace';

  // Machine-view class colours.
  const SENSE = { two: '#ff8a5c', auto: '#ffd447', car: '#5ee6ee', bus: '#8aa6ff', truck: '#c09bff', cycle: '#a4e86a', amb: '#ff4d6d', ped: '#ff7aa2', cow: '#f0b27a' };
  // Cityscapes / IDD-inspired semantic palette.
  const SEG = { road: '#6e3b74', walk: '#c44aa9', bldg: '#48525b', veg: '#6b8e23', lot: '#2b2f24', mark: '#e7d6ef', two: '#ff2a2a', auto: '#ffcc36', car: '#2f54ff', bus: '#00a0c8', truck: '#6a3dd0', cycle: '#b5123a', amb: '#ffffff', ped: '#ff3b6b', cow: '#c08a5b' };
  const SIG = { G: '#2bd576', Y: '#ffb020', R: '#ff4a3d' };
  const CAR_TINTS = ['#fbfbf9', '#eef0ee', '#dfe3e2', '#f3efe6', '#d6dbe1', '#e9ecef'];
  const BUS_STRIPES = ['#c8423a', '#2f7d6d', '#3a62b0'];
  const TARPS = ['#d39a3a', '#3f6fa8', '#c2582f'];
  const ROUTES = [
    ['mg-EB', 'MG Road, eastbound'], ['mg-WB', 'MG Road, westbound'],
    ['stn-EB', 'Station Road, eastbound'], ['stn-WB', 'Station Road, westbound'],
    ['nehru-SB', 'Nehru Marg, southbound'], ['nehru-NB', 'Nehru Marg, northbound'],
    ['tilak-SB', 'Tilak Road, southbound'], ['tilak-NB', 'Tilak Road, northbound'],
  ];

  const app = {
    sim: null, twin: null, speed: 1, paused: false, mode: 'lidar', rush: false, demand: 1,
    w: 0, h: 0, dpr: 1, cam: { x: 760, y: 470, base: 1, zoom: 1.2, s: 1, tx: null, ty: null },
    lens: { x: 470, y: 310, R: 170, sx: 0, sy: 0 },
    ptr: { inside: false, down: false, moved: false, sx: 0, sy: 0, x0: 0, y0: 0, cx0: 0, cy0: 0, id: null },
    touch: matchMedia('(pointer: coarse)').matches, manualUntil: 0,
    selected: null, hover: null, ents: [], pal: {}, layers: {}, res: 0, city: null,
    tour: { i: 0, next: 0 }, hist: [], lastHist: -99, nextAmbT: 0, nextCowT: 0,
    feedSeq: 0, lastUi: 0, cardTick: 0,
  };

  // ------------------------------------------------------------------ utils

  const clamp = (x, a, b) => Math.max(a, Math.min(b, x));
  const kmh = (v) => (v / U) * 3.6;
  function hash3(a, b, c) {
    let h = Math.imul(a | 0, 374761393) ^ Math.imul(b | 0, 668265263) ^ Math.imul(c | 0, 1274126177);
    h = Math.imul(h ^ (h >>> 13), 1274126177);
    return ((h ^ (h >>> 16)) >>> 0) / 4294967296;
  }
  function rr(g, x, y, w, h, r) {
    g.beginPath();
    if (g.roundRect) g.roundRect(x, y, w, h, r);
    else g.rect(x, y, w, h);
  }
  function clockText(t, secs = true) {
    const s = Math.floor(START + t);
    const hh = String(Math.floor(s / 3600) % 24).padStart(2, '0');
    const mm = String(Math.floor(s / 60) % 60).padStart(2, '0');
    const ss = String(s % 60).padStart(2, '0');
    return secs ? `${hh}:${mm}:${ss}` : `${hh}:${mm}`;
  }
  function hexA(hex, a) {
    const n = parseInt(hex.replace('#', '').trim().slice(0, 6), 16);
    return `rgba(${(n >> 16) & 255},${(n >> 8) & 255},${n & 255},${a})`;
  }
  const w2s = (x, y) => [(x - app.cam.x) * app.cam.s + app.w / 2, (y - app.cam.y) * app.cam.s + app.h / 2];
  const s2w = (sx, sy) => [(sx - app.w / 2) / app.cam.s + app.cam.x, (sy - app.h / 2) / app.cam.s + app.cam.y];
  const both = (fn) => { fn(app.sim); fn(app.twin); };
  const idTag = (e) => `${e.kind === 'ped' ? 'PED' : e.kind === 'cow' ? 'COW' : TYPES[e.ref.type].tag} ${String(e.ref.id % 100).padStart(2, '0')}`;

  function readPalette() {
    const cs = getComputedStyle(document.documentElement);
    const v = (n) => cs.getPropertyValue(n).trim();
    const dark = (cs.colorScheme || '').includes('dark') || v('--paper').toLowerCase() === '#0b1116';
    app.pal = {
      dark, ground: v('--city-ground'), lot: v('--city-lot'), walk: v('--city-walk'), road: v('--city-road'),
      mark: v('--city-mark'), roof: v('--city-roof'), wallS: v('--city-wall-s'), wallW: v('--city-wall-w'),
      shadow: v('--city-shadow'), tree: v('--city-tree'), treeHi: v('--city-tree-hi'), park: v('--city-park'),
      label: v('--city-label'), ink: v('--ink'), muted: v('--muted'), accent: v('--accent'), paper: v('--paper'),
      go: v('--go'), stop: v('--stop'), amber: v('--amber'), emergency: v('--emergency'), raised: v('--raised'),
    };
  }

  // ------------------------------------------------------------------ city model

  function genCity() {
    const r = S.mulberry32(99);
    const vx = ROADS.filter((q) => q.axis === 'v').map((q) => q.c);
    const hy = ROADS.filter((q) => q.axis === 'h').map((q) => q.c);
    const SET = HALF_ROAD + SIDEWALK + 8;
    const xb = [[-60, vx[0] - SET], [vx[0] + SET, vx[1] - SET], [vx[1] + SET, WORLD.w + 60]];
    const yb = [[-60, hy[0] - SET], [hy[0] + SET, hy[1] - SET], [hy[1] + SET, WORLD.h + 60]];
    const blocks = [], buildings = [], trees = [];
    let park = null;
    for (let i = 0; i < 3; i++) {
      for (let j = 0; j < 3; j++) {
        const [x0, x1] = xb[i], [y0, y1] = yb[j];
        blocks.push({ x0, y0, x1, y1 });
        if (i === 1 && j === 1) {
          park = { x0: x0 + 4, y0: y0 + 4, x1: x1 - 4, y1: y1 - 4 };
          const cx = (x0 + x1) / 2, cy = (y0 + y1) / 2;
          for (let x = park.x0 + 14; x < park.x1 - 8; x += 24 + r() * 6) {
            trees.push({ x, y: park.y0 + 12 + r() * 4, r: 8 + r() * 3 });
            trees.push({ x, y: park.y1 - 12 - r() * 4, r: 8 + r() * 3 });
          }
          for (let y = park.y0 + 36; y < park.y1 - 30; y += 24 + r() * 6) {
            trees.push({ x: park.x0 + 12 + r() * 4, y, r: 8 + r() * 3 });
            trees.push({ x: park.x1 - 12 - r() * 4, y, r: 8 + r() * 3 });
          }
          for (let k = 0; k < 9; k++) trees.push({ x: cx + 90 + r() * 90, y: cy - 60 + r() * 120, r: 7 + r() * 4 });
          buildings.push({ x: cx - 70, y: cy - 16, w: 34, h: 32, z: 9, tanks: [], pavilion: true });
          continue;
        }
        const fx0 = Math.max(x0, -20), fx1 = Math.min(x1, WORLD.w + 20);
        const fy0 = Math.max(y0, -20), fy1 = Math.min(y1, WORLD.h + 20);
        const nearMG = Math.abs(y1 - (hy[0] - SET)) < 1 || Math.abs(y0 - (hy[0] + SET)) < 1;
        const rows = fy1 - fy0 > 150 ? 2 : 1;
        const rowH = (fy1 - fy0) / rows;
        for (let row = 0; row < rows; row++) {
          let x = fx0;
          while (x < fx1 - 30) {
            const w = Math.min(fx1 - x, 52 + r() * 56);
            const ly0 = fy0 + row * rowH;
            const ins = 4 + r() * 5;
            if (r() < 0.12) {
              for (let k = 0; k < 3; k++) trees.push({ x: x + 10 + r() * (w - 20), y: ly0 + 14 + r() * (rowH - 28), r: 7 + r() * 4 });
            } else {
              let z = 12 + r() * 30 + (nearMG ? 10 : 0);
              if (r() < 0.12) z += 16;
              z = Math.min(z, 58);
              const b = { x: x + ins, y: ly0 + ins, w: w - ins * 2, h: rowH - ins * 2, z, tanks: [] };
              const nt = r() < 0.75 ? 1 + Math.floor(r() * 2) : 0;
              for (let k = 0; k < nt; k++) b.tanks.push({ u: 0.2 + r() * 0.6, v: 0.2 + r() * 0.6 });
              if (r() < 0.5) b.cabin = { u: 0.15 + r() * 0.5, v: 0.15 + r() * 0.5 };
              buildings.push(b);
            }
            x += w;
          }
        }
      }
    }
    // Street trees along the outer edge of each sidewalk.
    for (const road of ROADS) {
      const perp = ROADS.filter((q) => q.axis !== road.axis).map((q) => q.c);
      const L = road.axis === 'h' ? WORLD.w : WORLD.h;
      for (const side of [-1, 1]) {
        for (let s = 18 + r() * 20; s < L; s += 40 + r() * 22) {
          if (perp.some((c) => Math.abs(s - c) < 78)) continue;
          if (r() < 0.25) continue;
          const lat = road.c + side * (HALF_ROAD + SIDEWALK - 3);
          trees.push(road.axis === 'h' ? { x: s, y: lat, r: 6 + r() * 3, street: true } : { x: lat, y: s, r: 6 + r() * 3, street: true });
        }
      }
    }
    buildings.sort((a, b) => a.y + a.h - (b.y + b.h));
    trees.sort((a, b) => a.y - b.y);
    return { blocks, buildings, trees, park };
  }

  // ------------------------------------------------------------------ static layers

  function makeLayer(res) {
    const c = document.createElement('canvas');
    c.width = Math.ceil(WORLD.w * res);
    c.height = Math.ceil(WORLD.h * res);
    const g = c.getContext('2d');
    g.setTransform(res, 0, 0, res, 0, 0);
    return { c, g };
  }

  function roadSpans(road) {
    // Lengths along a road that are not inside a junction or its crosswalks.
    const perp = ROADS.filter((q) => q.axis !== road.axis).map((q) => q.c).sort((a, b) => a - b);
    const L = road.axis === 'h' ? WORLD.w : WORLD.h;
    const spans = [];
    let a = 0;
    for (const c of perp) { spans.push([a, c - HALF_ROAD - CROSSWALK - 4]); a = c + HALF_ROAD + CROSSWALK + 4; }
    spans.push([a, L]);
    return spans;
  }

  function along(road, s, lat) { return road.axis === 'h' ? [s, road.c + lat] : [road.c + lat, s]; }

  function lineAlong(g, road, a, b, lat) {
    const [x0, y0] = along(road, a, lat), [x1, y1] = along(road, b, lat);
    g.beginPath(); g.moveTo(x0, y0); g.lineTo(x1, y1); g.stroke();
  }

  function stripRect(g, road, lat0, lat1, s0 = -10, s1 = null) {
    const L = road.axis === 'h' ? WORLD.w : WORLD.h;
    if (s1 == null) s1 = L + 10;
    if (road.axis === 'h') g.fillRect(s0, road.c + lat0, s1 - s0, lat1 - lat0);
    else g.fillRect(road.c + lat0, s0, lat1 - lat0, s1 - s0);
  }

  function buildGround(res) {
    const { g, c } = makeLayer(res);
    const P = app.pal, city = app.city;
    g.fillStyle = P.ground; g.fillRect(-10, -10, WORLD.w + 20, WORLD.h + 20);
    g.fillStyle = P.lot;
    for (const b of city.blocks) g.fillRect(b.x0, b.y0, b.x1 - b.x0, b.y1 - b.y0);
    // park
    const pk = city.park;
    g.fillStyle = P.park; rr(g, pk.x0, pk.y0, pk.x1 - pk.x0, pk.y1 - pk.y0, 10); g.fill();
    g.strokeStyle = P.walk; g.lineWidth = 5;
    rr(g, pk.x0 + 26, pk.y0 + 26, pk.x1 - pk.x0 - 52, pk.y1 - pk.y0 - 52, 30); g.stroke();
    g.beginPath(); g.moveTo((pk.x0 + pk.x1) / 2, pk.y0 + 26); g.lineTo((pk.x0 + pk.x1) / 2, pk.y1 - 26); g.stroke();
    // sidewalks, then carriageways
    for (const road of ROADS) { g.fillStyle = P.walk; stripRect(g, road, -HALF_ROAD - SIDEWALK, HALF_ROAD + SIDEWALK); }
    for (const road of ROADS) { g.fillStyle = P.road; stripRect(g, road, -HALF_ROAD, HALF_ROAD); }
    // kerbs
    g.strokeStyle = hexA(P.shadow, P.dark ? 0.5 : 0.14); g.lineWidth = 0.8;
    for (const road of ROADS) for (const [a, b] of roadSpans(road)) for (const side of [-1, 1]) lineAlong(g, road, a + 4, b - 4, side * HALF_ROAD);

    // lane markings
    g.strokeStyle = P.mark;
    for (const road of ROADS) {
      for (const [a, b] of roadSpans(road)) {
        g.setLineDash([]); g.lineWidth = 0.7;
        lineAlong(g, road, a, b, -1.3); lineAlong(g, road, a, b, 1.3);
        g.lineWidth = 0.6; g.globalAlpha = 0.8;
        lineAlong(g, road, a, b, -(HALF_ROAD - 1.6)); lineAlong(g, road, a, b, HALF_ROAD - 1.6);
        g.globalAlpha = 1; g.setLineDash([7, 8]); g.lineWidth = 0.8;
        lineAlong(g, road, a, b, -(MEDIAN / 2 + LANE_W)); lineAlong(g, road, a, b, MEDIAN / 2 + LANE_W);
      }
    }
    g.setLineDash([]);

    // zebra crossings and stop lines
    g.fillStyle = P.mark;
    for (const j of app.sim.junctions) {
      for (const arm of [[0, -1], [0, 1], [-1, 0], [1, 0]]) {
        const [ax, ay] = arm;
        for (let k = -HALF_ROAD + 2; k < HALF_ROAD - 1; k += 4.6) {
          if (Math.abs(k) < 1.5) continue;
          if (ax === 0) g.fillRect(j.x + k, j.y + ay * 37 - 6, 2.3, 12);
          else g.fillRect(j.x + ax * 37 - 6, j.y + k, 12, 2.3);
        }
      }
    }
    for (const st of app.sim.streams) {
      for (const sj of st.junctions) {
        const s = sj.stopS + 1;
        const p0 = [st.start.x + st.hx * s + st.lx * (MEDIAN / 2), st.start.y + st.hy * s + st.ly * (MEDIAN / 2)];
        const p1 = [st.start.x + st.hx * s + st.lx * HALF_ROAD, st.start.y + st.hy * s + st.ly * HALF_ROAD];
        g.strokeStyle = P.mark; g.lineWidth = 1.8;
        g.beginPath(); g.moveTo(p0[0], p0[1]); g.lineTo(p1[0], p1[1]); g.stroke();
        // straight-ahead arrows painted in each lane
        for (const lane of [0, 1]) {
          const lat = S.laneLat(lane), sa = sj.stopS - 34;
          const bx = st.start.x + st.hx * sa + st.lx * lat, by = st.start.y + st.hy * sa + st.ly * lat;
          g.save(); g.translate(bx, by); g.rotate(Math.atan2(st.hy, st.hx));
          g.fillStyle = P.mark; g.globalAlpha = 0.85;
          g.fillRect(-9, -0.7, 9, 1.4);
          g.beginPath(); g.moveTo(0, -2.6); g.lineTo(4.5, 0); g.lineTo(0, 2.6); g.closePath(); g.fill();
          g.restore();
        }
      }
    }

    // shadows: drawn opaque on a scratch layer, then composited once so overlaps don't double up
    const sh = makeLayer(res);
    sh.g.fillStyle = '#000';
    for (const b of city.buildings) {
      const sx = b.z * 0.55, sy = b.z * 0.34;
      sh.g.beginPath();
      sh.g.moveTo(b.x, b.y); sh.g.lineTo(b.x + b.w, b.y); sh.g.lineTo(b.x + b.w + sx, b.y + sy);
      sh.g.lineTo(b.x + b.w + sx, b.y + b.h + sy); sh.g.lineTo(b.x + sx, b.y + b.h + sy); sh.g.lineTo(b.x, b.y + b.h);
      sh.g.closePath(); sh.g.fill();
    }
    for (const t of city.trees) { sh.g.beginPath(); sh.g.arc(t.x + 5, t.y + 3.5, t.r * 0.95, 0, Math.PI * 2); sh.g.fill(); }
    g.save(); g.globalAlpha = P.dark ? 0.42 : 0.12;
    g.drawImage(sh.c, 0, 0, WORLD.w, WORLD.h);
    g.restore();
    return c;
  }

  function drawBuilding(g, b, P) {
    const dx = b.z * OX, dy = b.z * OY;
    const { x, y, w, h } = b;
    g.fillStyle = P.wallS;
    g.beginPath(); g.moveTo(x, y + h); g.lineTo(x + w, y + h); g.lineTo(x + w + dx, y + h + dy); g.lineTo(x + dx, y + h + dy); g.closePath(); g.fill();
    g.fillStyle = P.wallW;
    g.beginPath(); g.moveTo(x, y); g.lineTo(x, y + h); g.lineTo(x + dx, y + h + dy); g.lineTo(x + dx, y + dy); g.closePath(); g.fill();
    // floor lines on the street-facing wall
    g.strokeStyle = hexA(P.shadow, P.dark ? 0.35 : 0.07); g.lineWidth = 0.5;
    for (let f = 10; f < b.z - 2; f += 10) {
      const fx = f * OX, fy = f * OY;
      g.beginPath(); g.moveTo(x + fx, y + h + fy); g.lineTo(x + w + fx, y + h + fy); g.stroke();
    }
    g.fillStyle = P.roof; g.fillRect(x + dx, y + dy, w, h);
    g.strokeStyle = hexA(P.shadow, P.dark ? 0.6 : 0.1); g.lineWidth = 0.6;
    g.strokeRect(x + dx + 1.6, y + dy + 1.6, w - 3.2, h - 3.2);
    if (b.cabin) {
      const cx = x + dx + b.cabin.u * (w - 10), cy = y + dy + b.cabin.v * (h - 8);
      g.fillStyle = P.wallS; g.fillRect(cx + 1.2, cy + 1.8, 9, 7);
      g.fillStyle = P.roof; g.fillRect(cx, cy, 9, 7);
      g.strokeRect(cx, cy, 9, 7);
    }
    for (const t of b.tanks) {
      const tx = x + dx + 4 + t.u * (w - 8), ty = y + dy + 4 + t.v * (h - 8);
      g.fillStyle = hexA('#000000', P.dark ? 0.4 : 0.12);
      g.beginPath(); g.arc(tx + 1.2, ty + 0.9, 2.2, 0, Math.PI * 2); g.fill();
      g.fillStyle = P.dark ? '#1c2328' : '#59636a';
      g.beginPath(); g.arc(tx, ty, 2.1, 0, Math.PI * 2); g.fill();
      g.fillStyle = P.dark ? '#3a454d' : '#8a949a';
      g.beginPath(); g.arc(tx - 0.5, ty - 0.5, 0.9, 0, Math.PI * 2); g.fill();
    }
    if (b.pavilion) {
      g.strokeStyle = hexA(P.shadow, 0.25);
      g.beginPath(); g.moveTo(x + dx, y + dy); g.lineTo(x + dx + w, y + dy + h); g.moveTo(x + dx + w, y + dy); g.lineTo(x + dx, y + dy + h); g.stroke();
    }
  }

  function buildTop(res) {
    const { g, c } = makeLayer(res);
    const P = app.pal;
    for (const b of app.city.buildings) drawBuilding(g, b, P);
    for (const t of app.city.trees) {
      const z = t.street ? 7 : 9;
      const x = t.x + z * OX, y = t.y + z * OY;
      g.fillStyle = P.tree; g.beginPath(); g.arc(x, y, t.r, 0, Math.PI * 2); g.fill();
      g.fillStyle = P.treeHi; g.beginPath(); g.arc(x - t.r * 0.28, y - t.r * 0.3, t.r * 0.55, 0, Math.PI * 2); g.fill();
      g.strokeStyle = hexA(P.shadow, P.dark ? 0.5 : 0.08); g.lineWidth = 0.6;
      g.beginPath(); g.arc(x, y, t.r, 0, Math.PI * 2); g.stroke();
    }
    return c;
  }

  function insideBuilding(x, y) {
    for (const b of app.city.buildings) if (x > b.x && x < b.x + b.w && y > b.y && y < b.y + b.h) return true;
    return false;
  }

  function buildLidar(res) {
    const { g, c } = makeLayer(res);
    const r = S.mulberry32(5);
    g.fillStyle = '#06131c'; g.fillRect(0, 0, WORLD.w, WORLD.h);
    g.fillStyle = '#0b1e2a';
    for (const road of ROADS) stripRect(g, road, -HALF_ROAD - SIDEWALK, HALF_ROAD + SIDEWALK);
    g.fillStyle = '#0f2735';
    for (const road of ROADS) stripRect(g, road, -HALF_ROAD, HALF_ROAD);
    // lane paint returns strongly to lidar: draw as bright dotted lines
    g.fillStyle = 'rgba(94,230,238,.55)';
    for (const road of ROADS) {
      for (const [a, b] of roadSpans(road)) {
        for (const lat of [-(MEDIAN / 2 + LANE_W), MEDIAN / 2 + LANE_W, -1.3, 1.3, -HALF_ROAD, HALF_ROAD]) {
          for (let s = a; s < b; s += 3.2) { const [x, y] = along(road, s, lat); g.fillRect(x - 0.6, y - 0.6, 1.2, 1.2); }
        }
      }
    }
    // concentric scan rings from each junction-mounted sensor, occluded by buildings
    for (const j of app.sim.junctions) {
      for (let rad = 22; rad < 460; rad += 11) {
        const n = Math.floor((2 * Math.PI * rad) / 4.6);
        const a = Math.max(0.1, 0.8 * (1 - rad / 500));
        g.fillStyle = `rgba(94,230,238,${a.toFixed(3)})`;
        for (let k = 0; k < n; k++) {
          const th = (k / n) * Math.PI * 2 + rad * 0.37;
          const rr2 = rad + (r() - 0.5) * 1.4;
          const x = j.x + Math.cos(th) * rr2, y = j.y + Math.sin(th) * rr2;
          if (x < 0 || y < 0 || x > WORLD.w || y > WORLD.h || insideBuilding(x, y)) continue;
          g.fillRect(x - 0.7, y - 0.7, 1.4, 1.4);
        }
      }
    }
    // building returns: edges and a sparse roof scatter
    for (const b of app.city.buildings) {
      g.fillStyle = '#10283a'; g.fillRect(b.x, b.y, b.w, b.h);
      g.fillStyle = 'rgba(150,240,246,.8)';
      const per = 2 * (b.w + b.h);
      for (let k = 0; k < per; k += 2.6) {
        let x, y;
        if (k < b.w) { x = b.x + k; y = b.y; }
        else if (k < b.w + b.h) { x = b.x + b.w; y = b.y + k - b.w; }
        else if (k < 2 * b.w + b.h) { x = b.x + b.w - (k - b.w - b.h); y = b.y + b.h; }
        else { x = b.x; y = b.y + b.h - (k - 2 * b.w - b.h); }
        g.fillRect(x - 0.65, y - 0.65, 1.3, 1.3);
      }
      g.fillStyle = `rgba(94,230,238,${(0.16 + b.z / 250).toFixed(3)})`;
      const n = Math.floor((b.w * b.h) / 26);
      for (let k = 0; k < n; k++) g.fillRect(b.x + r() * b.w, b.y + r() * b.h, 1.2, 1.2);
    }
    for (const t of app.city.trees) {
      g.fillStyle = 'rgba(164,232,106,.28)';
      for (let k = 0; k < 18; k++) { const a = r() * Math.PI * 2, d = Math.sqrt(r()) * t.r; g.fillRect(t.x + Math.cos(a) * d, t.y + Math.sin(a) * d, 1, 1); }
    }
    return c;
  }

  function buildSeg(res) {
    const { g, c } = makeLayer(res);
    g.fillStyle = SEG.lot; g.fillRect(0, 0, WORLD.w, WORLD.h);
    const pk = app.city.park;
    g.fillStyle = '#4f6a1a'; g.fillRect(pk.x0, pk.y0, pk.x1 - pk.x0, pk.y1 - pk.y0);
    g.fillStyle = SEG.walk;
    for (const road of ROADS) stripRect(g, road, -HALF_ROAD - SIDEWALK, HALF_ROAD + SIDEWALK);
    g.fillStyle = SEG.road;
    for (const road of ROADS) stripRect(g, road, -HALF_ROAD, HALF_ROAD);
    g.strokeStyle = SEG.mark; g.lineWidth = 1;
    for (const road of ROADS) for (const [a, b] of roadSpans(road)) {
      lineAlong(g, road, a, b, -(MEDIAN / 2 + LANE_W)); lineAlong(g, road, a, b, MEDIAN / 2 + LANE_W); lineAlong(g, road, a, b, 0);
    }
    for (const b of app.city.buildings) {
      g.fillStyle = SEG.bldg; g.fillRect(b.x, b.y, b.w, b.h);
      g.strokeStyle = '#5f6b75'; g.lineWidth = 0.8; g.strokeRect(b.x + 0.4, b.y + 0.4, b.w - 0.8, b.h - 0.8);
    }
    g.fillStyle = SEG.veg;
    for (const t of app.city.trees) { g.beginPath(); g.arc(t.x, t.y, t.r, 0, Math.PI * 2); g.fill(); }
    return c;
  }

  function buildFlow(res) {
    const { g, c } = makeLayer(res);
    g.fillStyle = '#061019'; g.fillRect(0, 0, WORLD.w, WORLD.h);
    for (const b of app.city.buildings) { g.fillStyle = '#0a1720'; g.fillRect(b.x, b.y, b.w, b.h); }
    g.fillStyle = '#0b1a25';
    for (const road of ROADS) stripRect(g, road, -HALF_ROAD, HALF_ROAD);
    g.strokeStyle = '#15394a'; g.lineWidth = 0.8;
    for (const road of ROADS) for (const [a, b] of roadSpans(road)) for (const lat of [-HALF_ROAD, HALF_ROAD, 0]) lineAlong(g, road, a, b, lat);
    // faint metre grid
    g.strokeStyle = 'rgba(94,230,238,.05)'; g.lineWidth = 0.5;
    for (let x = 0; x < WORLD.w; x += 40) { g.beginPath(); g.moveTo(x, 0); g.lineTo(x, WORLD.h); g.stroke(); }
    for (let y = 0; y < WORLD.h; y += 40) { g.beginPath(); g.moveTo(0, y); g.lineTo(WORLD.w, y); g.stroke(); }
    return c;
  }

  function layerRes() {
    const big = Math.min(2.4, Math.max(0.6, app.cam.s * app.dpr));
    return Math.round(big * 4) / 4;
  }

  function rebuildLayers(force) {
    const res = layerRes();
    if (!force && Math.abs(res - app.res) < 0.01) return;
    app.res = res;
    app.layers = { ground: buildGround(res), top: buildTop(res) };
  }

  function machineLayer(mode) {
    const key = 'm-' + mode;
    if (!app.layers[key]) {
      const res = Math.min(app.res, 1.75);
      app.layers[key] = mode === 'lidar' ? buildLidar(res) : mode === 'seg' ? buildSeg(res) : buildFlow(res);
    }
    return app.layers[key];
  }

  // ------------------------------------------------------------------ sizing & camera

  function resize() {
    const r = stage.getBoundingClientRect();
    app.w = r.width; app.h = r.height;
    app.dpr = Math.min(2, window.devicePixelRatio || 1);
    canvas.width = Math.round(app.w * app.dpr);
    canvas.height = Math.round(app.h * app.dpr);
    const narrow = app.w < 700;
    app.cam.base = Math.max(app.w / WORLD.w, app.h / WORLD.h) * (narrow ? 1.35 : 1.04);
    applyZoom();
    app.lens.R = clamp(Math.min(app.w, app.h) * 0.25, 105, 215);
    const B = Math.ceil(app.lens.R * 2.4);
    mcan.width = Math.ceil(B * app.dpr); mcan.height = Math.ceil(B * app.dpr);
    rebuildLayers(false);
  }

  function applyZoom() {
    app.cam.s = app.cam.base * app.cam.zoom;
    clampCam();
    const px = 200 * app.cam.s;
    const m = px > 140 ? 25 : px < 50 ? 100 : 50;
    $('#scale-bar').style.width = `${(m * U * app.cam.s).toFixed(0)}px`;
    $('#scale-lbl').textContent = `${m} m`;
  }

  function clampCam() {
    const hw = app.w / 2 / app.cam.s, hh = app.h / 2 / app.cam.s;
    app.cam.x = WORLD.w > hw * 2 ? clamp(app.cam.x, hw, WORLD.w - hw) : WORLD.w / 2;
    app.cam.y = WORLD.h > hh * 2 ? clamp(app.cam.y, hh, WORLD.h - hh) : WORLD.h / 2;
  }

  let zoomTimer = 0;
  function zoomBy(f, sx = app.w / 2, sy = app.h / 2) {
    const [wx, wy] = s2w(sx, sy);
    app.cam.zoom = clamp(app.cam.zoom * f, 1, 3.2);
    app.cam.s = app.cam.base * app.cam.zoom;
    app.cam.x = wx - (sx - app.w / 2) / app.cam.s;
    app.cam.y = wy - (sy - app.h / 2) / app.cam.s;
    app.cam.tx = null;
    applyZoom();
    clearTimeout(zoomTimer);
    zoomTimer = setTimeout(() => rebuildLayers(false), 220);
  }

  function flyTo(x, y) { app.cam.tx = x; app.cam.ty = y; app.manualUntil = performance.now() + 9000; app.lens.x = x; app.lens.y = y; }

  // ------------------------------------------------------------------ entities

  function collectEntities() {
    const out = [];
    for (const v of app.sim.vehicles.values()) {
      const p = S.vehiclePose(v);
      out.push({ kind: 'veh', ref: v, id: v.id, x: p.x, y: p.y, ang: p.ang, len: v.len, wid: v.wid });
    }
    for (const p of app.sim.peds) {
      const q = S.pedPose(p);
      out.push({ kind: 'ped', ref: p, id: p.id, x: q.x, y: q.y, ang: 0, len: 4, wid: 4 });
    }
    for (const h of app.sim.hazards) {
      const q = S.hazardPose(h, app.sim.t);
      const st = h.stream;
      const k = Math.min(1, (app.sim.t - h.t0) / 3);
      const ang = Math.atan2(st.hy, st.hx) + (1 - k) * -Math.PI / 2 + Math.sin(app.sim.t * 0.4 + h.id) * 0.25;
      out.push({ kind: 'cow', ref: h, id: h.id, x: q.x, y: q.y, ang, len: 13, wid: 6 });
    }
    app.ents = out;
  }

  function findEnt(sel) {
    if (!sel) return null;
    return app.ents.find((e) => e.kind === sel.kind && e.id === sel.id) || null;
  }

  function hitTest(sx, sy) {
    const [wx, wy] = s2w(sx, sy);
    const tol = Math.max(7, 16 / app.cam.s);
    let best = null, bd = Infinity;
    for (const e of app.ents) {
      const d = Math.hypot(e.x - wx, e.y - wy) - Math.max(e.len, e.wid) / 2;
      const pri = e.kind === 'ped' ? 1.5 : 0;
      if (d - pri < tol && d - pri < bd) { bd = d - pri; best = e; }
    }
    return best;
  }

  // ------------------------------------------------------------------ main view drawing

  function setWorld(g, dpr, ox = 0, oy = 0) {
    const s = app.cam.s;
    g.setTransform(dpr * s, 0, 0, dpr * s, dpr * (app.w / 2 - app.cam.x * s - ox), dpr * (app.h / 2 - app.cam.y * s - oy));
  }

  function drawVehicle(g, e, t) {
    const v = e.ref, L = v.len, W = v.wid, P = app.pal;
    g.save();
    g.translate(e.x, e.y); g.rotate(e.ang);
    g.fillStyle = hexA(P.shadow, P.dark ? 0.55 : 0.2);
    rr(g, -L / 2 + 1.1, -W / 2 + 1.5, L, W, 1.6); g.fill();
    if (P.dark && v.type !== 'cycle') {
      const hg = g.createLinearGradient(L / 2, 0, L / 2 + 20, 0);
      hg.addColorStop(0, 'rgba(255,226,160,.22)'); hg.addColorStop(1, 'rgba(255,226,160,0)');
      g.fillStyle = hg;
      g.beginPath(); g.moveTo(L / 2, -W / 2 + 1); g.lineTo(L / 2 + 20, -W / 2 - 2.5); g.lineTo(L / 2 + 20, W / 2 + 2.5); g.lineTo(L / 2, W / 2 - 1); g.fill();
      g.fillStyle = 'rgba(255,70,60,.75)';
      g.fillRect(-L / 2 - 0.4, -W / 2 + 0.5, 0.8, 1.4); g.fillRect(-L / 2 - 0.4, W / 2 - 1.9, 0.8, 1.4);
    }
    switch (v.type) {
      case 'two': {
        g.fillStyle = '#3b4349'; rr(g, -L / 2, -0.95, L, 1.9, 0.9); g.fill();
        g.fillStyle = '#20262b'; g.fillRect(L / 2 - 1.2, -1.6, 0.8, 3.2);
        const shirt = ['#e07a2f', '#2f8f8a', '#6b4fb8', '#3a6fc4', '#c2367a', '#d4a017'][v.id % 6];
        if (v.triple) { g.fillStyle = '#8a5a3c'; g.beginPath(); g.arc(-2.9, 0, 1.25, 0, 7); g.fill(); }
        if (v.triple || v.shade < 0.35) { g.fillStyle = '#b8452f'; g.beginPath(); g.arc(-1.8, 0, 1.35, 0, 7); g.fill(); }
        g.fillStyle = shirt; g.beginPath(); g.ellipse(-0.3, 0, 1.4, 1.75, 0, 0, 7); g.fill();
        g.fillStyle = v.noHelmet ? '#b07a4f' : '#1e2328'; g.beginPath(); g.arc(0.3, 0, 1.05, 0, 7); g.fill();
        break;
      }
      case 'cycle': {
        g.strokeStyle = '#3b4349'; g.lineWidth = 0.7;
        g.beginPath(); g.moveTo(-L / 2, 0); g.lineTo(L / 2, 0); g.stroke();
        g.fillStyle = '#4d7a3a'; g.beginPath(); g.ellipse(-0.4, 0, 1.2, 1.5, 0, 0, 7); g.fill();
        g.fillStyle = '#6b4a35'; g.beginPath(); g.arc(0.2, 0, 0.85, 0, 7); g.fill();
        break;
      }
      case 'auto': {
        const mumbai = v.shade > 0.62;
        const body = mumbai ? '#e9c84a' : '#3f8f55', canopy = mumbai ? '#23272b' : '#f0cb4e';
        g.fillStyle = body;
        rr(g, -L / 2, -W / 2, L - 2.6, W, 1.8); g.fill();
        g.beginPath(); g.moveTo(L / 2 - 2.8, -W * 0.34); g.lineTo(L / 2, -W * 0.16); g.lineTo(L / 2, W * 0.16); g.lineTo(L / 2 - 2.8, W * 0.34); g.closePath(); g.fill();
        g.fillStyle = canopy; rr(g, -L / 2 + 0.5, -W / 2 + 0.45, L - 4.4, W - 0.9, 2); g.fill();
        g.fillStyle = 'rgba(255,255,255,.18)'; g.fillRect(-L / 2 + 1.2, -W / 2 + 1, L - 6, 0.8);
        break;
      }
      case 'bus': {
        g.fillStyle = '#eef1f1'; rr(g, -L / 2, -W / 2, L, W, 2); g.fill();
        g.fillStyle = BUS_STRIPES[v.id % 3];
        g.fillRect(-L / 2 + 1, -W / 2, L - 2, 1.3); g.fillRect(-L / 2 + 1, W / 2 - 1.3, L - 2, 1.3);
        g.fillStyle = '#4f5d66'; g.fillRect(L / 2 - 2.2, -W / 2 + 1.4, 1.5, W - 2.8);
        g.fillStyle = '#cfd6d8';
        for (const u of [-0.32, -0.02, 0.26]) g.fillRect(u * L - 3, -2.3, 6, 4.6);
        break;
      }
      case 'truck': {
        g.fillStyle = TARPS[v.id % 3]; rr(g, -L / 2, -W / 2 - 0.2, L - 9.5, W + 0.4, 1.2); g.fill();
        g.strokeStyle = 'rgba(0,0,0,.18)'; g.lineWidth = 0.4;
        for (let u = -L / 2 + 4; u < L / 2 - 10; u += 4) { g.beginPath(); g.moveTo(u, -W / 2); g.lineTo(u, W / 2); g.stroke(); }
        g.fillStyle = v.id % 2 ? '#e8833a' : '#f0f1ef'; rr(g, L / 2 - 8.5, -W / 2 + 0.3, 8.5, W - 0.6, 1.6); g.fill();
        g.fillStyle = '#4f5d66'; g.fillRect(L / 2 - 2.2, -W / 2 + 1.3, 1.4, W - 2.6);
        break;
      }
      case 'amb': {
        g.fillStyle = '#ffffff'; rr(g, -L / 2, -W / 2, L, W, 2); g.fill();
        g.strokeStyle = 'rgba(0,0,0,.12)'; g.lineWidth = 0.4; g.stroke();
        g.fillStyle = '#d6332e'; g.fillRect(-L / 2 + 1, -W / 2, L - 3, 1.1); g.fillRect(-L / 2 + 1, W / 2 - 1.1, L - 3, 1.1);
        g.fillRect(-3.5, -0.8, 7, 1.6); g.fillRect(-0.8, -3.5, 1.6, 7);
        const on = Math.floor(t * 6) % 2 === 0;
        g.fillStyle = on ? '#ff2d2d' : '#5a1010'; g.fillRect(L / 2 - 6, -W / 2 + 0.8, 2, 2.8);
        g.fillStyle = on ? '#1c2f6b' : '#3d7bff'; g.fillRect(L / 2 - 6, W / 2 - 3.6, 2, 2.8);
        g.fillStyle = '#4f5d66'; g.fillRect(L / 2 - 2.6, -W / 2 + 1.2, 1.6, W - 2.4);
        break;
      }
      default: {
        const tint = CAR_TINTS[v.id % CAR_TINTS.length];
        g.fillStyle = tint; rr(g, -L / 2, -W / 2, L, W, 2.4); g.fill();
        g.strokeStyle = hexA(P.shadow, 0.12); g.lineWidth = 0.4; g.stroke();
        g.fillStyle = '#56656e'; rr(g, -L * 0.24, -W * 0.39, L * 0.52, W * 0.78, 1.4); g.fill();
        g.fillStyle = tint; rr(g, -L * 0.15, -W * 0.35, L * 0.31, W * 0.7, 1.1); g.fill();
      }
    }
    g.restore();
  }

  function drawCow(g, e) {
    const P = app.pal;
    g.save(); g.translate(e.x, e.y); g.rotate(e.ang);
    g.fillStyle = hexA(P.shadow, P.dark ? 0.55 : 0.2);
    g.beginPath(); g.ellipse(1.2, 1.4, 5.8, 2.9, 0, 0, 7); g.fill();
    g.fillStyle = '#f3efe4';
    g.beginPath(); g.ellipse(0, 0, 5.6, 2.7, 0, 0, 7); g.fill();
    g.fillStyle = '#e4dccb'; g.beginPath(); g.ellipse(1.6, 0, 1.7, 1.5, 0, 0, 7); g.fill();
    g.fillStyle = '#ece6d8'; g.beginPath(); g.ellipse(6.3, 0, 1.9, 1.3, 0, 0, 7); g.fill();
    g.strokeStyle = '#8a7a62'; g.lineWidth = 0.45;
    g.beginPath(); g.moveTo(6, -1.1); g.lineTo(5.6, -2.5); g.moveTo(6, 1.1); g.lineTo(5.6, 2.5); g.stroke();
    g.beginPath(); g.moveTo(-5.4, 0); g.lineTo(-7.2, 0.6); g.stroke();
    g.restore();
  }

  function drawPed(g, e) {
    const p = e.ref, P = app.pal;
    g.fillStyle = hexA(P.shadow, P.dark ? 0.6 : 0.22);
    g.beginPath(); g.arc(e.x + 0.9, e.y + 0.8, 1.8, 0, 7); g.fill();
    g.fillStyle = p.color; g.beginPath(); g.arc(e.x, e.y, 1.75, 0, 7); g.fill();
    g.fillStyle = '#2b2420'; g.beginPath(); g.arc(e.x + 0.15, e.y - 0.1, 0.85, 0, 7); g.fill();
  }

  function drawSignals(g) {
    for (const st of app.sim.streams) {
      for (const sj of st.junctions) {
        const s = sj.stopS + 3, lat = HALF_ROAD + 4.5;
        const x = st.start.x + st.hx * s + st.lx * lat, y = st.start.y + st.hy * s + st.ly * lat;
        const sig = S.signalOf(sj.j, sj.phase);
        g.fillStyle = '#1b2227';
        rr(g, x - 2.4, y - 2.4, 4.8, 4.8, 1.2); g.fill();
        g.fillStyle = hexA(SIG[sig], 0.28); g.beginPath(); g.arc(x, y, 4.6, 0, 7); g.fill();
        g.fillStyle = SIG[sig]; g.beginPath(); g.arc(x, y, 1.55, 0, 7); g.fill();
      }
    }
  }

  function brackets(g, e, color, pad, lw) {
    const hl = e.len / 2 + pad, hw = e.wid / 2 + pad;
    const k = Math.min(hl, hw) * 0.7 + 1;
    g.save(); g.translate(e.x, e.y); g.rotate(e.ang);
    g.strokeStyle = color; g.lineWidth = lw;
    g.beginPath();
    for (const [sx, sy] of [[-1, -1], [1, -1], [1, 1], [-1, 1]]) {
      g.moveTo(sx * hl, sy * hw + -sy * k); g.lineTo(sx * hl, sy * hw); g.lineTo(sx * hl + -sx * k, sy * hw);
    }
    g.stroke(); g.restore();
  }

  function drawLabels(g) {
    const P = app.pal;
    g.font = `600 10px ${FONT_MONO}`;
    g.fillStyle = P.label;
    g.textBaseline = 'middle';
    const spaced = (txt) => txt.toUpperCase().split('').join(' ');
    for (const road of ROADS) {
      const perp = ROADS.filter((q) => q.axis !== road.axis).map((q) => q.c);
      const L = road.axis === 'h' ? WORLD.w : WORLD.h;
      const mids = [(0 + perp[0]) / 2, (perp[0] + perp[1]) / 2, (perp[1] + L) / 2];
      for (const m of mids) {
        const [wx, wy] = along(road, m, -(HALF_ROAD + SIDEWALK / 2));
        const [sx, sy] = w2s(wx, wy);
        if (sx < -100 || sy < -100 || sx > app.w + 100 || sy > app.h + 100) continue;
        g.save(); g.translate(sx, sy);
        if (road.axis === 'v') g.rotate(-Math.PI / 2);
        g.textAlign = 'center';
        g.fillText(spaced(road.name), 0, 0.5);
        g.restore();
      }
    }
    const pk = app.city.park;
    const [px, py] = w2s((pk.x0 + pk.x1) / 2 + 40, (pk.y0 + pk.y1) / 2 + 66);
    g.textAlign = 'center'; g.fillText(spaced('Gandhi Maidan'), px, py);
    // junction ids
    g.textAlign = 'left';
    for (const j of app.sim.junctions) {
      const [sx, sy] = w2s(j.x - HALF_ROAD - SIDEWALK - 2, j.y - HALF_ROAD - SIDEWALK - 2);
      g.font = `600 10px ${FONT_MONO}`;
      const w = g.measureText(j.id).width + 10;
      g.fillStyle = P.ink; rr(g, sx - w, sy - 16, w, 16, 3); g.fill();
      g.fillStyle = P.paper; g.fillText(j.id, sx - w + 5, sy - 7.5);
    }
  }

  // ------------------------------------------------------------------ machine view

  const lensEnts = [];

  function tag(g, x, y, color, text, alpha = 1) {
    g.font = `500 9.5px ${FONT_MONO}`;
    const w = g.measureText(text).width + 17, h = 15;
    g.globalAlpha = alpha;
    g.fillStyle = 'rgba(4,13,20,.88)'; rr(g, x, y - h, w, h, 2); g.fill();
    g.fillStyle = color; g.fillRect(x + 5, y - h / 2 - 2.5, 5, 5);
    g.fillStyle = '#d9f7f9'; g.textBaseline = 'middle'; g.textAlign = 'left';
    g.fillText(text, x + 13, y - h / 2 + 0.5);
    g.globalAlpha = 1;
    return [x, y - h, w, h];
  }

  function nearestJunction(x, y) {
    let best = app.sim.junctions[0], bd = Infinity;
    for (const j of app.sim.junctions) { const d = Math.hypot(j.x - x, j.y - y); if (d < bd) { bd = d; best = j; } }
    return best;
  }

  function entColor(e) {
    if (e.kind === 'ped') return SENSE.ped;
    if (e.kind === 'cow') return SENSE.cow;
    return SENSE[e.ref.type];
  }

  function entLabel(e) {
    if (e.kind === 'ped') return `${idTag(e)}${e.ref.waiting ? ' · waiting' : ''}`;
    if (e.kind === 'cow') return 'CATTLE · HAZARD';
    const v = e.ref;
    if (v.type === 'amb') return `${v.callsign} · ${kmh(v.v).toFixed(0)} km/h`;
    return `${idTag(e)} · ${kmh(v.v).toFixed(0)} km/h`;
  }

  function drawMachine(t) {
    const L = app.lens, dpr = app.dpr, g = mctx;
    const B = mcan.width / dpr;
    const bx = L.sx - B / 2, by = L.sy - B / 2;
    g.setTransform(1, 0, 0, 1, 0, 0);
    g.clearRect(0, 0, mcan.width, mcan.height);
    setWorld(g, dpr, bx, by);
    g.imageSmoothingEnabled = true;
    g.drawImage(machineLayer(app.mode), 0, 0, WORLD.w, WORLD.h);

    const s = app.cam.s, px = 1 / s;
    const wr = (B / 2) / s;
    lensEnts.length = 0;
    for (const e of app.ents) if (Math.hypot(e.x - L.x, e.y - L.y) < wr + 20) lensEnts.push(e);
    const J = nearestJunction(L.x, L.y);

    // virtual stop lines, all modes
    for (const st of app.sim.streams) {
      for (const sj of st.junctions) {
        if (Math.hypot(sj.j.x - L.x, sj.j.y - L.y) > wr + 80) continue;
        const sig = S.signalOf(sj.j, sj.phase);
        const s0 = sj.stopS + 1;
        const a = [st.start.x + st.hx * s0 + st.lx * (MEDIAN / 2), st.start.y + st.hy * s0 + st.ly * (MEDIAN / 2)];
        const b = [st.start.x + st.hx * s0 + st.lx * HALF_ROAD, st.start.y + st.hy * s0 + st.ly * HALF_ROAD];
        g.strokeStyle = hexA(SIG[sig], 0.25); g.lineWidth = 6 * px;
        g.beginPath(); g.moveTo(a[0], a[1]); g.lineTo(b[0], b[1]); g.stroke();
        g.strokeStyle = SIG[sig]; g.lineWidth = 1.8 * px;
        g.beginPath(); g.moveTo(a[0], a[1]); g.lineTo(b[0], b[1]); g.stroke();
      }
    }

    if (app.mode === 'lidar') drawLidar(g, t, px, J);
    else if (app.mode === 'seg') drawSeg(g, px);
    else drawFlow(g, t, px);

    // screen-space labels with simple collision avoidance
    g.setTransform(dpr, 0, 0, dpr, -bx * dpr, -by * dpr);
    const sel = app.selected, hov = app.hover;
    const cand = lensEnts
      .map((e) => {
        const [sx, sy] = w2s(e.x, e.y);
        const d = Math.hypot(sx - L.sx, sy - L.sy);
        let pri = d;
        if (e.kind === 'veh' && e.ref.type === 'amb') pri -= 500;
        if (e.kind === 'cow') pri -= 400;
        if (e.kind === 'veh' && e.ref.flags.length) pri -= 120;
        if (sel && sel.id === e.id) pri -= 1000;
        if (hov && hov.id === e.id) pri -= 900;
        return { e, sx, sy, d, pri };
      })
      .filter((c) => c.d < L.R * 0.92)
      .sort((a, b) => a.pri - b.pri);
    const boxes = [];
    const maxLabels = app.mode === 'seg' ? 10 : 22;
    for (const c of cand) {
      if (boxes.length >= maxLabels) break;
      if (app.mode !== 'seg' && c.e.kind === 'ped' && c.pri > 0 && boxes.length > 12) continue;
      const half = (Math.max(c.e.len, c.e.wid) / 2) * s;
      const x = c.sx + half * 0.4 + 3, y = c.sy - half * 0.4 - 3;
      g.font = `500 9.5px ${FONT_MONO}`;
      const text = entLabel(c.e);
      const w = g.measureText(text).width + 17;
      const r0 = [x, y - 15, w, 15];
      if (boxes.some((b) => r0[0] < b[0] + b[2] && r0[0] + r0[2] > b[0] && r0[1] < b[1] + b[3] && r0[1] + r0[3] > b[1])) continue;
      const fade = clamp(1.25 - c.d / (L.R * 0.92), 0.35, 1);
      boxes.push(tag(g, x, y, app.mode === 'seg' ? segColor(c.e) : entColor(c.e), text, fade));
    }

    // lens HUD: junction readout inside the rim
    const readout = app.mode === 'flow'
      ? `${J.id} · ${J.decision.split('·')[0].trim().toUpperCase()}`
      : `${J.id} NODE · ${lensEnts.filter((e) => e.kind !== 'ped').length} TRACKS · ${app.mode === 'lidar' ? '10 HZ' : '0.97 mIoU*'}`;
    g.font = `500 9px ${FONT_MONO}`;
    g.fillStyle = 'rgba(217,247,249,.75)'; g.textAlign = 'center'; g.textBaseline = 'middle';
    g.fillText(readout, L.sx, L.sy + L.R * 0.8);
    g.textAlign = 'left';

    // vignette + scanlines
    g.setTransform(dpr, 0, 0, dpr, 0, 0);
    const cx = B / 2, cy = B / 2;
    const vg = g.createRadialGradient(cx, cy, L.R * 0.45, cx, cy, L.R * 1.15);
    vg.addColorStop(0, 'rgba(2,8,12,0)'); vg.addColorStop(1, 'rgba(2,8,12,.4)');
    g.fillStyle = vg; g.fillRect(0, 0, B, B);
    g.fillStyle = 'rgba(94,230,238,.035)';
    for (let y = 0; y < B; y += 3) g.fillRect(0, y, B, 1);
  }

  function drawLidar(g, t, px, J) {
    // rotating sweep from the nearest junction sensor
    if (!reduceMotion && g.createConicGradient) {
      const a = (t * 1.4) % (Math.PI * 2);
      const cg = g.createConicGradient(a, J.x, J.y);
      cg.addColorStop(0, 'rgba(94,230,238,.22)'); cg.addColorStop(0.09, 'rgba(94,230,238,0)'); cg.addColorStop(1, 'rgba(94,230,238,0)');
      g.fillStyle = cg; g.beginPath(); g.arc(J.x, J.y, 470, 0, Math.PI * 2); g.fill();
    }
    g.fillStyle = '#5ee6ee'; g.beginPath(); g.arc(J.x, J.y, 2.4, 0, 7); g.fill();
    g.strokeStyle = 'rgba(94,230,238,.6)'; g.lineWidth = px; g.beginPath(); g.arc(J.x, J.y, 6, 0, 7); g.stroke();

    for (const e of lensEnts) {
      const col = entColor(e);
      const ca = Math.cos(e.ang), sa = Math.sin(e.ang);
      if (e.kind === 'ped') {
        g.fillStyle = col;
        for (let k = 0; k < 7; k++) { const h = hash3(e.id, k, 1); const a2 = h * 6.28, d = 0.6 + hash3(e.id, k, 2) * 1.6; g.fillRect(e.x + Math.cos(a2) * d - 0.5, e.y + Math.sin(a2) * d - 0.5, 1, 1); }
        g.strokeStyle = hexA(col, 0.8); g.lineWidth = px; g.beginPath(); g.arc(e.x, e.y, 3.2, 0, 7); g.stroke();
        continue;
      }
      // point cloud: perimeter returns dense, interior sparse
      g.fillStyle = col;
      const hl = e.len / 2, hw = e.wid / 2, step = e.kind === 'cow' ? 1.4 : 1.9;
      let ui = 0;
      for (let u = -hl + 0.6; u <= hl - 0.6; u += step, ui++) {
        let wi = 0;
        for (let w = -hw + 0.5; w <= hw - 0.5; w += step, wi++) {
          const edge = u < -hl + step * 1.2 || u > hl - step * 1.2 || w < -hw + step || w > hw - step;
          if (e.kind === 'cow' && (u / hl) ** 2 + (w / hw) ** 2 > 1) continue;
          const h = hash3(e.id, ui * 31 + wi, Math.floor(t * 10));
          if (!edge && h > 0.45) continue;
          g.globalAlpha = edge ? 0.95 : 0.5;
          g.fillRect(e.x + ca * u - sa * w - 0.5, e.y + sa * u + ca * w - 0.5, 1, 1);
        }
      }
      g.globalAlpha = 1;
      g.save(); g.translate(e.x, e.y); g.rotate(e.ang);
      g.strokeStyle = hexA(col, 0.9); g.lineWidth = px;
      g.strokeRect(-hl - 1.2, -hw - 1.2, e.len + 2.4, e.wid + 2.4);
      if (e.kind === 'veh' && e.ref.v > 0.5) {
        g.strokeStyle = hexA(col, 0.6);
        g.beginPath(); g.moveTo(hl + 1.5, 0); g.lineTo(hl + 1.5 + e.ref.v * 0.6, 0); g.stroke();
      }
      g.restore();
      if (e.kind === 'veh' && e.ref.type === 'amb') {
        g.strokeStyle = hexA('#ff4d6d', 0.5 + 0.4 * Math.sin(t * 8)); g.lineWidth = 1.5 * px;
        g.beginPath(); g.arc(e.x, e.y, 18 + 4 * Math.sin(t * 4), 0, 7); g.stroke();
      }
      if (e.kind === 'cow') {
        g.strokeStyle = hexA('#ffb020', 0.6 + 0.3 * Math.sin(t * 5)); g.lineWidth = 1.2 * px;
        g.setLineDash([3 * px, 2 * px]); g.beginPath(); g.arc(e.x, e.y, 14, 0, 7); g.stroke(); g.setLineDash([]);
      }
    }
  }

  function segColor(e) {
    if (e.kind === 'ped') return SEG.ped;
    if (e.kind === 'cow') return SEG.cow;
    return SEG[e.ref.type];
  }

  function drawSeg(g, px) {
    for (const e of lensEnts) {
      const col = segColor(e);
      if (e.kind === 'ped') { g.fillStyle = col; g.beginPath(); g.arc(e.x, e.y, 2.3, 0, 7); g.fill(); continue; }
      g.save(); g.translate(e.x, e.y); g.rotate(e.ang);
      g.fillStyle = col;
      if (e.kind === 'cow') { g.beginPath(); g.ellipse(0, 0, 6, 3, 0, 0, 7); g.fill(); g.beginPath(); g.ellipse(6.4, 0, 2, 1.4, 0, 0, 7); g.fill(); }
      else { rr(g, -e.len / 2, -e.wid / 2, e.len, e.wid, e.ref.type === 'two' ? 1 : 1.8); g.fill(); }
      if (e.kind === 'veh' && (e.ref.type === 'two' || e.ref.type === 'cycle')) {
        g.fillStyle = '#ff0000'; g.beginPath(); g.arc(0, 0, 1.6, 0, 7); g.fill();
      }
      g.strokeStyle = 'rgba(255,255,255,.35)'; g.lineWidth = px; g.stroke();
      g.restore();
    }
  }

  function drawFlow(g, t, px) {
    const wr = (mcan.width / app.dpr / 2) / app.cam.s;
    // queue heat along each approach inside the detection zone
    for (const st of app.sim.streams) {
      for (const sj of st.junctions) {
        if (Math.hypot(sj.j.x - app.lens.x, sj.j.y - app.lens.y) > wr + DETECT) continue;
        let back = sj.stopS;
        for (const lane of st.lanes) for (const v of lane) {
          const d = sj.stopS - v.s;
          if (d >= -1 && d < DETECT && v.v < 1 * U) back = Math.min(back, v.s - v.len);
        }
        const q = sj.j.qByStream[st.id] || 0;
        const qlen = sj.stopS - back;
        // detection zone outline
        const z0 = sj.stopS - DETECT, z1 = sj.stopS;
        const pt = (s, lat) => [st.start.x + st.hx * s + st.lx * lat, st.start.y + st.hy * s + st.ly * lat];
        g.strokeStyle = 'rgba(94,230,238,.22)'; g.lineWidth = px; g.setLineDash([4 * px, 4 * px]);
        g.beginPath();
        const c = [pt(z0, MEDIAN / 2), pt(z1, MEDIAN / 2), pt(z1, HALF_ROAD), pt(z0, HALF_ROAD)];
        g.moveTo(c[0][0], c[0][1]); for (const p of c.slice(1)) g.lineTo(p[0], p[1]); g.closePath(); g.stroke();
        g.setLineDash([]);
        if (qlen > 2) {
          const a = pt(sj.stopS, (MEDIAN / 2 + HALF_ROAD) / 2), b = pt(back, (MEDIAN / 2 + HALF_ROAD) / 2);
          const gr = g.createLinearGradient(a[0], a[1], b[0], b[1]);
          const hot = q > 14 ? '#ff4a3d' : q > 7 ? '#ffb020' : '#2bd576';
          gr.addColorStop(0, hexA(hot, 0.55)); gr.addColorStop(1, hexA(hot, 0.05));
          g.strokeStyle = gr; g.lineWidth = HALF_ROAD - MEDIAN / 2; g.lineCap = 'butt';
          g.beginPath(); g.moveTo(a[0], a[1]); g.lineTo(b[0], b[1]); g.stroke();
        }
      }
    }
    // tracks: speed-coloured chevrons and 3 s predicted paths
    for (const e of lensEnts) {
      if (e.kind === 'ped') {
        g.fillStyle = e.ref.waiting ? '#ffb020' : 'rgba(255,122,162,.8)';
        g.beginPath(); g.arc(e.x, e.y, 1.8, 0, 7); g.fill();
        continue;
      }
      if (e.kind === 'cow') {
        g.fillStyle = '#ffb020'; g.beginPath(); g.arc(e.x, e.y, 4, 0, 7); g.fill();
        continue;
      }
      const v = e.ref, ratio = v.v / v.vmax;
      const col = v.type === 'amb' ? '#ff4d6d' : ratio < 0.15 ? '#ff4a3d' : ratio < 0.5 ? '#ffb020' : '#2bd576';
      const ca = Math.cos(e.ang), sa = Math.sin(e.ang);
      if (v.v > 0.5) {
        g.fillStyle = hexA(col, 0.7);
        const fx = e.x + ca * e.len / 2, fy = e.y + sa * e.len / 2;
        for (let k = 1; k <= 6; k++) {
          const d = (v.v * 3 * k) / 6;
          g.globalAlpha = 1 - k / 7;
          g.fillRect(fx + ca * d - 0.7, fy + sa * d - 0.7, 1.4, 1.4);
        }
        g.globalAlpha = 1;
      }
      g.save(); g.translate(e.x, e.y); g.rotate(e.ang);
      g.fillStyle = col;
      const hl = e.len / 2, hw = Math.max(1.4, e.wid / 2);
      g.beginPath(); g.moveTo(hl, 0); g.lineTo(-hl, -hw); g.lineTo(-hl + Math.min(4, hl), 0); g.lineTo(-hl, hw); g.closePath(); g.fill();
      g.restore();
    }
    // junction phase gauges
    for (const j of app.sim.junctions) {
      if (Math.hypot(j.x - app.lens.x, j.y - app.lens.y) > wr + 40) continue;
      const lim = app.sim.controller === 'fixed' ? j.fixedG[j.phase] : S.TUNE.maxG;
      const k = j.state === 'G' ? clamp(j.t / lim, 0, 1) : 1;
      const col = j.state === 'G' ? '#2bd576' : j.state === 'Y' ? '#ffb020' : '#ff4a3d';
      g.lineWidth = 2.6 * px * 1.6;
      g.strokeStyle = 'rgba(94,230,238,.18)'; g.beginPath(); g.arc(j.x, j.y, 16, 0, 7); g.stroke();
      g.strokeStyle = col; g.beginPath(); g.arc(j.x, j.y, 16, -Math.PI / 2, -Math.PI / 2 + k * Math.PI * 2); g.stroke();
      // axis bars
      g.fillStyle = j.phase === 0 && j.state === 'G' ? '#2bd576' : 'rgba(255,74,61,.7)';
      g.fillRect(j.x - 9, j.y - 1.2, 18, 2.4);
      g.fillStyle = j.phase === 1 && j.state === 'G' ? '#2bd576' : 'rgba(255,74,61,.7)';
      g.fillRect(j.x - 1.2, j.y - 9, 2.4, 18);
    }
    // decision text near each junction (screen space)
    const dpr = app.dpr, B = mcan.width / dpr;
    g.save();
    g.setTransform(dpr, 0, 0, dpr, -(app.lens.sx - B / 2) * dpr, -(app.lens.sy - B / 2) * dpr);
    for (const j of app.sim.junctions) {
      if (Math.hypot(j.x - app.lens.x, j.y - app.lens.y) > wr) continue;
      const [sx, sy] = w2s(j.x, j.y);
      const [a, b] = j.decision.split('·').map((x) => x.trim());
      g.font = `600 9.5px ${FONT_MONO}`; g.fillStyle = '#d9f7f9'; g.textAlign = 'left'; g.textBaseline = 'alphabetic';
      g.fillText(`${j.id} · ${(a || '').toUpperCase()}`, sx + 22, sy - 20);
      if (b) { g.font = `400 9px ${FONT_MONO}`; g.fillStyle = 'rgba(217,247,249,.7)'; g.fillText(b, sx + 22, sy - 8); }
    }
    g.restore();
  }

  function compositeLens(t) {
    const L = app.lens, dpr = app.dpr;
    const B = mcan.width / dpr;
    const bx = L.sx - B / 2, by = L.sy - B / 2;
    const inner = L.R * 0.82, outer = L.R * 1.14;
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.save();
    ctx.beginPath(); ctx.arc(L.sx, L.sy, inner, 0, Math.PI * 2); ctx.clip();
    ctx.drawImage(mcan, bx, by, B, B);
    ctx.restore();
    // dithered pixel rim
    const cs = app.w < 700 ? 6 : 8;
    const frame = reduceMotion ? 0 : Math.floor(t * 2.5);
    const x0 = Math.floor((L.sx - outer) / cs) * cs, x1 = L.sx + outer;
    const y0 = Math.floor((L.sy - outer) / cs) * cs, y1 = L.sy + outer;
    ctx.strokeStyle = 'rgba(94,230,238,.55)'; ctx.lineWidth = 1;
    for (let y = y0; y < y1; y += cs) {
      for (let x = x0; x < x1; x += cs) {
        const d = Math.hypot(x + cs / 2 - L.sx, y + cs / 2 - L.sy);
        if (d < inner - cs || d > outer) continue;
        const p = 1 - (d - (inner - cs)) / (outer - inner + cs);
        const h = hash3(x / cs, y / cs, frame + ((x / cs + y / cs) & 3));
        if (h < p * p * 1.05) {
          ctx.drawImage(mcan, (x - bx) * dpr, (y - by) * dpr, cs * dpr, cs * dpr, x, y, cs, cs);
        } else if (h < p + 0.06 && d > inner) {
          ctx.strokeRect(x + 1.5, y + 1.5, cs - 3, cs - 3);
        }
      }
    }
    // reticle
    ctx.strokeStyle = 'rgba(217,247,249,.8)'; ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.moveTo(L.sx - 9, L.sy); ctx.lineTo(L.sx - 4, L.sy); ctx.moveTo(L.sx + 4, L.sy); ctx.lineTo(L.sx + 9, L.sy);
    ctx.moveTo(L.sx, L.sy - 9); ctx.lineTo(L.sx, L.sy - 4); ctx.moveTo(L.sx, L.sy + 4); ctx.lineTo(L.sx, L.sy + 9);
    ctx.stroke();
  }

  // ------------------------------------------------------------------ frame

  function updateLens(now, dt) {
    const L = app.lens;
    const manual = now < app.manualUntil;
    if (app.touch) {
      // phones: the lens sits mid-screen and the map moves under it
      L.sx = app.w / 2; L.sy = app.h * 0.52;
      if (!manual && !app.ptr.down) tourStep(now, dt, true);
      const [wx, wy] = s2w(L.sx, L.sy);
      L.x = wx; L.y = wy;
      return;
    }
    if (app.ptr.inside && manual) {
      const [wx, wy] = s2w(app.ptr.sx, app.ptr.sy);
      L.x = wx; L.y = wy;
    } else if (!manual) {
      tourStep(now, dt, false);
    }
    [L.sx, L.sy] = w2s(L.x, L.y);
  }

  function tourTarget() {
    const amb = [...app.sim.vehicles.values()].find((v) => v.type === 'amb');
    if (amb) { const p = S.vehiclePose(amb); return [p.x + Math.cos(p.ang) * 40, p.y + Math.sin(p.ang) * 40]; }
    const j = app.sim.junctions[app.tour.i % app.sim.junctions.length];
    return [j.x + 30, j.y + 20];
  }

  function tourStep(now, dt, moveCam) {
    if (now > app.tour.next) { app.tour.i++; app.tour.next = now + 8000; }
    const [tx, ty] = tourTarget();
    const k = 1 - Math.exp(-dt * (reduceMotion ? 0.4 : 0.9));
    if (moveCam) { app.cam.x += (tx - app.cam.x) * k; app.cam.y += (ty - app.cam.y) * k; clampCam(); }
    else {
      app.lens.x += (tx - app.lens.x) * k; app.lens.y += (ty - app.lens.y) * k;
      // keep the lens target in view
      const [sx, sy] = w2s(app.lens.x, app.lens.y);
      const m = app.lens.R * 0.9;
      if (sx < m || sx > app.w - m || sy < m || sy > app.h - m) {
        app.cam.x += (app.lens.x - app.cam.x) * k * 0.6; app.cam.y += (app.lens.y - app.cam.y) * k * 0.6; clampCam();
      }
    }
  }

  function draw(t, now, dt) {
    const dpr = app.dpr, P = app.pal;
    if (app.cam.tx != null) {
      const k = 1 - Math.exp(-dt * 4);
      app.cam.x += (app.cam.tx - app.cam.x) * k; app.cam.y += (app.cam.ty - app.cam.y) * k;
      clampCam();
      if (Math.hypot(app.cam.tx - app.cam.x, app.cam.ty - app.cam.y) < 1) app.cam.tx = null;
    }
    collectEntities();
    updateLens(now, dt);

    ctx.setTransform(1, 0, 0, 1, 0, 0);
    ctx.fillStyle = P.ground; ctx.fillRect(0, 0, canvas.width, canvas.height);
    setWorld(ctx, dpr);
    ctx.imageSmoothingEnabled = true;
    ctx.drawImage(app.layers.ground, 0, 0, WORLD.w, WORLD.h);

    const hw = app.w / 2 / app.cam.s + 40, hh = app.h / 2 / app.cam.s + 40;
    const vis = (e) => Math.abs(e.x - app.cam.x) < hw && Math.abs(e.y - app.cam.y) < hh;
    for (const e of app.ents) if (e.kind === 'ped' && vis(e)) drawPed(ctx, e);
    for (const e of app.ents) if (e.kind === 'cow' && vis(e)) drawCow(ctx, e);
    for (const e of app.ents) if (e.kind === 'veh' && vis(e)) drawVehicle(ctx, e, t);
    // ambulance halo on the clay view
    for (const e of app.ents) {
      if (e.kind !== 'veh' || e.ref.type !== 'amb') continue;
      const ph = (t * 1.6) % 1;
      ctx.strokeStyle = hexA(Math.floor(t * 6) % 2 ? '#ff2d2d' : '#3d7bff', 0.5 * (1 - ph));
      ctx.lineWidth = 1.4 / app.cam.s;
      ctx.beginPath(); ctx.arc(e.x, e.y, 10 + ph * 34, 0, 7); ctx.stroke();
    }
    ctx.drawImage(app.layers.top, 0, 0, WORLD.w, WORLD.h);
    drawSignals(ctx);

    const hov = findEnt(app.hover), sel = findEnt(app.selected);
    if (hov && (!sel || hov.id !== sel.id)) brackets(ctx, hov, P.ink, 2.5, 1.2 / app.cam.s);
    if (sel) brackets(ctx, sel, P.accent, 3 + Math.sin(t * 5) * 0.8, 1.8 / app.cam.s);

    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    drawLabels(ctx);

    drawMachine(t);
    compositeLens(t);

    // leader line to the inspect card
    if (sel && !cardEl.hidden) {
      const [sx, sy] = w2s(sel.x, sel.y);
      const r = cardEl.getBoundingClientRect(), sr = stage.getBoundingClientRect();
      const cx = r.left - sr.left, cy = r.top - sr.top + 18;
      const tx = cx > sx ? cx : cx + r.width;
      ctx.strokeStyle = P.accent; ctx.lineWidth = 1;
      ctx.beginPath(); ctx.moveTo(sx, sy); ctx.lineTo(tx, cy); ctx.stroke();
      ctx.fillStyle = P.accent; ctx.beginPath(); ctx.arc(sx, sy, 2.4, 0, 7); ctx.fill();
    } else if (app.selected && !sel) {
      app.selected = null; cardEl.hidden = true;
    }
  }

  // ------------------------------------------------------------------ inspect card

  const cardEl = $('#card');
  function cardData(e) {
    const P = app.pal;
    if (e.kind === 'ped') {
      const p = e.ref;
      const j = nearestJunction(e.x, e.y);
      const dist = Math.hypot(j.x - e.x, j.y - e.y) / U;
      const waiting = !!p.waiting;
      const walk = p.waiting ? 'don’t walk' : p.crossing ? 'walk' : '—';
      return {
        tone: waiting ? P.amber : p.crossing ? P.accent : P.go, tag: 'Pedestrian · safe',
        status: waiting ? 'Waiting at the kerb' : p.crossing ? 'On the crosswalk' : 'On the sidewalk',
        id: idTag(e), title: waiting ? `Waiting ${p.wait.toFixed(0)} s for a walk phase` : `Walking at ${kmh(p.speed).toFixed(0)} km/h`,
        rows: [['class', 'person · 0.98'], ['speed', `${waiting ? 0 : kmh(p.speed).toFixed(0)} km/h`], ['signal', walk], ['range', `${dist.toFixed(0)} m from ${j.id} node`]],
        flags: [],
      };
    }
    if (e.kind === 'cow') {
      const h = e.ref;
      return {
        tone: P.amber, tag: 'Hazard · animal', status: 'Cattle on carriageway', id: idTag(e),
        title: `${h.stream.road.name}, ${h.stream.compass} kerb lane`,
        rows: [['class', 'animal · 0.94'], ['speed', '0 km/h'], ['blocking', 'lane 2 of 2'], ['on road', `${(app.sim.t - h.t0).toFixed(0)} s`], ['action', 'cattle squad paged']],
        flags: [],
      };
    }
    const v = e.ref;
    const st = v.stream;
    const sj = st.junctions[v.nextJ];
    const sig = sj ? S.signalOf(sj.j, sj.phase) : null;
    const sigTxt = sj ? `${{ G: 'green', Y: 'amber', R: 'red' }[sig]} · ${sj.j.id} in ${Math.max(0, (sj.stopS - v.s) / U).toFixed(0)} m` : 'clear · leaving network';
    let tone = P.go, status = 'Moving', tg = `${v.p.label} · tracked`;
    if (v.type === 'amb') { tone = P.emergency; status = app.sim.controller === 'adaptive' ? 'Green corridor active' : 'Emergency · no priority'; tg = 'Ambulance · 108'; }
    else if (v.flags.includes('Red-light jump')) { tone = P.stop; status = 'Red-light violation'; }
    else if (v.v < 0.5 * U) { tone = P.amber; status = sig === 'R' && sj && sj.stopS - v.s < 260 ? 'Waiting at red' : 'Stopped in queue'; }
    else if (v.v < 0.5 * v.vmax) { tone = P.amber; status = 'Slowing'; }
    return {
      tone, tag: tg, status, id: v.type === 'amb' ? v.callsign : idTag(e),
      title: `${st.road.name} ${st.compass} · ${kmh(v.v).toFixed(0)} km/h`,
      rows: [
        ['class', `${v.p.label.toLowerCase()} · ${v.conf.toFixed(2)}`],
        ['plate', v.plate],
        ['signal', sigTxt],
        ['stopped', `${v.stopped.toFixed(0)} s · ${v.stops} stop${v.stops === 1 ? '' : 's'}`],
        ['pcu', v.p.pcu.toFixed(1)],
      ],
      flags: v.flags,
    };
  }

  function updateCard(force) {
    const sel = findEnt(app.selected);
    if (!sel) { cardEl.hidden = true; return; }
    const now = performance.now();
    if (force || now - app.cardTick > 200) {
      app.cardTick = now;
      const d = cardData(sel);
      $('#card-head').style.setProperty('--tone', d.tone);
      $('#card-tag').textContent = d.tag;
      $('#card-status').textContent = d.status;
      $('#card-id').textContent = d.id;
      $('#card-title').textContent = d.title;
      $('#card-rows').innerHTML = d.rows.map(([k, v]) => `<dt>${k}</dt><dd>${v}</dd>`).join('');
      $('#card-flags').innerHTML = d.flags.map((f) => `<span class="flag">${f}</span>`).join('');
    }
    cardEl.hidden = false;
    const [sx, sy] = w2s(sel.x, sel.y);
    const cw = cardEl.offsetWidth, ch = cardEl.offsetHeight;
    let x = sx + 34, y = sy - ch / 2 - 30;
    if (x + cw > app.w - 12) x = sx - cw - 34;
    x = clamp(x, 12, app.w - cw - 12); y = clamp(y, 70, app.h - ch - 40);
    cardEl.style.transform = `translate(${x.toFixed(0)}px, ${y.toFixed(0)}px)`;
    cardEl.style.left = '0'; cardEl.style.top = '0';
  }

  function select(e) {
    app.selected = e ? { kind: e.kind, id: e.id } : null;
    updateCard(true);
  }

  // ------------------------------------------------------------------ panel UI

  const KPI = {
    avgDelay: { better: 'lower', fmt: (x) => x.toFixed(0) },
    throughput: { better: 'higher', fmt: (x) => x.toFixed(0) },
    queued: { better: 'lower', fmt: (x) => x.toFixed(0) },
    co2Rate: { better: 'lower', fmt: (x) => x.toFixed(0) },
  };

  function buildPanel() {
    const jl = $('#jlist');
    jl.innerHTML = app.sim.junctions.map((j) => `
      <button class="jrow" data-j="${j.id}">
        <canvas width="92" height="92"></canvas>
        <div style="min-width:0"><div class="jn"><em>${j.id}</em>${j.name}</div><div class="jd"></div></div>
        <div class="jt"><span class="jt-v">–</span><small class="jt-s"></small></div>
      </button>`).join('');
    jl.addEventListener('click', (ev) => {
      const b = ev.target.closest('.jrow');
      if (!b) return;
      const j = app.sim.junctions.find((x) => x.id === b.dataset.j);
      flyTo(j.x, j.y);
      if (app.w < 860) stage.scrollIntoView({ behavior: reduceMotion ? 'auto' : 'smooth' });
    });
    $('#amb-route').innerHTML = ROUTES.map(([id, n]) => `<option value="${id}">${n}</option>`).join('');
  }

  function drawJunctionGlyph(c, j) {
    const g = c.getContext('2d'), P = app.pal;
    const s = c.width / 46;
    g.setTransform(s, 0, 0, s, 0, 0);
    g.clearRect(0, 0, 46, 46);
    g.fillStyle = P.paper; rr(g, 0, 0, 46, 46, 9); g.fill();
    const col = (ph) => (j.phase === ph ? (j.state === 'G' ? P.go : j.state === 'Y' ? P.amber : P.stop) : P.stop);
    const qs = j.qByStream;
    const arm = (ph, dir, q) => {
      const len = 4 + Math.min(14, q * 0.8);
      g.fillStyle = col(ph);
      if (dir === 'W') g.fillRect(23 - 5 - len, 21, len, 4);
      if (dir === 'E') g.fillRect(23 + 5, 21, len, 4);
      if (dir === 'N') g.fillRect(21, 23 - 5 - len, 4, len);
      if (dir === 'S') g.fillRect(21, 23 + 5, 4, len);
    };
    const id = (r, c2) => (qs[`${r}-${c2}`] || 0);
    const h = j.h.id, v = j.v.id;
    arm(0, 'W', id(h, 'EB')); arm(0, 'E', id(h, 'WB'));
    arm(1, 'N', id(v, 'SB')); arm(1, 'S', id(v, 'NB'));
    g.fillStyle = P.ink; rr(g, 18, 18, 10, 10, 2); g.fill();
    if (j.preempt != null) { g.strokeStyle = P.emergency; g.lineWidth = 2; rr(g, 1, 1, 44, 44, 9); g.stroke(); }
  }

  function spark(c, a, b, better) {
    const g = c.getContext('2d'), P = app.pal;
    const dpr = app.dpr;
    const w = c.clientWidth, h = c.clientHeight;
    if (c.width !== Math.round(w * dpr)) { c.width = Math.round(w * dpr); c.height = Math.round(h * dpr); }
    g.setTransform(dpr, 0, 0, dpr, 0, 0);
    g.clearRect(0, 0, w, h);
    if (a.length < 2) return;
    const all = a.concat(b);
    let lo = Math.min(...all), hi = Math.max(...all);
    if (hi - lo < 1e-6) { hi += 1; lo -= 1; }
    const pad = 3;
    const X = (i) => (i / (Math.max(a.length, 2) - 1)) * (w - 6) + 1;
    const Y = (x) => h - pad - ((x - lo) / (hi - lo)) * (h - pad * 2);
    g.strokeStyle = hexA(P.muted, 0.15); g.lineWidth = 1;
    g.beginPath(); g.moveTo(0, h - 0.5); g.lineTo(w, h - 0.5); g.stroke();
    g.setLineDash([3, 2]); g.strokeStyle = P.muted; g.globalAlpha = 0.7; g.lineWidth = 1.2;
    g.beginPath(); b.forEach((x, i) => (i ? g.lineTo(X(i), Y(x)) : g.moveTo(X(i), Y(x)))); g.stroke();
    g.setLineDash([]); g.globalAlpha = 1;
    const grad = g.createLinearGradient(0, 0, 0, h);
    grad.addColorStop(0, hexA(P.accent, 0.22)); grad.addColorStop(1, hexA(P.accent, 0));
    g.beginPath(); a.forEach((x, i) => (i ? g.lineTo(X(i), Y(x)) : g.moveTo(X(i), Y(x))));
    g.lineTo(X(a.length - 1), h); g.lineTo(X(0), h); g.closePath(); g.fillStyle = grad; g.fill();
    g.strokeStyle = P.accent; g.lineWidth = 1.6;
    g.beginPath(); a.forEach((x, i) => (i ? g.lineTo(X(i), Y(x)) : g.moveTo(X(i), Y(x)))); g.stroke();
    g.fillStyle = P.accent; g.beginPath(); g.arc(X(a.length - 1), Y(a[a.length - 1]), 2.4, 0, 7); g.fill();
  }

  function recordHistory() {
    if (app.sim.t - app.lastHist < 5) return;
    app.lastHist = app.sim.t;
    app.hist.push({ a: S.metrics(app.sim, 180), b: S.metrics(app.twin, 180) });
    if (app.hist.length > 72) app.hist.shift();
  }

  function updatePanel() {
    const sim = app.sim;
    $('#clock').textContent = clockText(sim.t);
    $('#clock-sub').textContent = `IST · simulated · ${app.paused ? 'paused' : `×${app.speed}`}`;
    const ai = sim.controller === 'adaptive';
    $('#leg-a').textContent = ai ? 'Drishti AI (shown)' : 'Fixed-time (shown)';
    $('#leg-b').textContent = ai ? 'Fixed-time twin, same demand' : 'Drishti AI twin, same demand';
    $('#mode-note').textContent = ai
      ? 'Each junction reads queues from its cameras in PCU and decides every 50 ms: extend green, gap out, or switch. Ambulances get a green corridor.'
      : `A ${sim.junctions[0].fixedG[0]} s / ${sim.junctions[0].fixedG[1]} s timer runs whatever the traffic. This is how most Indian junctions work today.`;

    const m = S.metrics(sim, 180), mt = S.metrics(app.twin, 180);
    for (const el of document.querySelectorAll('.kpi')) {
      const k = el.dataset.k, cfg = KPI[k];
      const a = k === 'queued' ? m.queued : m[k], b = k === 'queued' ? mt.queued : mt[k];
      el.querySelector('strong').textContent = cfg.fmt(a);
      const diff = b ? ((a - b) / b) * 100 : 0;
      const good = cfg.better === 'lower' ? diff < -1 : diff > 1;
      const bad = cfg.better === 'lower' ? diff > 1 : diff < -1;
      const cls = good ? 'good' : bad ? 'bad' : '';
      el.querySelector('.cmp').innerHTML = `<b class="${cls}">${diff > 0 ? '+' : ''}${diff.toFixed(0)}%</b> vs twin ${cfg.fmt(b)}`;
      spark(el.querySelector('canvas'), app.hist.map((h) => h.a[k]), app.hist.map((h) => h.b[k]), cfg.better);
    }

    const rows = document.querySelectorAll('.jrow');
    sim.junctions.forEach((j, i) => {
      const row = rows[i];
      drawJunctionGlyph(row.querySelector('canvas'), j);
      row.querySelector('.jd').textContent = j.decision;
      const axis = j.phase === 0 ? 'E–W' : 'N–S';
      let v, sub;
      if (j.state === 'G') {
        if (sim.controller === 'fixed') { v = `${Math.max(0, j.fixedG[j.phase] - j.t).toFixed(0)}s`; sub = `${axis} green`; }
        else { v = `${j.t.toFixed(0)}s`; sub = `${axis} green`; }
      } else if (j.state === 'Y') { v = 'AMB'; sub = 'amber'; v = `${(S.YELLOW - j.t).toFixed(0)}s`; }
      else { v = '—'; sub = 'all red'; }
      row.querySelector('.jt-v').textContent = v;
      row.querySelector('.jt-s').textContent = sub;
    });

    // feed
    const evs = sim.events;
    const last = evs.length ? evs[evs.length - 1].seq : 0;
    if (last !== app.feedSeq) {
      app.feedSeq = last;
      const shown = evs.slice(-8).reverse();
      $('#feed').innerHTML = shown.map((e) => `
        <li data-sev="${e.sev}" data-kind="${e.kind}" data-ref="${e.ref ?? ''}" tabindex="0">
          <time>${clockText(e.t)}</time><span class="t">${e.text}</span><span class="s">${e.sub}</span>
        </li>`).join('');
      const v = sim.stats.violations;
      $('#feed-count').textContent = `${v.red} red-light · ${v.helmet} helmet · ${v.triple} triple`;
    }
  }

  function feedClick(ev) {
    const li = ev.target.closest('li');
    if (!li || !li.dataset.ref) return;
    const id = Number(li.dataset.ref);
    const e = app.ents.find((x) => x.id === id);
    if (e) { select(e); flyTo(e.x, e.y); }
  }

  // ------------------------------------------------------------------ benchmark

  const WORKER_SRC = `self.onmessage = (e) => { const r = self.DrishtiSim.runScenario(e.data); self.postMessage(r); };`;
  let workerP = null;
  function getRunner() {
    if (!workerP) {
      workerP = fetch('sim.js')
        .then((r) => { if (!r.ok) throw new Error('fetch'); return r.text(); })
        .then((src) => {
          const url = URL.createObjectURL(new Blob([src, '\n', WORKER_SRC], { type: 'text/javascript' }));
          const w = new Worker(url);
          return (job) => new Promise((res, rej) => {
            const to = setTimeout(() => rej(new Error('timeout')), 30000);
            w.onmessage = (e) => { clearTimeout(to); res(e.data); };
            w.onerror = (e) => { clearTimeout(to); rej(e); };
            w.postMessage(job);
          });
        })
        .catch(() => null);
    }
    return workerP.then((run) => run || ((job) => new Promise((res) => setTimeout(() => res(S.runScenario(job)), 20))));
  }

  async function runBench() {
    const btn = $('#bench'), prog = $('#bench-progress'), bar = prog.querySelector('i');
    btn.disabled = true; prog.hidden = false; bar.style.width = '0%';
    $('#bench-meta').textContent = 'running…';
    const seeds = [11, 23, 37];
    const jobs = [];
    for (const controller of ['fixed', 'adaptive']) for (const seed of seeds) jobs.push({ seed, controller, demand: app.demand, surge: app.rush, duration: 900 });
    let run = await getRunner();
    const out = { fixed: [], adaptive: [] };
    for (let i = 0; i < jobs.length; i++) {
      let r;
      try { r = await run(jobs[i]); }
      catch { run = (job) => Promise.resolve(S.runScenario(job)); r = await run(jobs[i]); }
      out[jobs[i].controller].push(r);
      bar.style.width = `${(((i + 1) / jobs.length) * 100).toFixed(0)}%`;
    }
    const mean = (rs, k) => rs.reduce((s, x) => s + x[k], 0) / rs.length;
    const rows = [
      ['Delay per vehicle', 'avgDelay', 's', 'lower', 1],
      ['Stops per vehicle', 'avgStops', '', 'lower', 2],
      ['Vehicles served', 'throughput', '/min', 'higher', 0],
      ['Idling CO₂', 'co2PerHour', 'kg/h', 'lower', 0],
      ['Pedestrian wait', 'pedWait', 's', 'lower', 1],
      ['Ambulance delay', 'ambDelay', 's', 'lower', 0],
    ];
    $('#bench-out').innerHTML = `
      <table class="bench-table">
        <thead><tr><th>Metric</th><th>Fixed</th><th>Drishti</th><th>Change</th></tr></thead>
        <tbody>${rows.map(([lbl, k, unit, better, dp]) => {
          const f = mean(out.fixed, k), a = mean(out.adaptive, k);
          const ch = f ? ((a - f) / f) * 100 : 0;
          const good = better === 'lower' ? ch < -1 : ch > 1, bad = better === 'lower' ? ch > 1 : ch < -1;
          return `<tr><td>${lbl}</td><td>${f.toFixed(dp)}${unit ? ` <small>${unit}</small>` : ''}</td><td>${a.toFixed(dp)}</td><td class="chg ${good ? 'good' : bad ? 'bad' : 'flat'}">${ch > 0 ? '+' : ''}${ch.toFixed(0)}%</td></tr>`;
        }).join('')}</tbody>
      </table>`;
    btn.disabled = false;
    $('#bench-meta').textContent = `demand ×${app.demand.toFixed(2)}${app.rush ? ' + rush' : ''} · run again`;
    setTimeout(() => { prog.hidden = true; }, 600);
  }

  // ------------------------------------------------------------------ world events

  function scheduleWorld() {
    const t = app.sim.t;
    const r = Math.random;
    if (t > app.nextAmbT) {
      const id = ROUTES[Math.floor(r() * ROUTES.length)][0];
      both((s) => S.dispatchAmbulance(s, id));
      app.nextAmbT = t + 160 + r() * 80;
    }
    if (t > app.nextCowT) {
      if (!app.sim.hazards.length) {
        const id = ROUTES[Math.floor(r() * ROUTES.length)][0];
        both((s) => S.addCattle(s, id));
      }
      app.nextCowT = t + 210 + r() * 90;
    }
  }

  // ------------------------------------------------------------------ input

  function bindInput() {
    const P = app.ptr;
    stage.addEventListener('pointerdown', (e) => {
      if (e.target !== canvas) return;
      app.touch = e.pointerType !== 'mouse' ? true : app.touch && e.pointerType !== 'mouse';
      document.body.classList.toggle('is-touch', app.touch);
      P.down = true; P.moved = false; P.id = e.pointerId;
      P.x0 = e.offsetX; P.y0 = e.offsetY; P.cx0 = app.cam.x; P.cy0 = app.cam.y;
      canvas.setPointerCapture(e.pointerId);
      app.cam.tx = null;
    });
    stage.addEventListener('pointermove', (e) => {
      if (e.pointerType === 'mouse' && app.touch) { app.touch = false; document.body.classList.remove('is-touch'); }
      const r = canvas.getBoundingClientRect();
      P.sx = e.clientX - r.left; P.sy = e.clientY - r.top; P.inside = true;
      if (e.pointerType === 'mouse') app.manualUntil = performance.now() + 4000;
      if (P.down && e.pointerId === P.id) {
        const dx = P.sx - P.x0, dy = P.sy - P.y0;
        if (Math.hypot(dx, dy) > 5) { P.moved = true; stage.classList.add('dragging'); }
        if (P.moved) {
          app.cam.x = P.cx0 - dx / app.cam.s; app.cam.y = P.cy0 - dy / app.cam.s; clampCam();
          app.manualUntil = performance.now() + (app.touch ? 9000 : 4000);
        }
      } else if (e.pointerType === 'mouse') {
        const h = hitTest(P.sx, P.sy);
        app.hover = h ? { kind: h.kind, id: h.id } : null;
        canvas.style.cursor = h ? 'pointer' : '';
      }
    });
    const end = (e) => {
      if (!P.down || e.pointerId !== P.id) return;
      P.down = false; stage.classList.remove('dragging');
      if (!P.moved) {
        const h = hitTest(P.sx, P.sy);
        select(h);
        if (app.touch) app.manualUntil = performance.now() + 9000;
      }
    };
    stage.addEventListener('pointerup', end);
    stage.addEventListener('pointercancel', end);
    stage.addEventListener('pointerleave', (e) => {
      if (e.pointerType === 'mouse') { P.inside = false; app.manualUntil = performance.now() + 1200; app.hover = null; }
    });
    stage.addEventListener('wheel', (e) => {
      e.preventDefault();
      zoomBy(Math.exp(-e.deltaY * 0.0015), e.offsetX, e.offsetY);
    }, { passive: false });
    $('#zoom-in').addEventListener('click', () => zoomBy(1.3));
    $('#zoom-out').addEventListener('click', () => zoomBy(1 / 1.3));
    $('#card-x').addEventListener('click', () => select(null));

    for (const b of document.querySelectorAll('.chip')) b.addEventListener('click', () => setMode(b.dataset.mode));
    for (const b of document.querySelectorAll('.mode button')) b.addEventListener('click', () => setController(b.dataset.ctrl));
    for (const b of document.querySelectorAll('.speed button')) b.addEventListener('click', () => setSpeed(Number(b.dataset.speed)));
    $('#act-amb').addEventListener('click', () => {
      const id = $('#amb-route').value;
      both((s) => S.dispatchAmbulance(s, id));
      app.nextAmbT = app.sim.t + 200;
      app.manualUntil = 0; app.tour.next = 0;
    });
    $('#act-cow').addEventListener('click', () => {
      const id = $('#amb-route').value;
      let hz = null;
      both((s) => { const h = S.addCattle(s, id); if (s === app.sim) hz = h; });
      app.nextCowT = app.sim.t + 240;
      const p = S.hazardPose(hz, app.sim.t + 3);
      flyTo(p.x, p.y);
      setTimeout(() => { const e = app.ents.find((x) => x.id === hz.id); if (e) select(e); }, 60);
    });
    $('#act-rush').addEventListener('click', (e) => {
      app.rush = !app.rush;
      both((s) => { s.surge = app.rush; });
      e.currentTarget.setAttribute('aria-pressed', String(app.rush));
    });
    $('#act-pause').addEventListener('click', togglePause);
    $('#demand').addEventListener('input', (e) => {
      app.demand = Number(e.target.value);
      both((s) => { s.demand = app.demand; });
      $('#demand-out').textContent = `×${app.demand.toFixed(2)}`;
    });
    $('#bench').addEventListener('click', runBench);
    $('#feed').addEventListener('click', feedClick);
    $('#feed').addEventListener('keydown', (e) => { if (e.key === 'Enter') feedClick(e); });

    window.addEventListener('keydown', (e) => {
      if (e.target.closest('input, select, textarea')) return;
      if (e.key === '1') setMode('lidar');
      else if (e.key === '2') setMode('seg');
      else if (e.key === '3') setMode('flow');
      else if (e.key === ' ' && !e.target.closest('button')) { e.preventDefault(); togglePause(); }
      else if (e.key === 'Escape') select(null);
    });
    new ResizeObserver(() => { resize(); }).observe(stage);
    const mq = matchMedia('(prefers-color-scheme: dark)');
    const retheme = () => { readPalette(); app.layers = {}; rebuildLayers(true); };
    mq.addEventListener('change', retheme);
    new MutationObserver(retheme).observe(document.documentElement, { attributes: true, attributeFilter: ['data-theme'] });
  }

  function setMode(m) {
    app.mode = m;
    for (const b of document.querySelectorAll('.chip')) b.setAttribute('aria-pressed', String(b.dataset.mode === m));
  }
  function setController(c) {
    app.sim.controller = c;
    app.twin.controller = c === 'adaptive' ? 'fixed' : 'adaptive';
    for (const b of document.querySelectorAll('.mode button')) b.setAttribute('aria-checked', String(b.dataset.ctrl === c));
    app.hist.length = 0;
  }
  function setSpeed(n) {
    app.speed = n;
    for (const b of document.querySelectorAll('.speed button')) b.setAttribute('aria-pressed', String(Number(b.dataset.speed) === n));
  }
  function togglePause() {
    app.paused = !app.paused;
    $('#act-pause').setAttribute('aria-pressed', String(app.paused));
    $('#pause-lbl').textContent = app.paused ? 'Resume' : 'Pause';
    $('#pause-glyph').setAttribute('d', app.paused ? 'M11 8.5l8 5.5-8 5.5z' : 'M11 9v10M17 9v10');
  }

  // ------------------------------------------------------------------ boot

  function boot() {
    app.sim = S.createSim({ seed: SEED, controller: 'adaptive' });
    app.twin = S.createSim({ seed: SEED, controller: 'fixed' });
    both((s) => S.advance(s, 140));
    both((s) => S.addCattle(s, 'mg-WB'));
    both((s) => S.advance(s, 8));
    both((s) => S.dispatchAmbulance(s, 'mg-EB'));
    both((s) => S.advance(s, 5));
    app.nextAmbT = app.sim.t + 150; app.nextCowT = app.sim.t + 200;
    for (let i = 0; i < 12; i++) { both((s) => S.advance(s, 5)); app.lastHist = -99; recordHistory(); }

    app.city = genCity();
    readPalette();
    document.body.classList.toggle('is-touch', app.touch);
    buildPanel();
    bindInput();
    resize();
    const amb = [...app.sim.vehicles.values()].find((v) => v.type === 'amb');
    const p = amb ? S.vehiclePose(amb) : { x: 470, y: 310 };
    app.cam.x = p.x + 200; app.cam.y = p.y + 120; clampCam();
    app.lens.x = p.x + 30; app.lens.y = p.y;

    let last = performance.now(), acc = 0;
    const frame = (now) => {
      const dt = Math.min(0.1, (now - last) / 1000);
      last = now;
      if (!app.paused) {
        acc += dt * app.speed;
        let n = 0;
        while (acc >= S.DT && n < 360) { S.step(app.sim); S.step(app.twin); acc -= S.DT; n++; }
        if (n >= 360) acc = 0;
        scheduleWorld();
        recordHistory();
      }
      draw(app.sim.t, now, dt);
      updateCard(false);
      if (now - app.lastUi > 250) { app.lastUi = now; updatePanel(); }
      requestAnimationFrame(frame);
    };
    requestAnimationFrame(frame);
    if (document.fonts && document.fonts.ready) document.fonts.ready.then(() => updatePanel());
  }

  boot();
})();
