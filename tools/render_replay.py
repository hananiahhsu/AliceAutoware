#!/usr/bin/env python3
import csv
import json
import pathlib
import sys

LANE_WIDTH = 3.7
LANES = [-LANE_WIDTH, 0.0, LANE_WIDTH]


def load_rows(csv_path: pathlib.Path):
    rows = []
    with csv_path.open('r', encoding='utf-8') as fp:
        reader = csv.DictReader(fp)
        for row in reader:
            actors = []
            if row['actors']:
                for token in row['actors'].split('|'):
                    parts = token.split(':')
                    actor_id, x, y, speed, lane = parts[:5]
                    behavior = parts[5] if len(parts) > 5 else 'cruise'
                    actors.append({
                        'id': int(actor_id),
                        'x': float(x),
                        'y': float(y),
                        'speed': float(speed),
                        'lane': int(lane),
                        'behavior': behavior,
                    })
            rows.append({
                'time': float(row['time']),
                'ego_x': float(row['ego_x']),
                'ego_y': float(row['ego_y']),
                'ego_speed': float(row['ego_speed']),
                'decision': row['decision'],
                'target_lane': int(row['target_lane']),
                'collided': int(row['collided']),
                'actors': actors,
            })
    return rows


def main():
    if len(sys.argv) != 3:
        print('Usage: render_replay.py <input.csv> <output.html>')
        return 1

    input_path = pathlib.Path(sys.argv[1])
    output_path = pathlib.Path(sys.argv[2])
    rows = load_rows(input_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    html_template = """<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8" />
<title>Replay - __TITLE__</title>
<style>
body { font-family: Arial, sans-serif; margin: 0; background: #111; color: #eee; }
header { padding: 12px 16px; background: #1b1b1b; }
#hud { padding: 8px 16px; display: flex; gap: 24px; flex-wrap: wrap; }
canvas { display: block; margin: 0 auto; background: #202020; border: 1px solid #444; }
button { margin: 8px 16px; padding: 8px 14px; }
</style>
</head>
<body>
<header><h2>Momenta Autodrive Stack Replay</h2></header>
<div id="hud">
  <div id="time"></div>
  <div id="decision"></div>
  <div id="speed"></div>
  <div id="collision"></div>
</div>
<button onclick="togglePlay()">Play / Pause</button>
<canvas id="view" width="1200" height="420"></canvas>
<script>
const frames = __FRAMES__;
const laneCenters = __LANES__;
const canvas = document.getElementById('view');
const ctx = canvas.getContext('2d');
let playing = true;
let index = 0;

function worldToCanvas(x, y, egoX) {
  const px = (x - egoX) * 8 + 250;
  const py = 210 - y * 40;
  return [px, py];
}

function drawVehicle(x, y, egoX, isEgo) {
  const [px, py] = worldToCanvas(x, y, egoX);
  ctx.fillStyle = isEgo ? '#00d084' : '#f2c94c';
  ctx.fillRect(px - 20, py - 10, 40, 20);
}

function draw() {
  const frame = frames[index];
  ctx.clearRect(0, 0, canvas.width, canvas.height);
  ctx.fillStyle = '#2a2a2a';
  ctx.fillRect(0, 0, canvas.width, canvas.height);
  ctx.strokeStyle = '#666';
  ctx.lineWidth = 2;
  laneCenters.forEach((laneY) => {
    const py = worldToCanvas(frame.ego_x, laneY, frame.ego_x)[1];
    ctx.setLineDash([12, 10]);
    ctx.beginPath();
    ctx.moveTo(0, py);
    ctx.lineTo(canvas.width, py);
    ctx.stroke();
  });
  ctx.setLineDash([]);
  drawVehicle(frame.ego_x, frame.ego_y, frame.ego_x, true);
  frame.actors.forEach(actor => drawVehicle(actor.x, actor.y, frame.ego_x, false));
  document.getElementById('time').innerText = 'time: ' + frame.time.toFixed(2) + ' s';
  document.getElementById('decision').innerText = 'decision: ' + frame.decision + ' -> lane ' + frame.target_lane;
  document.getElementById('speed').innerText = 'ego speed: ' + frame.ego_speed.toFixed(2) + ' m/s';
  document.getElementById('collision').innerText = 'collision: ' + (frame.collided ? 'YES' : 'NO');
}

function tick() {
  if (playing) {
    index = Math.min(index + 1, frames.length - 1);
  }
  draw();
  requestAnimationFrame(tick);
}

function togglePlay() { playing = !playing; }

draw();
requestAnimationFrame(tick);
</script>
</body>
</html>
"""
    html_text = html_template.replace('__TITLE__', input_path.stem).replace('__FRAMES__', json.dumps(rows)).replace('__LANES__', json.dumps(LANES))
    output_path.write_text(html_text, encoding='utf-8')
    print(f'[MAD] replay written to: {output_path}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
