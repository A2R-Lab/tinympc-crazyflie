#!/usr/bin/env python3
"""Build an inline comparison of banked-turn path and attitude tracking."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path

import numpy as np


def rows(path: Path) -> list[dict[str, str]]:
    with path.open(newline="") as stream:
        return list(csv.DictReader(stream))


def quaternion_rpy(qw: float, qx: float, qy: float, qz: float) -> tuple[float, float, float]:
    return (
        math.atan2(2 * (qw * qx + qy * qz), 1 - 2 * (qx * qx + qy * qy)),
        math.asin(float(np.clip(2 * (qw * qy - qz * qx), -1, 1))),
        math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz)),
    )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--trajectory", type=Path, required=True)
    parser.add_argument("--level", type=Path, required=True)
    parser.add_argument("--bank15", type=Path, required=True)
    parser.add_argument("--bank30", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()

    reference = []
    for row in rows(args.trajectory):
        roll, pitch, yaw = quaternion_rpy(*[float(row[key]) for key in ("qw", "qx", "qy", "qz")])
        reference.append({
            "t": round(float(row["t"]), 4), "x": round(float(row["x"]), 5),
            "y": round(float(row["y"]), 5), "roll": round(math.degrees(roll), 3),
            "pitch": round(math.degrees(pitch), 3), "yaw": round(math.degrees(yaw), 3),
            "tilt": round(math.degrees(math.acos(np.clip(math.cos(roll) * math.cos(pitch), -1, 1))), 3),
        })

    runs = []
    for label, path in (("Level", args.level), ("15° model", args.bank15), ("30° model", args.bank30)):
        values = []
        source = rows(path / "closed_loop.csv")
        for index, row in enumerate(source):
            if index % 5 and index != len(source) - 1:
                continue
            roll, pitch, yaw = [float(row[key]) for key in ("roll_rad", "pitch_rad", "yaw_rad")]
            values.append({
                "t": round(float(row["t"]), 4), "x": round(float(row["next_x"]), 5),
                "y": round(float(row["next_y"]), 5), "roll": round(math.degrees(roll), 3),
                "pitch": round(math.degrees(pitch), 3), "yaw": round(math.degrees(yaw), 3),
                "tilt": round(math.degrees(math.acos(np.clip(math.cos(roll) * math.cos(pitch), -1, 1))), 3),
            })
        runs.append({"label": label, "values": values, "maxTilt": round(max(value["tilt"] for value in values), 2)})

    data = json.dumps({"reference": reference, "runs": runs}, separators=(",", ":"))
    template = r'''<div id="bank-attitude-viz">
  <style>
    #bank-attitude-viz { color: var(--foreground); font-family: ui-sans-serif, system-ui, sans-serif; }
    #bank-attitude-viz h2 { margin: 0 0 6px; font-size: 18px; font-weight: 650; }
    #bank-attitude-viz .legend { display: flex; flex-wrap: wrap; gap: 6px 16px; margin: 0 0 10px; font-size: 12px; }
    #bank-attitude-viz .legend button { border: 0; background: transparent; color: var(--foreground); padding: 2px 0; cursor: pointer; }
    #bank-attitude-viz .legend button[aria-pressed="false"] { opacity: .38; }
    #bank-attitude-viz .swatch { display: inline-block; width: 14px; height: 3px; margin-right: 5px; vertical-align: middle; }
    #bank-attitude-viz .grid { display: grid; grid-template-columns: repeat(3,minmax(0,1fr)); gap: 12px; }
    #bank-attitude-viz .panel-title { font-size: 13px; font-weight: 600; margin: 0 0 3px; text-align: center; }
    #bank-attitude-viz svg { display: block; width: 100%; min-height: 300px; }
    #bank-attitude-viz text { fill: var(--foreground); font-size: 12px; }
    #bank-attitude-viz .axis path, #bank-attitude-viz .axis line, #bank-attitude-viz [data-chart-frame] { stroke: var(--border); }
    #bank-attitude-viz .tooltip { position: absolute; pointer-events: none; display: none; padding: 7px 9px; background: var(--popover); color: var(--popover-foreground); border: 1px solid var(--border); font-size: 12px; z-index: 5; }
    @media (max-width: 720px) { #bank-attitude-viz .grid { grid-template-columns: 1fr; } #bank-attitude-viz svg { min-height: 280px; } }
  </style>
  <h2>30° bank request — path and attitude tracking</h2>
  <div class="legend"></div>
  <div class="grid">
    <section><div class="panel-title">Top-down path · arrows show heading · marker size shows tilt</div><svg id="bank-path"></svg></section>
    <section><div class="panel-title">Roll tracking</div><svg id="bank-roll"></svg></section>
    <section><div class="panel-title">Yaw tracking</div><svg id="bank-yaw"></svg></section>
  </div>
  <div class="tooltip" role="tooltip"></div>
  <script src="https://cdn.jsdelivr.net/npm/d3@7.9.0/dist/d3.min.js"></script>
  <script>
  (() => {
    const root=document.getElementById('bank-attitude-viz'), data=__DATA__;
    const colors=['var(--viz-series-1)','var(--viz-series-2)','var(--viz-series-3)'];
    const series=[{label:'Reference',values:data.reference,color:'var(--viz-series-4)',reference:true},...data.runs.map((d,i)=>({...d,color:colors[i]}))];
    const visible=new Set(series.map(d=>d.label)), legend=d3.select(root).select('.legend'), tip=d3.select(root).select('.tooltip');
    legend.selectAll('button').data(series).join('button').attr('type','button').attr('aria-pressed','true').html(d=>`<span class="swatch" style="background:${d.color}"></span>${d.label}${d.maxTilt?` · max tilt ${d.maxTilt}°`:''}`).on('click',function(e,d){visible.has(d.label)?visible.delete(d.label):visible.add(d.label);d3.select(this).attr('aria-pressed',visible.has(d.label));drawAll();});
    function setup(id){const el=root.querySelector(id),w=Math.max(330,el.getBoundingClientRect().width),h=310,m={t:12,r:18,b:48,l:62};return {svg:d3.select(el).attr('viewBox',`0 0 ${w} ${h}`),w,h,m};}
    function axes(c,x,y,xlabel,ylabel){c.svg.selectAll('*').remove();c.svg.append('rect').attr('data-chart-frame','').attr('x',c.m.l).attr('y',c.m.t).attr('width',c.w-c.m.l-c.m.r).attr('height',c.h-c.m.t-c.m.b).attr('fill','none');c.svg.append('g').attr('class','axis').attr('transform',`translate(0,${c.h-c.m.b})`).call(d3.axisBottom(x).ticks(c.w<420?4:6));c.svg.append('g').attr('class','axis').attr('transform',`translate(${c.m.l},0)`).call(d3.axisLeft(y).ticks(5));c.svg.append('text').attr('class','axis-title').attr('data-axis','x').attr('x',(c.m.l+c.w-c.m.r)/2).attr('y',c.h-8).attr('text-anchor','middle').text(xlabel);c.svg.append('text').attr('class','axis-title').attr('data-axis','y').attr('transform','rotate(-90)').attr('x',-(c.m.t+c.h-c.m.b)/2).attr('y',16).attr('text-anchor','middle').text(ylabel);}
    function pathPlot(){const c=setup('#bank-path'),all=series.flatMap(d=>d.values),xe=d3.extent(all,d=>d.x),ye=d3.extent(all,d=>d.y),pad=.12,x=d3.scaleLinear().domain([xe[0]-pad,xe[1]+pad]).range([c.m.l,c.w-c.m.r]),y=d3.scaleLinear().domain([ye[0]-pad,ye[1]+pad]).range([c.h-c.m.b,c.m.t]);axes(c,x,y,'x [m]','y [m]');series.filter(s=>visible.has(s.label)).forEach(s=>{c.svg.append('path').datum(s.values).attr('fill','none').attr('stroke',s.color).attr('stroke-width',s.reference?2:2.5).attr('stroke-dasharray',s.reference?'6 4':null).attr('d',d3.line().x(d=>x(d.x)).y(d=>y(d.y)));if(!s.reference){const marks=s.values.filter((d,i)=>i%Math.max(1,Math.floor(s.values.length/9))===0);c.svg.selectAll(`.m-${s.label}`).data(marks).join('circle').attr('cx',d=>x(d.x)).attr('cy',d=>y(d.y)).attr('r',d=>3+d.tilt/10).attr('fill',s.color).attr('fill-opacity',.28).attr('stroke',s.color);c.svg.selectAll(`.h-${s.label}`).data(marks).join('line').attr('x1',d=>x(d.x)).attr('y1',d=>y(d.y)).attr('x2',d=>x(d.x+.12*Math.cos(d.yaw*Math.PI/180))).attr('y2',d=>y(d.y+.12*Math.sin(d.yaw*Math.PI/180))).attr('stroke',s.color).attr('stroke-width',2);}});}
    function timePlot(id,key,ylabel){const c=setup(id),all=series.flatMap(d=>d.values),x=d3.scaleLinear().domain(d3.extent(all,d=>d.t)).nice().range([c.m.l,c.w-c.m.r]),ext=d3.extent(all,d=>d[key]),p=Math.max(3,(ext[1]-ext[0])*.08),y=d3.scaleLinear().domain([ext[0]-p,ext[1]+p]).nice().range([c.h-c.m.b,c.m.t]);axes(c,x,y,'time [s]',ylabel);series.filter(s=>visible.has(s.label)).forEach(s=>c.svg.append('path').datum(s.values).attr('fill','none').attr('stroke',s.color).attr('stroke-width',s.reference?2:2.4).attr('stroke-dasharray',s.reference?'6 4':null).attr('d',d3.line().x(d=>x(d.t)).y(d=>y(d[key]))));const overlay=c.svg.append('rect').attr('data-chart-hit','').attr('data-chart-hover-overlay','cross-series').attr('x',c.m.l).attr('y',c.m.t).attr('width',c.w-c.m.l-c.m.r).attr('height',c.h-c.m.t-c.m.b).attr('fill','transparent');const guide=c.svg.append('line').attr('data-chart-hover-guide','').attr('y1',c.m.t).attr('y2',c.h-c.m.b).attr('stroke','var(--foreground)').attr('stroke-opacity',.35).style('display','none');overlay.on('mousemove',e=>{const [px]=d3.pointer(e),tv=x.invert(px),rows=series.filter(s=>visible.has(s.label)).map(s=>{const i=d3.bisector(d=>d.t).center(s.values,tv),d=s.values[i];return `${s.label}: ${d[key].toFixed(1)}° · pitch ${d.pitch.toFixed(1)}° · tilt ${d.tilt.toFixed(1)}°`;});guide.attr('x1',px).attr('x2',px).style('display',null);tip.style('display','block').style('left',`${e.pageX-root.getBoundingClientRect().left+12}px`).style('top',`${e.pageY-root.getBoundingClientRect().top+12}px`).html(rows.join('<br>'));}).on('mouseleave',()=>{guide.style('display','none');tip.style('display','none');});}
    function drawAll(){pathPlot();timePlot('#bank-roll','roll','roll [deg]');timePlot('#bank-yaw','yaw','yaw [deg]');}
    new ResizeObserver(drawAll).observe(root);drawAll();
  })();
  </script>
</div>'''
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(template.replace("__DATA__", data))
    print(f"wrote {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
