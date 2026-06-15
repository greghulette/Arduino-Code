#pragma once
// HTML for the SBUS Controller web UI.
// Kept in a separate header so the Arduino preprocessor does not scan
// the raw-string contents looking for function prototypes (which causes
// false-positive matches on JS "function name() { ... }" patterns).

static const char HTML[] PROGMEM = R"rawhtml(<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1.0,user-scalable=no">
<title>SBUS Controller</title>
<style>
  :root {
    --bg:       #0d0f14;
    --panel:    #161920;
    --border:   #2a2d38;
    --accent:   #4fc3f7;
    --green:    #66bb6a;
    --red:      #ef5350;
    --yellow:   #ffa726;
    --text:     #e0e4f0;
    --muted:    #6b7280;
    --stick-bg: #1a1d28;
    --stick-rim:#2e3450;
    --thumb:    #4fc3f7;
  }
  *{box-sizing:border-box;margin:0;padding:0;}
  body{
    background:var(--bg);color:var(--text);
    font-family:'Segoe UI',system-ui,sans-serif;
    min-height:100vh;display:flex;flex-direction:column;
    align-items:center;padding:12px;gap:10px;
  }

  header{
    width:100%;max-width:960px;
    display:flex;align-items:center;justify-content:space-between;
    flex-wrap:wrap;gap:8px;
  }
  .logo{font-size:1.05rem;font-weight:700;letter-spacing:.1em;color:var(--accent);}
  #connBadge{
    font-size:.7rem;font-weight:600;letter-spacing:.06em;
    padding:4px 12px;border-radius:20px;
    background:var(--panel);border:1px solid var(--red);color:var(--red);
    transition:all .3s;
  }
  #connBadge.connected{border-color:var(--green);color:var(--green);}
  .hdr-btn{
    font-size:.7rem;font-weight:700;letter-spacing:.07em;
    padding:4px 12px;border-radius:20px;cursor:pointer;
    background:var(--panel);border:1px solid var(--border);color:var(--muted);
    transition:all .2s;user-select:none;
  }
  .hdr-btn.active-16{border-color:var(--yellow);color:var(--yellow);}
  .hdr-btn.active-24{border-color:var(--accent);color:var(--accent);}
  .hdr-btn.dbg-on   {border-color:var(--green);color:var(--green);}
  /* Transport picker (WiFi / USB Serial dropdown) — popover-style menu.
     position:fixed because it's placed from getBoundingClientRect() viewport
     coords; absolute would drift by the scroll offset on a scrolled page. */
  .tx-menu{
    position:fixed;display:none;flex-direction:column;
    background:var(--panel);border:1px solid var(--border);border-radius:8px;
    padding:4px;min-width:130px;z-index:300;box-shadow:0 6px 24px rgba(0,0,0,.5);
  }
  .tx-menu.show{display:flex;}
  .tx-menu button{
    appearance:none;background:none;border:none;color:var(--text);
    padding:7px 12px;text-align:left;cursor:pointer;font-size:.7rem;
    font-weight:600;letter-spacing:.05em;border-radius:4px;font-family:inherit;
  }
  .tx-menu button:hover{background:var(--bg);color:var(--accent);}
  .tx-menu button.active{color:var(--accent);}
  .tx-menu .tx-note{
    font-size:.58rem;color:var(--muted);padding:4px 12px 6px;letter-spacing:.04em;
  }

  .sw-pyramid{
    display:flex;flex-direction:column;align-items:center;gap:5px;flex-shrink:0;
  }
  .sw-pair{
    display:flex;flex-direction:row;gap:5px;
  }
  .sw-card{
    background:var(--panel);border:1px solid var(--border);border-radius:10px;
    padding:7px 6px;display:flex;flex-direction:column;align-items:center;gap:4px;
    width:72px;
  }
  .sw-name{font-size:.62rem;font-weight:700;letter-spacing:.1em;color:var(--accent);text-transform:uppercase;}
  .sw-ch  {font-size:.55rem;color:var(--muted);}
  .sw-toggle{display:flex;flex-direction:column;gap:3px;width:100%;}
  .sw-seg{
    padding:5px 0;width:100%;border-radius:5px;cursor:pointer;
    border:1px solid var(--border);background:var(--bg);
    color:var(--muted);font-size:.68rem;font-weight:600;text-align:center;
    transition:all .12s;user-select:none;
  }
  .sw-seg.sel{background:var(--accent);color:#000;border-color:var(--accent);}
  .sw-seg:hover:not(.sel){border-color:var(--accent);color:var(--text);}
  .sw-mom{
    padding:10px 0;width:100%;border-radius:6px;cursor:pointer;
    border:1px solid var(--border);background:var(--bg);
    color:var(--text);font-size:.75rem;font-weight:700;text-align:center;
    user-select:none;touch-action:none;transition:all .1s;
  }
  .sw-mom.held{background:var(--accent);color:#000;border-color:var(--accent);}

  .ctrl-area{
    width:100%;max-width:960px;
    display:flex;gap:6px;align-items:flex-start;justify-content:center;flex-wrap:wrap;
  }

  .ts-col{
    display:flex;flex-direction:column;align-items:center;gap:8px;
    flex:0 0 auto;
  }

  .trim-widget{
    background:var(--panel);border:1px solid var(--border);border-radius:8px;
    padding:7px 8px;display:flex;flex-direction:column;align-items:center;gap:3px;
  }
  .trim-lbl{font-size:.58rem;font-weight:700;color:var(--accent);letter-spacing:.08em;}
  .trim-ch {font-size:.52rem;color:var(--muted);}
  .trim-btn{
    width:40px;height:26px;border-radius:4px;cursor:pointer;
    border:1px solid var(--border);background:var(--bg);color:var(--text);
    font-size:.8rem;font-weight:700;user-select:none;touch-action:none;
    transition:background .1s,border-color .1s;
  }
  .trim-btn:active,.trim-btn.held{background:var(--accent);color:#000;border-color:var(--accent);}
  .trim-val{
    font-size:.65rem;font-variant-numeric:tabular-nums;
    color:var(--text);min-width:40px;text-align:center;
  }
  .trim-rst{
    width:40px;height:16px;border-radius:3px;cursor:pointer;
    border:1px solid var(--border);background:transparent;color:var(--muted);
    font-size:.52rem;font-weight:700;
  }

  .slider-widget{
    background:var(--panel);border:1px solid var(--border);border-radius:8px;
    padding:8px;display:flex;flex-direction:column;align-items:center;gap:5px;
  }
  .slider-lbl{font-size:.6rem;font-weight:700;color:var(--accent);letter-spacing:.08em;}
  .slider-ch {font-size:.52rem;color:var(--muted);}
  .slider-wrap{width:40px;height:160px;display:flex;align-items:center;justify-content:center;overflow:visible;}
  .slider-inp{
    width:140px;
    transform:rotate(-90deg);transform-origin:center;
    cursor:pointer;accent-color:var(--accent);
  }
  .slider-val{font-size:.65rem;color:var(--muted);font-variant-numeric:tabular-nums;}

  .stick-block{
    display:flex;flex-direction:column;align-items:center;gap:8px;flex:0 0 auto;
  }
  .stick-card{
    background:var(--panel);border:1px solid var(--border);border-radius:12px;
    padding:12px;display:flex;flex-direction:column;align-items:center;gap:7px;
  }
  .stick-lbl{font-size:.62rem;font-weight:700;letter-spacing:.09em;color:var(--muted);text-transform:uppercase;}
  .stick-wrap{
    position:relative;width:min(220px,38vw);aspect-ratio:1;
    border-radius:50%;background:var(--stick-bg);border:2px solid var(--stick-rim);
    touch-action:none;user-select:none;cursor:crosshair;
  }
  .stick-wrap::before,.stick-wrap::after{content:'';position:absolute;background:var(--stick-rim);}
  .stick-wrap::before{width:1px;height:100%;left:50%;top:0;}
  .stick-wrap::after {width:100%;height:1px;top:50%;left:0;}
  .thumb{
    position:absolute;width:36px;height:36px;border-radius:50%;
    background:radial-gradient(circle at 35% 35%,#7fd6ff,var(--thumb));
    box-shadow:0 0 10px rgba(79,195,247,.4);
    transform:translate(-50%,-50%);pointer-events:none;
  }
  .stick-readout{font-size:.65rem;color:var(--muted);font-variant-numeric:tabular-nums;white-space:nowrap;}

  .trim-row{display:flex;gap:8px;justify-content:center;}
  .trim-h{
    background:var(--panel);border:1px solid var(--border);border-radius:8px;
    padding:6px 8px;display:flex;flex-direction:column;align-items:center;gap:3px;
  }
  .trim-h-btns{display:flex;gap:4px;align-items:center;}

  .stick-center{
    display:flex;flex-direction:column;align-items:center;gap:6px;flex:0 0 auto;
  }
  .sticks-row{
    display:flex;gap:10px;align-items:flex-start;
  }

  .pot-row{
    display:flex;gap:12px;justify-content:center;flex-wrap:wrap;width:100%;
  }
  .pot-widget{
    background:var(--panel);border:1px solid var(--border);border-radius:8px;
    padding:8px 16px;display:flex;flex-direction:column;align-items:center;gap:5px;
    min-width:160px;
  }
  .pot-lbl{font-size:.62rem;font-weight:700;color:var(--accent);letter-spacing:.08em;}
  .pot-ch {font-size:.52rem;color:var(--muted);}
  .pot-inp{width:180px;cursor:pointer;accent-color:var(--accent);}
  .pot-val{font-size:.65rem;color:var(--muted);font-variant-numeric:tabular-nums;}

  /* ── Transmitter SVG (Phase-1 X18 visualisation) ──────────────────────── */
  /* touch-action:pan-y on the wrap + svg lets the user scroll the page
     vertically by dragging on the transmitter background (and two-finger
     pans/pinches still pan).  The INTERACTIVE elements below override this
     with touch-action:none so finger drags there are gestures, not scrolls
     — the browser commits to "no scroll for this pointer" the instant the
     touch starts on an element with touch-action:none, so the joystick
     drag is never reinterpreted as a scroll mid-gesture. */
  .tx-svg-wrap{max-width:900px;margin:0 auto;width:100%;touch-action:pan-y;}
  .tx-svg-wrap svg{width:100%;height:auto;display:block;touch-action:pan-y;
                   -webkit-user-select:none;user-select:none;
                   -webkit-touch-callout:none;}
  /* Interactive SVG hit-targets: must NOT permit page scroll while dragging. */
  .svg-btn,.svg-tr,.svg-sw,.svg-knob,.svg-slider,.svg-stick,
  .stick-hit,.svg-lua{touch-action:none;-webkit-user-select:none;user-select:none;
                      -webkit-touch-callout:none;}
  .svg-btn,.svg-tr,.svg-sw,.svg-knob,.svg-slider,.svg-lua{cursor:pointer;}
  .svg-btn:hover>circle:first-of-type,
  .svg-tr:hover>rect:first-of-type,
  .svg-sw:hover>circle:first-of-type{opacity:.7;}
  .svg-btn.pressed>circle:first-of-type{fill:#4fc3f7 !important;stroke:#fff !important;}
  .svg-tr.pressed>rect:first-of-type{fill:#4fc3f7 !important;stroke:#fff !important;}
  .svg-tr.pressed>rect:nth-of-type(2),
  .svg-tr.pressed>polygon{fill:#000 !important;}
  /* Switch lever rotation is applied via JS using SVG transform="rotate(deg,cx,cy)"
     so each switch pivots around its own bushing centre.  See svgSwCycle() in JS. */
  .svg-sw .lever{transition:transform .15s ease;}

  /* The old switch/trim/button/stick/slider/pot widgets are replaced by the
     SVG.  We keep the elements in the DOM (state-tracking still runs against
     them) but hide them visually so the SVG is the sole control surface. */
  #swPyramidLeft, #swPyramidRight, #trimBank, #btnBank,
  #slLSCol, #slRSCol, .sticks-row, #potRow { display:none !important; }
  .ctrl-area { display:none !important; }   /* the whole legacy row */

  /* X18-style trim layout (legacy — used only as a fallback if SVG hidden):
       row 1 (vertical): T3 ……… T2
       row 2 (horizontal): T4  T5    T6  T1                                  */
  .trim-bank{
    display:flex;flex-direction:column;gap:10px;align-items:center;width:100%;
  }
  .trim-row{display:flex;gap:14px;justify-content:center;align-items:center;flex-wrap:wrap;}
  .trim-row-top{gap:120px;}                 /* big gap between T3 and T2 */
  .trim-row-bot{gap:14px;}
  .trim-gap{width:80px;}                    /* mid-gap in bottom row (under sticks) */
  .trim-pair{display:flex;gap:6px;}          /* T4+T5 or T6+T1 sub-pair */
  .trim-widget.horizontal{flex-direction:row;align-items:center;gap:6px;padding:5px 8px;}
  .trim-widget.horizontal .trim-btn{width:30px;height:30px;}

  .lua-section{width:100%;max-width:960px;}
  .lua-header{margin-bottom:6px;}
  .lua-grid{display:grid;grid-template-columns:repeat(5,1fr);gap:7px;}
  @media(max-width:600px){.lua-grid{grid-template-columns:repeat(3,1fr);}}
  .lua-btn{
    --btn-color: #4fc3f7;
    padding:14px 6px;
    border:1px solid var(--border);border-radius:8px;
    background:#1a1d28;color:var(--text);
    font-size:.75rem;font-weight:600;cursor:pointer;text-align:center;word-break:break-word;
    transition:border-color .12s,background .12s,color .12s,transform .1s;
    -webkit-tap-highlight-color:transparent;user-select:none;touch-action:none;
  }
  .lua-btn:not(.unassigned){background:var(--btn-color);color:#000;border-color:var(--btn-color);}
  .lua-btn.unassigned{background:transparent;border-color:var(--btn-color);border-style:dashed;color:var(--muted);opacity:.55;cursor:default;}
  .lua-btn.pressed{filter:brightness(.75);transform:scale(.96);}
  .lua-btn:not(.unassigned):hover{filter:brightness(1.15);}

  /* X18-style matrix button layout:
       S1/S2/S3 col  |  S4/S5/S6 col   (RB1/RB2 promoted to SI/SJ switches)*/
  .btn-bank{
    display:flex;gap:24px;justify-content:center;align-items:center;width:100%;flex-wrap:wrap;
  }
  .btn-col{display:flex;flex-direction:column;gap:6px;}
  .btn-group{display:flex;gap:6px;flex-wrap:wrap;justify-content:center;}
  .sep{width:1px;background:var(--border);align-self:stretch;margin:0 4px;}
  .ctrl-btn{
    padding:12px 8px;min-width:56px;
    border:1px solid var(--border);border-radius:8px;
    background:#1a1d28;color:var(--text);
    font-size:.75rem;font-weight:600;cursor:pointer;text-align:center;
    transition:all .12s;user-select:none;touch-action:none;
    -webkit-tap-highlight-color:transparent;
  }
  .ctrl-btn.unassigned{color:var(--muted);border-style:dashed;cursor:default;}
  .ctrl-btn.pressed{background:var(--accent);color:#000;border-color:var(--accent);}
  .ctrl-btn:not(.unassigned):hover{border-color:var(--accent);}

  .settings-wrap{width:100%;max-width:960px;}
  details{width:100%;}
  summary{
    cursor:pointer;font-size:.7rem;font-weight:700;letter-spacing:.08em;text-transform:uppercase;
    color:var(--muted);padding:9px 14px;
    background:var(--panel);border:1px solid var(--border);border-radius:10px;
    list-style:none;display:flex;align-items:center;gap:8px;user-select:none;
  }
  summary::-webkit-details-marker{display:none;}
  summary::before{content:'▶';transition:transform .2s;font-size:.58rem;}
  details[open] summary{border-radius:10px 10px 0 0;}
  details[open] summary::before{transform:rotate(90deg);}
  .settings-body{
    background:var(--panel);border:1px solid var(--border);border-top:none;
    border-radius:0 0 10px 10px;padding:14px;display:flex;flex-direction:column;gap:14px;
  }
  .sec-title{font-size:.62rem;font-weight:700;letter-spacing:.1em;text-transform:uppercase;color:var(--muted);margin-bottom:4px;}
  .cfg-table{width:100%;border-collapse:collapse;font-size:.74rem;}
  .cfg-table th{font-size:.6rem;font-weight:700;letter-spacing:.07em;text-transform:uppercase;
    color:var(--muted);padding:4px 6px;text-align:left;border-bottom:1px solid var(--border);}
  .cfg-table td{padding:4px 6px;vertical-align:middle;}
  .cfg-table tr:nth-child(even) td{background:rgba(255,255,255,.02);}
  .cfg-table input,.cfg-table select{
    background:var(--bg);border:1px solid var(--border);border-radius:4px;
    color:var(--text);padding:4px 6px;font-size:.72rem;width:100%;
  }
  .cfg-table input:focus,.cfg-table select:focus{outline:none;border-color:var(--accent);}
  .axis-grid{display:grid;grid-template-columns:repeat(4,1fr);gap:8px;}
  .axis-item{display:flex;flex-direction:column;gap:4px;}
  .axis-item label{font-size:.65rem;color:var(--muted);}
  .axis-item select{background:var(--bg);border:1px solid var(--border);border-radius:4px;
    color:var(--text);padding:5px 6px;font-size:.74rem;}
  .save-btn{
    padding:8px 20px;border-radius:8px;
    border:1px solid var(--accent);background:transparent;color:var(--accent);
    font-size:.78rem;font-weight:700;cursor:pointer;transition:all .15s;
  }
  .save-btn:hover{background:var(--accent);color:#000;}
  .cfg-io-row{display:flex;gap:8px;align-items:center;flex-wrap:wrap;justify-content:flex-end;}
  .io-btn{
    padding:8px 16px;border-radius:8px;font-size:.78rem;font-weight:700;cursor:pointer;transition:all .15s;
  }
  .io-btn.export{border:1px solid var(--green);background:transparent;color:var(--green);}
  .io-btn.export:hover{background:var(--green);color:#000;}
  .io-btn.import{border:1px solid var(--yellow);background:transparent;color:var(--yellow);}
  .io-btn.import:hover{background:var(--yellow);color:#000;}
  #importStatus{font-size:.72rem;color:var(--muted);}

  .wifi-status{font-size:.72rem;color:var(--muted);display:flex;gap:14px;flex-wrap:wrap;margin-bottom:8px;}
  .wifi-status b{color:var(--text);}
  .wifi-net-row{display:grid;grid-template-columns:auto 1fr 1fr auto auto auto;gap:6px;align-items:center;margin-bottom:5px;}
  .wifi-net-row .drag-num{font-size:.65rem;color:var(--muted);text-align:center;width:18px;}
  .wifi-net-row input{background:var(--bg);border:1px solid var(--border);border-radius:4px;color:var(--text);padding:4px 6px;font-size:.72rem;}
  .wifi-net-row input:focus{outline:none;border-color:var(--accent);}
  .wifi-pass-wrap{position:relative;display:flex;}
  .wifi-pass-wrap input{flex:1;padding-right:28px;}
  .wifi-pass-wrap button{position:absolute;right:4px;top:50%;transform:translateY(-50%);
    background:none;border:none;color:var(--muted);cursor:pointer;font-size:.8rem;padding:0;}
  .wifi-arrow{background:none;border:1px solid var(--border);border-radius:4px;color:var(--muted);
    cursor:pointer;font-size:.7rem;padding:3px 6px;line-height:1;}
  .wifi-arrow:hover{border-color:var(--accent);color:var(--accent);}
  .wifi-del{background:none;border:1px solid var(--border);border-radius:4px;color:var(--muted);
    cursor:pointer;font-size:.7rem;padding:3px 6px;}
  .wifi-del:hover{border-color:var(--red);color:var(--red);}
  .wifi-add{background:none;border:1px dashed var(--border);border-radius:6px;color:var(--muted);
    cursor:pointer;font-size:.72rem;padding:5px;width:100%;margin-top:4px;transition:all .15s;}
  .wifi-add:hover{border-color:var(--accent);color:var(--accent);}
  .wifi-footer{display:flex;gap:8px;align-items:center;margin-top:10px;flex-wrap:wrap;}
  .wifi-footer label{font-size:.7rem;color:var(--muted);}
  .wifi-footer select{background:var(--bg);border:1px solid var(--border);border-radius:4px;
    color:var(--text);padding:4px 8px;font-size:.72rem;}
  .wifi-apply{padding:7px 18px;border-radius:8px;border:1px solid var(--accent);
    background:transparent;color:var(--accent);font-size:.78rem;font-weight:700;cursor:pointer;transition:all .15s;}
  .wifi-apply:hover{background:var(--accent);color:#000;}
  #wifiMsg{font-size:.72rem;color:var(--yellow);display:none;}

  .dbg-grid{display:grid;grid-template-columns:repeat(8,1fr);gap:4px;}
  @media(max-width:600px){.dbg-grid{grid-template-columns:repeat(4,1fr);}}
  .dbg-cell{background:var(--bg);border:1px solid var(--border);border-radius:4px;
    padding:4px;text-align:center;transition:border-color .1s;}
  .dbg-cell .dcn{font-size:.55rem;color:var(--muted);}
  .dbg-cell .dcv{font-size:.75rem;font-variant-numeric:tabular-nums;color:var(--text);display:block;margin-top:1px;}
  .dbg-cell.lit{border-color:var(--accent);}
  .dbg-cell.lit .dcv{color:var(--accent);}
  .dbg-info{font-size:.65rem;color:var(--muted);margin-top:5px;display:flex;gap:14px;flex-wrap:wrap;}
  .dbg-info .di-val{color:var(--text);}

  @media(max-width:768px){.axis-grid{grid-template-columns:repeat(2,1fr);}.wifi-net-row{grid-template-columns:auto 1fr 1fr auto auto;}}
  @media(max-width:540px){
    body{padding:6px;gap:6px;}
    header{flex-direction:column;align-items:flex-start;gap:5px;}
    header>div{width:100%;justify-content:space-between;}
    .ctrl-area{display:grid;grid-template-columns:1fr 1fr;grid-template-rows:auto auto auto;gap:6px;align-items:start;}
    .stick-center{grid-column:1/3;grid-row:1;width:100%;}
    #swPyramidLeft{grid-column:1;grid-row:2;justify-self:center;}
    #swPyramidRight{grid-column:2;grid-row:2;justify-self:center;}
    #slLSCol{grid-column:1;grid-row:3;justify-self:stretch;align-self:center;}
    #slRSCol{grid-column:2;grid-row:3;justify-self:stretch;align-self:center;}
    .sticks-row{width:100%;justify-content:space-around;}
    .stick-block{flex:1;}.stick-card{padding:8px;}
    .stick-wrap{width:min(42vw,185px);}.stick-lbl{font-size:.55rem;}
    .stick-readout{font-size:.6rem;}.thumb{width:30px;height:30px;}
    .sw-pyramid{gap:4px;}.sw-pair{gap:4px;}.sw-card{width:62px;padding:6px 4px;}
    .ts-col{width:100%;}
    .slider-widget{flex-direction:column;align-items:center;padding:6px 8px;width:100%;}
    .slider-wrap{width:100%;height:28px;}.slider-inp{width:90%;transform:none;transform-origin:unset;}
    .lua-grid{grid-template-columns:repeat(4,1fr);gap:5px;}.lua-btn{padding:10px 4px;font-size:.7rem;}
    .trim-bank{gap:4px;}.trim-widget{padding:5px 6px;}.trim-btn{width:34px;height:24px;}
    .pot-row{gap:8px;}.pot-widget{min-width:0;flex:1;}.pot-inp{width:100%;}
    .cfg-table{display:block;overflow-x:auto;-webkit-overflow-scrolling:touch;}
  }
  @media(max-width:820px) and (orientation:landscape){
    body{padding:4px;gap:4px;}.stick-wrap{width:min(28vw,155px);}.stick-card{padding:8px;}
    .thumb{width:30px;height:30px;}.sw-card{width:60px;padding:5px 4px;}
    .lua-grid{grid-template-columns:repeat(6,1fr);}.lua-btn{padding:8px 4px;font-size:.65rem;}
    .trim-btn{width:32px;height:22px;font-size:.7rem;}
  }
</style>
</head>
<body>

<!-- ── Header ──────────────────────────────────────────────────────────────── -->
<header>
  <div class="logo">SBUS CONTROLLER</div>
  <div style="display:flex;gap:7px;align-items:center;flex-wrap:wrap;">
    <!-- Unit display toggle: storage stays SBUS, display switches between SBUS (172-1811) and PWM µs (988-2012). -->
    <span style="font-size:.6rem;color:var(--muted);letter-spacing:.05em">UNITS:</span>
    <label style="font-size:.7rem;color:var(--accent);cursor:pointer">
      <input type="radio" name="unit-toggle" value="sbus" checked style="vertical-align:middle"> SBUS
    </label>
    <label style="font-size:.7rem;color:var(--accent);cursor:pointer">
      <input type="radio" name="unit-toggle" value="us" style="vertical-align:middle"> µs
    </label>
    <span style="opacity:.4;color:var(--muted);font-size:.7rem">·</span>
    <button class="hdr-btn" id="modeBtn"  onclick="toggleMode()">SBUS-24</button>
    <button class="hdr-btn" id="debugBtn" onclick="toggleDebug()">DEBUG OFF</button>
    <!-- Transport picker — click to switch between WiFi and USB Serial. The
         current transport is reflected in the button label. -->
    <button class="hdr-btn" id="transportBtn" onclick="toggleTransportMenu(event)">WiFi ▾</button>
    <div id="connBadge">DISCONNECTED</div>
  </div>
</header>

<!-- Floating transport-picker menu — positioned next to #transportBtn at open. -->
<div id="transportMenu" class="tx-menu">
  <button data-kind="wifi"   onclick="chooseTransport('wifi')">WiFi</button>
  <button data-kind="serial" onclick="chooseTransport('serial')">USB Serial</button>
  <div class="tx-note">USB Serial needs Chrome/Edge.</div>
</div>

<!-- ── Lua / virtual buttons — top of page for easy access ─────────────────── -->
<div class="lua-section" id="luaSection">
  <div class="lua-header">
    <span style="font-size:.62rem;font-weight:700;letter-spacing:.1em;text-transform:uppercase;color:var(--muted);">Lua Buttons</span>
  </div>
  <div class="lua-grid" id="luaGrid"></div>
</div>

<!-- ── Transmitter SVG (X18 visualisation, Phase 1) ────────────────────────── -->
<!--   Click switches / matrix buttons / trims here to actuate them.            -->
<!--   The sticks, knobs (S1/S2) and LS/RS sliders remain as HTML controls      -->
<!--   below for now — Phase 2 will make them SVG-native.                       -->
<div class="tx-svg-wrap">
<svg viewBox="0 0 1000 830" xmlns="http://www.w3.org/2000/svg">
  <defs>
    <linearGradient id="tx-body-grad" x1="0" y1="0" x2="0" y2="1">
      <stop offset="0%"   stop-color="#2e4a82"/>
      <stop offset="45%"  stop-color="#1f3870"/>
      <stop offset="100%" stop-color="#13234a"/>
    </linearGradient>
    <linearGradient id="tx-body-highlight" x1="0" y1="0" x2="0" y2="1">
      <stop offset="0%"   stop-color="#3a5a92" stop-opacity="0.7"/>
      <stop offset="100%" stop-color="#3a5a92" stop-opacity="0"/>
    </linearGradient>
    <radialGradient id="tx-screen-grad" cx="50%" cy="40%" r="70%">
      <stop offset="0%"   stop-color="#0e2440"/>
      <stop offset="100%" stop-color="#050d1c"/>
    </radialGradient>
    <radialGradient id="tx-knob-grad" cx="50%" cy="30%" r="70%">
      <stop offset="0%"   stop-color="#e0e0e0"/>
      <stop offset="40%"  stop-color="#909090"/>
      <stop offset="100%" stop-color="#202020"/>
    </radialGradient>
    <radialGradient id="tx-bush-grad" cx="50%" cy="30%" r="65%">
      <stop offset="0%"   stop-color="#e8e8e8"/>
      <stop offset="50%"  stop-color="#9a9a9a"/>
      <stop offset="100%" stop-color="#303030"/>
    </radialGradient>
    <linearGradient id="tx-lever-grad" x1="0" y1="0" x2="0" y2="1">
      <stop offset="0%"   stop-color="#d0d0d0"/>
      <stop offset="100%" stop-color="#606060"/>
    </linearGradient>
    <radialGradient id="tx-cap-grad" cx="50%" cy="25%" r="60%">
      <stop offset="0%"   stop-color="#6a6a6a"/>
      <stop offset="55%"  stop-color="#1a1a1a"/>
      <stop offset="100%" stop-color="#000"/>
    </radialGradient>
    <filter id="tx-shadow" x="-20%" y="-20%" width="140%" height="140%">
      <feDropShadow dx="0" dy="6" stdDeviation="10" flood-opacity=".55"/>
    </filter>
  </defs>

  <!-- Main body -->
  <path d="M 70 320 Q 70 280 110 280 L 270 280 Q 290 280 295 270 Q 300 250 320 250 L 680 250 Q 700 250 705 270 Q 710 280 730 280 L 890 280 Q 930 280 930 320 L 930 640 Q 930 730 870 760 L 770 790 Q 750 798 720 800 L 580 808 Q 540 812 500 810 Q 460 812 420 808 L 280 800 Q 250 798 230 790 L 130 760 Q 70 730 70 640 Z" fill="url(#tx-body-grad)" stroke="#0a1428" stroke-width="2" filter="url(#tx-shadow)"/>
  <path d="M 70 320 Q 70 280 110 280 L 270 280 Q 290 280 295 270 Q 300 250 320 250 L 680 250 Q 700 250 705 270 Q 710 280 730 280 L 890 280 Q 930 280 930 320 L 930 360 L 70 360 Z" fill="url(#tx-body-highlight)" opacity="0.5"/>

  <!-- Screen pod -->
  <rect x="290" y="15" width="420" height="290" rx="22" fill="#0a0a0a" stroke="#1a2a4a" stroke-width="2" filter="url(#tx-shadow)"/>
  <rect x="305" y="32" width="390" height="258" rx="6" fill="url(#tx-screen-grad)"/>
  <text x="688" y="56" text-anchor="end" fill="#a8b2c8" font-size="16" font-weight="700" font-style="italic" font-family="serif">FrSky</text>
  <rect x="320" y="72" width="360" height="24" rx="3" fill="#1a3a6a"/>
  <text x="332" y="89" fill="#7acdf6" font-size="12" font-family="monospace" font-weight="700">SBUS Controller</text>
  <text x="668" y="89" fill="#7acdf6" font-size="10" font-family="monospace" text-anchor="end" id="tx-screen-status">—</text>
  <text x="332" y="112" fill="#5a9ac6" font-size="10" font-family="monospace" id="tx-screen-mode">Mode: SBUS-24</text>
  <text x="668" y="112" fill="#5a9ac6" font-size="10" font-family="monospace" text-anchor="end" id="tx-screen-frames">Frames: 0</text>

  <!-- LUA button grid mirrored from the HTML row above (5 cols × 3 rows).
       Populated by renderScreenLuaButtons() once the cfg arrives. -->
  <g id="screenLuaGrid"></g>

  <!-- Forward/Rear orientation legend — small directional indicators sitting
       between the two switch clusters above the screen pod.                   -->
  <g font-family="monospace" font-size="11" font-weight="700">
    <text x="385" y="30" text-anchor="end" fill="#7acdf6">REAR &#x2190;</text>
    <text x="500" y="30" text-anchor="middle" fill="#5a9ac6">lever lean</text>
    <text x="615" y="30" text-anchor="start" fill="#7acdf6">&#x2192; FWD</text>
  </g>

  <!-- Switches SA-SH (cycle on click).
       The <g class="lever"> sub-group wraps the stem + tip rects so a JS
       transform rotates them as a unit around the bushing centre. -->
  <g class="svg-sw" data-i="5" data-pos="0" data-px="135" data-py="282" onclick="svgSwCycle(5)">
    <circle cx="135" cy="282" r="11" fill="url(#tx-bush-grad)" stroke="#0a0a0a"/>
    <g class="lever">
      <rect x="132" y="195" width="6" height="87" rx="2" fill="url(#tx-lever-grad)" stroke="#0a0a0a" stroke-width="0.5"/>
      <rect x="128" y="187" width="14" height="14" rx="3" fill="#e8e8e8" stroke="#0a0a0a"/>
    </g>
    <text x="115" y="287" text-anchor="end" fill="#cdd6e8" font-size="11" font-weight="700">SF</text>
  </g>
  <g class="svg-sw" data-i="4" data-pos="0" data-px="180" data-py="310" onclick="svgSwCycle(4)">
    <circle cx="180" cy="310" r="10" fill="url(#tx-bush-grad)" stroke="#0a0a0a"/>
    <g class="lever">
      <rect x="177" y="240" width="6" height="70" rx="2" fill="url(#tx-lever-grad)" stroke="#0a0a0a" stroke-width="0.5"/>
      <rect x="173" y="234" width="14" height="14" rx="3" fill="#e8e8e8" stroke="#0a0a0a"/>
    </g>
    <text x="160" y="315" text-anchor="end" fill="#cdd6e8" font-size="11" font-weight="700">SE</text>
  </g>
  <g class="svg-sw" data-i="0" data-pos="0" data-px="95" data-py="335" onclick="svgSwCycle(0)">
    <circle cx="95" cy="335" r="10" fill="url(#tx-bush-grad)" stroke="#0a0a0a"/>
    <g class="lever">
      <rect x="92" y="275" width="6" height="60" rx="2" fill="url(#tx-lever-grad)" stroke="#0a0a0a" stroke-width="0.5"/>
      <rect x="88" y="269" width="14" height="14" rx="3" fill="#e8e8e8" stroke="#0a0a0a"/>
    </g>
    <text x="95" y="358" text-anchor="middle" fill="#cdd6e8" font-size="11" font-weight="700">SA</text>
  </g>
  <g class="svg-sw" data-i="1" data-pos="0" data-px="225" data-py="335" onclick="svgSwCycle(1)">
    <circle cx="225" cy="335" r="10" fill="url(#tx-bush-grad)" stroke="#0a0a0a"/>
    <g class="lever">
      <rect x="222" y="275" width="6" height="60" rx="2" fill="url(#tx-lever-grad)" stroke="#0a0a0a" stroke-width="0.5"/>
      <rect x="218" y="269" width="14" height="14" rx="3" fill="#e8e8e8" stroke="#0a0a0a"/>
    </g>
    <text x="225" y="358" text-anchor="middle" fill="#cdd6e8" font-size="11" font-weight="700">SB</text>
  </g>
  <g class="svg-sw" data-i="7" data-pos="0" data-px="865" data-py="282"
     onpointerdown="svgSwMomentaryPress(event,7)"
     onpointerup="svgSwMomentaryRelease(event,7)"
     onpointerleave="svgSwMomentaryRelease(event,7)"
     onpointercancel="svgSwMomentaryRelease(event,7)">
    <circle cx="865" cy="282" r="11" fill="url(#tx-bush-grad)" stroke="#0a0a0a"/>
    <g class="lever">
      <rect x="862" y="195" width="6" height="87" rx="2" fill="url(#tx-lever-grad)" stroke="#0a0a0a" stroke-width="0.5"/>
      <rect x="858" y="187" width="14" height="14" rx="3" fill="#e8e8e8" stroke="#0a0a0a"/>
    </g>
    <text x="845" y="287" text-anchor="end" fill="#cdd6e8" font-size="11" font-weight="700">SH</text>
  </g>
  <g class="svg-sw" data-i="6" data-pos="0" data-px="820" data-py="310" onclick="svgSwCycle(6)">
    <circle cx="820" cy="310" r="10" fill="url(#tx-bush-grad)" stroke="#0a0a0a"/>
    <g class="lever">
      <rect x="817" y="240" width="6" height="70" rx="2" fill="url(#tx-lever-grad)" stroke="#0a0a0a" stroke-width="0.5"/>
      <rect x="813" y="234" width="14" height="14" rx="3" fill="#e8e8e8" stroke="#0a0a0a"/>
    </g>
    <text x="800" y="315" text-anchor="end" fill="#cdd6e8" font-size="11" font-weight="700">SG</text>
  </g>
  <g class="svg-sw" data-i="3" data-pos="0" data-px="905" data-py="335" onclick="svgSwCycle(3)">
    <circle cx="905" cy="335" r="10" fill="url(#tx-bush-grad)" stroke="#0a0a0a"/>
    <g class="lever">
      <rect x="902" y="275" width="6" height="60" rx="2" fill="url(#tx-lever-grad)" stroke="#0a0a0a" stroke-width="0.5"/>
      <rect x="898" y="269" width="14" height="14" rx="3" fill="#e8e8e8" stroke="#0a0a0a"/>
    </g>
    <text x="905" y="358" text-anchor="middle" fill="#cdd6e8" font-size="11" font-weight="700">SD</text>
  </g>
  <g class="svg-sw" data-i="2" data-pos="0" data-px="775" data-py="335" onclick="svgSwCycle(2)">
    <circle cx="775" cy="335" r="10" fill="url(#tx-bush-grad)" stroke="#0a0a0a"/>
    <g class="lever">
      <rect x="772" y="275" width="6" height="60" rx="2" fill="url(#tx-lever-grad)" stroke="#0a0a0a" stroke-width="0.5"/>
      <rect x="768" y="269" width="14" height="14" rx="3" fill="#e8e8e8" stroke="#0a0a0a"/>
    </g>
    <text x="775" y="358" text-anchor="middle" fill="#cdd6e8" font-size="11" font-weight="700">SC</text>
  </g>

  <!-- SI / SJ button caps (cycle through positions on click — defaults to 3-pos
       switch but reflects sw[i].t at runtime).  Positions match the RC Controller
       config tool so the controller graphic stays consistent across the two apps. -->
  <g class="svg-sw" data-i="8" data-pos="0" data-px="265" data-py="359" onclick="svgSwCycle(8)" style="cursor:pointer">
    <rect x="254" y="348" width="22" height="22" rx="5" fill="#1a1a2e" stroke="#3a3a6a"/>
    <circle cx="265" cy="359" r="7" fill="url(#tx-cap-grad)" stroke="#1a1a1a"/>
    <text x="265" y="384" text-anchor="middle" fill="#cdd6e8" font-size="11" font-weight="700">SI</text>
  </g>
  <g class="svg-sw" data-i="9" data-pos="0" data-px="735" data-py="359" onclick="svgSwCycle(9)" style="cursor:pointer">
    <rect x="724" y="348" width="22" height="22" rx="5" fill="#1a1a2e" stroke="#3a3a6a"/>
    <circle cx="735" cy="359" r="7" fill="url(#tx-cap-grad)" stroke="#1a1a1a"/>
    <text x="735" y="384" text-anchor="middle" fill="#cdd6e8" font-size="11" font-weight="700">SJ</text>
  </g>

  <!-- S1, S2 knobs (draggable — vertical drag = value).
       cfg.sl index: S1=2, S2=3.  Drag up = increase, down = decrease.        -->
  <g class="svg-knob" data-i="2"
     onpointerdown="svgKnobDown(event,2)"
     onpointermove="svgKnobMove(event,2)"
     onpointerup="svgKnobUp(event,2)"
     onpointercancel="svgKnobUp(event,2)">
    <circle cx="430" cy="390" r="28" fill="#08101e" stroke="#1a2a4a"/>
    <circle cx="430" cy="390" r="23" fill="url(#tx-knob-grad)" stroke="#ffffff" stroke-width="0.6"/>
    <line id="knobIndS1" x1="430" y1="370" x2="430" y2="378" stroke="#fff" stroke-width="2" stroke-linecap="round" transform="rotate(0 430 390)"/>
    <text x="430" y="438" text-anchor="middle" fill="#cdd6e8" font-size="11" font-weight="700">S1</text>
    <text id="knobValS1" x="430" y="408" text-anchor="middle" fill="#7acdf6" font-size="9" font-family="monospace">50</text>
  </g>
  <g class="svg-knob" data-i="3"
     onpointerdown="svgKnobDown(event,3)"
     onpointermove="svgKnobMove(event,3)"
     onpointerup="svgKnobUp(event,3)"
     onpointercancel="svgKnobUp(event,3)">
    <circle cx="570" cy="390" r="28" fill="#08101e" stroke="#1a2a4a"/>
    <circle cx="570" cy="390" r="23" fill="url(#tx-knob-grad)" stroke="#ffffff" stroke-width="0.6"/>
    <line id="knobIndS2" x1="570" y1="370" x2="570" y2="378" stroke="#fff" stroke-width="2" stroke-linecap="round" transform="rotate(0 570 390)"/>
    <text x="570" y="438" text-anchor="middle" fill="#cdd6e8" font-size="11" font-weight="700">S2</text>
    <text id="knobValS2" x="570" y="408" text-anchor="middle" fill="#7acdf6" font-size="9" font-family="monospace">50</text>
  </g>

  <!-- Power button + scroll wheel (decorative) -->
  <circle cx="500" cy="475" r="24" fill="#08101e" stroke="#1a2a4a"/>
  <circle cx="500" cy="475" r="19" fill="#0d0d0d" stroke="#2a2a2a"/>
  <path d="M 500 465 L 500 479 M 491 470 A 13 13 0 1 0 509 470" stroke="#888" stroke-width="2.2" fill="none" stroke-linecap="round"/>
  <rect x="488" y="510" width="24" height="16" rx="4" fill="#0a0a0a" stroke="#1a1a1a"/>
  <rect x="495" y="514" width="10" height="8" rx="1" fill="#3a3a3a"/>

  <!-- ── LEFT gimbal (draggable) ── -->
  <!-- Body data attrs: cx/cy = centre, half = half-extent of hit box -->
  <g class="svg-stick" data-side="L" data-cx="240" data-cy="485" data-half="78">
    <rect x="150" y="395" width="180" height="180" rx="22" fill="#000" stroke="#1a1a1a" stroke-width="2"/>
    <rect x="162" y="407" width="156" height="156" rx="14" fill="#0a0a0a"/>
    <line x1="240" y1="410" x2="240" y2="424" stroke="#5a5a5a" stroke-width="1.5"/>
    <line x1="240" y1="546" x2="240" y2="560" stroke="#5a5a5a" stroke-width="1.5"/>
    <line x1="166" y1="485" x2="180" y2="485" stroke="#5a5a5a" stroke-width="1.5"/>
    <line x1="300" y1="485" x2="314" y2="485" stroke="#5a5a5a" stroke-width="1.5"/>
    <rect x="208" y="453" width="64" height="64" rx="8" fill="#000" stroke="#252525"/>
    <!-- Thumb cap (gets translated on drag) -->
    <g id="stickThumbL">
      <circle cx="240" cy="485" r="15" fill="url(#tx-cap-grad)" stroke="#1a1a1a"/>
      <circle cx="240" cy="485" r="6" fill="#0a0a0a"/>
    </g>
    <!-- Transparent hit overlay covering the whole gimbal cutout -->
    <rect class="stick-hit" x="150" y="395" width="180" height="180" rx="22"
          fill="transparent" style="cursor:grab"
          onpointerdown="svgStickDown(event,'L')"
          onpointermove="svgStickMove(event,'L')"
          onpointerup="svgStickUp(event,'L')"
          onpointercancel="svgStickUp(event,'L')"/>
  </g>

  <!-- ── RIGHT gimbal (draggable) ── -->
  <g class="svg-stick" data-side="R" data-cx="760" data-cy="485" data-half="78">
    <rect x="670" y="395" width="180" height="180" rx="22" fill="#000" stroke="#1a1a1a" stroke-width="2"/>
    <rect x="682" y="407" width="156" height="156" rx="14" fill="#0a0a0a"/>
    <line x1="760" y1="410" x2="760" y2="424" stroke="#5a5a5a" stroke-width="1.5"/>
    <line x1="760" y1="546" x2="760" y2="560" stroke="#5a5a5a" stroke-width="1.5"/>
    <line x1="686" y1="485" x2="700" y2="485" stroke="#5a5a5a" stroke-width="1.5"/>
    <line x1="820" y1="485" x2="834" y2="485" stroke="#5a5a5a" stroke-width="1.5"/>
    <rect x="728" y="453" width="64" height="64" rx="8" fill="#000" stroke="#252525"/>
    <g id="stickThumbR">
      <circle cx="760" cy="485" r="15" fill="url(#tx-cap-grad)" stroke="#1a1a1a"/>
      <circle cx="760" cy="485" r="6" fill="#0a0a0a"/>
    </g>
    <rect class="stick-hit" x="670" y="395" width="180" height="180" rx="22"
          fill="transparent" style="cursor:grab"
          onpointerdown="svgStickDown(event,'R')"
          onpointermove="svgStickMove(event,'R')"
          onpointerup="svgStickUp(event,'R')"
          onpointercancel="svgStickUp(event,'R')"/>
  </g>

  <!-- ── LS / RS edge sliders (draggable, vertical) ──
       Styling matches the RC Controller config tool: short rail (10×120) set
       slightly inboard from the body edge, with a wider knob (22×16) straddling
       the rail and a horizontal centre indicator line.  cfg.sl index 0 = LS
       (left rail), 1 = RS (right rail).  The JS constants _sliderTop/_sliderBot
       in the slider section below must stay in sync with these coordinates. -->
  <g class="svg-slider" data-i="0" data-x="81" data-top="430" data-bot="550"
     onpointerdown="svgSliderDown(event,0)"
     onpointermove="svgSliderMove(event,0)"
     onpointerup="svgSliderUp(event,0)"
     onpointercancel="svgSliderUp(event,0)">
    <rect x="76" y="430" width="10" height="120" rx="5" fill="#08101e" stroke="#1a2a4a"/>
    <line x1="73" y1="490" x2="89" y2="490" stroke="#2a3a5a" stroke-width="1.5"/>
    <rect x="70" y="482" width="22" height="16" rx="4" id="sliderThumbLS" fill="url(#tx-knob-grad)" stroke="#fff" stroke-width="0.6"/>
    <text x="81" y="570" text-anchor="middle" fill="#cdd6e8" font-size="11" font-weight="700">LS</text>
  </g>
  <g class="svg-slider" data-i="1" data-x="919" data-top="430" data-bot="550"
     onpointerdown="svgSliderDown(event,1)"
     onpointermove="svgSliderMove(event,1)"
     onpointerup="svgSliderUp(event,1)"
     onpointercancel="svgSliderUp(event,1)">
    <rect x="914" y="430" width="10" height="120" rx="5" fill="#08101e" stroke="#1a2a4a"/>
    <line x1="911" y1="490" x2="927" y2="490" stroke="#2a3a5a" stroke-width="1.5"/>
    <rect x="908" y="482" width="22" height="16" rx="4" id="sliderThumbRS" fill="url(#tx-knob-grad)" stroke="#fff" stroke-width="0.6"/>
    <text x="919" y="570" text-anchor="middle" fill="#cdd6e8" font-size="11" font-weight="700">RS</text>
  </g>

  <!-- ── J5 / J6 horizontal sliders (X20 3-axis stick twist) ──────────────────
       Slim spring-loaded rails just below the joystick block bottoms (y=575).
       Pulled up tight against the sticks so the SK/SL labels can sit beside
       the buttons without overlapping the T4-T5-T6-T1 trim row (y=600).
       Geometry MUST match _sliderGeom entries for indices 5/6 in JS below. -->
  <g class="svg-slider" data-i="5" data-orient="h" data-left="140" data-right="335"
     onpointerdown="svgSliderDown(event,5)"
     onpointermove="svgSliderMove(event,5)"
     onpointerup="svgSliderUp(event,5)"
     onpointercancel="svgSliderUp(event,5)">
    <rect x="140" y="580" width="195" height="6" rx="3" fill="#08101e" stroke="#1a2a4a"/>
    <line x1="237" y1="575" x2="237" y2="591" stroke="#2a3a5a" stroke-width="1.5"/>
    <rect x="230" y="575" width="14" height="16" rx="3" id="sliderThumbJ5" fill="url(#tx-knob-grad)" stroke="#fff" stroke-width="0.6"/>
    <text x="132" y="587" text-anchor="end" fill="#cdd6e8" font-size="11" font-weight="700">J5</text>
  </g>
  <g class="svg-slider" data-i="6" data-orient="h" data-left="665" data-right="860"
     onpointerdown="svgSliderDown(event,6)"
     onpointermove="svgSliderMove(event,6)"
     onpointerup="svgSliderUp(event,6)"
     onpointercancel="svgSliderUp(event,6)">
    <rect x="665" y="580" width="195" height="6" rx="3" fill="#08101e" stroke="#1a2a4a"/>
    <line x1="762" y1="575" x2="762" y2="591" stroke="#2a3a5a" stroke-width="1.5"/>
    <rect x="755" y="575" width="14" height="16" rx="3" id="sliderThumbJ6" fill="url(#tx-knob-grad)" stroke="#fff" stroke-width="0.6"/>
    <text x="868" y="587" text-anchor="start" fill="#cdd6e8" font-size="11" font-weight="700">J6</text>
  </g>

  <!-- SK / SL stick-click momentaries (X20 3-axis gimbal upgrade).
       cfg.btn[6]=SK, cfg.btn[7]=SL.  Same press/release path as S1-S6.
       Labels sit on the INBOARD side (toward the centre power button) so they
       don't overlap the trim row immediately below the buttons. -->
  <g class="svg-btn" data-i="6" onpointerdown="svgBtnPress(event,6)" onpointerup="svgBtnRelease(event,6)" onpointerleave="svgBtnRelease(event,6)">
    <circle cx="360" cy="583" r="9" fill="#0a0a0a" stroke="#4a4a4a" stroke-width="2"/>
    <circle cx="360" cy="583" r="5" fill="#1a1a1a"/>
    <text x="375" y="587" text-anchor="start" fill="#cdd6e8" font-size="10" font-weight="700">SK</text>
  </g>
  <g class="svg-btn" data-i="7" onpointerdown="svgBtnPress(event,7)" onpointerup="svgBtnRelease(event,7)" onpointerleave="svgBtnRelease(event,7)">
    <circle cx="640" cy="583" r="9" fill="#0a0a0a" stroke="#4a4a4a" stroke-width="2"/>
    <circle cx="640" cy="583" r="5" fill="#1a1a1a"/>
    <text x="625" y="587" text-anchor="end" fill="#cdd6e8" font-size="10" font-weight="700">SL</text>
  </g>

  <!-- Trim button zones — press/release fires {t:'tr', i, d, p}.
       trim idx: T1=0 T2=1 T3=2 T4=3 T5=4 T6=5 -->
  <g class="svg-tr" data-i="2" data-d="1"  onpointerdown="svgTrPress(event,2,1)"  onpointerup="svgTrRelease(event,2)"  onpointerleave="svgTrRelease(event,2)">
    <rect x="358" y="435" width="26" height="22" rx="3" fill="#0a0a0a" stroke="#3a3a3a"/>
    <rect x="367" y="438" width="8" height="10" rx="2" fill="#aaa"/>
    <polygon points="371,430 366,436 376,436" fill="#cdd6e8"/>
  </g>
  <g class="svg-tr" data-i="2" data-d="-1" onpointerdown="svgTrPress(event,2,-1)" onpointerup="svgTrRelease(event,2)" onpointerleave="svgTrRelease(event,2)">
    <rect x="358" y="515" width="26" height="22" rx="3" fill="#0a0a0a" stroke="#3a3a3a"/>
    <rect x="367" y="525" width="8" height="10" rx="2" fill="#aaa"/>
    <polygon points="371,545 366,539 376,539" fill="#cdd6e8"/>
  </g>
  <text x="371" y="485" text-anchor="middle" fill="#cdd6e8" font-size="11" font-weight="700">T3</text>

  <g class="svg-tr" data-i="1" data-d="1"  onpointerdown="svgTrPress(event,1,1)"  onpointerup="svgTrRelease(event,1)"  onpointerleave="svgTrRelease(event,1)">
    <rect x="616" y="435" width="26" height="22" rx="3" fill="#0a0a0a" stroke="#3a3a3a"/>
    <rect x="625" y="438" width="8" height="10" rx="2" fill="#aaa"/>
    <polygon points="629,430 624,436 634,436" fill="#cdd6e8"/>
  </g>
  <g class="svg-tr" data-i="1" data-d="-1" onpointerdown="svgTrPress(event,1,-1)" onpointerup="svgTrRelease(event,1)" onpointerleave="svgTrRelease(event,1)">
    <rect x="616" y="515" width="26" height="22" rx="3" fill="#0a0a0a" stroke="#3a3a3a"/>
    <rect x="625" y="525" width="8" height="10" rx="2" fill="#aaa"/>
    <polygon points="629,545 624,539 634,539" fill="#cdd6e8"/>
  </g>
  <text x="629" y="485" text-anchor="middle" fill="#cdd6e8" font-size="11" font-weight="700">T2</text>

  <g class="svg-tr" data-i="3" data-d="-1" onpointerdown="svgTrPress(event,3,-1)" onpointerup="svgTrRelease(event,3)" onpointerleave="svgTrRelease(event,3)">
    <rect x="160" y="600" width="22" height="20" rx="3" fill="#0a0a0a" stroke="#3a3a3a"/>
    <rect x="163" y="606" width="9" height="8" rx="2" fill="#aaa"/>
    <polygon points="156,610 162,606 162,614" fill="#cdd6e8"/>
  </g>
  <g class="svg-tr" data-i="3" data-d="1"  onpointerdown="svgTrPress(event,3,1)"  onpointerup="svgTrRelease(event,3)"  onpointerleave="svgTrRelease(event,3)">
    <rect x="195" y="600" width="22" height="20" rx="3" fill="#0a0a0a" stroke="#3a3a3a"/>
    <rect x="205" y="606" width="9" height="8" rx="2" fill="#aaa"/>
    <polygon points="221,610 215,606 215,614" fill="#cdd6e8"/>
  </g>
  <text x="188" y="640" text-anchor="middle" fill="#cdd6e8" font-size="11" font-weight="700">T4</text>

  <g class="svg-tr" data-i="4" data-d="-1" onpointerdown="svgTrPress(event,4,-1)" onpointerup="svgTrRelease(event,4)" onpointerleave="svgTrRelease(event,4)">
    <rect x="295" y="600" width="22" height="20" rx="3" fill="#0a0a0a" stroke="#3a3a3a"/>
    <rect x="298" y="606" width="9" height="8" rx="2" fill="#aaa"/>
    <polygon points="291,610 297,606 297,614" fill="#cdd6e8"/>
  </g>
  <g class="svg-tr" data-i="4" data-d="1"  onpointerdown="svgTrPress(event,4,1)"  onpointerup="svgTrRelease(event,4)"  onpointerleave="svgTrRelease(event,4)">
    <rect x="330" y="600" width="22" height="20" rx="3" fill="#0a0a0a" stroke="#3a3a3a"/>
    <rect x="340" y="606" width="9" height="8" rx="2" fill="#aaa"/>
    <polygon points="356,610 350,606 350,614" fill="#cdd6e8"/>
  </g>
  <text x="323" y="640" text-anchor="middle" fill="#cdd6e8" font-size="11" font-weight="700">T5</text>

  <g class="svg-tr" data-i="5" data-d="-1" onpointerdown="svgTrPress(event,5,-1)" onpointerup="svgTrRelease(event,5)" onpointerleave="svgTrRelease(event,5)">
    <rect x="648" y="600" width="22" height="20" rx="3" fill="#0a0a0a" stroke="#3a3a3a"/>
    <rect x="651" y="606" width="9" height="8" rx="2" fill="#aaa"/>
    <polygon points="644,610 650,606 650,614" fill="#cdd6e8"/>
  </g>
  <g class="svg-tr" data-i="5" data-d="1"  onpointerdown="svgTrPress(event,5,1)"  onpointerup="svgTrRelease(event,5)"  onpointerleave="svgTrRelease(event,5)">
    <rect x="683" y="600" width="22" height="20" rx="3" fill="#0a0a0a" stroke="#3a3a3a"/>
    <rect x="693" y="606" width="9" height="8" rx="2" fill="#aaa"/>
    <polygon points="709,610 703,606 703,614" fill="#cdd6e8"/>
  </g>
  <text x="676" y="640" text-anchor="middle" fill="#cdd6e8" font-size="11" font-weight="700">T6</text>

  <g class="svg-tr" data-i="0" data-d="-1" onpointerdown="svgTrPress(event,0,-1)" onpointerup="svgTrRelease(event,0)" onpointerleave="svgTrRelease(event,0)">
    <rect x="783" y="600" width="22" height="20" rx="3" fill="#0a0a0a" stroke="#3a3a3a"/>
    <rect x="786" y="606" width="9" height="8" rx="2" fill="#aaa"/>
    <polygon points="779,610 785,606 785,614" fill="#cdd6e8"/>
  </g>
  <g class="svg-tr" data-i="0" data-d="1"  onpointerdown="svgTrPress(event,0,1)"  onpointerup="svgTrRelease(event,0)"  onpointerleave="svgTrRelease(event,0)">
    <rect x="818" y="600" width="22" height="20" rx="3" fill="#0a0a0a" stroke="#3a3a3a"/>
    <rect x="828" y="606" width="9" height="8" rx="2" fill="#aaa"/>
    <polygon points="844,610 838,606 838,614" fill="#cdd6e8"/>
  </g>
  <text x="811" y="640" text-anchor="middle" fill="#cdd6e8" font-size="11" font-weight="700">T1</text>

  <!-- Centre logo -->
  <text x="500" y="640" text-anchor="middle" fill="#3a5a8a" font-size="24" font-family="sans-serif" font-weight="700" font-style="italic">X18</text>
  <rect x="465" y="650" width="70" height="13" rx="6" fill="#08101e" stroke="#1a2a4a"/>
  <g fill="#1a3a6a">
    <circle cx="475" cy="656" r="1.4"/><circle cx="485" cy="656" r="1.4"/>
    <circle cx="495" cy="656" r="1.4"/><circle cx="505" cy="656" r="1.4"/>
    <circle cx="515" cy="656" r="1.4"/><circle cx="525" cy="656" r="1.4"/>
  </g>
  <text x="500" y="680" text-anchor="middle" fill="#5a7aa8" font-size="9" font-weight="700" letter-spacing="1">900M/2.4G DUAL BAND</text>

  <!-- Nav dials (decorative) -->
  <circle cx="185" cy="700" r="36" fill="#08101e" stroke="#1a2a4a"/>
  <circle cx="185" cy="700" r="31" fill="#0d0d0d" stroke="#2a2a2a"/>
  <circle cx="185" cy="700" r="15" fill="#1a1a1a" stroke="#0a0a0a"/>
  <text x="185" y="675" text-anchor="middle" fill="#a0a8b8" font-size="8" font-weight="700">MDL</text>
  <text x="217" y="703" text-anchor="middle" fill="#a0a8b8" font-size="8" font-weight="700">DISP</text>
  <text x="185" y="732" text-anchor="middle" fill="#a0a8b8" font-size="8" font-weight="700">RTN</text>
  <text x="153" y="703" text-anchor="middle" fill="#a0a8b8" font-size="8" font-weight="700">SYS</text>
  <circle cx="815" cy="700" r="36" fill="#08101e" stroke="#1a2a4a"/>
  <circle cx="815" cy="700" r="31" fill="#0d0d0d" stroke="#2a2a2a"/>
  <circle cx="815" cy="700" r="20" fill="#1a1a1a"/>

  <!-- Matrix buttons S1-S6 (idx 0-5 in cfg.btn) — press/release -->
  <g class="svg-btn" data-i="0" onpointerdown="svgBtnPress(event,0)" onpointerup="svgBtnRelease(event,0)" onpointerleave="svgBtnRelease(event,0)">
    <circle cx="320" cy="668" r="12" fill="#0a0a0a" stroke="#4a4a4a" stroke-width="2"/>
    <circle cx="320" cy="668" r="7" fill="#1a1a1a"/>
    <text x="320" y="671" text-anchor="middle" fill="#a0a8b8" font-size="9" font-weight="700">S1</text>
  </g>
  <g class="svg-btn" data-i="1" onpointerdown="svgBtnPress(event,1)" onpointerup="svgBtnRelease(event,1)" onpointerleave="svgBtnRelease(event,1)">
    <circle cx="350" cy="695" r="12" fill="#0a0a0a" stroke="#4a4a4a" stroke-width="2"/>
    <circle cx="350" cy="695" r="7" fill="#1a1a1a"/>
    <text x="350" y="698" text-anchor="middle" fill="#a0a8b8" font-size="9" font-weight="700">S2</text>
  </g>
  <g class="svg-btn" data-i="2" onpointerdown="svgBtnPress(event,2)" onpointerup="svgBtnRelease(event,2)" onpointerleave="svgBtnRelease(event,2)">
    <circle cx="320" cy="722" r="12" fill="#0a0a0a" stroke="#4a4a4a" stroke-width="2"/>
    <circle cx="320" cy="722" r="7" fill="#1a1a1a"/>
    <text x="320" y="725" text-anchor="middle" fill="#a0a8b8" font-size="9" font-weight="700">S3</text>
  </g>
  <g class="svg-btn" data-i="3" onpointerdown="svgBtnPress(event,3)" onpointerup="svgBtnRelease(event,3)" onpointerleave="svgBtnRelease(event,3)">
    <circle cx="680" cy="668" r="12" fill="#0a0a0a" stroke="#4a4a4a" stroke-width="2"/>
    <circle cx="680" cy="668" r="7" fill="#1a1a1a"/>
    <text x="680" y="671" text-anchor="middle" fill="#a0a8b8" font-size="9" font-weight="700">S4</text>
  </g>
  <g class="svg-btn" data-i="4" onpointerdown="svgBtnPress(event,4)" onpointerup="svgBtnRelease(event,4)" onpointerleave="svgBtnRelease(event,4)">
    <circle cx="650" cy="695" r="12" fill="#0a0a0a" stroke="#4a4a4a" stroke-width="2"/>
    <circle cx="650" cy="695" r="7" fill="#1a1a1a"/>
    <text x="650" y="698" text-anchor="middle" fill="#a0a8b8" font-size="9" font-weight="700">S5</text>
  </g>
  <g class="svg-btn" data-i="5" onpointerdown="svgBtnPress(event,5)" onpointerup="svgBtnRelease(event,5)" onpointerleave="svgBtnRelease(event,5)">
    <circle cx="680" cy="722" r="12" fill="#0a0a0a" stroke="#4a4a4a" stroke-width="2"/>
    <circle cx="680" cy="722" r="7" fill="#1a1a1a"/>
    <text x="680" y="725" text-anchor="middle" fill="#a0a8b8" font-size="9" font-weight="700">S6</text>
  </g>

  <!-- Former rear buttons RB1/RB2 have been promoted to switches SI/SJ — they now
       live in the bottom of the switch pyramids (left/right) and in the Switches
       config section, with low/mid/high values like SA-SH. -->

  <!-- ── S3 vertical slider (X20 middle slider, was MS) ─────────────────────────
       Sits in the bottom-centre between the L/R nav dials, below the X18 logo.
       Geometry MUST match _sliderGeom[4] in the JS section. -->
  <g class="svg-slider" data-i="4" data-orient="v" data-top="710" data-bot="790"
     onpointerdown="svgSliderDown(event,4)"
     onpointermove="svgSliderMove(event,4)"
     onpointerup="svgSliderUp(event,4)"
     onpointercancel="svgSliderUp(event,4)">
    <rect x="494" y="710" width="10" height="80" rx="5" fill="#08101e" stroke="#1a2a4a"/>
    <line x1="491" y1="750" x2="507" y2="750" stroke="#2a3a5a" stroke-width="1.5"/>
    <rect x="488" y="743" width="22" height="14" rx="3" id="sliderThumbS3" fill="url(#tx-knob-grad)" stroke="#fff" stroke-width="0.6"/>
    <text x="499" y="807" text-anchor="middle" fill="#cdd6e8" font-size="11" font-weight="700">S3</text>
  </g>
</svg>
</div>

<!-- ── Main row: [L-pyramid] [LS] [stick-center] [RS] [R-pyramid] -->
<div class="ctrl-area">

  <!-- Left pyramid: SF top, SE mid, SA+SB bottom pair -->
  <div class="sw-pyramid" id="swPyramidLeft"></div>

  <!-- LS slider -->
  <div class="ts-col" id="slLSCol"></div>

  <!-- Center: sticks + all sub-controls tucked underneath -->
  <div class="stick-center">
    <div class="sticks-row">

      <!-- Left stick -->
      <div class="stick-block">
        <div class="stick-card">
          <div class="stick-lbl" id="lblL">LEFT  LX:CH4  LY:CH3</div>
          <div class="stick-wrap" id="stickL">
            <div class="thumb" id="thumbL" style="left:50%;top:50%"></div>
          </div>
          <div class="stick-readout" id="readL">LX: 992&nbsp;&nbsp;LY: 992</div>
        </div>
      </div>

      <!-- Right stick -->
      <div class="stick-block">
        <div class="stick-card">
          <div class="stick-lbl" id="lblR">RIGHT  RX:CH1  RY:CH2</div>
          <div class="stick-wrap" id="stickR">
            <div class="thumb" id="thumbR" style="left:50%;top:50%"></div>
          </div>
          <div class="stick-readout" id="readR">RX: 992&nbsp;&nbsp;RY: 992</div>
        </div>
      </div>

    </div><!-- /sticks-row -->

    <!-- S1/S2 pots tucked directly under sticks -->
    <div class="pot-row" id="potRow"></div>

    <!-- Trims tucked under pots -->
    <div class="trim-bank" id="trimBank"></div>

    <!-- Physical buttons tucked under trims -->
    <div class="btn-bank" id="btnBank"></div>

  </div><!-- /stick-center -->

  <!-- RS slider -->
  <div class="ts-col" id="slRSCol"></div>

  <!-- Right pyramid: SH top, SG mid, SC+SD bottom pair -->
  <div class="sw-pyramid" id="swPyramidRight"></div>

</div>

<!-- ── Debug panel ───────────────────────────────────────────────────────────── -->
<div class="settings-wrap">
  <details id="dbgDetails" open>
    <summary>Live Channel Monitor</summary>
    <div class="settings-body">
      <div id="dbgOffMsg" style="font-size:.74rem;color:var(--muted);text-align:center;padding:6px 0;">
        Enable <strong style="color:var(--text)">DEBUG</strong> to start live data
      </div>
      <div class="dbg-grid" id="dbgGrid" style="display:none"></div>
      <div class="dbg-info" id="dbgInfo" style="display:none">
        <span>Mode: <span class="di-val" id="diMode">—</span></span>
        <span>Frame: <span class="di-val" id="diFrame">—</span> bytes</span>
        <span>Channels: <span class="di-val" id="diCh">—</span></span>
        <span>Updates: <span class="di-val" id="diRate">—</span></span>
      </div>
    </div>
  </details>
</div>

<!-- ── Settings ─────────────────────────────────────────────────────────────── -->
<div class="settings-wrap">
  <details>
    <summary>Settings</summary>
    <div class="settings-body">

      <div>
        <div class="sec-title">Joystick Channels &amp; Range</div>
        <table class="cfg-table" style="min-width:480px;">
          <thead><tr><th>Axis</th><th>Channel</th><th>Min <span class="unit-label">(SBUS)</span></th><th>Max <span class="unit-label">(SBUS)</span></th><th title="Flip stick direction for this axis">Reverse</th></tr></thead>
          <tbody>
            <!-- XYXY ordering: Right X, Right Y, Left X, Left Y.  Internal aMin/aMax/aRev
                 array indices kept the same (0/1=R, 2=LY, 3=LX) — only display row order swapped. -->
            <tr><td>Right X (AIL)</td><td><select id="selRX"></select></td><td><input type="number" id="axMin0" min="1" max="2047" value="172"  data-axis-min="0" class="axis-input" style="width:70px"></td><td><input type="number" id="axMax0" min="1" max="2047" value="1811" data-axis-max="0" class="axis-input" style="width:70px"></td><td style="text-align:center"><input type="checkbox" id="axRev0" data-axis-rev="0"></td></tr>
            <tr><td>Right Y (ELE)</td><td><select id="selRY"></select></td><td><input type="number" id="axMin1" min="1" max="2047" value="172"  data-axis-min="1" class="axis-input" style="width:70px"></td><td><input type="number" id="axMax1" min="1" max="2047" value="1811" data-axis-max="1" class="axis-input" style="width:70px"></td><td style="text-align:center"><input type="checkbox" id="axRev1" data-axis-rev="1"></td></tr>
            <tr><td>Left X (RUD)</td> <td><select id="selLX"></select></td><td><input type="number" id="axMin3" min="1" max="2047" value="172"  data-axis-min="3" class="axis-input" style="width:70px"></td><td><input type="number" id="axMax3" min="1" max="2047" value="1811" data-axis-max="3" class="axis-input" style="width:70px"></td><td style="text-align:center"><input type="checkbox" id="axRev3" data-axis-rev="3"></td></tr>
            <tr><td>Left Y (THR)</td> <td><select id="selLY"></select></td><td><input type="number" id="axMin2" min="1" max="2047" value="172"  data-axis-min="2" class="axis-input" style="width:70px"></td><td><input type="number" id="axMax2" min="1" max="2047" value="1811" data-axis-max="2" class="axis-input" style="width:70px"></td><td style="text-align:center"><input type="checkbox" id="axRev2" data-axis-rev="2"></td></tr>
          </tbody>
        </table>
      </div>

      <div>
        <div class="sec-title">Switches</div>
        <table class="cfg-table">
          <thead><tr><th>Name</th><th>Type</th><th>Channel</th><th>Low Val</th><th>Mid Val</th><th>High Val</th><th>Default Pos</th></tr></thead>
          <tbody id="swCfgBody"></tbody>
        </table>
      </div>

      <div>
        <div class="sec-title">Sliders &amp; Pots (LS / RS / S1 / S2)</div>
        <table class="cfg-table">
          <thead><tr><th>Name</th><th>Channel</th></tr></thead>
          <tbody id="slCfgBody"></tbody>
        </table>
      </div>

      <div>
        <div class="sec-title">Trims</div>
        <table class="cfg-table">
          <thead><tr>
            <th>Name</th><th>Channel</th><th>Mode</th>
            <th>Step</th><th>Left/Down val</th><th>Right/Up val</th>
          </tr></thead>
          <tbody id="trCfgBody"></tbody>
        </table>
      </div>

      <div>
        <div class="sec-title">Physical Buttons (S1–S6)</div>
        <table class="cfg-table">
          <thead><tr><th>Name</th><th>Label</th><th>Channel</th><th>Pressed Value</th></tr></thead>
          <tbody id="btnCfgBody"></tbody>
        </table>
      </div>

      <div>
        <div class="sec-title">Lua Buttons (15 virtual)</div>
        <table class="cfg-table">
          <thead><tr><th>#</th><th>Label</th><th>Channel</th><th>Pressed Value</th><th>Color</th></tr></thead>
          <tbody id="luaCfgBody"></tbody>
        </table>
      </div>

      <div>
        <div class="sec-title" style="display:flex;align-items:center;gap:12px;">
          RC PWM Outputs (GPIO 4/6/15/17 &mdash; WCB v3.x Serial1-4 TX)
          <label style="font-size:.8rem;font-weight:normal;display:flex;align-items:center;gap:5px;cursor:pointer;">
            <input type="checkbox" id="pwmExtChk" onchange="onPwmExtChange()">
            Extended range (500&ndash;2500&nbsp;&micro;s)
          </label>
        </div>
        <table class="cfg-table">
          <thead><tr><th>Output</th><th>GPIO</th><th>SBUS Channel</th></tr></thead>
          <tbody id="pwmCfgBody"></tbody>
        </table>
      </div>

      <div class="cfg-io-row">
        <span id="importStatus"></span>
        <button class="io-btn export" onclick="exportConfig()">&#11015; Export JSON</button>
        <label class="io-btn import" style="display:inline-block;">
          &#11014; Import JSON
          <input type="file" id="importFile" accept=".json,application/json"
                 style="display:none" onchange="importConfig(this)">
        </label>
        <button class="save-btn" onclick="saveConfig()">&#128190; Save Config</button>
      </div>
    </div>
  </details>
</div>

<!-- ── WiFi panel ────────────────────────────────────────────────────────────── -->
<div class="settings-wrap">
  <details id="wifiDetails">
    <summary>WiFi Networks</summary>
    <div class="settings-body">
      <div class="wifi-status">
        <span>Status: <b id="wifiStatusSSID">—</b></span>
        <span>IP: <b id="wifiStatusIP">—</b></span>
        <span>mDNS: <b id="wifiStatusMDNS">—</b></span>
      </div>
      <div id="wifiNetList"></div>
      <button class="wifi-add" onclick="wifiAddRow()">+ Add Network</button>
      <div class="wifi-footer">
        <label>Connect:</label>
        <select id="wifiPrefSel"></select>
        <button class="wifi-apply" onclick="wifiApply()">&#8646; Save &amp; Connect</button>
        <span id="wifiMsg"></span>
      </div>
    </div>
  </details>
</div>

<script>
// =============================================================================
//  Constants & state
// =============================================================================
const SBUS_MIN = 172, SBUS_MAX = 1811, SBUS_CENTER = 992;
const SBUS_USER_MIN = 1, SBUS_USER_MAX = 2047;  // extended range for testing

// ─── Unit display ─────────────────────────────────────────────────────────────
//   Internal storage and over-the-wire values stay as raw SBUS.  The toggle
//   switches the DISPLAY between raw SBUS (172-1811) and PWM microseconds
//   (988-2012 standard).  User preference persists in localStorage.
const US_CENTER = 1500, SBUS_TO_US = 0.625, US_TO_SBUS = 1.6;
let   displayUnit = localStorage.getItem('sbusDisplayUnit') || 'sbus';
const fmtVal   = (sbus) => displayUnit === 'us'
                            ? Math.round(US_CENTER + (sbus - SBUS_CENTER) * SBUS_TO_US)
                            : Math.round(sbus);
const parseVal = (raw)  => {
  const n = parseInt(raw); if (isNaN(n)) return null;
  return displayUnit === 'us'
           ? Math.round(SBUS_CENTER + (n - US_CENTER) * US_TO_SBUS)
           : n;
};
const unitLbl = () => displayUnit === 'us' ? 'µs' : 'SBUS';
const altLbl  = (sbus) => displayUnit === 'us'
                            ? `${Math.round(sbus)} SBUS`
                            : `${Math.round(US_CENTER + (sbus - SBUS_CENTER) * SBUS_TO_US)} µs`;

let transport;   // active Transport (WsTransport or SerialTransport — see below)
let cfg = {
  rx:1, ry:2, ly:3, lx:4, sbus24:true,
  aMin: [172, 172, 172, 172],
  aMax: [1811,1811,1811,1811],
  aRev: [false,false,false,false],
  pwm: Array.from({length:4}, (_,i)=>({c:i+1})),
  pwmExt: false,
  sw:  Array.from({length:10}, (_,i)=>({l:`S${i}`, c:0, t:0, v:[172,992,1811], pos:1})),
  sl:  Array.from({length:4},  (_,i)=>({l:['LS','RS','S1','S2'][i]||`SL${i}`, c:0, pct:50})),
  tr:  Array.from({length:6},  (_,i)=>({l:`T${i+1}`, c:0, s:10, cur:992})),
  btn: Array.from({length:6},  (_,i)=>({l:`Btn${i+1}`, c:0, v:1811})),
  lua: Array.from({length:15}, (_,i)=>({l:`Button ${i+1}`, c:0, v:1811}))
};
let g_dbgOn = false;

// Local trim values (SBUS units) — mirrored from server on connect, then tracked locally
let trimCur = Array(6).fill(SBUS_CENTER);

// Joystick state
const sticks = { L:{x:0,y:0,active:false,touchId:null}, R:{x:0,y:0,active:false,touchId:null} };
let joyTimer = null;

// =============================================================================
//  Transport layer  (WebSocket-over-WiFi  ·  Web-Serial-over-USB)
// =============================================================================
// All inbound and outbound messaging is funneled through a Transport instance
// so the same UI can ride either transport without scattering "ws.send" /
// "ws.onmessage" calls throughout the file.  The wire format is identical on
// both paths — JSON objects per message.  Inbound dispatch is centralised in
// handleInboundMessage so every transport reaches the same handlers.
//
// Add a new transport by implementing the four methods below; the Connect
// button (top bar) decides which one to activate.

// --- Base interface (also handy as a no-op fallback) ---
class Transport {
  constructor() {
    this.label = '';                              // user-facing name
    this.onmessage = (_msg) => {};                // inbound dispatcher
    this.onstate   = (_connected) => {};          // connection state change
  }
  open()           { /* override */ }
  close()          { /* override */ }
  send(_obj)       { /* override */ }
  isConnected()    { return false; }
}

// --- WebSocket-over-WiFi (original behaviour) ---
class WsTransport extends Transport {
  constructor() { super(); this.label = 'WiFi'; this.ws = null; this._reconnectTimer = null; }
  open() {
    this.ws = new WebSocket(`ws://${location.hostname}/ws`);
    this.ws.onopen  = () => this.onstate(true);
    this.ws.onclose = () => {
      this.onstate(false);
      // Auto-reconnect — only while THIS transport is still the active one.
      // Switching to SerialTransport sets transport=… elsewhere and we stop.
      this._reconnectTimer = setTimeout(() => { if (transport === this) this.open(); }, 3000);
    };
    this.ws.onerror = () => {};
    this.ws.onmessage = (e) => {
      try { this.onmessage(JSON.parse(e.data)); } catch(_) {}
    };
  }
  close() {
    if (this._reconnectTimer) { clearTimeout(this._reconnectTimer); this._reconnectTimer = null; }
    if (this.ws) {
      // Detach handlers BEFORE closing: the close event fires async, and a
      // stale onclose would call onstate(false) AFTER a newly-activated
      // transport already reported connected — leaving the badge wrongly red.
      this.ws.onopen = this.ws.onclose = this.ws.onerror = this.ws.onmessage = null;
      try { this.ws.close(); } catch(_) {}
    }
    this.ws = null;
  }
  send(obj) {
    if (this.ws && this.ws.readyState === WebSocket.OPEN) this.ws.send(JSON.stringify(obj));
  }
  isConnected() { return this.ws && this.ws.readyState === WebSocket.OPEN; }
}

// --- Web Serial over USB (Chrome / Edge only) -------------------------------
// Newline-delimited JSON over a serial port at 115200 8N1.  Same wire format
// as the WebSocket path.  PINGs the firmware on connect and every 3 s so the
// board's broadcastJson() mirrors WS broadcasts to Serial too (5 s TTL on the
// firmware side).
class SerialTransport extends Transport {
  constructor() {
    super();
    this.label    = 'USB Serial';
    this.port     = null;
    this.reader   = null;
    this.writer   = null;
    this.pingTimer = null;
    this.connected = false;
    this._closing  = false;
  }

  async open() {
    if (!navigator.serial) {
      alert('USB Serial requires a Chromium-based browser (Chrome, Edge, etc.)\n' +
            'In Safari or Firefox, use the WiFi connection instead.');
      this.onstate(false);
      return;
    }
    try {
      this.port = await navigator.serial.requestPort();   // user picks COM port
      await this.port.open({ baudRate: 115200 });

      // Writer: utf8-encode JSON + newline into the port's WritableStream.
      this.writer = this.port.writable.getWriter();

      // Reader: byte stream → TextDecoder → string chunks → our line buffer.
      const decoder = new TextDecoderStream();
      this.port.readable.pipeTo(decoder.writable).catch(() => {});
      this.reader = decoder.readable.getReader();

      this.connected = true;
      this.onstate(true);

      // Read loop runs forever until cancel(); when it exits we treat the
      // port as disconnected.
      this._readLoop();

      // Identify ourselves and keep the host-alive stamp fresh so the
      // firmware echoes broadcastJson() to Serial.
      this.send({ t: 'ping' });
      this.pingTimer = setInterval(() => this.send({ t: 'ping' }), 3000);
      // Pull the firmware's current config — Serial has no equivalent of the
      // WebSocket's open-event auto-sync, so without this the UI would show
      // its initial JS defaults until the user triggers something that fires
      // a buildCfgJson() broadcast.  Small delay so the PING above lands
      // first (serialHostAlive must be true for the broadcast to mirror).
      setTimeout(() => this.send({ t: 'getcfg' }), 100);
    } catch (err) {
      console.error('[SerialTransport] open failed:', err);
      this.connected = false;
      this.onstate(false);
      // Tidy up half-opened state if any.
      try { this.writer && this.writer.releaseLock(); } catch (_) {}
      try { this.port   && await this.port.close();   } catch (_) {}
      this.writer = null;
      this.port   = null;
    }
  }

  async _readLoop() {
    let buf = '';
    try {
      while (true) {
        const { value, done } = await this.reader.read();
        if (done) break;
        buf += value;
        let idx;
        while ((idx = buf.indexOf('\n')) >= 0) {
          const line = buf.substring(0, idx).trim();
          buf = buf.substring(idx + 1);
          // Drop non-JSON noise (CLI prints, boot banner, hand-typed input)
          if (line.length === 0 || line[0] !== '{') continue;
          try { this.onmessage(JSON.parse(line)); } catch (_) {}
        }
      }
    } catch (err) {
      if (!this._closing) console.error('[SerialTransport] read error:', err);
    } finally {
      this.connected = false;
      // Stop pinging a dead port (close() clears this for deliberate closes,
      // but an unexpected unplug exits the loop without close() running).
      if (this.pingTimer) { clearInterval(this.pingTimer); this.pingTimer = null; }
      // Only report disconnected if we're still the ACTIVE transport — this
      // finally can run after the user has already switched transports, and a
      // late onstate(false) would stomp the new transport's badge.
      if (transport === this) this.onstate(false);
    }
  }

  async close() {
    this._closing = true;
    if (this.pingTimer) { clearInterval(this.pingTimer); this.pingTimer = null; }
    try { this.reader && await this.reader.cancel(); } catch (_) {}
    try { this.writer && this.writer.releaseLock();  } catch (_) {}
    try { this.port   && await this.port.close();    } catch (_) {}
    this.reader = null;
    this.writer = null;
    this.port   = null;
    this.connected = false;
    this._closing  = false;
  }

  send(obj) {
    if (!this.writer || !this.connected) return;
    const text = JSON.stringify(obj) + '\n';
    // Fire-and-forget — keeps send() synchronous to match WsTransport's shape.
    // Errors here usually mean the port has been unplugged; the read loop
    // will see the EOF and trigger onstate(false) shortly after.
    this.writer.write(new TextEncoder().encode(text))
      .catch(err => console.error('[SerialTransport] write failed:', err));
  }

  isConnected() { return this.connected; }
}

// Centralised inbound dispatch — every transport routes here.
// Events from the firmware use the 'e' field; direct command replies (PONG)
// use 't' to mirror the outbound command shape.
function handleInboundMessage(msg) {
  if      (msg.e === 'cfg')            applyCfg(msg);
  else if (msg.e === 'chdata')         handleChData(msg);
  else if (msg.e === 'wifi_switching') handleWifiSwitching(msg);
  else if (msg.t === 'pong')           handlePong(msg);
}

// PONG receipt = the firmware is alive and speaks our cfg version.  We don't
// currently surface this to the UI beyond logging, but it's the hook for
// later (e.g. compatibility check, latency display).
function handlePong(msg) {
  console.log('[transport] PONG  ver=' + msg.ver);
}

// Heuristic: are we being served by the ESP itself?  If yes the WebSocket
// path will work; if no (GitHub Pages, file://, anything else off-device) the
// page should default to USB Serial since there's no /ws endpoint to talk to.
// Treats *.github.io and file:// as external; any other hostname (an IP, a
// .local mDNS name, plain "esp32") is assumed to be ESP-served.
function _isEspHosted() {
  if (location.protocol === 'file:') return false;
  if (location.hostname.endsWith('.github.io')) return false;
  return true;
}

// Backwards-compat entry points — every existing call site that does
// connect() or send({...}) still works; they just flow through the transport.
function connect() {
  if (!transport) transport = _isEspHosted() ? new WsTransport() : new SerialTransport();
  transport.onmessage = handleInboundMessage;
  transport.onstate   = setBadge;
  _updateTransportBtn();
  // Auto-open WiFi only.  Web Serial's navigator.serial.requestPort() requires
  // a user gesture (security) so SerialTransport waits for the user to click
  // "USB Serial" in the picker — which provides that gesture.
  if (transport instanceof WsTransport) transport.open();
}
function send(obj) { if (transport) transport.send(obj); }

// ── Transport picker UI ─────────────────────────────────────────────────────
// Toggle the floating menu under the transport button.
function toggleTransportMenu(ev) {
  if (ev) ev.stopPropagation();
  const m   = document.getElementById('transportMenu');
  const btn = document.getElementById('transportBtn');
  if (!m || !btn) return;
  const willShow = !m.classList.contains('show');
  if (willShow) {
    // Position the menu under the button.  Using fixed coords so the menu
    // floats correctly even when the header wraps to a new line.
    const r = btn.getBoundingClientRect();
    m.style.top  = (r.bottom + 4) + 'px';
    m.style.left = Math.max(4, r.right - 130) + 'px';   // right-align to button
    // Mark current transport as active in the menu.
    const kind = (transport instanceof SerialTransport) ? 'serial' : 'wifi';
    m.querySelectorAll('button[data-kind]').forEach(b => {
      b.classList.toggle('active', b.dataset.kind === kind);
    });
  }
  m.classList.toggle('show', willShow);
}

// Close the menu when the user clicks anywhere else.
document.addEventListener('click', (ev) => {
  const m = document.getElementById('transportMenu');
  if (m && m.classList.contains('show') && !m.contains(ev.target) &&
      ev.target?.id !== 'transportBtn') {
    m.classList.remove('show');
  }
});

// Swap the active transport.  Closes the old one (which kills its reconnect
// timer / read loop) and opens the new one.  setBadge will fire on state
// changes so the badge tracks the new connection.
async function chooseTransport(kind) {
  document.getElementById('transportMenu').classList.remove('show');
  // Skip if user picked the already-active kind — avoids a pointless reset.
  const currentKind = (transport instanceof SerialTransport) ? 'serial' : 'wifi';
  if (kind === currentKind && transport.isConnected()) return;
  // Tear down the current transport first so its reconnect timer doesn't keep
  // resurrecting it under the new one.
  if (transport) {
    try { await transport.close(); } catch (_) {}
  }
  setBadge(false);                                     // immediate visual feedback
  transport = (kind === 'serial') ? new SerialTransport() : new WsTransport();
  transport.onmessage = handleInboundMessage;
  transport.onstate   = setBadge;
  _updateTransportBtn();
  transport.open();
}

// Update the picker button label to match the active transport.
function _updateTransportBtn() {
  const btn = document.getElementById('transportBtn');
  if (btn && transport) btn.textContent = transport.label + ' ▾';
}

// ─── SVG transmitter handlers (Phase 1) ───────────────────────────────────
//   svgBtn   = matrix buttons S1-S6 (cfg.btn[0..5]) — momentary press/release
//   svgTr    = trim arrow zones — fires {t:'tr', i, d, p} like the old widgets
//   svgSw    = switches SA-SH — click cycles through positions 0/1/2
//   (arrow-function style avoids Arduino preprocessor scanning these as C++)

const svgBtnPress = (ev, i) => {
  if (ev) ev.preventDefault();
  send({t:'btn', i, p:true});
  const el = document.querySelector('.svg-btn[data-i="'+i+'"]');
  if (el) el.classList.add('pressed');
};
const svgBtnRelease = (ev, i) => {
  if (ev) ev.preventDefault();
  const el = document.querySelector('.svg-btn[data-i="'+i+'"]');
  if (!el || !el.classList.contains('pressed')) return;
  el.classList.remove('pressed');
  send({t:'btn', i, p:false});
};

// Trim zone press/release.  i = trim idx (0-5), d = -1 or +1 direction.
const _svgTrActive = {};
const svgTrPress = (ev, i, d) => {
  if (ev) ev.preventDefault();
  _svgTrActive[i+'_'+d] = true;
  const el = document.querySelector('.svg-tr[data-i="'+i+'"][data-d="'+d+'"]');
  if (el) el.classList.add('pressed');
  send({t:'tr', i, d, p:true});
};
const svgTrRelease = (ev, i) => {
  if (ev) ev.preventDefault();
  let any = false;
  document.querySelectorAll('.svg-tr[data-i="'+i+'"]').forEach(el => {
    if (el.classList.contains('pressed')) { any = true; el.classList.remove('pressed'); }
  });
  for (const k in _svgTrActive) if (k.startsWith(i+'_')) delete _svgTrActive[k];
  if (any) send({t:'tr', i, d:0, p:false});
};

// Switch cycle on click.  Skips position 1 for 2-pos / momentary switches.
// Rotates the lever sub-group around the bushing centre to visualise position.
const svgSwCycle = (i) => {
  if (!cfg || !cfg.sw || !cfg.sw[i]) return;
  const sw = cfg.sw[i];
  const positions = sw.t === 0 ? [0,1,2] : [0,2];
  const el = document.querySelector('.svg-sw[data-i="'+i+'"]');
  if (!el) return;
  let cur = +el.dataset.pos;
  let idx = positions.indexOf(cur);
  if (idx < 0) idx = 0;
  const next = positions[(idx + 1) % positions.length];
  el.dataset.pos = next;
  applySvgSwRotation(el, next);
  send({t:'sw', i, p:next});
};

// Apply a rotation to the switch lever based on position (0/1/2).
// Pivot point comes from the switch's data-px/data-py attrs (= bushing centre).
const applySvgSwRotation = (el, pos) => {
  const lever = el.querySelector('.lever');
  if (!lever) return;
  const px = +el.dataset.px, py = +el.dataset.py;
  // pos 0 → lever leans LEFT (-25°), pos 1 → straight up (0°), pos 2 → leans RIGHT (+25°)
  const deg = (pos === 0) ? -25 : (pos === 2) ? 25 : 0;
  lever.setAttribute('transform', `rotate(${deg}, ${px}, ${py})`);
};

// Initialise lever positions from cfg on first render (default pos 0/1).
const initSvgSwitchPositions = () => {
  document.querySelectorAll('.svg-sw').forEach(el => {
    const pos = +el.dataset.pos || 0;
    applySvgSwRotation(el, pos);
  });
};

// ─── Joystick gimbals (Phase 2 — draggable in SVG) ─────────────────────────
// Drag the thumb anywhere inside the gimbal cutout; release springs back.
// Sends {t:'a', rx, ry, ly, lx} with all four axes (each -1.0 .. +1.0).
const _stickState = {
  L: { x:0, y:0, active:false, ptrId:null, cx:240, cy:485, half:78 },
  R: { x:0, y:0, active:false, ptrId:null, cx:760, cy:485, half:78 },
};
let _stickThrottleTimer = null;

const _stickSend = () => {
  if (_stickThrottleTimer) return;
  _stickThrottleTimer = setTimeout(() => {
    _stickThrottleTimer = null;
    send({ t:'a',
      rx: _stickState.R.x,
      ry: _stickState.R.y,
      lx: _stickState.L.x,
      ly: _stickState.L.y });
  }, 30);
};

const _stickThumbXY = (side) => {
  const s = _stickState[side];
  // Translate the thumb circle/socket so it follows the pointer.
  const dx = s.x * s.half;
  const dy = -s.y * s.half;   // SVG y grows DOWN, but joystick "up" is +y
  const g = document.getElementById('stickThumb'+side);
  if (g) g.setAttribute('transform', `translate(${dx},${dy})`);
};

// Cache the SVG root so window-level listeners (where ev.target may be a
// non-SVG element like body) can still compute viewBox coordinates.
let _txSvgRoot = null;
const _getTxSvg = (ev) => {
  if (_txSvgRoot && _txSvgRoot.isConnected) return _txSvgRoot;
  // ev.target.ownerSVGElement works when the pointer is inside the SVG;
  // otherwise look up by the wrapper id.
  let svg = ev && ev.target && ev.target.ownerSVGElement;
  if (!svg) {
    const wrap = document.querySelector('.tx-svg-wrap svg');
    if (wrap) svg = wrap;
  }
  if (svg) _txSvgRoot = svg;
  return svg;
};

const _stickEventToXY = (ev, side) => {
  const s   = _stickState[side];
  // Convert client coords to SVG viewBox coords using getCTM-inverse approach
  const svg = _getTxSvg(ev);
  if (!svg) return null;
  const pt  = svg.createSVGPoint();
  pt.x = ev.clientX; pt.y = ev.clientY;
  const m   = svg.getScreenCTM();
  if (!m) return null;
  const lp  = pt.matrixTransform(m.inverse());
  let nx = (lp.x - s.cx) / s.half;
  let ny = (s.cy - lp.y) / s.half;   // invert Y so up = +1
  // Clamp to unit circle/square
  nx = Math.max(-1, Math.min(1, nx));
  ny = Math.max(-1, Math.min(1, ny));
  return { x:nx, y:ny };
};

// Mobile-robust pattern: on pointerdown we ALSO attach window-level
// pointermove/up/cancel listeners.  Even if Safari/Chrome on iOS/Android
// silently drops the SVG element's pointer capture mid-gesture (very common
// on SVG hit-tests), the window listeners keep the drag alive until the
// finger genuinely lifts.  The window listeners auto-detach on release.
const _stickWinHandlers = { L:null, R:null };

const svgStickDown = (ev, side) => {
  ev.preventDefault();
  const s = _stickState[side];
  s.active = true;
  s.ptrId  = ev.pointerId;
  try { ev.target.setPointerCapture(ev.pointerId); } catch(e) {}
  const p = _stickEventToXY(ev, side);
  if (p) { s.x = p.x; s.y = p.y; }
  _stickThumbXY(side);
  _stickSend();
  // Tear down any prior listeners for this side, then install fresh ones.
  _stickDetachWin(side);
  const onMove   = (e) => { if (e.pointerId === s.ptrId) svgStickMove(e, side); };
  const onUpCxl  = (e) => { if (e.pointerId === s.ptrId) svgStickUp(e, side); };
  _stickWinHandlers[side] = { onMove, onUpCxl };
  window.addEventListener('pointermove',   onMove,  { passive:false });
  window.addEventListener('pointerup',     onUpCxl, { passive:false });
  window.addEventListener('pointercancel', onUpCxl, { passive:false });
};
const _stickDetachWin = (side) => {
  const h = _stickWinHandlers[side];
  if (!h) return;
  window.removeEventListener('pointermove',   h.onMove);
  window.removeEventListener('pointerup',     h.onUpCxl);
  window.removeEventListener('pointercancel', h.onUpCxl);
  _stickWinHandlers[side] = null;
};
const svgStickMove = (ev, side) => {
  const s = _stickState[side];
  if (!s.active || ev.pointerId !== s.ptrId) return;
  ev.preventDefault();
  const p = _stickEventToXY(ev, side);
  if (!p) return;
  s.x = p.x; s.y = p.y;
  _stickThumbXY(side);
  _stickSend();
};
const svgStickUp = (ev, side) => {
  const s = _stickState[side];
  if (!s.active) return;
  ev.preventDefault();
  s.active = false;
  const pid = s.ptrId;
  s.ptrId  = null;
  try { ev.target && ev.target.releasePointerCapture && ev.target.releasePointerCapture(pid); } catch(e) {}
  _stickDetachWin(side);
  s.x = 0; s.y = 0;
  _stickThumbXY(side);
  _stickSend();
};

// ─── LS / RS vertical sliders (Phase 2) ───────────────────────────────────
// Geometry MUST match the SVG rect dims above:
//   LS / RS (vertical, sides):       rail x=76/914  y=430  w=10  h=120,  thumb 22×16
//   S3    (vertical, bottom-centre): rail x=494     y=710  w=10  h=80,   thumb 22×14
//   J5/J6 (horizontal, below sticks): rail x=140/665 y=587 w=195 h=6,    thumb 14×16
//
// Geometry is per-slider so vertical and horizontal rails share the same handler
// path.  thumbExtent = thumb length along the drag axis (h for vertical sliders,
// w for horizontal); used to clamp the rendered thumb so it doesn't run off the
// rail end.  pct convention: 100 = top (vertical) or right (horizontal); 0 = the
// opposite end.
const _sliderState = {};   // idx → { active, ptrId, pct }
const _sliderGeom  = {
  // LS — left rail (cfg.sl[0])
  0: { orient:'v', thumb:'sliderThumbLS', top:430, bot:550, thumbExtent:16 },
  // RS — right rail (cfg.sl[1])
  1: { orient:'v', thumb:'sliderThumbRS', top:430, bot:550, thumbExtent:16 },
  // S3 — bottom-centre vertical (cfg.sl[4]; X20 middle slider, was MS)
  4: { orient:'v', thumb:'sliderThumbS3', top:710, bot:790, thumbExtent:14 },
  // J5 — horizontal below left stick (cfg.sl[5]; X20 L-stick twist).
  // spring:true → snaps back to centre (pct=50) on release, like a stick axis.
  5: { orient:'h', thumb:'sliderThumbJ5', left:140, right:335, thumbExtent:14, spring:true },
  // J6 — horizontal below right stick (cfg.sl[6]; X20 R-stick twist).  Spring-back.
  6: { orient:'h', thumb:'sliderThumbJ6', left:665, right:860, thumbExtent:14, spring:true },
};

const _sliderPctFromEv = (ev, idx) => {
  const g = _sliderGeom[idx];
  if (!g) return null;
  const svg = _getTxSvg(ev);
  if (!svg) return null;
  const pt  = svg.createSVGPoint();
  pt.x = ev.clientX; pt.y = ev.clientY;
  const m   = svg.getScreenCTM();
  if (!m) return null;
  const lp = pt.matrixTransform(m.inverse());
  // Use the FULL rail length for input mapping so dragging anywhere on the rail
  // feels right; thumb position is clamped to the usable range during render.
  const v = (g.orient === 'v')
    ? (g.bot - lp.y) / (g.bot - g.top)
    : (lp.x - g.left) / (g.right - g.left);
  return Math.max(0, Math.min(100, Math.round(v * 100)));
};
const _sliderRenderThumb = (idx) => {
  const g = _sliderGeom[idx];
  if (!g) return;
  const pct   = (_sliderState[idx] && _sliderState[idx].pct !== undefined) ? _sliderState[idx].pct : 50;
  const thumb = document.getElementById(g.thumb);
  if (!thumb) return;
  if (g.orient === 'v') {
    const usableTop = g.top;
    const usableBot = g.bot - g.thumbExtent;
    const y = usableBot - (pct / 100) * (usableBot - usableTop);
    thumb.setAttribute('y', y);
  } else {
    const usableLeft  = g.left;
    const usableRight = g.right - g.thumbExtent;
    const x = usableLeft + (pct / 100) * (usableRight - usableLeft);
    thumb.setAttribute('x', x);
  }
};
// Window-level fallback listeners (same pattern as the sticks).
const _sliderWinHandlers = {};
const _sliderDetachWin = (idx) => {
  const h = _sliderWinHandlers[idx];
  if (!h) return;
  window.removeEventListener('pointermove',   h.onMove);
  window.removeEventListener('pointerup',     h.onUpCxl);
  window.removeEventListener('pointercancel', h.onUpCxl);
  _sliderWinHandlers[idx] = null;
};
const svgSliderDown = (ev, idx) => {
  ev.preventDefault();
  if (!_sliderState[idx]) _sliderState[idx] = { active:false, ptrId:null, pct:50 };
  _sliderState[idx].active = true;
  _sliderState[idx].ptrId  = ev.pointerId;
  try { ev.target.setPointerCapture(ev.pointerId); } catch(e) {}
  const pct = _sliderPctFromEv(ev, idx);
  if (pct !== null) {
    _sliderState[idx].pct = pct;
    _sliderRenderThumb(idx);
    send({t:'sl', i:idx, v:pct});
  }
  _sliderDetachWin(idx);
  const s = _sliderState[idx];
  const onMove  = (e) => { if (e.pointerId === s.ptrId) svgSliderMove(e, idx); };
  const onUpCxl = (e) => { if (e.pointerId === s.ptrId) svgSliderUp(e, idx); };
  _sliderWinHandlers[idx] = { onMove, onUpCxl };
  window.addEventListener('pointermove',   onMove,  { passive:false });
  window.addEventListener('pointerup',     onUpCxl, { passive:false });
  window.addEventListener('pointercancel', onUpCxl, { passive:false });
};
const svgSliderMove = (ev, idx) => {
  const s = _sliderState[idx];
  if (!s || !s.active || ev.pointerId !== s.ptrId) return;
  ev.preventDefault();
  const pct = _sliderPctFromEv(ev, idx);
  if (pct !== null && pct !== s.pct) {
    s.pct = pct;
    _sliderRenderThumb(idx);
    send({t:'sl', i:idx, v:pct});
  }
};
const svgSliderUp = (ev, idx) => {
  const s = _sliderState[idx];
  if (!s || !s.active) return;
  ev.preventDefault();
  s.active = false;
  const pid = s.ptrId;
  s.ptrId  = null;
  try { ev.target && ev.target.releasePointerCapture && ev.target.releasePointerCapture(pid); } catch(e) {}
  _sliderDetachWin(idx);

  // Spring-back sliders (J5/J6) snap their thumb to centre on release and
  // immediately push a centred SBUS value, same idea as joystick axes
  // springing back to neutral when you let go.
  const g = _sliderGeom[idx];
  if (g && g.spring && s.pct !== 50) {
    s.pct = 50;
    _sliderRenderThumb(idx);
    send({t:'sl', i:idx, v:50});
  }
};

// ─── S1 / S2 rotary knobs (Phase 2) ────────────────────────────────────────
// Drag vertically across the knob: up = increase, down = decrease.  ~150px
// of vertical travel = full 0-100 sweep.  Indicator line rotates with value.
const _knobState = {};   // idx → { active, ptrId, pct, startY, startPct }
const _knobIndicators = { 2:'knobIndS1', 3:'knobIndS2' };
const _knobValueLabels = { 2:'knobValS1', 3:'knobValS2' };
const _knobCenters     = { 2:{cx:430,cy:390}, 3:{cx:570,cy:390} };

const _knobRender = (idx) => {
  const pct = (_knobState[idx] && _knobState[idx].pct !== undefined) ? _knobState[idx].pct : 50;
  // pct 0 → -150°,  pct 50 → 0°,  pct 100 → +150°
  const deg = (pct - 50) / 50 * 150;
  const c   = _knobCenters[idx];
  const ind = document.getElementById(_knobIndicators[idx]);
  if (ind) ind.setAttribute('transform', `rotate(${deg} ${c.cx} ${c.cy})`);
  const lbl = document.getElementById(_knobValueLabels[idx]);
  if (lbl) lbl.textContent = pct;
};
// Window-level fallback listeners (same pattern as sticks/sliders).
const _knobWinHandlers = {};
const _knobDetachWin = (idx) => {
  const h = _knobWinHandlers[idx];
  if (!h) return;
  window.removeEventListener('pointermove',   h.onMove);
  window.removeEventListener('pointerup',     h.onUpCxl);
  window.removeEventListener('pointercancel', h.onUpCxl);
  _knobWinHandlers[idx] = null;
};
const svgKnobDown = (ev, idx) => {
  ev.preventDefault();
  if (!_knobState[idx]) _knobState[idx] = { pct:50 };
  const s = _knobState[idx];
  s.active   = true;
  s.ptrId    = ev.pointerId;
  s.startY   = ev.clientY;
  s.startPct = s.pct;
  try { ev.target.setPointerCapture(ev.pointerId); } catch(e) {}
  _knobDetachWin(idx);
  const onMove  = (e) => { if (e.pointerId === s.ptrId) svgKnobMove(e, idx); };
  const onUpCxl = (e) => { if (e.pointerId === s.ptrId) svgKnobUp(e, idx); };
  _knobWinHandlers[idx] = { onMove, onUpCxl };
  window.addEventListener('pointermove',   onMove,  { passive:false });
  window.addEventListener('pointerup',     onUpCxl, { passive:false });
  window.addEventListener('pointercancel', onUpCxl, { passive:false });
};
const svgKnobMove = (ev, idx) => {
  const s = _knobState[idx];
  if (!s || !s.active || ev.pointerId !== s.ptrId) return;
  ev.preventDefault();
  const dy = s.startY - ev.clientY;     // drag UP = positive
  const newPct = Math.max(0, Math.min(100, Math.round(s.startPct + dy * 0.7)));
  if (newPct !== s.pct) {
    s.pct = newPct;
    _knobRender(idx);
    send({t:'sl', i:idx, v:newPct});
  }
};
const svgKnobUp = (ev, idx) => {
  const s = _knobState[idx];
  if (!s || !s.active) return;
  ev.preventDefault();
  s.active = false;
  const pid = s.ptrId;
  s.ptrId  = null;
  try { ev.target && ev.target.releasePointerCapture && ev.target.releasePointerCapture(pid); } catch(e) {}
  _knobDetachWin(idx);
};

const initSvgAnalogs = () => {
  // Slider indices match _sliderGeom keys: 0=LS, 1=RS, 4=S3, 5=J5, 6=J6.
  // (cfg.sl[2]/[3] are the S1/S2 knobs — rendered via _knobRender instead.)
  Object.keys(_sliderGeom).forEach(k => _sliderRenderThumb(+k));
  _knobRender(2);
  _knobRender(3);
};

// ─── LUA buttons mirrored in the SVG screen pod ────────────────────────────
// Layout: 5 columns × 3 rows inside the screen (x=320..680, y=130..280).
const svgLuaPress = (ev, i) => {
  if (ev) ev.preventDefault();
  send({t:'lua', i, p:true});
  const r = document.querySelector('.svg-lua[data-i="'+i+'"] rect');
  if (r) r.setAttribute('opacity', '0.55');
};
const svgLuaRelease = (ev, i) => {
  if (ev) ev.preventDefault();
  send({t:'lua', i, p:false});
  const r = document.querySelector('.svg-lua[data-i="'+i+'"] rect');
  if (r) r.setAttribute('opacity', '1');
};
const renderScreenLuaButtons = () => {
  const grid = document.getElementById('screenLuaGrid');
  if (!grid || !cfg || !cfg.lua) return;
  while (grid.firstChild) grid.removeChild(grid.firstChild);
  // Fill the screen pod cleanly — 5×3 grid sized so it fits inside the inner
  // screen rect (x=305..695, y=125..285) with a small margin on each side.
  const cols = 5, cw = 72, ch = 44, gx = 4, gy = 4;
  const totalW = cols * cw + (cols - 1) * gx;      // 5×72 + 4×4 = 376
  const totalH = 3    * ch + 2        * gy;        // 3×44 + 2×4 = 140
  const x0 = 305 + (390 - totalW) / 2;             // centred horizontally
  const y0 = 130;                                  // sits below the header line
  const NS = 'http://www.w3.org/2000/svg';
  cfg.lua.forEach((btn, i) => {
    if (i >= 15) return;
    const col = i % cols, row = Math.floor(i / cols);
    const x = x0 + col * (cw + gx);
    const y = y0 + row * (ch + gy);
    const assigned = btn.c !== 0;
    const color = btn.k || '#4fc3f7';

    const g = document.createElementNS(NS, 'g');
    g.setAttribute('class', 'svg-lua');
    g.setAttribute('data-i', i);
    if (assigned) {
      g.setAttribute('onpointerdown', 'svgLuaPress(event,'+i+')');
      g.setAttribute('onpointerup',   'svgLuaRelease(event,'+i+')');
      g.setAttribute('onpointerleave','svgLuaRelease(event,'+i+')');
      g.setAttribute('onpointercancel','svgLuaRelease(event,'+i+')');
    }
    const r = document.createElementNS(NS, 'rect');
    r.setAttribute('x', x); r.setAttribute('y', y);
    r.setAttribute('width', cw); r.setAttribute('height', ch);
    r.setAttribute('rx', 4);
    r.setAttribute('fill', assigned ? color : 'transparent');
    r.setAttribute('stroke', color);
    r.setAttribute('stroke-width', assigned ? '1' : '1.5');
    if (!assigned) r.setAttribute('stroke-dasharray', '3,2');
    if (!assigned) r.setAttribute('opacity', '0.45');
    g.appendChild(r);

    const t = document.createElementNS(NS, 'text');
    t.setAttribute('x', x + cw/2);
    t.setAttribute('y', y + ch/2 + 3);
    t.setAttribute('text-anchor', 'middle');
    t.setAttribute('fill', assigned ? '#000' : '#888');
    t.setAttribute('font-size', '8');
    t.setAttribute('font-weight', '700');
    t.setAttribute('font-family', 'sans-serif');
    t.style.pointerEvents = 'none';
    let label = (btn.l || ('Btn'+(i+1)));
    if (label.length > 14) label = label.substring(0,13) + '…';
    t.textContent = label;
    g.appendChild(t);

    grid.appendChild(g);
  });
};

// Momentary switches: press → "active" position, release → back to default.
//   defaultPos 0 (low)  → active = 2 (high)
//   defaultPos 1 (mid)  → active = 2 (high)   — mid isn't a sensible momentary idle
//   defaultPos 2 (high) → active = 0 (low)
const _swActivePos = (sw) => ((sw && sw.d === 2) ? 0 : 2);
const _swDefaultPos = (sw) => (sw && sw.d != null ? sw.d : 0);

const svgSwMomentaryPress = (ev, i) => {
  if (ev) ev.preventDefault();
  const el = document.querySelector('.svg-sw[data-i="'+i+'"]');
  if (!el || !cfg || !cfg.sw || !cfg.sw[i]) return;
  if (el.dataset.held === '1') return;       // already held — ignore re-entry
  el.dataset.held = '1';
  const active = _swActivePos(cfg.sw[i]);
  el.dataset.pos = active;
  applySvgSwRotation(el, active);
  send({t:'sw', i, p:active});
};
const svgSwMomentaryRelease = (ev, i) => {
  if (ev) ev.preventDefault();
  const el = document.querySelector('.svg-sw[data-i="'+i+'"]');
  if (!el || !cfg || !cfg.sw || !cfg.sw[i]) return;
  if (el.dataset.held !== '1') return;
  el.dataset.held = '0';
  const def = _swDefaultPos(cfg.sw[i]);
  el.dataset.pos = def;
  applySvgSwRotation(el, def);
  send({t:'sw', i, p:def});
};

// ── Unified SVG switch event dispatcher ──────────────────────────────────────
// Every .svg-sw element gets the same handlers; behavior is decided at runtime
// from cfg.sw[i].t so the Type dropdown (3-way / 2-way / Momentary) takes effect
// without needing to re-render the SVG.
//   Non-momentary:  pointerup cycles through valid positions for the type.
//   Momentary:      pointerdown → active value, release/leave/cancel → default.
const svgSwOnDown = (ev, i) => {
  if (!cfg || !cfg.sw || !cfg.sw[i]) return;
  if (cfg.sw[i].t === 2) svgSwMomentaryPress(ev, i);
};
const svgSwOnUp = (ev, i) => {
  if (!cfg || !cfg.sw || !cfg.sw[i]) return;
  if (cfg.sw[i].t === 2) svgSwMomentaryRelease(ev, i);
  else                    svgSwCycle(i);
};
const svgSwOnLeave = (ev, i) => {
  if (!cfg || !cfg.sw || !cfg.sw[i]) return;
  if (cfg.sw[i].t === 2) svgSwMomentaryRelease(ev, i);
};

// Strip the static inline handlers from every .svg-sw and rewire them to the
// unified dispatcher above.  Called once at startup; safe to call again later.
function wireSvgSwitchHandlers() {
  document.querySelectorAll('.svg-sw').forEach(el => {
    if (el.dataset.unified === '1') return;     // already wired
    const i = +el.dataset.i;
    ['click','pointerdown','pointerup','pointerleave','pointercancel'].forEach(evt => {
      el['on' + evt] = null;
      el.removeAttribute('on' + evt);
    });
    el.addEventListener('pointerdown',  (ev) => svgSwOnDown(ev, i));
    el.addEventListener('pointerup',    (ev) => svgSwOnUp(ev, i));
    el.addEventListener('pointerleave', (ev) => svgSwOnLeave(ev, i));
    el.addEventListener('pointercancel',(ev) => svgSwOnLeave(ev, i));
    el.style.cursor = 'pointer';
    el.dataset.unified = '1';
  });
}

function setBadge(on) {
  const b = document.getElementById('connBadge');
  b.textContent = on ? 'CONNECTED' : 'DISCONNECTED';
  b.className   = on ? 'connected' : '';
}

// =============================================================================
//  Config
// =============================================================================
function applyCfg(msg) {
  cfg.rx     = msg.rx    || 1;
  cfg.ry     = msg.ry    || 2;
  cfg.ly     = msg.ly    || 3;
  cfg.lx     = msg.lx    || 4;
  cfg.sbus24 = (msg.sbus24 !== undefined) ? msg.sbus24 : true;
  g_dbgOn    = msg.dbg   || false;
  if (Array.isArray(msg.aMin)) cfg.aMin = msg.aMin;
  if (Array.isArray(msg.aMax)) cfg.aMax = msg.aMax;
  if (Array.isArray(msg.aRev)) cfg.aRev = msg.aRev.map(v => !!v);
  if (Array.isArray(msg.sw))  cfg.sw  = msg.sw;
  if (Array.isArray(msg.sl))  cfg.sl  = msg.sl;
  if (Array.isArray(msg.tr))  {
    cfg.tr  = msg.tr;
    trimCur = cfg.tr.map(t => t.cur !== undefined ? t.cur : SBUS_CENTER);
  }
  if (Array.isArray(msg.btn)) cfg.btn = msg.btn;
  if (Array.isArray(msg.lua)) {
    cfg.lua = msg.lua;
  } else if (!cfg.lua) {
    cfg.lua = Array.from({length:15}, (_,i)=>({l:`Button ${i+1}`,c:0,v:1811,k:'#4fc3f7'}));
  }
  // PWM outputs
  if (Array.isArray(msg.pwm)) cfg.pwm = msg.pwm;
  if (msg.pwmExt != null) cfg.pwmExt = !!msg.pwmExt;
  // WiFi
  if (Array.isArray(msg.wifiNets)) cfg.wifiNets = msg.wifiNets;
  if (msg.wifiPref  != null) cfg.wifiPref  = msg.wifiPref;
  if (msg.wifiNet   != null) cfg.wifiNet   = msg.wifiNet;
  if (msg.wifiIP    != null) cfg.wifiIP    = msg.wifiIP;
  if (msg.wifiMDNS  != null) cfg.wifiMDNS  = msg.wifiMDNS;
  renderAll();
  renderWifiPanel();
}

function toggleMode() {
  cfg.sbus24 = !cfg.sbus24;
  send({t:'mode', sbus24:cfg.sbus24});
  updateModeBtn();
}
function toggleDebug() {
  g_dbgOn = !g_dbgOn;
  send({t:'dbg', on:g_dbgOn});
  updateDebugBtn();
}
function updateModeBtn() {
  const btn = document.getElementById('modeBtn');
  if (cfg.sbus24) { btn.textContent='SBUS-24'; btn.className='hdr-btn active-24'; }
  else            { btn.textContent='SBUS-16'; btn.className='hdr-btn active-16'; }
}
function updateDebugBtn() {
  const btn    = document.getElementById('debugBtn');
  const offMsg = document.getElementById('dbgOffMsg');
  if (g_dbgOn) {
    btn.textContent='DEBUG ON'; btn.className='hdr-btn dbg-on';
    if (offMsg) offMsg.style.display='none';
  } else {
    btn.textContent='DEBUG OFF'; btn.className='hdr-btn';
    if (offMsg) offMsg.style.display='';
    const grid=document.getElementById('dbgGrid');
    const info=document.getElementById('dbgInfo');
    if (grid) grid.style.display='none';
    if (info) info.style.display='none';
  }
}

// =============================================================================
//  Render
// =============================================================================
function axisToSbus(v) {
  return Math.round((v*0.5+0.5)*(SBUS_MAX-SBUS_MIN)+SBUS_MIN);
}
function axisToSbusRange(v, mn, mx) {
  return Math.max(SBUS_USER_MIN, Math.min(SBUS_USER_MAX, Math.round((v*0.5+0.5)*(mx-mn)+mn)));
}
function sbusFromPct(pct) {
  return Math.round(pct/100*(SBUS_MAX-SBUS_MIN)+SBUS_MIN);
}
function trimOffset(cur) {
  const d = cur - SBUS_CENTER;
  return (d >= 0 ? '+' : '') + d;
}
function buildChSel(val, includeNone) {
  const sel = document.createElement('select');
  if (includeNone) {
    const o=document.createElement('option'); o.value='0'; o.textContent='None';
    if (!val) o.selected=true; sel.appendChild(o);
  }
  for (let c=1;c<=24;c++) {
    const o=document.createElement('option'); o.value=String(c); o.textContent=`CH${c}`;
    if (c===val) o.selected=true; sel.appendChild(o);
  }
  return sel;
}

function renderAll() {
  updateModeBtn();
  updateDebugBtn();
  updateStickLabels();
  updateReadouts();
  renderSwitchCols();
  renderSliders();      // LS beside left stick, RS beside right stick
  renderPots();         // S1/S2 in pot row above sticks (hidden — SVG is the real UI)
  renderTrimBank();     // all 6 trims in a row below sticks (hidden — SVG)
  renderButtons();      // physical S1-S6 (hidden — SVG); RB1/RB2 → switches SI/SJ
  renderLuaButtons();        // 15 configurable virtual buttons (HTML row above)
  renderScreenLuaButtons();  // same buttons mirrored inside the SVG screen pod
  renderSettings();
}

function updateStickLabels() {
  document.getElementById('lblL').textContent =
    `LEFT  LX:CH${cfg.lx}  LY:CH${cfg.ly}`;
  document.getElementById('lblR').textContent =
    `RIGHT  RX:CH${cfg.rx}  RY:CH${cfg.ry}`;
}
function updateReadouts() {
  // Use per-axis range + reverse so readout reflects actual SBUS output
  const rev = (i) => (cfg.aRev && cfg.aRev[i]) ? -1 : 1;
  // Live readouts respect the unit toggle (SBUS or µs).
  document.getElementById('readL').innerHTML =
    `LX:&nbsp;${fmtVal(axisToSbusRange(rev(3)* sticks.L.x,  cfg.aMin[3],cfg.aMax[3]))}&nbsp;&nbsp;` +
    `LY:&nbsp;${fmtVal(axisToSbusRange(rev(2)*-sticks.L.y, cfg.aMin[2],cfg.aMax[2]))}`;
  document.getElementById('readR').innerHTML =
    `RX:&nbsp;${fmtVal(axisToSbusRange(rev(0)* sticks.R.x,  cfg.aMin[0],cfg.aMax[0]))}&nbsp;&nbsp;` +
    `RY:&nbsp;${fmtVal(axisToSbusRange(rev(1)*-sticks.R.y, cfg.aMin[1],cfg.aMax[1]))}`;
}

// ── Switches ──────────────────────────────────────────────────────────────────
// Build a single switch card for index i in cfg.sw
function makeSwitchCard(i) {
  const sw   = cfg.sw[i];
  const card = document.createElement('div');
  card.className = 'sw-card';

  const nm = document.createElement('div'); nm.className='sw-name'; nm.textContent=sw.l;
  const ch = document.createElement('div'); ch.className='sw-ch';
  ch.textContent = sw.c ? `CH${sw.c}` : 'N/A';
  card.appendChild(nm); card.appendChild(ch);

  if (sw.t === 2) {
    // Momentary — press jumps to the "active" pos (opposite extreme from
    // defaultPos); release returns to defaultPos (not hard-coded 0).
    const btn = document.createElement('button');
    btn.className = 'sw-mom'; btn.textContent = sw.l;
    const active = _swActivePos(sw);
    const def    = _swDefaultPos(sw);
    const press = () => { btn.classList.add('held');    send({t:'sw',i,p:active}); };
    const rel   = () => { btn.classList.remove('held'); send({t:'sw',i,p:def}); };
    btn.addEventListener('mousedown',  press);
    btn.addEventListener('mouseup',    rel);
    btn.addEventListener('mouseleave', rel);
    btn.addEventListener('touchstart', (e)=>{e.preventDefault();press();},{passive:false});
    btn.addEventListener('touchend',   (e)=>{e.preventDefault();rel();},  {passive:false});
    card.appendChild(btn);
  } else {
    const toggle  = document.createElement('div'); toggle.className='sw-toggle';
    const numPos  = (sw.t===1) ? 2 : 3;
    const labels  = numPos===2 ? ['▼','▲'] : ['▼','—','▲'];
    // Render high→mid→low top-to-bottom
    for (let p = numPos-1; p >= 0; p--) {
      const seg = document.createElement('button');
      seg.className = 'sw-seg' + (sw.pos===p ? ' sel' : '');
      seg.textContent = labels[p];
      seg.addEventListener('click', () => {
        sw.pos = p;
        toggle.querySelectorAll('.sw-seg').forEach((s,si) => {
          s.classList.toggle('sel', (numPos-1-si)===p);
        });
        send({t:'sw', i, p});
      });
      toggle.appendChild(seg);
    }
    card.appendChild(toggle);
  }
  return card;
}

// Build a switch pyramid:  topIdx single card, midIdx single card, then pair
// side-by-side, then optional bottomIdx single card under the pair.
//   Left:  SF(5) top, SE(4) mid, SA(0)+SB(1) pair, SI(8) bottom
//   Right: SH(7) top, SG(6) mid, SC(2)+SD(3) pair, SJ(9) bottom
function renderSwitchCols() {
  function buildPyramid(id, topIdx, midIdx, pairA, pairB, bottomIdx) {
    const root = document.getElementById(id);
    root.innerHTML = '';
    root.appendChild(makeSwitchCard(topIdx));   // SF or SH
    root.appendChild(makeSwitchCard(midIdx));   // SE or SG
    const pair = document.createElement('div');
    pair.className = 'sw-pair';
    pair.appendChild(makeSwitchCard(pairA));    // SA or SC
    pair.appendChild(makeSwitchCard(pairB));    // SB or SD
    root.appendChild(pair);
    if (bottomIdx != null && cfg.sw[bottomIdx]) {
      root.appendChild(makeSwitchCard(bottomIdx));  // SI or SJ
    }
  }
  buildPyramid('swPyramidLeft',  5, 4, 0, 1, 8);  // SF SE | SA SB | SI
  buildPyramid('swPyramidRight', 7, 6, 2, 3, 9);  // SH SG | SC SD | SJ
}

// ── Trim widget ──────────────────────────────────────────────────────────────
// orientation: 'v' (vertical: ▲ val ▼) or 'h' (horizontal: ◄ val ►).
// Defaults to vertical for back-compat.
function makeTrimWidget(trIdx, orientation = 'v') {
  const tr  = cfg.tr[trIdx];
  const div = document.createElement('div');
  div.className = 'trim-widget' + (orientation === 'h' ? ' horizontal' : '');

  const lbl = document.createElement('div'); lbl.className='trim-lbl'; lbl.textContent=tr.l;
  const ch  = document.createElement('div'); ch.className='trim-ch';
  ch.textContent = tr.c ? `CH${tr.c}` : 'N/A';

  const valEl = document.createElement('div'); valEl.className='trim-val';
  valEl.id = `trv${trIdx}`; valEl.textContent = trimOffset(trimCur[trIdx]);

  // Arrows depend on orientation
  const upChar = orientation === 'h' ? '◄' : '▲';
  const dnChar = orientation === 'h' ? '►' : '▼';
  const btnUp  = document.createElement('button'); btnUp.className='trim-btn'; btnUp.textContent=upChar;
  const btnDn  = document.createElement('button'); btnDn.className='trim-btn'; btnDn.textContent=dnChar;
  const btnRst = document.createElement('button'); btnRst.className='trim-rst'; btnRst.textContent='RST';

  const isBtnMode = () => (tr.m || 0) === 1;

  btnRst.onclick = () => {
    trimCur[trIdx] = SBUS_CENTER;
    valEl.textContent = trimOffset(trimCur[trIdx]);
    send({t:'tr', i:trIdx, d:0});
  };

  function applyTrimDelta(delta) {
    const step = tr.s || 10;
    trimCur[trIdx] = Math.max(SBUS_MIN, Math.min(SBUS_MAX, trimCur[trIdx] + delta*step));
    valEl.textContent = trimOffset(trimCur[trIdx]);
    send({t:'tr', i:trIdx, d:delta});
  }

  // Button-mode press / release — send specific value, return to center on up.
  function trimButtonPress(delta) {
    const target = delta > 0 ? (tr.vR ?? 1811) : (tr.vL ?? 172);
    trimCur[trIdx] = target;
    valEl.textContent = trimOffset(trimCur[trIdx]);
    send({t:'tr', i:trIdx, d:delta, p:true});
  }
  function trimButtonRelease() {
    trimCur[trIdx] = SBUS_CENTER;
    valEl.textContent = trimOffset(trimCur[trIdx]);
    send({t:'tr', i:trIdx, d:0, p:false});
  }

  function makeRepeater(btn, delta) {
    let hold=null, rep=null;
    const start = () => {
      if (isBtnMode()) {
        trimButtonPress(delta);   // single fire, no repeat
      } else {
        applyTrimDelta(delta);
        hold = setTimeout(()=>{ rep=setInterval(()=>applyTrimDelta(delta), 80); }, 400);
      }
    };
    const stop = () => {
      if (hold) { clearTimeout(hold); hold=null; }
      if (rep)  { clearInterval(rep); rep=null;  }
      if (isBtnMode()) trimButtonRelease();
    };
    btn.addEventListener('mousedown',  start);
    btn.addEventListener('mouseup',    stop);
    btn.addEventListener('mouseleave', stop);
    btn.addEventListener('touchstart', (e)=>{e.preventDefault();start();},{passive:false});
    btn.addEventListener('touchend',   (e)=>{e.preventDefault();stop(); },{passive:false});
  }
  // For vertical: ▲ = +1 (vR),  ▼ = -1 (vL)
  // For horizontal: ◄ = -1 (vL), ► = +1 (vR)
  if (orientation === 'h') {
    makeRepeater(btnUp, -1);   // ◄
    makeRepeater(btnDn, +1);   // ►
  } else {
    makeRepeater(btnUp, +1);   // ▲
    makeRepeater(btnDn, -1);   // ▼
  }

  if (orientation === 'h') {
    // Horizontal: RST  ◄  val  ►  +  label/ch
    div.appendChild(btnRst);
    div.appendChild(btnUp);
    div.appendChild(valEl);
    div.appendChild(btnDn);
    const lblWrap = document.createElement('div');
    lblWrap.style.cssText = 'display:flex;flex-direction:column;align-items:center;gap:1px;';
    lblWrap.appendChild(lbl); lblWrap.appendChild(ch);
    div.appendChild(lblWrap);
  } else {
    div.appendChild(lbl); div.appendChild(ch); div.appendChild(btnUp);
    div.appendChild(valEl); div.appendChild(btnDn); div.appendChild(btnRst);
  }
  return div;
}

function makeSliderWidget(slIdx) {
  const sl  = cfg.sl[slIdx];
  const div = document.createElement('div'); div.className='slider-widget';

  const lbl = document.createElement('div'); lbl.className='slider-lbl'; lbl.textContent=sl.l;
  const ch  = document.createElement('div'); ch.className='slider-ch';
  ch.textContent = sl.c ? `CH${sl.c}` : 'N/A';

  const wrap = document.createElement('div'); wrap.className='slider-wrap';
  const inp  = document.createElement('input');
  inp.type='range'; inp.className='slider-inp';
  inp.min='0'; inp.max='100'; inp.value=String(sl.pct||50);
  wrap.appendChild(inp);

  const valEl = document.createElement('div'); valEl.className='slider-val';
  valEl.id = `slv${slIdx}`;
  valEl.textContent = sbusFromPct(sl.pct||50);

  inp.oninput = () => {
    const pct = parseInt(inp.value);
    valEl.textContent = sbusFromPct(pct);
    send({t:'sl', i:slIdx, v:pct});
  };

  div.appendChild(lbl); div.appendChild(ch); div.appendChild(wrap); div.appendChild(valEl);
  return div;
}

// ── Sliders: LS left of left stick, RS right of right stick ──────────────────
function renderSliders() {
  // LS (index 0) beside left stick
  const lsCol = document.getElementById('slLSCol');
  lsCol.innerHTML = '';
  lsCol.appendChild(makeSliderWidget(0));

  // RS (index 1) beside right stick
  const rsCol = document.getElementById('slRSCol');
  rsCol.innerHTML = '';
  rsCol.appendChild(makeSliderWidget(1));
}

// ── S1/S2 pots + X20 extras (S3/J5/J6): horizontal sliders in their own row ──
// This entire row is HIDDEN by the .ctrl-area { display:none } rule — the SVG
// is the real UI.  Kept in the DOM so legacy state-tracking (slv# ids) still
// has something to read/write, and so the slider config table behind it can
// look up element ids if needed.
function renderPots() {
  const row = document.getElementById('potRow');
  row.innerHTML = '';
  // cfg.sl[0/1] = LS/RS (rendered as vertical sliders elsewhere);
  // cfg.sl[2] = S1, cfg.sl[3] = S2 (centre pots / knobs in the SVG);
  // cfg.sl[4] = S3 (X20 middle slider, was "MS");
  // cfg.sl[5/6] = J5/J6 (X20 stick-twist 3rd axes — meaningful only on an X20
  //   with the 3-axis gimbal upgrade, but always present so the SVG controls
  //   work without a model-specific toggle).
  for (let i = 2; i < cfg.sl.length; i++) {
    const sl  = cfg.sl[i];
    const div = document.createElement('div'); div.className='pot-widget';

    const lbl = document.createElement('div'); lbl.className='pot-lbl'; lbl.textContent=sl.l;
    const ch  = document.createElement('div'); ch.className='pot-ch';
    ch.textContent = sl.c ? `CH${sl.c}` : 'N/A';

    const inp = document.createElement('input');
    inp.type='range'; inp.className='pot-inp';
    inp.min='0'; inp.max='100'; inp.value=String(sl.pct||50);

    const valEl = document.createElement('div'); valEl.className='pot-val';
    valEl.id=`slv${i}`; valEl.textContent=sbusFromPct(sl.pct||50);

    const idx = i; // capture
    inp.oninput = () => {
      const pct = parseInt(inp.value);
      valEl.textContent = sbusFromPct(pct);
      send({t:'sl', i:idx, v:pct});
    };
    div.appendChild(lbl); div.appendChild(ch); div.appendChild(inp); div.appendChild(valEl);
    row.appendChild(div);
  }
}

// ── All 6 trims in a single row below the sticks ─────────────────────────────
// X18-style trim layout:
//   row 1 (vertical trims):     T3 ……………… T2
//   row 2 (horizontal trims):   T4 T5  ……  T6 T1
function renderTrimBank() {
  const bank = document.getElementById('trimBank');
  bank.innerHTML = '';

  // cfg.tr index mapping: 0=T1, 1=T2, 2=T3, 3=T4, 4=T5, 5=T6
  const T1=0, T2=1, T3=2, T4=3, T5=4, T6=5;

  // Top row — verticals T3 (left), T2 (right)
  const top = document.createElement('div');
  top.className = 'trim-row trim-row-top';
  top.appendChild(makeTrimWidget(T3, 'v'));
  top.appendChild(makeTrimWidget(T2, 'v'));
  bank.appendChild(top);

  // Bottom row — horizontals T4 + T5 (left pair), gap, T6 + T1 (right pair)
  const bot = document.createElement('div');
  bot.className = 'trim-row trim-row-bot';

  const leftPair = document.createElement('div'); leftPair.className = 'trim-pair';
  leftPair.appendChild(makeTrimWidget(T4, 'h'));
  leftPair.appendChild(makeTrimWidget(T5, 'h'));
  bot.appendChild(leftPair);

  const gap = document.createElement('div'); gap.className = 'trim-gap';
  bot.appendChild(gap);

  const rightPair = document.createElement('div'); rightPair.className = 'trim-pair';
  rightPair.appendChild(makeTrimWidget(T6, 'h'));
  rightPair.appendChild(makeTrimWidget(T1, 'h'));
  bot.appendChild(rightPair);

  bank.appendChild(bot);
}

// ── Buttons S1-S6 ────────────────────────────────────────────────────────────
// X18-style matrix:  S1/S2/S3 column  |  S4/S5/S6 column
// (Former rear buttons RB1/RB2 are now switches SI/SJ; see the switch pyramids.)
function renderButtons() {
  const bank = document.getElementById('btnBank');
  bank.innerHTML = '';

  // Build a button element by index
  function makeBtn(i) {
    const btn = cfg.btn[i];
    const el  = document.createElement('button');
    el.textContent = btn.l || `Btn${i+1}`;
    el.className   = 'ctrl-btn' + (btn.c === 0 ? ' unassigned' : '');
    if (btn.c !== 0) {
      const press = () => { el.classList.add('pressed');   send({t:'btn', i, p:true}); };
      const rel   = () => { el.classList.remove('pressed'); send({t:'btn', i, p:false}); };
      el.addEventListener('mousedown',  press);
      el.addEventListener('mouseup',    rel);
      el.addEventListener('mouseleave', rel);
      el.addEventListener('touchstart', (e)=>{e.preventDefault();press();},{passive:false});
      el.addEventListener('touchend',   (e)=>{e.preventDefault();rel();},  {passive:false});
    }
    return el;
  }

  // cfg.btn ordering: 0=S1, 1=S2, 2=S3, 3=S4, 4=S5, 5=S6, 6=L-Stick Click, 7=R-Stick Click
  // X18-style layout: two 3-row columns for S1-S6.  X20 adds a third
  // column with the two stick-click momentaries (slots 19/20 on the
  // RC-Controller matrix), rendered alongside the regular face buttons
  // so the user can press them to test the X20's 3-axis features.
  if (cfg.btn.length >= 6) {
    const sCol = document.createElement('div'); sCol.className = 'btn-col';
    sCol.appendChild(makeBtn(0));                                // S1
    sCol.appendChild(makeBtn(1));                                // S2
    sCol.appendChild(makeBtn(2));                                // S3
    bank.appendChild(sCol);
    const sCol2 = document.createElement('div'); sCol2.className = 'btn-col';
    sCol2.appendChild(makeBtn(3));                               // S4
    sCol2.appendChild(makeBtn(4));                               // S5
    sCol2.appendChild(makeBtn(5));                               // S6
    bank.appendChild(sCol2);
    if (cfg.btn.length >= 8) {
      const sCol3 = document.createElement('div'); sCol3.className = 'btn-col';
      sCol3.appendChild(makeBtn(6));                             // L-Stick Click (X20 slot 19)
      sCol3.appendChild(makeBtn(7));                             // R-Stick Click (X20 slot 20)
      bank.appendChild(sCol3);
    }
  } else {
    // Fallback for unexpected smaller config
    cfg.btn.forEach((_, i) => bank.appendChild(makeBtn(i)));
  }
}

// ── Lua / virtual button grid (15 buttons) ───────────────────────────────────
function renderLuaButtons() {
  const grid = document.getElementById('luaGrid');
  grid.innerHTML = '';
  (cfg.lua || []).forEach((btn, i) => {
    const el = document.createElement('button');
    el.textContent = btn.l || `Button ${i+1}`;
    const assigned = btn.c !== 0;
    el.className   = 'lua-btn' + (assigned ? '' : ' unassigned');
    // Always apply color so it's visible even on unassigned buttons
    el.style.setProperty('--btn-color', btn.k && btn.k.length === 7 ? btn.k : '#4fc3f7');
    if (assigned) {
      const press = () => { el.classList.add('pressed');    send({t:'lua',i,p:true}); };
      const rel   = () => { el.classList.remove('pressed'); send({t:'lua',i,p:false}); };
      el.addEventListener('mousedown',  press);
      el.addEventListener('mouseup',    rel);
      el.addEventListener('mouseleave', rel);
      el.addEventListener('touchstart', (e)=>{e.preventDefault();press();},{passive:false});
      el.addEventListener('touchend',   (e)=>{e.preventDefault();rel();},  {passive:false});
    }
    grid.appendChild(el);
  });
}

// ── Settings panel ────────────────────────────────────────────────────────────
function renderSettings() {
  // Axis channel dropdowns
  [['selRX',cfg.rx],['selRY',cfg.ry],['selLY',cfg.ly],['selLX',cfg.lx]].forEach(([id,val])=>{
    const cont = document.getElementById(id).parentNode;
    const sel  = buildChSel(val, false); sel.id=id;
    cont.replaceChild(sel, document.getElementById(id));
  });
  // Axis min/max inputs — values shown in current unit (SBUS or µs)
  [0,1,2,3].forEach(i=>{
    const mnEl=document.getElementById(`axMin${i}`);
    const mxEl=document.getElementById(`axMax${i}`);
    const rvEl=document.getElementById(`axRev${i}`);
    if(mnEl){ mnEl.value=fmtVal(cfg.aMin[i]??172);  mnEl.title=altLbl(cfg.aMin[i]??172); }
    if(mxEl){ mxEl.value=fmtVal(cfg.aMax[i]??1811); mxEl.title=altLbl(cfg.aMax[i]??1811); }
    if(rvEl){ rvEl.checked = !!(cfg.aRev && cfg.aRev[i]); }
  });

  // Switch table — Type dropdown (cell 1) drives which positions exist and
  // whether the switch is momentary; the Default Pos dropdown's labels update
  // live when Type changes so users see sensible choices for each mode.
  const swTb = document.getElementById('swCfgBody'); swTb.innerHTML='';
  const swSelStyle = 'background:var(--bg);border:1px solid var(--border);border-radius:4px;color:var(--text);padding:4px 6px;font-size:.74rem;';
  cfg.sw.forEach((sw, i)=>{
    const tr=document.createElement('tr');
    tr.innerHTML=`<td>${sw.l}</td>`;

    // Type dropdown (3-way / 2-way / Momentary).  Matches SwType enum in firmware:
    //   0 = SW_3POS, 1 = SW_2POS, 2 = SW_MOMENT
    const tdT=document.createElement('td');
    const tSel=document.createElement('select');
    tSel.style.cssText=swSelStyle;
    [['0','3-way'],['1','2-way'],['2','Momentary']].forEach(([v,lbl])=>{
      const opt=document.createElement('option'); opt.value=v; opt.textContent=lbl;
      if((+v)===(sw.t??0)) opt.selected=true;
      tSel.appendChild(opt);
    });
    tdT.appendChild(tSel); tr.appendChild(tdT);

    // Channel
    const tdCh=document.createElement('td'); tdCh.appendChild(buildChSel(sw.c,true)); tr.appendChild(tdCh);

    // Low / Mid / High value inputs
    const valInputs=[];
    for(let j=0;j<3;j++){
      const td=document.createElement('td');
      const inp=document.createElement('input'); inp.type='number';
      const raw=sw.v[j]||SBUS_CENTER;
      inp.value=fmtVal(raw); inp.title=altLbl(raw);
      inp.classList.add('sbus-val');  // mark so saveConfig() knows to parseVal()
      td.appendChild(inp); tr.appendChild(td);
      valInputs.push(inp);
    }

    // Default Position dropdown — labels depend on Type
    const tdD=document.createElement('td');
    const dSel=document.createElement('select');
    dSel.style.cssText=swSelStyle;
    const buildDefaultOptions = (curType) => {
      const cur = parseInt(dSel.value);   // remember current selection across rebuild
      dSel.innerHTML='';
      // 3-way: low / center / high  ·  2-way and momentary: low / — / high (mid unused)
      const labels = (curType === 0)
        ? ['▼ Down','Center','▲ Up']
        : ['▼ Down','—','▲ Up'];
      labels.forEach((lbl,idx)=>{
        if (curType !== 0 && idx === 1) return;  // hide unused mid for 2-way/momentary
        const opt=document.createElement('option'); opt.value=idx; opt.textContent=lbl;
        dSel.appendChild(opt);
      });
      // Restore selection if it's still valid for this type; else fall back to sw.d / 0
      const fallback = (sw.d ?? 0);
      const want = Number.isNaN(cur) ? fallback : cur;
      dSel.value = String([...dSel.options].some(o=>+o.value===want) ? want : fallback);
      // For momentary: also grey the Mid Val input since it's unused
      const isMom = curType === 2;
      valInputs[1].disabled = isMom;
      valInputs[1].style.opacity = isMom ? .35 : 1;
    };
    buildDefaultOptions(sw.t ?? 0);
    tSel.addEventListener('change', () => buildDefaultOptions(parseInt(tSel.value)));
    tdD.appendChild(dSel); tr.appendChild(tdD);

    swTb.appendChild(tr);
  });

  // Slider/pot table — all sliders in cfg.sl:
  //   0=LS, 1=RS (X18 side sliders)
  //   2=S1, 3=S2 (X18 centre pots)
  //   4=MS    (X20 middle slider)
  //   5=J5, 6=J6 (X20 stick-twist 3rd axes — 3-axis gimbal upgrade)
  const slTb = document.getElementById('slCfgBody'); slTb.innerHTML='';
  cfg.sl.forEach((sl,i)=>{
    const tr=document.createElement('tr');
    tr.innerHTML=`<td>${sl.l}</td>`;
    const tdCh=document.createElement('td'); tdCh.appendChild(buildChSel(sl.c,true)); tr.appendChild(tdCh);
    slTb.appendChild(tr);
  });

  // Trim table  (mode 0 = step, 1 = button)
  const trTb = document.getElementById('trCfgBody'); trTb.innerHTML='';
  cfg.tr.forEach((t,i)=>{
    const tr=document.createElement('tr');
    tr.innerHTML=`<td>${t.l}</td>`;
    // Channel
    const tdCh=document.createElement('td'); tdCh.appendChild(buildChSel(t.c,false)); tr.appendChild(tdCh);
    // Mode dropdown
    const tdM=document.createElement('td');
    const mSel=document.createElement('select');
    mSel.innerHTML = `<option value="0">Step</option><option value="1">Button (momentary)</option>`;
    mSel.value = String(t.m || 0);
    tdM.appendChild(mSel); tr.appendChild(tdM);
    // Step
    const tdS=document.createElement('td');
    const si=document.createElement('input'); si.type='number'; si.min=1; si.max=100; si.value=t.s||10;
    si.style.width='60px'; tdS.appendChild(si); tr.appendChild(tdS);
    // valL  (left / down) — value shown in current unit
    const tdL=document.createElement('td');
    const vL=document.createElement('input'); vL.type='number';
    const rawL = (t.vL===undefined ? 172 : t.vL);
    vL.value = fmtVal(rawL); vL.title = altLbl(rawL);
    vL.classList.add('sbus-val'); vL.style.width='70px';
    tdL.appendChild(vL); tr.appendChild(tdL);
    // valR  (right / up) — value shown in current unit
    const tdR=document.createElement('td');
    const vR=document.createElement('input'); vR.type='number';
    const rawR = (t.vR===undefined ? 1811 : t.vR);
    vR.value = fmtVal(rawR); vR.title = altLbl(rawR);
    vR.classList.add('sbus-val'); vR.style.width='70px';
    tdR.appendChild(vR); tr.appendChild(tdR);

    // Grey out step in button mode, and L/R in step mode
    const refresh = () => {
      const isBtn = mSel.value === '1';
      si.disabled = isBtn;        si.style.opacity = isBtn ? .3 : 1;
      vL.disabled = !isBtn;       vL.style.opacity = !isBtn ? .3 : 1;
      vR.disabled = !isBtn;       vR.style.opacity = !isBtn ? .3 : 1;
    };
    mSel.onchange = refresh; refresh();

    trTb.appendChild(tr);
  });

  // Physical button table
  const btnTb = document.getElementById('btnCfgBody'); btnTb.innerHTML='';
  cfg.btn.forEach((b,i)=>{
    const tr=document.createElement('tr');
    tr.innerHTML=`<td>${b.l}</td>`;
    const tdL=document.createElement('td');
    const li=document.createElement('input'); li.type='text'; li.value=b.l; li.maxLength=31;
    tdL.appendChild(li); tr.appendChild(tdL);
    const tdCh=document.createElement('td'); tdCh.appendChild(buildChSel(b.c,true)); tr.appendChild(tdCh);
    const tdV=document.createElement('td');
    const vi=document.createElement('input'); vi.type='number';
    vi.value=fmtVal(b.v); vi.title=altLbl(b.v);
    vi.classList.add('sbus-val'); vi.style.width='70px';
    tdV.appendChild(vi); tr.appendChild(tdV);
    btnTb.appendChild(tr);
  });

  // Lua button table
  const luaTb = document.getElementById('luaCfgBody'); luaTb.innerHTML='';
  (cfg.lua||[]).forEach((b,i)=>{
    const tr=document.createElement('tr');
    // # column
    const tdN=document.createElement('td'); tdN.textContent=i+1;
    tdN.style.cssText='font-size:.68rem;color:var(--muted);text-align:center;';
    tr.appendChild(tdN);
    // Label
    const tdL=document.createElement('td');
    const li=document.createElement('input'); li.type='text'; li.value=b.l; li.maxLength=31;
    tdL.appendChild(li); tr.appendChild(tdL);
    // Channel
    const tdCh=document.createElement('td'); tdCh.appendChild(buildChSel(b.c,true)); tr.appendChild(tdCh);
    // Value
    const tdV=document.createElement('td');
    const vi=document.createElement('input'); vi.type='number'; vi.min=SBUS_MIN; vi.max=SBUS_MAX; vi.value=b.v||SBUS_MAX;
    vi.style.width='70px'; tdV.appendChild(vi); tr.appendChild(tdV);
    // Color
    const tdK=document.createElement('td'); tdK.style.textAlign='center';
    const ki=document.createElement('input'); ki.type='color'; ki.value=b.k||'#4fc3f7';
    ki.style.cssText='width:40px;height:28px;padding:2px;border:none;border-radius:4px;cursor:pointer;background:none;';
    // Live-preview: update the button color in the grid as the user picks
    ki.addEventListener('input', () => {
      const btns = document.querySelectorAll('#luaGrid .lua-btn');
      if (btns[i]) btns[i].style.setProperty('--btn-color', ki.value);
    });
    tdK.appendChild(ki); tr.appendChild(tdK);
    luaTb.appendChild(tr);
  });

  // PWM output table
  document.getElementById('pwmExtChk').checked = !!cfg.pwmExt;
  const pwmPins = [4, 6, 15, 17];
  const pwmTb = document.getElementById('pwmCfgBody'); pwmTb.innerHTML='';
  (cfg.pwm || Array.from({length:4}, (_,i)=>({c:i+1}))).forEach((p, i) => {
    const tr = document.createElement('tr');
    tr.innerHTML = `<td>PWM ${i+1}</td><td style="color:var(--muted);font-size:.75rem;">GPIO ${pwmPins[i]}</td>`;
    const tdCh = document.createElement('td'); tdCh.appendChild(buildChSel(p.c, true)); tr.appendChild(tdCh);
    pwmTb.appendChild(tr);
  });
}

// =============================================================================
//  WiFi panel
// =============================================================================

function renderWifiPanel() {
  // Status bar
  const nets = cfg.wifiNets || [];
  const ni   = cfg.wifiNet != null ? cfg.wifiNet : -1;
  document.getElementById('wifiStatusSSID').textContent =
    ni >= 0 && nets[ni] ? nets[ni].s : (ni === -1 ? 'AP (' + (cfg.wifiIP||'') + ')' : '—');
  document.getElementById('wifiStatusIP').textContent   = cfg.wifiIP   || '—';
  document.getElementById('wifiStatusMDNS').textContent = cfg.wifiMDNS || '—';

  // Network rows
  const list = document.getElementById('wifiNetList');
  list.innerHTML = '';
  nets.forEach((net, i) => {
    const row = document.createElement('div');
    row.className = 'wifi-net-row';
    row.dataset.idx = i;

    const num  = document.createElement('span'); num.className='drag-num'; num.textContent=i+1+'.';
    const ssid = document.createElement('input'); ssid.type='text'; ssid.placeholder='SSID';
    ssid.value = net.s || ''; ssid.maxLength = 32;

    const passWrap = document.createElement('div'); passWrap.className='wifi-pass-wrap';
    const pass = document.createElement('input'); pass.type='password'; pass.placeholder='Password';
    pass.value = net.p || ''; pass.maxLength = 64;
    const eye  = document.createElement('button'); eye.textContent='👁'; eye.title='Show/hide';
    eye.onclick = () => { pass.type = pass.type==='password' ? 'text' : 'password'; };
    passWrap.appendChild(pass); passWrap.appendChild(eye);

    const up  = document.createElement('button'); up.className='wifi-arrow';  up.textContent='▲';
    const dn  = document.createElement('button'); dn.className='wifi-arrow';  dn.textContent='▼';
    const del = document.createElement('button'); del.className='wifi-del';   del.textContent='✕';
    up.onclick  = () => { if(i>0){const t=nets[i];nets[i]=nets[i-1];nets[i-1]=t;renderWifiPanel();} };
    dn.onclick  = () => { if(i<nets.length-1){const t=nets[i];nets[i]=nets[i+1];nets[i+1]=t;renderWifiPanel();} };
    del.onclick = () => { nets.splice(i,1); renderWifiPanel(); };

    row.appendChild(num); row.appendChild(ssid); row.appendChild(passWrap);
    row.appendChild(up);  row.appendChild(dn);   row.appendChild(del);
    list.appendChild(row);
  });

  // Preference dropdown
  const sel = document.getElementById('wifiPrefSel');
  sel.innerHTML = '<option value="0">Auto (try in order)</option>';
  nets.forEach((n, i) => {
    const o = document.createElement('option');
    o.value = i + 1;
    o.textContent = `Net ${i+1}: ${n.s || '(blank)'}`;
    sel.appendChild(o);
  });
  const apOpt = document.createElement('option'); apOpt.value=255; apOpt.textContent='AP Only';
  sel.appendChild(apOpt);
  sel.value = cfg.wifiPref != null ? cfg.wifiPref : 0;
}

function wifiAddRow() {
  if (!cfg.wifiNets) cfg.wifiNets = [];
  // Cap matches MAX_WIFI_NETS in firmware (8).  Firmware would silently drop
  // anything beyond, so we stop the user here for clarity.
  if (cfg.wifiNets.length >= 8) return;
  cfg.wifiNets.push({s:'', p:''});
  renderWifiPanel();
}

// Read current row values back into cfg.wifiNets before sending
function wifiCollect() {
  const rows = document.getElementById('wifiNetList').children;
  cfg.wifiNets = [];
  for (const row of rows) {
    const inputs = row.querySelectorAll('input');
    const s = inputs[0].value.trim();
    const p = inputs[1].value;
    if (s) cfg.wifiNets.push({s, p});
  }
}

function wifiApply() {
  wifiCollect();
  cfg.wifiPref = parseInt(document.getElementById('wifiPrefSel').value) || 0;
  send({t:'wificfg', pref: cfg.wifiPref, nets: cfg.wifiNets});
  const msg = document.getElementById('wifiMsg');
  msg.style.display = 'inline';
  msg.textContent = 'Connecting…';
}

function handleWifiSwitching(msg) {
  const el = document.getElementById('wifiMsg');
  el.style.display = 'inline';
  el.textContent = `Switching to "${msg.ssid}"… reconnect at http://${msg.mdns}`;
}

// =============================================================================
//  Export / Import
// =============================================================================

// ── Military DTG: DDHHMMZMONYR  e.g. 091234ZAPR26 ───────────────────────────
function getDTG() {
  const now = new Date();
  const mo  = ['JAN','FEB','MAR','APR','MAY','JUN','JUL','AUG','SEP','OCT','NOV','DEC'];
  const dd  = String(now.getUTCDate()).padStart(2,'0');
  const hh  = String(now.getUTCHours()).padStart(2,'0');
  const mm  = String(now.getUTCMinutes()).padStart(2,'0');
  const yy  = String(now.getUTCFullYear()).slice(-2);
  return `${dd}${hh}${mm}Z${mo[now.getUTCMonth()]}${yy}`;
}

// ── Export: fetch the saved JSON from the ESP and trigger a browser download ──
function exportConfig() {
  fetch('/cfg')
    .then(r => { if (!r.ok) throw new Error('No config'); return r.blob(); })
    .then(blob => {
      const url = URL.createObjectURL(blob);
      const a   = document.createElement('a');
      a.href = url;
      a.download = `sbus_config_${getDTG()}.json`;
      a.click();
      URL.revokeObjectURL(url);
    })
    .catch(e => {
      const el = document.getElementById('importStatus');
      el.style.color = 'var(--red)';
      el.textContent = 'Export failed: ' + e.message;
    });
}

// ── Import: read a JSON file and send it as a cfg WS message ─────────────────
function importConfig(input) {
  const file = input.files[0];
  if (!file) return;
  const status = document.getElementById('importStatus');
  status.style.color = 'var(--muted)';
  status.textContent = 'Reading…';
  const reader = new FileReader();
  reader.onload = e => {
    try {
      const data = JSON.parse(e.target.result);
      // Remap LittleFS format to WS cfg message format
      const payload = { t: 'cfg' };
      if (data.rx    != null) payload.rx = data.rx;
      if (data.ry    != null) payload.ry = data.ry;
      if (data.ly    != null) payload.ly = data.ly;
      if (data.lx    != null) payload.lx = data.lx;
      if (data.aMin  != null) payload.aMin = data.aMin;
      if (data.aMax  != null) payload.aMax = data.aMax;
      if (data.aRev  != null) payload.aRev = data.aRev;
      if (data.sw    != null) payload.sw   = data.sw;
      if (data.sl    != null) payload.sl   = data.sl;
      if (data.tr    != null) payload.tr   = data.tr;
      if (data.btn   != null) payload.btn  = data.btn;
      if (data.lua   != null) payload.lua  = data.lua;
      if (data.pwm   != null) payload.pwm  = data.pwm;
      if (data.pwmExt != null) payload.pwmExt = data.pwmExt;
      send(payload);
      // WiFi networks live in a separate firmware message ('wificfg'), so an
      // exported file's networks were silently dropped by import before.
      // nosw:1 stores them without switching networks out from under the
      // session doing the import.
      if (Array.isArray(data.wifiNets) && data.wifiNets.length) {
        const nets = data.wifiNets.map(n => ({s:(n.s||'').trim(), p:n.p||''})).filter(n => n.s);
        if (nets.length) {
          const pref = (data.wifiPref != null) ? data.wifiPref : cfg.wifiPref;
          setTimeout(() => send({t:'wificfg', nosw:1, pref, nets}), 150);
        }
      }
      // Belt-and-suspenders: the firmware fires its own buildCfgJson broadcast
      // right after applying the import, but if that response is missed (TX
      // path busy, transport switch in flight, etc.) the UI would stay on the
      // pre-import state.  An explicit getcfg afterwards guarantees the UI
      // catches up to what's actually saved (delay sits after the wificfg
      // follow-up above so it reflects the networks too).
      setTimeout(() => send({t:'getcfg'}), 450);
      status.style.color = 'var(--green)';
      status.textContent = '\u2713 Imported — config applied & saved.';
    } catch(err) {
      status.style.color = 'var(--red)';
      status.textContent = 'Import failed: ' + err.message;
    }
    input.value = '';   // reset so the same file can be re-imported if needed
  };
  reader.readAsText(file);
}

function saveConfig() {
  const payload = {t:'cfg'};
  payload.rx = parseInt(document.getElementById('selRX').value);
  payload.ry = parseInt(document.getElementById('selRY').value);
  payload.ly = parseInt(document.getElementById('selLY').value);
  payload.lx = parseInt(document.getElementById('selLX').value);
  // Axis min/max — input values are in the current display unit; parseVal converts back to raw SBUS.
  payload.aMin = [0,1,2,3].map(i => parseVal(document.getElementById(`axMin${i}`).value) ?? 172);
  payload.aMax = [0,1,2,3].map(i => parseVal(document.getElementById(`axMax${i}`).value) ?? 1811);
  payload.aRev = [0,1,2,3].map(i => !!document.getElementById(`axRev${i}`).checked);

  const swRows = document.getElementById('swCfgBody').rows;
  payload.sw = cfg.sw.map((sw, i) => {
    const row = swRows[i];
    // cells: 0=Name(td), 1=Type(select), 2=Channel(select), 3-5=Low/Mid/High(input), 6=DefaultPos(select)
    const t  = parseInt(row.cells[1].querySelector('select').value);
    const ch = parseInt(row.cells[2].querySelector('select').value);
    const v  = [0,1,2].map(j=>parseVal(row.cells[3+j].querySelector('input').value));
    const d  = parseInt(row.cells[6].querySelector('select').value);
    return {l:sw.l, c:ch, t, d, v};
  });

  const slRows = document.getElementById('slCfgBody').rows;
  payload.sl = cfg.sl.map((sl, i) => ({
    l: sl.l,
    c: parseInt(slRows[i].cells[1].querySelector('select').value)
  }));

  const trRows = document.getElementById('trCfgBody').rows;
  payload.tr = cfg.tr.map((t, i) => ({
    l:  t.l,
    c:  parseInt(trRows[i].cells[1].querySelector('select').value),
    m:  parseInt(trRows[i].cells[2].querySelector('select').value),
    s:  parseInt(trRows[i].cells[3].querySelector('input').value),
    vL: parseVal(trRows[i].cells[4].querySelector('input').value),  // unit-aware
    vR: parseVal(trRows[i].cells[5].querySelector('input').value)   // unit-aware
  }));

  const btnRows = document.getElementById('btnCfgBody').rows;
  payload.btn = cfg.btn.map((b, i) => ({
    l: btnRows[i].cells[1].querySelector('input').value,
    c: parseInt(btnRows[i].cells[2].querySelector('select').value),
    v: parseVal(btnRows[i].cells[3].querySelector('input').value)   // unit-aware
  }));

  const luaRows = document.getElementById('luaCfgBody').rows;
  payload.lua = (cfg.lua||[]).map((b, i) => ({
    l: luaRows[i].cells[1].querySelector('input').value,
    c: parseInt(luaRows[i].cells[2].querySelector('select').value),
    v: parseInt(luaRows[i].cells[3].querySelector('input').value),
    k: luaRows[i].cells[4].querySelector('input[type=color]').value
  }));

  const pwmRows = document.getElementById('pwmCfgBody').rows;
  payload.pwm = Array.from(pwmRows).map(row => ({
    c: parseInt(row.cells[2].querySelector('select').value) || 0
  }));
  payload.pwmExt = document.getElementById('pwmExtChk').checked;

  send(payload);
  // Same belt-and-suspenders as importConfig — guarantee the UI re-syncs to
  // what the firmware actually saved, even if the implicit post-save broadcast
  // happens to be missed.
  setTimeout(() => send({t:'getcfg'}), 250);
}

function onPwmExtChange() {
  // Live-update the firmware immediately when the checkbox is toggled
  send({ t:'cfg', pwmExt: document.getElementById('pwmExtChk').checked,
         pwm: Array.from(document.getElementById('pwmCfgBody').rows).map(row => ({
           c: parseInt(row.cells[2].querySelector('select').value) || 0
         }))
  });
}

// =============================================================================
//  Debug channel grid
// =============================================================================
let dbgCells=[], dbgLitTimers=[], dbgUpdatesInWindow=0, dbgRateTimer=null;

function initDbgGrid(n) {
  const grid=document.getElementById('dbgGrid'); if(!grid)return;
  grid.innerHTML=''; dbgCells=[]; dbgLitTimers=new Array(n).fill(null);
  for(let i=0;i<n;i++){
    const cell=document.createElement('div'); cell.className='dbg-cell';
    const cn=document.createElement('div'); cn.className='dcn'; cn.textContent=`CH${String(i+1).padStart(2,'0')}`;
    const cv=document.createElement('span'); cv.className='dcv'; cv.textContent='—';
    cell.appendChild(cn); cell.appendChild(cv); grid.appendChild(cell);
    dbgCells.push({cell,cv});
  }
}

function handleChData(msg) {
  const ch=msg.ch, mode=msg.mode, fl=msg.fl;
  if(!Array.isArray(ch)) return;
  const offMsg=document.getElementById('dbgOffMsg');
  const grid=document.getElementById('dbgGrid');
  const info=document.getElementById('dbgInfo');
  if(offMsg) offMsg.style.display='none';
  if(grid)   grid.style.display='';
  if(info)   info.style.display='';
  if(dbgCells.length!==ch.length) initDbgGrid(ch.length);
  ch.forEach((val,i)=>{
    const e=dbgCells[i]; if(!e) return;
    const prev=parseInt(e.cv.textContent)||-1;
    e.cv.textContent=val;
    if(val!==prev){
      e.cell.classList.add('lit');
      if(dbgLitTimers[i]) clearTimeout(dbgLitTimers[i]);
      dbgLitTimers[i]=setTimeout(()=>{e.cell.classList.remove('lit');dbgLitTimers[i]=null;},300);
    }
  });
  const mEl=document.getElementById('diMode'); if(mEl) mEl.textContent=`SBUS-${mode}`;
  const fEl=document.getElementById('diFrame'); if(fEl) fEl.textContent=fl;
  const cEl=document.getElementById('diCh');   if(cEl) cEl.textContent=ch.length;
  dbgUpdatesInWindow++;
  if(!dbgRateTimer){
    dbgRateTimer=setInterval(()=>{
      const rEl=document.getElementById('diRate'); if(rEl) rEl.textContent=`${dbgUpdatesInWindow}/s`;
      dbgUpdatesInWindow=0;
    },1000);
  }
}

// =============================================================================
//  Joystick
// =============================================================================
function scheduleJoySend() {
  if(joyTimer) return;
  joyTimer=setTimeout(()=>{
    joyTimer=null;
    send({t:'a', lx:sticks.L.x, ly:sticks.L.y, rx:sticks.R.x, ry:sticks.R.y});
    updateReadouts();
  },30);
}
function posFromXY(cx,cy,wrap){
  const r=wrap.getBoundingClientRect();
  let nx=(cx-r.left-r.width/2)/(r.width/2);
  let ny=(cy-r.top-r.height/2)/(r.height/2);
  const mag=Math.sqrt(nx*nx+ny*ny); if(mag>1){nx/=mag;ny/=mag;}
  return[+nx.toFixed(3),+ny.toFixed(3)];
}
function applyStick(side,nx,ny){
  sticks[side].x=nx; sticks[side].y=ny;
  const th=document.getElementById('thumb'+side);
  th.style.left=(50+nx*44)+'%'; th.style.top=(50+ny*44)+'%';
  scheduleJoySend();
}
function springBack(side){ applyStick(side,0,0); }

function initStick(side){
  const wrap=document.getElementById('stick'+side);
  wrap.addEventListener('touchstart',(e)=>{
    e.preventDefault(); const t=e.changedTouches[0];
    sticks[side].touchId=t.identifier; sticks[side].active=true;
    wrap.classList.add('active');
    const[nx,ny]=posFromXY(t.clientX,t.clientY,wrap); applyStick(side,nx,ny);
  },{passive:false});
  wrap.addEventListener('touchmove',(e)=>{
    e.preventDefault();
    for(const t of e.changedTouches){
      if(t.identifier===sticks[side].touchId){
        const[nx,ny]=posFromXY(t.clientX,t.clientY,wrap); applyStick(side,nx,ny);
      }
    }
  },{passive:false});
  wrap.addEventListener('touchend',(e)=>{
    e.preventDefault();
    for(const t of e.changedTouches){
      if(t.identifier===sticks[side].touchId){
        sticks[side].active=false; sticks[side].touchId=null;
        wrap.classList.remove('active'); springBack(side);
      }
    }
  },{passive:false});
  wrap.addEventListener('mousedown',(e)=>{
    sticks[side].active=true; wrap.classList.add('active');
    const[nx,ny]=posFromXY(e.clientX,e.clientY,wrap); applyStick(side,nx,ny);
    const onMove=(ev)=>{ if(!sticks[side].active)return; const[nx2,ny2]=posFromXY(ev.clientX,ev.clientY,wrap); applyStick(side,nx2,ny2); };
    const onUp=()=>{ sticks[side].active=false; wrap.classList.remove('active'); springBack(side); document.removeEventListener('mousemove',onMove); document.removeEventListener('mouseup',onUp); };
    document.addEventListener('mousemove',onMove); document.addEventListener('mouseup',onUp);
  });
}

// =============================================================================
//  Serial command hint (displayed on Serial Monitor)
// =============================================================================

// =============================================================================
//  Unit toggle wiring — display can flip between SBUS and µs without reload
// =============================================================================
function applyUnitToggle() {
  // Sync radio buttons to current displayUnit
  document.querySelectorAll('input[name="unit-toggle"]').forEach(r => {
    r.checked = (r.value === displayUnit);
  });
  // Refresh all "(SBUS)" / "(µs)" badges next to table headers
  document.querySelectorAll('.unit-label').forEach(el => {
    el.textContent = `(${unitLbl()})`;
  });
  // Re-render every panel that shows SBUS values
  renderAll();
  updateReadouts();
}
document.querySelectorAll('input[name="unit-toggle"]').forEach(r => {
  r.addEventListener('change', () => {
    displayUnit = r.value;
    localStorage.setItem('sbusDisplayUnit', displayUnit);
    applyUnitToggle();
  });
});

// =============================================================================
//  Boot
// =============================================================================
initStick('L');
initStick('R');
renderAll();
renderWifiPanel();
initSvgSwitchPositions();
wireSvgSwitchHandlers();   // strip static inline handlers, rewire to type-aware dispatcher
initSvgAnalogs();
applyUnitToggle();   // pick up persisted unit preference on first paint
connect();
</script>
</body>
</html>)rawhtml";
