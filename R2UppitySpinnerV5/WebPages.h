// WebPages.h — R2 Uppity Spinner v3
// Changes in this revision:
//   • Lift joystick now scales by Lift Speed; "down" button uses :PH{speed}
//   • Actions section moved ABOVE light kit
//   • "Spin Deg" renamed "random °" with explanation
//   • Status bar expanded to 5 cells: height, rotation (or "home"), safety, motors, cmd
//   • Rotary dial syncs in real time from /api/status rotDeg field
//   • Setup sub-pages (Marcduino, Parameters, WiFi, Remote, Firmware) restyled
//   • "fault" and "safety" chip labels replaced with plain-English tooltips
//   • Live status + switches duplicated into the Periscope tab (bottom)

#ifdef USE_WIFI_WEB
#include "web-images.h"

bool sOTAInProgress;

///////////////////////////////////////////////////////////////////////////////
// SHARED CSS + SHELL HELPERS
///////////////////////////////////////////////////////////////////////////////

static void printPageHead(Print& out, const char* title)
{
    out.print("<!DOCTYPE html><html lang='en'><head>");
    out.print("<meta charset='utf-8'>");
    out.print("<meta name='viewport' content='width=device-width,initial-scale=1,viewport-fit=cover'>");
    out.print("<title>"); out.print(title); out.print("</title><style>");

    // CSS Variables — dark theme (default)
    out.print(":root{"
              "--bg:#0f1117;--panel:#1a1d28;--border:#2a2d3a;--border2:#353c47;"
              "--text:#e0e4f0;--muted:#6b7280;"
              "--accent:#4fc3f7;--green:#4ade80;--red:#ef5350;--yellow:#ffa726;"
              "}");
    // Light theme override
    out.print("body.light{"
              "--bg:#f0f2f7;--panel:#ffffff;--border:#d1d5e0;--border2:#b0b8c8;"
              "--text:#1a1d2e;--muted:#6b7280;"
              "--accent:#0284c7;--green:#16a34a;--red:#dc2626;--yellow:#d97706;"
              "}");

    // Reset & base
    out.print("*{box-sizing:border-box;margin:0;padding:0}");
    out.print("html{height:100%}");
    out.print("body{min-height:100%;background:var(--bg);color:var(--text);"
              "font-family:'Trebuchet MS',Arial,sans-serif;"
              "font-size:15px;-webkit-tap-highlight-color:transparent}");

    // Top nav bar
    out.print("#topnav{display:flex;align-items:stretch;"
              "background:var(--panel);border-bottom:1px solid var(--border);"
              "position:sticky;top:0;z-index:200;"
              "box-shadow:0 2px 8px rgba(0,0,0,.2)}");
    out.print(".tn-title{font-size:11px;font-weight:700;color:var(--muted);"
              "padding:0 14px;display:flex;align-items:center;"
              "border-right:1px solid var(--border);white-space:nowrap;"
              "letter-spacing:.08em;text-transform:uppercase}");
    out.print("#topnav a{color:var(--muted);text-decoration:none;"
              "padding:12px 13px;font-size:13px;font-weight:600;"
              "white-space:nowrap;display:flex;align-items:center;"
              "letter-spacing:.03em;transition:color .15s,background .15s}");
    out.print("#topnav a:hover{color:var(--text);background:var(--panel)}");
    out.print("#topnav a.active{color:var(--accent);"
              "border-bottom:2px solid var(--accent)}");

    // Content area
    out.print("#page{padding:12px 14px 24px;max-width:680px;margin:0 auto;width:100%}");

    // Status bar
    out.print("#sbar{display:grid;grid-template-columns:repeat(5,1fr);gap:4px;"
              "padding:8px 10px;background:var(--panel);"
              "border-bottom:1px solid var(--border);"
              "position:sticky;top:46px;z-index:100}");
    out.print(".ss{background:var(--bg);border:1px solid var(--border);"
              "border-radius:8px;padding:6px 4px;text-align:center}");
    out.print(".sv{font-size:13px;font-weight:700;line-height:1.2}");
    out.print(".sl{font-size:9px;color:var(--muted);margin-top:2px;"
              "text-transform:uppercase;letter-spacing:.05em}");
    out.print(".c-g{color:var(--green)}.c-y{color:var(--yellow)}"
              ".c-r{color:var(--red)}.c-b{color:var(--accent)}"
              ".c-m{color:var(--muted);font-size:10px}");

    // Section labels
    out.print(".sec{font-size:11px;color:var(--muted);font-weight:700;"
              "letter-spacing:.1em;text-transform:uppercase;"
              "margin:16px 0 8px;padding-bottom:4px;"
              "border-bottom:1px solid var(--border)}");
    out.print(".sec:first-child{margin-top:6px}");

    // Buttons
    out.print("button{appearance:none;-webkit-appearance:none;"
              "border:1px solid var(--border2);background:var(--panel);color:var(--text);"
              "border-radius:8px;padding:12px 8px;font-size:14px;"
              "font-family:inherit;cursor:pointer;text-align:center;"
              "width:100%;font-weight:600;transition:all .15s;letter-spacing:.02em}");
    out.print("button:hover{background:var(--border2);color:var(--text)}");
    out.print("button:active{opacity:.75}");
    out.print("button.danger{border-color:rgba(239,83,80,.35);"
              "color:var(--red);background:rgba(239,83,80,.07)}");
    out.print("button.danger:hover{background:rgba(239,83,80,.16)}");
    out.print("button.primary{border-color:rgba(79,195,247,.35);"
              "color:var(--accent);background:rgba(79,195,247,.07)}");
    out.print("button.primary:hover{background:rgba(79,195,247,.16)}");
    out.print("button.success{border-color:rgba(74,222,128,.35);"
              "color:var(--green);background:rgba(74,222,128,.07)}");
    out.print("button.success:hover{background:rgba(74,222,128,.16)}");
    out.print("button.estop{border-color:var(--red);background:var(--red);"
              "color:#fff;font-size:17px;font-weight:700;"
              "padding:16px;width:100%;border-radius:10px;margin-top:10px}");
    out.print("button.estop:hover{filter:brightness(1.15)}");

    // Grids
    out.print(".g2{display:grid;grid-template-columns:1fr 1fr;gap:8px}");
    out.print(".g3{display:grid;grid-template-columns:1fr 1fr 1fr;gap:8px}");
    out.print(".g5{display:grid;grid-template-columns:repeat(5,1fr);gap:6px}");

    // Light-kit pills
    out.print(".lk{display:flex;gap:6px;flex-wrap:wrap;padding-bottom:2px}");
    out.print(".lp{font-size:13px;padding:7px 13px;"
              "border-radius:20px;border:1px solid var(--border2);"
              "color:var(--muted);background:var(--panel);cursor:pointer;"
              "white-space:nowrap;font-weight:600;transition:all .15s}");
    out.print(".lp:hover{border-color:var(--accent);color:var(--text)}");
    out.print(".lp.on{background:rgba(79,195,247,.12);"
              "border-color:var(--accent);color:var(--accent)}");

    // Sequence buttons
    out.print(".sq{background:var(--panel);border:1px solid var(--border2);"
              "border-radius:8px;color:var(--muted);font-size:15px;"
              "padding:11px 0;cursor:pointer;text-align:center;"
              "font-weight:700;transition:all .15s}");
    out.print(".sq:hover{border-color:var(--accent);color:var(--text)}");
    out.print(".sq.on{background:rgba(79,195,247,.12);"
              "border-color:var(--accent);color:var(--accent)}");

    // Position row
    out.print(".pos-row{display:flex;gap:12px;align-items:stretch;height:190px}");
    out.print(".lift-col{display:flex;flex-direction:column;"
              "align-items:center;gap:6px;width:70px}");
    out.print(".lift-lbl{font-size:11px;color:var(--muted);"
              "text-transform:uppercase;letter-spacing:.05em;font-weight:700}");
    out.print(".lift-val{font-size:16px;font-weight:700;color:var(--accent)}");
    out.print("canvas{display:block;touch-action:none;cursor:grab}");
    out.print("canvas:active{cursor:grabbing}");
    out.print(".dial-col{flex:1;display:flex;flex-direction:column;"
              "align-items:center;justify-content:center;gap:5px}");
    out.print(".dial-val{font-size:16px;font-weight:700;color:var(--accent)}");
    out.print(".dial-hint{font-size:11px;color:var(--muted);text-align:center}");

    // Speed sliders
    out.print(".spd-row{display:flex;gap:10px;margin-top:10px}");
    out.print(".spd-item{flex:1;background:var(--panel);border:1px solid var(--border);"
              "border-radius:8px;padding:10px 12px}");
    out.print(".spd-lbl{font-size:11px;color:var(--muted);text-transform:uppercase;"
              "letter-spacing:.05em;margin-bottom:8px;font-weight:700}");
    out.print(".spd-val{font-size:12px;color:var(--accent);text-align:right;margin-top:5px}");
    out.print("input[type=range]{-webkit-appearance:none;appearance:none;"
              "width:100%;height:4px;background:var(--border2);"
              "border-radius:2px;outline:none;cursor:pointer}");
    out.print("input[type=range]::-webkit-slider-thumb{"
              "-webkit-appearance:none;width:20px;height:20px;"
              "background:var(--accent);border-radius:50%;border:none}");
    out.print("input[type=range]::-moz-range-thumb{"
              "width:20px;height:20px;background:var(--accent);"
              "border-radius:50%;border:none}");

    // Status chips
    out.print(".chip-grid{display:grid;grid-template-columns:repeat(3,1fr);gap:6px}");
    out.print(".chip{background:var(--panel);border:1px solid var(--border);"
              "border-radius:8px;padding:8px 10px}");
    out.print(".chip .sv{font-size:14px;font-weight:700}");
    out.print(".chip .sl{font-size:10px;color:var(--muted);margin-top:2px;"
              "text-transform:uppercase;letter-spacing:.04em}");

    // Card
    out.print(".card{background:var(--panel);border:1px solid var(--border);"
              "border-radius:10px;padding:14px}");

    // Settings list
    out.print(".set-row{display:flex;align-items:center;padding:13px 0;"
              "border-bottom:1px solid var(--border);"
              "cursor:pointer;text-decoration:none;color:inherit}");
    out.print(".set-row:hover .set-name{color:var(--accent)}");
    out.print(".set-row:last-child{border-bottom:none}");
    out.print(".set-icon{width:34px;height:34px;border-radius:8px;"
              "display:flex;align-items:center;justify-content:center;"
              "margin-right:14px;flex-shrink:0}");
    out.print(".set-main{flex:1;min-width:0}");
    out.print(".set-name{font-size:15px;color:var(--text);font-weight:600;"
              "transition:color .15s}");
    out.print(".set-val{font-size:12px;color:var(--muted);margin-top:3px;"
              "white-space:nowrap;overflow:hidden;text-overflow:ellipsis}");
    out.print(".set-arr{color:var(--border2);font-size:18px;"
              "flex-shrink:0;margin-left:6px}");
    out.print(".danger-zone{margin-top:20px;border-top:1px solid var(--border);"
              "padding-top:14px}");
    out.print(".dz-lbl{font-size:11px;color:var(--muted);text-transform:uppercase;"
              "letter-spacing:.07em;margin-bottom:10px;font-weight:700}");

    // Form inputs
    out.print("input[type=text],input[type=password],input[type=number]{"
              "width:100%;padding:10px 12px;font-size:14px;"
              "border:1px solid var(--border2);border-radius:8px;"
              "background:var(--bg);color:var(--text);font-family:inherit;"
              "appearance:none;-webkit-appearance:none;transition:border-color .15s}");
    out.print("input[type=text]:focus,input[type=password]:focus,"
              "input[type=number]:focus{outline:none;border-color:var(--accent);"
              "box-shadow:0 0 0 2px rgba(79,195,247,.1)}");
    out.print("select{width:100%;padding:10px 12px;font-size:14px;"
              "border:1px solid var(--border2);border-radius:8px;"
              "background:var(--bg);color:var(--text);font-family:inherit;"
              "appearance:none;-webkit-appearance:none}");
    out.print("select:focus{outline:none;border-color:var(--accent)}");
    out.print("label.fl{display:block;font-size:11px;color:var(--muted);"
              "font-weight:700;letter-spacing:.07em;text-transform:uppercase;"
              "margin-bottom:6px}");
    out.print(".fg{margin-bottom:14px}");
    out.print(".form-actions{display:flex;gap:8px;margin-top:16px}");
    out.print(".form-actions button{flex:1}");
    out.print(".back-link{display:inline-flex;align-items:center;gap:6px;"
              "font-size:13px;color:var(--accent);text-decoration:none;"
              "font-weight:600;padding:12px 0}");
    out.print(".pg-title{font-size:15px;font-weight:700;color:var(--text);"
              "padding:10px 0 14px;border-bottom:1px solid var(--border);"
              "margin-bottom:14px;letter-spacing:.02em}");
    out.print(".hint{font-size:12px;color:var(--muted);margin-top:5px}");
    out.print(".cb-row{display:flex;align-items:center;gap:10px;padding:10px 0;"
              "border-bottom:1px solid var(--border);cursor:pointer}");
    out.print(".cb-row:last-child{border-bottom:none}");
    out.print(".cb-row input[type=checkbox]{width:18px;height:18px;"
              "accent-color:var(--accent);cursor:pointer;flex-shrink:0}");
    out.print(".cb-lbl{font-size:14px;color:var(--text);font-weight:500}");
    out.print(".cb-hint{font-size:12px;color:var(--muted);margin-top:2px}");
    out.print(".msg{font-size:13px;padding:10px 14px;border-radius:8px;"
              "margin-top:10px;text-align:center}");
    out.print(".msg.ok{background:rgba(74,222,128,.1);color:var(--green);"
              "border:1px solid rgba(74,222,128,.3)}");
    out.print(".msg.err{background:rgba(239,83,80,.1);color:var(--red);"
              "border:1px solid rgba(239,83,80,.3)}");

    // Misc
    out.print(".jog{touch-action:none;user-select:none;-webkit-user-select:none}");
    out.print("hr{border:none;border-top:1px solid var(--border);margin:12px 0}");
    out.print("a{color:var(--accent)}");

    // Log viewer
    out.print("#lb{background:#050709;color:#4ade80;font-family:monospace;"
              "font-size:12px;height:calc(100vh - 200px);overflow-y:auto;"
              "padding:12px;border-radius:8px;border:1px solid var(--border)}");

    // Tablet >= 640px
    out.print("@media(min-width:640px){");
    out.print("#page{padding:16px 24px 24px;max-width:720px}");
    out.print(".sv{font-size:15px}");
    out.print(".ss{padding:8px 6px}");
    out.print(".pos-row{height:240px}");
    out.print(".lift-col{width:80px}");
    out.print(".chip-grid{grid-template-columns:repeat(5,1fr)}");
    out.print(".g5{grid-template-columns:repeat(9,1fr)}");
    out.print("#lb{height:calc(100vh - 220px)}");
    out.print("}");

    // Desktop >= 1024px
    out.print("@media(min-width:1024px){");
    out.print("body{font-size:14px}");
    out.print("#sbar{padding:10px 24px;gap:8px}");
    out.print(".ss{padding:8px 6px;border-radius:8px}");
    out.print(".sv{font-size:16px}");
    out.print("#page{padding:20px 32px 32px;max-width:860px;margin:0 auto}");
    out.print(".pos-row{height:280px}");
    out.print(".lift-col{width:90px}");
    out.print(".chip-grid{grid-template-columns:repeat(6,1fr);gap:8px}");
    out.print(".g5{grid-template-columns:repeat(9,1fr)}");
    out.print("#lb{height:calc(100vh - 200px)}");
    out.print("}");

    // Theme toggle button styling (sits flush right in the nav bar)
    out.print("#themetoggle{margin-left:auto;border:none;background:transparent;"
              "color:var(--muted);cursor:pointer;padding:0 14px;"
              "font-size:18px;display:flex;align-items:center;"
              "transition:color .15s;width:auto}");
    out.print("#themetoggle:hover{color:var(--text);background:var(--panel)}");

    out.print("</style>");
    // Apply saved theme before first paint (avoids flash)
    out.print("<script>"
              "if(localStorage.getItem('theme')==='light'){"
              "document.documentElement.classList.add('light-pending');}"
              "</script>");
    out.print("<style>.light-pending body,.light-pending #topnav{"
              "transition:none!important}</style>");
    out.print("</head><body>");
    // Apply to body immediately
    out.print("<script>"
              "(function(){"
              "if(localStorage.getItem('theme')==='light')"
              "document.body.classList.add('light');"
              "})();"
              "</script>");
}

// Top navigation bar — always visible on all screen sizes
static void printTopNav(Print& out, const char* active)
{
    struct { const char* href; const char* id; const char* label; } tabs[] = {
        { "/periscope", "periscope", "periscope" },
        { "/rescue",    "rescue",    "rescue"    },
        { "/log",       "log",       "log"       },
        { "/setup",     "setup",     "setup"     },
    };
    out.print("<nav id='topnav'>");
    out.print("<div class='tn-title'>R2 Uppity</div>");
    for (auto& t : tabs) {
        bool on = (strcmp(t.id, active) == 0);
        out.print("<a href='"); out.print(t.href); out.print("'");
        if (on) out.print(" class='active'");
        out.print(">"); out.print(t.label); out.print("</a>");
    }
    // Theme toggle — right-aligned in the nav bar
    out.print("<button id='themetoggle' onclick='_toggleTheme()' title='Toggle light/dark'>"
              "<span id='theme_icon'>&#9790;</span>"   // crescent moon = dark mode indicator
              "</button>");
    out.print("<script>"
              "function _toggleTheme(){"
              "var b=document.body,isLight=b.classList.toggle('light');"
              "localStorage.setItem('theme',isLight?'light':'dark');"
              "document.getElementById('theme_icon').textContent=isLight?'\\u2600':'\\u263E';}"
              // Set correct icon on load
              "(function(){"
              "if(localStorage.getItem('theme')==='light')"
              "document.getElementById('theme_icon').textContent='\\u2600';"
              "})();"
              "</script>");
    out.print("</nav>");
}

// Writes the 5-cell status bar
static void printStatusBar(Print& out)
{
    out.print("<div id='sbar'>");
    out.print("<div class='ss'><div class='sv c-b' id='st_h'>--</div>"
              "<div class='sl'>height</div></div>");
    out.print("<div class='ss'><div class='sv c-b' id='st_r'>--</div>"
              "<div class='sl'>rotation</div></div>");
    out.print("<div class='ss'><div class='sv' id='st_s'>--</div>"
              "<div class='sl'>safety</div></div>");
    out.print("<div class='ss'><div class='sv' id='st_m'>--</div>"
              "<div class='sl'>motors</div></div>");
    out.print("<div class='ss'><div class='sv c-m' id='st_c' "
              "style='overflow:hidden;white-space:nowrap;text-overflow:ellipsis'>--</div>"
              "<div class='sl'>cmd</div></div>");
    out.print("</div>");
}

// printTabBar kept for call-site compatibility — nav is now in printTopNav
static void printTabBar(Print& out, const char* active)
{
    (void)out; (void)active; // nav bar is rendered by printTopNav at page top
}

// Shared status-bar poll JS block (used by setup + calibrate pages that have no other poll)
static void printStatusPollJS(Print& out)
{
    out.print("<script>");
    out.print("function _sc(id,cls,txt){var e=document.getElementById(id);"
              "if(!e)return;e.textContent=txt;e.className='sv '+cls;}");
    out.print("function _poll(){fetch('/api/status')"
              ".then(function(r){return r.json();})"
              ".then(function(d){");
    out.print("_sc('st_h','c-b',d.height+'%');");
    out.print("var rd=Math.round(d.rotDeg||0);"
              "_sc('st_r','c-b',rd<5||rd>355?'home':rd+'\\u00b0');");
    out.print("_sc('st_s',d.fault?'c-r':(d.safety?'c-g':'c-y'),"
              "d.fault?'FAULT':(d.safety?'Ready':'WAIT'));");
    out.print("_sc('st_m',d.motors?'c-g':'c-r',d.motors?'ON':'off');");
    out.print("var e=document.getElementById('st_c');"
              "if(e&&d.lastCmd)e.textContent=d.lastCmd;");
    out.print("}).catch(function(){});}");
    out.print("setInterval(_poll,1000);_poll();");
    out.print("</script>");
}

// Shared live-status + switches HTML block (used on both periscope and rescue pages)
// prefix = unique ID prefix to avoid conflicts when both pages share the DOM
static void printDefinitionsBox(Print& out)
{
    out.print("<div style='font-size:12px;color:var(--muted);line-height:1.6;"
              "background:var(--panel);border:1px solid var(--border);border-radius:10px;"
              "padding:10px 12px;margin-bottom:8px'>");
    out.print("<b style='color:var(--text)'>Safety</b> — the startup maneuver: "
              "the board lowers the periscope and finds the rotary home position "
              "before allowing any movement. Shows <b style='color:var(--green)'>Ready</b> "
              "once complete, <b style='color:var(--yellow)'>WAIT</b> while running, "
              "<b style='color:var(--red)'>FAULT</b> if it failed.<br>"
              "<b style='color:var(--text)'>Fault</b> — a motor driver fault: "
              "the motor controller detected an overcurrent or hardware error. "
              "Usually cleared by the E-STOP button on the Rescue tab.");
    out.print("</div>");
}

static void printStatusChips(Print& out, const char* prefix)
{
    out.print("<div class='sec'>live status</div>");
    out.print("<div class='chip-grid'>");
    char id[24];
    snprintf(id, sizeof(id), "%s_h", prefix);
    out.print("<div class='chip'><div class='sv c-b' id='"); out.print(id);
    out.print("'>--</div><div class='sl'>height</div></div>");
    snprintf(id, sizeof(id), "%s_t", prefix);
    out.print("<div class='chip'><div class='sv c-m' id='"); out.print(id);
    out.print("'>--</div><div class='sl'>enc ticks</div></div>");
    snprintf(id, sizeof(id), "%s_r", prefix);
    out.print("<div class='chip'><div class='sv c-m' id='"); out.print(id);
    out.print("'>--</div><div class='sl'>rot deg</div></div>");
    snprintf(id, sizeof(id), "%s_s", prefix);
    out.print("<div class='chip'><div class='sv' id='"); out.print(id);
    out.print("'>--</div><div class='sl'>safety</div></div>");
    snprintf(id, sizeof(id), "%s_mo", prefix);
    out.print("<div class='chip'><div class='sv' id='"); out.print(id);
    out.print("'>--</div><div class='sl'>motors</div></div>");
    snprintf(id, sizeof(id), "%s_f", prefix);
    out.print("<div class='chip'><div class='sv' id='"); out.print(id);
    out.print("'>--</div><div class='sl'>driver fault</div></div>");
    out.print("</div>");

    out.print("<div class='sec'>limit switches</div>");
    out.print("<div class='chip-grid'>");
    snprintf(id, sizeof(id), "%s_sw_t", prefix);
    out.print("<div class='chip'><div class='sv' id='"); out.print(id);
    out.print("'>--</div><div class='sl'>top limit</div></div>");
    snprintf(id, sizeof(id), "%s_sw_b", prefix);
    out.print("<div class='chip'><div class='sv' id='"); out.print(id);
    out.print("'>--</div><div class='sl'>bot limit</div></div>");
    snprintf(id, sizeof(id), "%s_sw_r", prefix);
    out.print("<div class='chip'><div class='sv' id='"); out.print(id);
    out.print("'>--</div><div class='sl'>rot home</div></div>");
    snprintf(id, sizeof(id), "%s_sw_l", prefix);
    out.print("<div class='chip'><div class='sv' id='"); out.print(id);
    out.print("'>--</div><div class='sl'>lift fault</div></div>");
    snprintf(id, sizeof(id), "%s_sw_f", prefix);
    out.print("<div class='chip'><div class='sv' id='"); out.print(id);
    out.print("'>--</div><div class='sl'>rot fault</div></div>");
    out.print("</div>");
}

// JS that updates a status+switches chip block given a data object and prefix string
// Emitted inside a script tag; call updateChips(d, 'prefix') from each poll callback
static void printUpdateChipsJS(Print& out)
{
    out.print("function _sc(id,cls,txt){var e=document.getElementById(id);"
              "if(!e)return;e.textContent=txt;e.className='sv '+cls;}");
    out.print("function _sw(id,v){var e=document.getElementById(id);"
              "if(!e)return;e.textContent=v?'ON':'off';"
              "e.className='sv '+(v?'c-g':'c-r');}");
    out.print("function updateChips(d,p){"
              "_sc(p+'_h','c-b',d.height+'%');"
              "var e=document.getElementById(p+'_t');if(e)e.textContent=d.ticks;"
              "_sc(p+'_r','c-b',(Math.round(d.rotDeg||0)<5||Math.round(d.rotDeg||0)>355)?'home':Math.round(d.rotDeg||0)+'\\u00b0');"
              "_sc(p+'_s',d.fault?'c-r':(d.safety?'c-g':'c-y'),"
              "d.fault?'FAULT':(d.safety?'Ready':'WAIT'));"
              "_sc(p+'_mo',d.motors?'c-g':'c-r',d.motors?'ON':'off');"
              "_sc(p+'_f',(d.liftFault||d.rotFault)?'c-r':'c-g',"
              "(d.liftFault||d.rotFault)?'FAULT':'ok');"
              "_sw(p+'_sw_t',d.topLimit);"
              "_sw(p+'_sw_b',d.botLimit);"
              "_sw(p+'_sw_r',d.rotHome);"
              "_sw(p+'_sw_l',d.liftFault);"
              "_sw(p+'_sw_f',d.rotFault);}");
}

///////////////////////////////////////////////////////////////////////////////
// PERISCOPE PAGE
///////////////////////////////////////////////////////////////////////////////

static void printPeriscopePage(Print& out)
{
    printPageHead(out, "R2 Periscope");
    printTopNav(out, "periscope");
    printStatusBar(out);
    out.print("<div id='page'>");

    // Position controls
    out.print("<div class='sec'>position</div>");
    out.print("<div class='pos-row'>");

    // Lift canvas joystick column — JS resizes on load
    out.print("<div class='lift-col'>");
    out.print("<div class='lift-val' id='lv'>0%</div>");
    out.print("<canvas id='lc' width='60' height='130' "
              "style='border-radius:14px;flex:1'></canvas>");
    out.print("<div class='lift-lbl'>lift</div>");
    out.print("</div>");

    // Rotary dial — JS resizes on load
    out.print("<div class='dial-col'>");
    out.print("<canvas id='dc' width='160' height='160' "
              "style='border-radius:50%'></canvas>");
    out.print("<div class='dial-val'><span id='dv'>0</span>&deg;</div>");
    out.print("<div class='dial-hint'>drag &amp; release to set position</div>");
    out.print("</div>");

    out.print("</div>"); // .pos-row

    // ── STOP — big, prominent, easy to hit ─────────────────────────────────
    out.print("<button class='estop' "
              "onclick='estop()'>&#9632; STOP</button>");

    // ── Lifter actions ───────────────────────────────────────────────────
    out.print("<div class='sec'>lifter</div>");
    out.print("<div class='g2'>");
    out.print("<button class='primary' "
              "onclick='cmd(\":PP100,\"+lspd())'>&#8679; up</button>");
    // :PH homes rotary before descending — critical for dome clearance.
    out.print("<button onclick='cmd(\":PH\"+lspd())'>&#8681; down</button>");
    out.print("</div>");

    // ── Rotary actions ───────────────────────────────────────────────────
    out.print("<div class='sec'>rotary</div>");
    out.print("<div class='g3'>");
    out.print("<button class='primary' "
              "onclick='cmd(\":PR\"+rspd())'>&#8634; spin left</button>");
    out.print("<button class='primary' "
              "onclick='cmd(\":PR-\"+rspd())'>spin right &#8635;</button>");
    out.print("<button onclick='fetch(\"/api/action?do=homerotary\")'>&#8962; home</button>");
    out.print("</div>");

    // ── Random ───────────────────────────────────────────────────────────
    out.print("<div style='margin-top:8px'>");
    out.print("<button class='primary' style='width:100%' "
              "onclick='cmd(\":PM\")'>&#127922; random</button>");
    out.print("</div>");

    // ── Light kit ─────────────────────────────────────────────────────────
    out.print("<div class='sec'>light kit</div>");
    out.print("<div class='lk' id='lkrow'>");
    const char* lkNames[] = { "full","off","obi wan","yoda",
                               "sith","search","dagobah","sparkle" };
    for (int i = 0; i < 8; i++) {
        out.print("<div class='lp' id='lk"); out.print(i);
        out.print("' onclick='setLK("); out.print(i); out.print(")'>");
        out.print(lkNames[i]); out.print("</div>");
    }
    out.print("</div>");

    // Sequences
    out.print("<div class='sec'>sequences</div>");
    out.print("<div class='g5'>");
    for (int i = 0; i < 9; i++) {
        out.print("<div class='sq' id='sq"); out.print(i);
        out.print("' onclick='runSeq("); out.print(i); out.print(")'>");
        out.print(i); out.print("</div>");
    }
    out.print("</div>");

    // ── Speed sliders (at bottom for easy access without cluttering actions) ──
    out.print("<div class='sec'>speed</div>");
    out.print("<div class='spd-row'>");
    out.print("<div class='spd-item'>"
              "<div class='spd-lbl'>lift speed</div>"
              "<input type='range' id='lspd' min='0' max='100' value='50' step='1'>"
              "<div class='spd-val'><span id='lsv'>50</span>%</div>"
              "</div>");
    out.print("<div class='spd-item'>"
              "<div class='spd-lbl'>rotary speed</div>"
              "<input type='range' id='rspd' min='0' max='100' value='50' step='1'>"
              "<div class='spd-val'><span id='rsv'>50</span>%</div>"
              "</div>");
    out.print("</div>");

    // Live status + switches (duplicated from rescue)
    printStatusChips(out, "p");

    // Definitions box — at very bottom
    printDefinitionsBox(out);

    out.print("</div>"); // #page
    printTabBar(out, "periscope");

    out.print("<script>");
    printUpdateChipsJS(out);

    out.print("function lspd(){return parseInt(document.getElementById('lspd').value);}");
    out.print("function rspd(){return parseInt(document.getElementById('rspd').value);}");
    out.print("document.getElementById('lspd').oninput=function(){"
              "document.getElementById('lsv').textContent=this.value;};");
    out.print("document.getElementById('rspd').oninput=function(){"
              "document.getElementById('rsv').textContent=this.value;};");
    out.print("function cmd(c){"
              "fetch('/api/cmd?c='+encodeURIComponent(c)).catch(function(){});}");
    out.print("function estop(){fetch('/api/estop').catch(function(){});}");

    // ── Lift canvas joystick ──────────────────────────────────────────────
    // Throttle = direction × (position fraction) × (lift speed / 100)
    // Canvas dimensions derived from viewport at init time for responsiveness.
    out.print("var _lc=document.getElementById('lc'),"
              "_lctx=_lc.getContext('2d');");
    // Responsive sizing: phone=60×130, tablet=70×160, desktop=80×200
    out.print("var _vw=window.innerWidth;");
    out.print("var _lcW=_vw>=1024?80:(_vw>=640?70:60);");
    out.print("var _lcH=_vw>=1024?200:(_vw>=640?160:130);");
    out.print("_lc.width=_lcW;_lc.height=_lcH;");
    out.print("var _lcActive=false,_lcDir=0,_lcKA=null,_lcPY=_lcH/2;");

    out.print("function drawLift(py){");
    out.print("_lctx.clearRect(0,0,_lcW,_lcH);");
    out.print("_lctx.fillStyle='#0f1117';"
              "_lctx.beginPath();_lctx.roundRect(0,0,_lcW,_lcH,14);"
              "_lctx.fill();");
    out.print("_lctx.strokeStyle='#2a2d3a';_lctx.lineWidth=1;"
              "_lctx.beginPath();_lctx.moveTo(8,_lcH/2);"
              "_lctx.lineTo(_lcW-8,_lcH/2);_lctx.stroke();");
    out.print("_lctx.fillStyle='rgba(79,195,247,0.07)';"
              "_lctx.beginPath();"
              "_lctx.roundRect(1,1,_lcW-2,_lcH/2-1,"
              "{upperLeft:13,upperRight:13});_lctx.fill();");
    out.print("_lctx.fillStyle='rgba(239,83,80,0.07)';"
              "_lctx.beginPath();"
              "_lctx.roundRect(1,_lcH/2,_lcW-2,_lcH/2-1,"
              "{lowerLeft:13,lowerRight:13});_lctx.fill();");
    out.print("_lctx.font='bold 20px sans-serif';_lctx.textAlign='center';"
              "_lctx.fillStyle='#4fc3f7';"
              "_lctx.fillText('\\u25B2',_lcW/2,26);"
              "_lctx.fillStyle='#ef5350';"
              "_lctx.fillText('\\u25BC',_lcW/2,_lcH-8);");
    out.print("if(_lcActive&&py!==null){"
              "_lctx.beginPath();_lctx.arc(_lcW/2,py,16,0,6.283);"
              "_lctx.fillStyle=_lcDir>0?'#4fc3f7':'#ef5350';"
              "_lctx.fill();}");
    out.print("}");
    out.print("drawLift(null);");

    // _lcSend: throttle = dir × posFrac × (liftSpeed/100)
    // This means dragging to the very top with speed=50 → t=0.5, not t=1.0
    out.print("function _lcSend(){"
              "var frac=Math.min(Math.abs(_lcPY-_lcH/2)/(_lcH/2),1);"
              "var spd=lspd()/100;"
              "var t=(_lcDir*frac*spd).toFixed(3);"
              "fetch('/api/lift?t='+t).catch(function(){});}");
    out.print("function _lcStop(){"
              "_lcActive=false;_lcDir=0;"
              "clearInterval(_lcKA);_lcKA=null;"
              "fetch('/api/lift?t=0').catch(function(){});"
              "drawLift(null);}");
    out.print("_lc.addEventListener('pointerdown',function(e){"
              "_lc.setPointerCapture(e.pointerId);"
              "_lcActive=true;e.preventDefault();"
              "var r=_lc.getBoundingClientRect();"
              "_lcPY=Math.max(0,Math.min(_lcH,e.clientY-r.top));"
              "_lcDir=_lcPY<_lcH/2?1:-1;"
              "_lcSend();drawLift(_lcPY);"
              "_lcKA=setInterval(_lcSend,1500);});");
    out.print("_lc.addEventListener('pointermove',function(e){"
              "if(!_lcActive)return;e.preventDefault();"
              "var r=_lc.getBoundingClientRect();"
              "_lcPY=Math.max(0,Math.min(_lcH,e.clientY-r.top));"
              "_lcDir=_lcPY<_lcH/2?1:-1;drawLift(_lcPY);});");
    out.print("_lc.addEventListener('pointerup',function(e){_lcStop();});");
    out.print("_lc.addEventListener('pointercancel',function(e){_lcStop();});");

    // ── Rotary dial — responsive size ─────────────────────────────────────
    out.print("var _dc=document.getElementById('dc'),"
              "_dctx=_dc.getContext('2d');");
    // phone=160, tablet=200, desktop=240
    out.print("var _dsz=_vw>=1024?240:(_vw>=640?200:160);");
    out.print("_dc.width=_dsz;_dc.height=_dsz;");
    out.print("var _dang=0,_dactive=false,"
              "_dcx=_dsz/2,_dcy=_dsz/2,_dcr=_dsz/2-6;");

    out.print("function drawDial(){");
    out.print("_dctx.clearRect(0,0,_dsz,_dsz);");
    out.print("_dctx.beginPath();_dctx.arc(_dcx,_dcy,_dcr,0,6.283);"
              "_dctx.fillStyle='#1a1d28';_dctx.fill();"
              "_dctx.strokeStyle='#2a2d3a';_dctx.lineWidth=2;_dctx.stroke();");
    out.print("for(var i=0;i<12;i++){"
              "var a=i*Math.PI/6,r1=_dcr-5,r2=_dcr-13;"
              "_dctx.beginPath();"
              "_dctx.moveTo(_dcx+r1*Math.sin(a),_dcy-r1*Math.cos(a));"
              "_dctx.lineTo(_dcx+r2*Math.sin(a),_dcy-r2*Math.cos(a));"
              "_dctx.strokeStyle='#353c47';_dctx.lineWidth=1.5;"
              "_dctx.stroke();}");
    // Home tick (top, 0°) highlighted green
    out.print("_dctx.beginPath();"
              "_dctx.moveTo(_dcx,_dcy-(_dcr-5));"
              "_dctx.lineTo(_dcx,_dcy-(_dcr-14));"
              "_dctx.strokeStyle='#4ade80';_dctx.lineWidth=2.5;"
              "_dctx.stroke();");
    out.print("var na=_dang*Math.PI/180;"
              "_dctx.beginPath();_dctx.moveTo(_dcx,_dcy);"
              "_dctx.lineTo(_dcx+(_dcr-10)*Math.sin(na),"
              "_dcy-(_dcr-10)*Math.cos(na));"
              "_dctx.strokeStyle='#4fc3f7';_dctx.lineWidth=3;"
              "_dctx.lineCap='round';_dctx.stroke();");
    out.print("_dctx.beginPath();_dctx.arc(_dcx,_dcy,6,0,6.283);"
              "_dctx.fillStyle='#4fc3f7';_dctx.fill();");
    // "home" label at top
    out.print("_dctx.fillStyle='#4ade80';_dctx.font='bold 10px sans-serif';"
              "_dctx.textAlign='center';"
              "_dctx.fillText('home',_dcx,_dcy-_dcr+26);");
    out.print("}");
    out.print("drawDial();");

    out.print("function _dptr(e){"
              "var r=_dc.getBoundingClientRect();"
              "return{x:e.clientX-r.left-_dcx,y:e.clientY-r.top-_dcy};}");
    out.print("function _dangle(p){"
              "var a=Math.atan2(p.x,-p.y)*180/Math.PI;return(a+360)%360;}");
    out.print("_dc.addEventListener('pointerdown',function(e){"
              "_dc.setPointerCapture(e.pointerId);_dactive=true;"
              "e.preventDefault();});");
    out.print("_dc.addEventListener('pointermove',function(e){"
              "if(!_dactive)return;"
              "var p=_dptr(e),a=_dangle(p);"
              "_dang=a;drawDial();"
              "document.getElementById('dv').textContent=Math.round(a);"
              "e.preventDefault();});");
    out.print("_dc.addEventListener('pointerup',function(e){"
              "if(!_dactive)return;_dactive=false;"
              "cmd(':PA'+Math.round(_dang)+','+rspd());"
              "e.preventDefault();});");
    out.print("_dc.addEventListener('pointercancel',"
              "function(e){_dactive=false;});");

    // Light kit
    out.print("var _lk=-1;");
    out.print("function setLK(n){"
              "if(_lk>=0){var p=document.getElementById('lk'+_lk);"
              "if(p)p.className='lp';}"
              "_lk=n;"
              "var p=document.getElementById('lk'+n);"
              "if(p)p.className='lp on';"
              "cmd(':PL'+n);}");

    // Sequences
    out.print("var _sq=-1;");
    out.print("function runSeq(n){"
              "if(_sq>=0){var s=document.getElementById('sq'+_sq);"
              "if(s)s.className='sq';}"
              "_sq=n;"
              "var s=document.getElementById('sq'+n);"
              "if(s)s.className='sq on';"
              "cmd(':PS'+n);}");

    // Status poll — updates status bar, syncs rotary dial from board, updates chips
    out.print("function _poll(){fetch('/api/status')"
              ".then(function(r){return r.json();})"
              ".then(function(d){");
    out.print("_sc('st_h','c-b',d.height+'%');");
    // Rotary status bar cell — show "home" when at/near 0°
    out.print("var rd=Math.round(d.rotDeg||0);"
              "_sc('st_r','c-b',rd<5||rd>355?'home':rd+'\\u00b0');");
    out.print("_sc('st_s',d.fault?'c-r':(d.safety?'c-g':'c-y'),"
              "d.fault?'FAULT':(d.safety?'Ready':'WAIT'));");
    out.print("_sc('st_m',d.motors?'c-g':'c-r',d.motors?'ON':'off');");
    out.print("var e=document.getElementById('st_c');"
              "if(e&&d.lastCmd)e.textContent=d.lastCmd;");
    // Sync rotary dial needle from board (only when user isn't dragging)
    out.print("if(!_dactive&&typeof d.rotDeg!=='undefined'){"
              "_dang=d.rotDeg;"
              "drawDial();"
              "document.getElementById('dv').textContent=Math.round(d.rotDeg);}");
    // Sync lift height display when joystick not active
    out.print("if(!_lcActive)"
              "document.getElementById('lv').textContent=d.height+'%';");
    // Sync light kit indicator
    out.print("if(typeof d.lightkit!=='undefined'&&d.lightkit!==_lk){"
              "if(_lk>=0){var p=document.getElementById('lk'+_lk);"
              "if(p)p.className='lp';}"
              "_lk=d.lightkit;"
              "var p=document.getElementById('lk'+d.lightkit);"
              "if(p)p.className='lp on';}");
    // Update status chips
    out.print("updateChips(d,'p');");
    out.print("}).catch(function(){});}");
    out.print("setInterval(_poll,1000);_poll();");
    out.print("</script></body></html>");
}

///////////////////////////////////////////////////////////////////////////////
// RESCUE PAGE
///////////////////////////////////////////////////////////////////////////////

static void printRescuePage(Print& out)
{
    printPageHead(out, "R2 Rescue");
    printTopNav(out, "rescue");
    printStatusBar(out);
    out.print("<div id='page'>");

    out.print("<button class='estop' "
              "onclick='fetch(\"/api/estop\")'>&#9632; EMERGENCY STOP</button>");

    out.print("<div class='sec'>lifter jog</div>");
    out.print("<div class='card' style='margin-bottom:10px'>");
    out.print("<div style='display:flex;align-items:center;"
              "gap:10px;margin-bottom:10px'>");
    out.print("<span style='font-size:13px;color:var(--muted);"
              "white-space:nowrap;font-weight:600'>jog speed</span>");
    out.print("<input type='range' id='jspd' min='0' max='100' "
              "value='80' step='1' style='flex:1'>");
    out.print("<span style='font-size:15px;color:var(--accent);"
              "min-width:38px;text-align:right;font-weight:700'>"
              "<span id='jsv'>80</span>%</span>");
    out.print("</div>");
    out.print("<div class='g3'>");
    out.print("<button class='jog primary' "
              "onpointerdown='jogStart(1,this)' onpointerup='jogStop()' "
              "onpointercancel='jogStop()'>&#8679; up</button>");
    out.print("<button class='danger' "
              "onclick='fetch(\"/api/lift?t=0\")'>&#9632; stop</button>");
    out.print("<button class='jog' "
              "onpointerdown='jogStart(-1,this)' onpointerup='jogStop()' "
              "onpointercancel='jogStop()'>down &#8681;</button>");
    out.print("</div></div>");

    out.print("<div class='sec'>rotary</div>");
    out.print("<div class='card' style='margin-bottom:10px'>");
    out.print("<div style='display:flex;flex-direction:column;"
              "align-items:center;gap:6px'>");
    out.print("<canvas id='rd' width='180' height='180' "
              "style='border-radius:50%'></canvas>");
    out.print("<div style='font-size:13px;color:var(--muted)'>"
              "drag left/right &bull; release to stop</div>");
    out.print("<div style='font-size:16px;color:var(--accent);font-weight:700'>"
              "<span id='rv'>0</span>% &nbsp;"
              "<span id='rd2'>&#9632; stopped</span></div>");
    out.print("</div>");
    out.print("<div class='g2' style='margin-top:10px'>");
    out.print("<button class='danger' onclick='rdStop()'>&#9632; stop</button>");
    out.print("<button class='primary' "
              "onclick='fetch(\"/api/action?do=sethome\")'>set home</button>");
    out.print("</div></div>");

    out.print("<div class='sec'>recovery</div>");
    out.print("<div class='card' style='margin-bottom:10px'>"
              "<div class='g2'>");
    out.print("<button class='danger' "
              "onclick='fetch(\"/api/action?do=clearfault\")'>"
              "clear fault</button>");
    out.print("<button class='primary' "
              "onclick='fetch(\"/api/action?do=enablemotors\")'>"
              "enable motors</button>");
    out.print("<button class='primary' "
              "onclick='fetch(\"/api/action?do=homerotary\")'>"
              "home rotary</button>");
    out.print("<button class='primary' "
              "onclick='fetch(\"/api/action?do=safety\")'>"
              "re-run safety</button>");
    out.print("<button onclick='fetch(\"/api/action?do=clearenc\")'>"
              "clear encoder</button>");
    out.print("</div></div>");

    // ── Safe Spin Height ─────────────────────────────────────────────────
    out.print("<div class='sec'>safe spin height</div>");
    out.print("<div class='card' style='margin-bottom:10px'>");
    out.print("<p style='font-size:13px;color:var(--text);line-height:1.6;margin-bottom:12px'>"
              "Raise the periscope to a height where the head safely clears the dome, "
              "then press the button to save that position as the minimum allowed spin height. "
              "The rotary motor will not be permitted to run below this point.</p>");
    out.print("<div class='g2' style='margin-bottom:12px'>");
    out.print("<div class='chip'>"
              "<div class='sv c-b' id='ssh_cur'>--</div>"
              "<div class='sl'>current height</div></div>");
    out.print("<div class='chip'>"
              "<div class='sv c-g' id='ssh_min'>--</div>"
              "<div class='sl'>saved min height</div></div>");
    out.print("</div>");
    out.print("<button class='primary' style='width:100%' id='ssh_btn' "
              "onclick='setSafeHeight()'>&#128204; set safe spin height to current</button>");
    out.print("<div id='ssh_msg' style='text-align:center;font-size:13px;"
              "margin-top:8px;min-height:18px'></div>");
    out.print("</div>");

    // ── Safety Override ──────────────────────────────────────────────────
    out.print("<div class='sec'>safety override</div>");
    out.print("<div class='card' id='override-section'>"
              "<div id='ovr-off'>"
              "<p style='font-size:0.85em;color:#aaa;margin:0 0 8px'>Hold button 3 seconds to temporarily bypass safety checks (60s).</p>"
              "<button id='ovr-btn' class='danger' style='width:100%;height:48px;font-size:1.1em'>"
              "&#9888; hold to override safeties</button>"
              "</div>"
              "<div id='ovr-on' style='display:none'>"
              "<div style='background:#b00;color:#fff;padding:10px;border-radius:6px;"
              "text-align:center;font-weight:bold;margin-bottom:8px'>"
              "&#9888; SAFETIES OVERRIDDEN &mdash; <span id='ovr-time'>60</span>s remaining</div>"
              "<button class='primary' style='width:100%' "
              "onclick='fetch(\"/api/action?do=override_off\")'>cancel override</button>"
              "</div>"
              "</div>");

    // Status chips (rescue prefix = "c")
    printStatusChips(out, "c");
    printDefinitionsBox(out);

    out.print("</div>"); // #page
    printTabBar(out, "rescue");

    out.print("<script>");
    printUpdateChipsJS(out);

    out.print("document.getElementById('jspd').oninput=function(){"
              "document.getElementById('jsv').textContent=this.value;};");

    // Jog hold-to-run with keepalive
    out.print("var _jt=null,_jd=0;");
    out.print("function jogStart(dir,el){"
              "el.setPointerCapture(event.pointerId);"
              "_jd=dir;_sendJog();"
              "_jt=setInterval(_sendJog,1500);}");
    out.print("function _sendJog(){"
              "var v=(parseInt(document.getElementById('jspd').value)"
              "/100*_jd).toFixed(3);"
              "fetch('/api/lift?t='+v).catch(function(){});}");
    out.print("function jogStop(){"
              "clearInterval(_jt);_jd=0;"
              "fetch('/api/lift?t=0').catch(function(){});}");

    // Rotary shuttle dial — responsive size
    out.print("var _rds=0,_rda=false;");
    out.print("var _vw=window.innerWidth;");
    out.print("var _rdsz=_vw>=1024?260:(_vw>=640?220:180);");
    out.print("var _rc=document.getElementById('rd'),"
              "_rx=_rc.getContext('2d');");
    out.print("_rc.width=_rdsz;_rc.height=_rdsz;");
    out.print("var _rcx=_rdsz/2,_rcy=_rdsz/2,_rcr=_rdsz/2-8;");
    out.print("function drawRd(){");
    out.print("_rx.clearRect(0,0,_rdsz,_rdsz);");
    out.print("_rx.beginPath();_rx.arc(_rcx,_rcy,_rcr,0,6.283);"
              "_rx.fillStyle='#1a1d28';_rx.fill();"
              "_rx.strokeStyle='#2a2d3a';_rx.lineWidth=2;_rx.stroke();");
    out.print("_rx.save();_rx.beginPath();_rx.moveTo(_rcx,_rcy);"
              "_rx.arc(_rcx,_rcy,_rcr,1.571,4.712);_rx.closePath();"
              "_rx.fillStyle='rgba(79,195,247,0.08)';_rx.fill();_rx.restore();");
    out.print("_rx.save();_rx.beginPath();_rx.moveTo(_rcx,_rcy);"
              "_rx.arc(_rcx,_rcy,_rcr,-1.571,1.571);_rx.closePath();"
              "_rx.fillStyle='rgba(239,83,80,0.08)';_rx.fill();_rx.restore();");
    out.print("_rx.strokeStyle='#2a2d3a';_rx.lineWidth=1;"
              "_rx.beginPath();"
              "_rx.moveTo(_rcx-_rcr,_rcy);_rx.lineTo(_rcx+_rcr,_rcy);"
              "_rx.stroke();");
    out.print("_rx.fillStyle='#6b7280';_rx.font='bold 15px sans-serif';"
              "_rx.textAlign='center';"
              "_rx.fillText('\\u21b6 L',_rcx-Math.round(_rcr*0.54),_rcy+5);"
              "_rx.fillText('R \\u21b7',_rcx+Math.round(_rcr*0.54),_rcy+5);");
    out.print("_rx.beginPath();_rx.arc(_rcx,_rcy,14,0,6.283);"
              "_rx.fillStyle='#2a2d3a';_rx.fill();"
              "_rx.strokeStyle='#353c47';_rx.lineWidth=1;_rx.stroke();");
    out.print("if(_rda){"
              "_rx.beginPath();"
              "_rx.arc(_rcx+_rds*_rcr,_rcy,14,0,6.283);"
              "_rx.fillStyle=_rds<0?'#4fc3f7':(_rds>0?'#ef5350':'#6b7280');"
              "_rx.fill();}");
    out.print("}");
    out.print("drawRd();");

    out.print("function _rdsp(dx){"
              "var d=Math.abs(dx);if(d<14)return 0;"
              "return Math.round(Math.min(d/_rcr,1)*100*(dx>=0?1:-1))/100;}");
    out.print("function rdStop(){"
              "_rda=false;_rds=0;drawRd();"
              "document.getElementById('rv').textContent='0';"
              "document.getElementById('rd2').textContent='\\u25a0 stopped';"
              "fetch('/api/rotary?s=0').catch(function(){});}");
    out.print("setInterval(function(){"
              "if(_rda||_rds!==0)"
              "fetch('/api/rotary?s='+_rds.toFixed(3)).catch(function(){});"
              "},500);");
    out.print("_rc.addEventListener('pointerdown',function(e){"
              "_rc.setPointerCapture(e.pointerId);_rda=true;e.preventDefault();});");
    out.print("_rc.addEventListener('pointermove',function(e){"
              "if(!_rda)return;"
              "var r=_rc.getBoundingClientRect(),"
              "dx=e.clientX-r.left-_rcx;"
              "_rds=_rdsp(dx);drawRd();"
              "document.getElementById('rv').textContent="
              "Math.round(Math.abs(_rds)*100);"
              "document.getElementById('rd2').textContent="
              "_rds<0?'\\u21b6 left':(_rds>0?'right \\u21b7':'\\u25a0 stopped');"
              "e.preventDefault();});");
    out.print("_rc.addEventListener('pointerup',"
              "function(e){rdStop();e.preventDefault();});");
    out.print("_rc.addEventListener('pointercancel',function(){rdStop();});");

    // Status poll
    out.print("function _poll(){fetch('/api/status')"
              ".then(function(r){return r.json();})"
              ".then(function(d){");
    out.print("_sc('st_h','c-b',d.height+'%');");
    out.print("var rd=Math.round(d.rotDeg||0);"
              "_sc('st_r','c-b',rd<5||rd>355?'home':rd+'\\u00b0');");
    out.print("_sc('st_s',d.fault?'c-r':(d.safety?'c-g':'c-y'),"
              "d.fault?'FAULT':(d.safety?'Ready':'WAIT'));");
    out.print("_sc('st_m',d.motors?'c-g':'c-r',d.motors?'ON':'off');");
    out.print("var e=document.getElementById('st_c');"
              "if(e&&d.lastCmd)e.textContent=d.lastCmd;");
    out.print("updateChips(d,'c');");
    // Update safe spin height section chips
    out.print("var _sc2=document.getElementById('ssh_cur');"
              "if(_sc2)_sc2.textContent=d.height+'%';");
    out.print("var _sm=document.getElementById('ssh_min');"
              "if(_sm)_sm.textContent=(d.minHeightPct||0)+'%';");
    // Sync override state from server
    out.print("var oon=document.getElementById('ovr-on'),"
              "ooff=document.getElementById('ovr-off'),"
              "otm=document.getElementById('ovr-time');");
    out.print("if(d.override>0){oon.style.display='';ooff.style.display='none';"
              "otm.textContent=d.override;}"
              "else{oon.style.display='none';ooff.style.display='';}");
    out.print("}).catch(function(){});}");
    out.print("setInterval(_poll,500);_poll();");

    // Override long-press (3 seconds)
    out.print("var _ot=null,_oc=0;");
    out.print("var _ob=document.getElementById('ovr-btn');");
    out.print("function _oStart(e){e.preventDefault();_oc=3;_ob.textContent='3...';"
              "_ot=setInterval(function(){"
              "_oc--;if(_oc>0){_ob.textContent=_oc+'...';}"
              "else{clearInterval(_ot);_ot=null;_ob.textContent='activating...';"
              "fetch('/api/action?do=override_on');}},1000);}");
    out.print("function _oEnd(e){e.preventDefault();"
              "if(_ot){clearInterval(_ot);_ot=null;_ob.textContent="
              "'\\u26a0 hold to override safeties';}}");

    // Safe spin height button
    out.print("function setSafeHeight(){"
              "var btn=document.getElementById('ssh_btn');"
              "var msg=document.getElementById('ssh_msg');"
              "btn.disabled=true;btn.textContent='saving...';"
              "fetch('/api/action?do=setsafeheight')"
              ".then(function(){return fetch('/api/status');})"
              ".then(function(r){return r.json();})"
              ".then(function(d){"
              "msg.style.color='#4ade80';"
              "msg.textContent='\\u2713 Saved! Min spin height set to '+(d.minHeightPct||0)+'%';"
              "btn.disabled=false;"
              "btn.textContent='\\u{1F4CC} set safe spin height to current';"
              "setTimeout(function(){msg.textContent='';},5000);"
              "}).catch(function(){"
              "msg.style.color='#ef5350';"
              "msg.textContent='Error saving. Try again.';"
              "btn.disabled=false;"
              "btn.textContent='\\u{1F4CC} set safe spin height to current';"
              "});"
              "}");
    out.print("_ob.addEventListener('pointerdown',_oStart);"
              "_ob.addEventListener('pointerup',_oEnd);"
              "_ob.addEventListener('pointercancel',_oEnd);");

    out.print("</script></body></html>");
}

///////////////////////////////////////////////////////////////////////////////
// LOG PAGE
///////////////////////////////////////////////////////////////////////////////

static void printLogPage(Print& out)
{
    printPageHead(out, "R2 Log");
    printTopNav(out, "log");
    printStatusBar(out);
    out.print("<div id='page'>");
    out.print("<div style='display:flex;gap:10px;margin-bottom:10px'>");
    out.print("<button onclick=\"document.getElementById('lb').innerHTML=''\""
              " style='flex:0 0 auto;width:auto;padding:10px 20px'>clear</button>");
    out.print("<button id='asb' "
              "onclick=\"_as=!_as;this.textContent='auto-scroll: '+(_as?'on':'off')\""
              " style='flex:0 0 auto;width:auto;padding:10px 20px'>"
              "auto-scroll: on</button>");
    out.print("</div>");
    out.print("<div id='lb'></div>");
    out.print("</div>");
    printTabBar(out, "log");
    out.print("<script>");
    out.print("function _sc(id,cls,txt){var e=document.getElementById(id);"
              "if(!e)return;e.textContent=txt;e.className='sv '+cls;}");
    out.print("function _pollsb(){fetch('/api/status')"
              ".then(function(r){return r.json();})"
              ".then(function(d){");
    out.print("_sc('st_h','c-b',d.height+'%');");
    out.print("var rd=Math.round(d.rotDeg||0);"
              "_sc('st_r','c-b',rd<5||rd>355?'home':rd+'\\u00b0');");
    out.print("_sc('st_s',d.fault?'c-r':(d.safety?'c-g':'c-y'),"
              "d.fault?'FAULT':(d.safety?'Ready':'WAIT'));");
    out.print("_sc('st_m',d.motors?'c-g':'c-r',d.motors?'ON':'off');");
    out.print("var e=document.getElementById('st_c');"
              "if(e&&d.lastCmd)e.textContent=d.lastCmd;");
    out.print("}).catch(function(){});}");
    out.print("setInterval(_pollsb,2000);_pollsb();");
    out.print("var _s=-1,_as=true;");
    out.print("function _pl(){fetch('/api/log?since='+_s)"
              ".then(function(r){return r.text();})"
              ".then(function(t){");
    out.print("if(!t||!t.trim())return;"
              "var b=document.getElementById('lb');");
    out.print("t.split('\\n').forEach(function(l){"
              "if(!l.trim())return;"
              "var i=l.indexOf('\\t');if(i<0)return;"
              "var n=parseInt(l.substring(0,i));"
              "if(!isNaN(n)&&n>_s){"
              "_s=n;"
              "var d=document.createElement('div');"
              "d.textContent=l.substring(i+1);"
              "b.appendChild(d);}});");
    out.print("if(_as)b.scrollTop=b.scrollHeight;"
              "}).catch(function(){});}");
    out.print("setInterval(_pl,1000);_pl();");
    out.print("</script></body></html>");
}

///////////////////////////////////////////////////////////////////////////////
// SETUP PAGE
///////////////////////////////////////////////////////////////////////////////

static void printSetupPage(Print& out)
{
    printPageHead(out, "R2 Setup");
    printTopNav(out, "setup");
    printStatusBar(out);
    out.print("<div id='page'>");
    out.print("<div style='font-size:11px;color:var(--muted);font-weight:700;"
              "padding:10px 0 12px;border-bottom:1px solid var(--border);"
              "margin-bottom:6px;text-transform:uppercase;letter-spacing:.07em'>"
              "configuration</div>");

    // Calibrate
    out.print("<a class='set-row' href='/calibrate'>");
    out.print("<div class='set-icon' style='background:rgba(79,195,247,.12)'>"
              "<svg width='18' height='18' viewBox='0 0 18 18' fill='none' "
              "stroke='#4fc3f7' stroke-width='1.6' stroke-linecap='round'>"
              "<circle cx='9' cy='9' r='6'/>"
              "<path d='M9 5.5v3.5l2.5 1.5'/></svg></div>");
    out.print("<div class='set-main'>"
              "<div class='set-name'>calibration</div>"
              "<div class='set-val'>clear distance, reset rotary, run calibration</div>"
              "</div><div class='set-arr'>&#8250;</div></a>");

    // Marcduino
    out.print("<a class='set-row' href='/marcduino'>");
    out.print("<div class='set-icon' style='background:rgba(74,222,128,.12)'>"
              "<svg width='18' height='18' viewBox='0 0 18 18' fill='none' "
              "stroke='#4ade80' stroke-width='1.6' stroke-linecap='round'>"
              "<rect x='2' y='5' width='14' height='9' rx='2'/>"
              "<path d='M6 5V4a3 3 0 016 0v1'/></svg></div>");
    out.print("<div class='set-main'>"
              "<div class='set-name'>marcduino</div>"
              "<div class='set-val'>ID: ");
    out.print(sSettings.fID);
    out.print(" &bull; baud: "); out.print(sSettings.fBaudRate);
    out.print("</div></div><div class='set-arr'>&#8250;</div></a>");

    // WiFi
    out.print("<a class='set-row' href='/wifi'>");
    out.print("<div class='set-icon' style='background:rgba(167,139,250,.12)'>"
              "<svg width='18' height='18' viewBox='0 0 18 18' fill='none' "
              "stroke='#a78bfa' stroke-width='1.6' stroke-linecap='round'>"
              "<path d='M2 7a9 9 0 0114 0'/>"
              "<path d='M5 10a5 5 0 018 0'/>"
              "<path d='M8 13a2 2 0 014 0'/></svg></div>");
    out.print("<div class='set-main'><div class='set-name'>wifi</div>"
              "<div class='set-val'>");
    {
        String ssid = preferences.getString(PREFERENCE_WIFI_SSID, WIFI_AP_NAME);
        bool ap = preferences.getBool(PREFERENCE_WIFI_AP, WIFI_ACCESS_POINT);
        bool en = preferences.getBool(PREFERENCE_WIFI_ENABLED, WIFI_ENABLED);
        out.print(ssid.c_str());
        out.print(en ? (ap ? " &bull; access point" : " &bull; client")
                     : " &bull; disabled");
    }
    out.print("</div></div><div class='set-arr'>&#8250;</div></a>");

    // Remote
    out.print("<a class='set-row' href='/remote'>");
    out.print("<div class='set-icon' style='background:rgba(255,167,38,.12)'>"
              "<svg width='18' height='18' viewBox='0 0 18 18' fill='none' "
              "stroke='#ffa726' stroke-width='1.6' stroke-linecap='round'>"
              "<rect x='5' y='2' width='8' height='14' rx='3'/>"
              "<line x1='9' y1='12' x2='9' y2='13' stroke-width='2'/>"
              "</svg></div>");
    out.print("<div class='set-main'><div class='set-name'>remote</div>"
              "<div class='set-val'>");
    {
        String rhost = preferences.getString(PREFERENCE_REMOTE_HOSTNAME, SMQ_HOSTNAME);
        bool ren = preferences.getBool(PREFERENCE_REMOTE_ENABLED, REMOTE_ENABLED);
        out.print(rhost.c_str());
        out.print(ren ? " &bull; enabled" : " &bull; disabled");
    }
    out.print("</div></div><div class='set-arr'>&#8250;</div></a>");

    // Parameters
    out.print("<a class='set-row' href='/parameters'>");
    out.print("<div class='set-icon' style='background:rgba(79,195,247,.08)'>"
              "<svg width='18' height='18' viewBox='0 0 18 18' fill='none' "
              "stroke='#818cf8' stroke-width='1.6' stroke-linecap='round'>"
              "<path d='M3 9h3l2-6 2.5 12L13 9h3'/></svg></div>");
    out.print("<div class='set-main'><div class='set-name'>parameters</div>"
              "<div class='set-val'>min lift: ");
    out.print(LIFTER_MINIMUM_POWER);
    out.print("% &bull; safe spin: ");
    {
        int dist = sLifterParameters.fLifterDistance;
        out.print(dist > 0 ? sLifterParameters.fRotaryMinHeight * 100 / dist : 0);
    }
    out.print("%</div></div><div class='set-arr'>&#8250;</div></a>");

    // Firmware
    out.print("<a class='set-row' href='/firmware'>");
    out.print("<div class='set-icon' style='background:rgba(74,222,128,.12)'>"
              "<svg width='18' height='18' viewBox='0 0 18 18' fill='none' "
              "stroke='#4ade80' stroke-width='1.6' stroke-linecap='round'>"
              "<path d='M2 14h14M5 14V9l4-6 4 6v5'/></svg></div>");
    out.print("<div class='set-main'><div class='set-name'>firmware</div>"
              "<div class='set-val'>"); out.print(__DATE__);
#ifdef BUILD_VERSION
    out.print(" &bull; <a href='"); out.print(BUILD_VERSION);
    out.print("' style='color:var(--accent)'>sources</a>");
#endif
    out.print("</div></div><div class='set-arr'>&#8250;</div></a>");

    // Danger zone
    out.print("<div class='danger-zone'>");
    out.print("<div class='dz-lbl'>danger zone</div>");
    out.print("<div class='g2'>");
    out.print("<button class='danger' "
              "onclick=\"if(confirm('Clear all preferences?')){"
              "fetch('/api/action?do=clearprefs')"
              ".then(function(){location.href='/setup';})}\">clear prefs</button>");
    out.print("<button class='danger' "
              "onclick=\"if(confirm('Reboot now?')){"
              "fetch('/api/action?do=reboot')}\">reboot</button>");
    out.print("</div></div>");

    out.print("</div>"); // #page
    printTabBar(out, "setup");
    printStatusPollJS(out);
    out.print("</body></html>");
}

///////////////////////////////////////////////////////////////////////////////
// CALIBRATE PAGE
///////////////////////////////////////////////////////////////////////////////

static void printCalibratePage(Print& out)
{
    printPageHead(out, "R2 Calibration");
    printTopNav(out, "setup");
    printStatusBar(out);
    out.print("<div id='page'>");
    out.print("<div class='pg-title'>Calibration</div>");

    // ── Calibration ───────────────────────────────────────────────────────
    out.print("<div class='sec'>calibration</div>");
    out.print("<div class='card' style='margin-bottom:14px'>");
    out.print("<p style='font-size:14px;color:var(--text);margin-bottom:10px;line-height:1.6'>"
              "Runs the automatic calibration routine. The lifter moves up and down at "
              "increasing speeds to measure minimum power and full travel distance. "
              "Results are saved to flash.</p>");
    out.print("<p style='font-size:13px;color:var(--red);margin-bottom:14px'>"
              "&#9888; Keep clear of the periscope during calibration.</p>");
    out.print("<div class='g2' style='margin-bottom:10px'>");
    out.print("<button class='danger' onclick='sendCmd(\"#PCLRD\")'>"
              "clear distance</button>");
    out.print("<button onclick='sendCmd(\"#PRRC\")'>"
              "reset rotary count</button>");
    out.print("</div>");
    out.print("<p class='hint' style='margin-bottom:14px'>"
              "Clear distance before recalibrating after a motor swap. "
              "Reset rotary count if the rotary home is not found.</p>");
    out.print("<button class='primary' style='font-size:16px;padding:15px;width:100%' "
              "onclick='sendCmd(\"#PSC\")'>&#9654; start calibration</button>");
    out.print("<div id='hwmsg' style='margin-top:10px'></div>");
    out.print("</div>");

    out.print("<div style='text-align:center;padding-bottom:8px'>"
              "<a href='/setup' class='back-link'>&#8592; back to setup</a></div>");

    out.print("</div>"); // #page
    printTabBar(out, "setup");

    out.print("<script>");
    // Status bar updater
    out.print("function _sc(id,cls,txt){var e=document.getElementById(id);"
              "if(!e)return;e.textContent=txt;e.className='sv '+cls;}");
    out.print("function _poll(){fetch('/api/status')"
              ".then(function(r){return r.json();})"
              ".then(function(d){"
              "_sc('st_h','c-b',d.height+'%');"
              "var rd=Math.round(d.rotDeg||0);"
              "_sc('st_r','c-b',rd<5||rd>355?'home':rd+'\\u00b0');"
              "_sc('st_s',d.fault?'c-r':(d.safety?'c-g':'c-y'),"
              "d.fault?'FAULT':(d.safety?'Ready':'WAIT'));"
              "_sc('st_m',d.motors?'c-g':'c-r',d.motors?'ON':'off');"
              "}).catch(function(){});}");
    out.print("setInterval(_poll,1000);_poll();");
    // Command sender
    out.print("function sendCmd(c){"
              "fetch('/api/cmd?c='+encodeURIComponent(c))"
              ".then(function(){setTimeout(_poll,400);})"
              ".catch(function(){});"
              "if(c==='#PSC'){"
              "var m=document.getElementById('hwmsg');"
              "m.className='msg ok';"
              "m.textContent='Calibration started \u2014 watch the log page for progress.';}"
              "else if(c==='#PCLRD'){"
              "var m=document.getElementById('hwmsg');"
              "m.className='msg ok';"
              "m.textContent='Distance cleared. Now run Start Calibration.';}"
              "else if(c==='#PRRC'){"
              "var m=document.getElementById('hwmsg');"
              "m.className='msg ok';"
              "m.textContent='Rotary count reset.';}"
              "}");
    out.print("</script></body></html>");
}

///////////////////////////////////////////////////////////////////////////////
// SETUP SUB-PAGES — custom WAPI pages with dark-theme CSS
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
// MARCDUINO PAGE
///////////////////////////////////////////////////////////////////////////////
static void printMarcduinoPage(Print& out)
{
    printPageHead(out, "R2 Marcduino");
    printTopNav(out, "setup");
    printStatusBar(out);
    out.print("<div id='page'>");
    out.print("<div class='pg-title'>Marcduino</div>");
    out.print("<div class='fg'><label class='fl'>ID #</label>"
              "<input type='number' id='mid' min='1' max='255' value='");
    out.print(sSettings.fID);
    out.print("'></div>");
    out.print("<div class='fg'><label class='fl'>Serial baud rate</label>"
              "<select id='mbaud'>"
              "<option value='2400'");
    if (sSettings.fBaudRate == 2400) out.print(" selected");
    out.print(">2400</option><option value='9600'");
    if (sSettings.fBaudRate == 9600) out.print(" selected");
    out.print(">9600</option></select></div>");
    out.print("<div class='cb-row'>"
              "<input type='checkbox' id='mwifi'");
    if (preferences.getBool(PREFERENCE_MARCWIFI_ENABLED, MARC_WIFI_ENABLED)) out.print(" checked");
    out.print("><div><div class='cb-lbl'>Marcduino on WiFi (port 2000)</div>"
              "<div class='cb-hint'>Accept Marcduino commands over WiFi TCP connection</div>"
              "</div></div>");
    out.print("<div class='fg' style='margin-top:14px'>"
              "<label class='fl'>Last command received</label>"
              "<input type='text' id='mrcv' readonly value='' "
              "style='color:var(--muted)'></div>");
    out.print("<div class='form-actions'>"
              "<button class='primary' onclick='save()'>save</button>"
              "<button onclick=\"location.href='/setup'\">back</button>"
              "</div>");
    out.print("<div id='msg'></div>");
    out.print("</div>");
    printTabBar(out, "setup");
    out.print("<script>");
    // Poll last command
    out.print("function _poll(){fetch('/api/status').then(r=>r.json()).then(d=>{"
              "var e=document.getElementById('mrcv');"
              "if(e&&d.lastCmd)e.value=d.lastCmd;}).catch(()=>{})}");
    out.print("setInterval(_poll,2000);_poll();");
    // Save
    out.print("function save(){"
              "var id=document.getElementById('mid').value;"
              "var baud=document.getElementById('mbaud').value;"
              "var wifi=document.getElementById('mwifi').checked?1:0;"
              "fetch('/api/save/marcduino?id='+id+'&baud='+baud+'&wifi='+wifi)"
              ".then(r=>r.json()).then(d=>{"
              "var m=document.getElementById('msg');"
              "m.className='msg '+(d.ok?'ok':'err');"
              "m.textContent=d.ok?'Saved.':('Error: '+d.err);"
              "}).catch(()=>{"
              "document.getElementById('msg').className='msg err';"
              "document.getElementById('msg').textContent='Network error';});}");
    // Status bar poll
    out.print("function _sc(id,cls,txt){var e=document.getElementById(id);"
              "if(!e)return;e.textContent=txt;e.className='sv '+cls;}");
    out.print("function _sb(){fetch('/api/status').then(r=>r.json()).then(d=>{"
              "_sc('st_h','c-b',d.height+'%');"
              "var rd=Math.round(d.rotDeg||0);"
              "_sc('st_r','c-b',rd<5||rd>355?'home':rd+'\\u00b0');"
              "_sc('st_s',d.fault?'c-r':(d.safety?'c-g':'c-y'),"
              "d.fault?'FAULT':(d.safety?'Ready':'WAIT'));"
              "_sc('st_m',d.motors?'c-g':'c-r',d.motors?'ON':'off');"
              "}).catch(()=>{})}");
    out.print("setInterval(_sb,1000);_sb();");
    out.print("</script></body></html>");
}

///////////////////////////////////////////////////////////////////////////////
// PARAMETERS PAGE
///////////////////////////////////////////////////////////////////////////////
static void printParametersPage(Print& out)
{
    printPageHead(out, "R2 Parameters");
    printTopNav(out, "setup");
    printStatusBar(out);
    out.print("<div id='page'>");
    out.print("<div class='pg-title'>Parameters</div>");

    out.print("<div class='fg'><label class='fl'>Min lifter power (%)</label>"
              "<input type='number' id='lmp' min='0' max='100' value='");
    out.print(LIFTER_MINIMUM_POWER); out.print("'></div>");

    out.print("<div class='fg'><label class='fl'>Min seek-bottom power (%)</label>"
              "<input type='number' id='sbp' min='0' max='100' value='");
    out.print(LIFTER_SEEKBOTTTOM_POWER); out.print("'></div>");

    out.print("<div class='fg'><label class='fl'>Min rotary power (%)</label>"
              "<input type='number' id='rmp' min='0' max='100' value='");
    out.print(ROTARY_MINIMUM_POWER); out.print("'></div>");

    out.print("<div class='fg'><label class='fl'>Lifter distance (encoder ticks)</label>"
              "<input type='number' id='ld' min='0' value='");
    out.print(LIFTER_DISTANCE); out.print("'></div>");

    {
        int dist = max(1, LIFTER_DISTANCE);
        int pct  = ROTARY_MINIMUM_HEIGHT * 100 / dist;
        out.print("<div class='fg'><label class='fl'>Safe spin height (% of travel)</label>"
                  "<input type='range' id='ssh' min='0' max='100' value='");
        out.print(pct);
        out.print("' oninput=\"document.getElementById('sshv').textContent=this.value\">"
                  "<div class='spd-val'><span id='sshv'>");
        out.print(pct);
        out.print("</span>%</div></div>");
    }

    out.print("<div class='fg'><label class='fl'>Drift correction threshold (0=off)</label>"
              "<input type='range' id='dc' min='0' max='20' value='");
    out.print(sLifterParameters.fDriftCorrectionPct);
    out.print("' oninput=\"document.getElementById('dcv').textContent=this.value\">"
              "<div class='spd-val'><span id='dcv'>");
    out.print(sLifterParameters.fDriftCorrectionPct);
    out.print("</span>%</div></div>");

    out.print("<div class='cb-row'>"
              "<input type='checkbox' id='rotd'");
    if (sSettings.fDisableRotary) out.print(" checked");
    out.print("><div><div class='cb-lbl'>Disable rotary unit</div>"
              "<div class='cb-hint'>Ignore rotary motor entirely</div></div></div>");

    // ── Motor direction ───────────────────────────────────────────────────
    out.print("<div class='sec' style='margin-top:16px'>motor direction</div>");
    out.print("<div class='card' style='margin-bottom:4px'>");
    // Invert lifter motor
    out.print("<div style='display:flex;align-items:center;justify-content:space-between;"
              "padding:10px 0;border-bottom:1px solid var(--border)'>");
    out.print("<div><div style='font-size:14px;font-weight:600;color:var(--text)'>"
              "Invert lifter motor</div>"
              "<div class='hint'>Toggle if lifter moves the wrong direction</div></div>");
    out.print("<span id='pilm_b' style='font-size:12px;font-weight:700;"
              "padding:3px 10px;border-radius:12px;margin-right:8px'></span>");
    out.print("<button style='width:auto;padding:8px 16px;font-size:13px' "
              "onclick='sendCmd(\"#PILM\")'>toggle</button>");
    out.print("</div>");
    // Invert lifter encoder
    out.print("<div style='display:flex;align-items:center;justify-content:space-between;"
              "padding:10px 0;border-bottom:1px solid var(--border)'>");
    out.print("<div><div style='font-size:14px;font-weight:600;color:var(--text)'>"
              "Invert lifter encoder</div>"
              "<div class='hint'>Toggle if encoder counts backward (independent of motor)</div></div>");
    out.print("<span id='pile_b' style='font-size:12px;font-weight:700;"
              "padding:3px 10px;border-radius:12px;margin-right:8px'></span>");
    out.print("<button style='width:auto;padding:8px 16px;font-size:13px' "
              "onclick='sendCmd(\"#PILE\")'>toggle</button>");
    out.print("</div>");
    // Lifter limit switch polarity
    out.print("<div style='display:flex;align-items:center;gap:8px;"
              "padding:10px 0;border-bottom:1px solid var(--border)'>");
    out.print("<div style='flex:1'><div style='font-size:14px;font-weight:600;color:var(--text)'>"
              "Lifter limit switch</div>"
              "<div class='hint'>NC = closed at rest, NO = open at rest</div></div>");
    out.print("<button id='btn_lnc' style='width:auto;padding:8px 14px;font-size:13px' "
              "onclick='sendCmd(\"#PNCL\")'>NC</button>");
    out.print("<button id='btn_lno' style='width:auto;padding:8px 14px;font-size:13px' "
              "onclick='sendCmd(\"#PNOL\")'>NO</button>");
    out.print("</div>");
    // Rotary home switch polarity
    out.print("<div style='display:flex;align-items:center;gap:8px;padding:10px 0'>");
    out.print("<div style='flex:1'><div style='font-size:14px;font-weight:600;color:var(--text)'>"
              "Rotary home switch</div>"
              "<div class='hint'>NC = closed at rest, NO = open at rest</div></div>");
    out.print("<button id='btn_rnc' style='width:auto;padding:8px 14px;font-size:13px' "
              "onclick='sendCmd(\"#PNCR\")'>NC</button>");
    out.print("<button id='btn_rno' style='width:auto;padding:8px 14px;font-size:13px' "
              "onclick='sendCmd(\"#PNOR\")'>NO</button>");
    out.print("</div>");
    out.print("</div>"); // card

    // Motor preset selector
    out.print("<div class='sec' style='margin-top:16px'>motor preset</div>");
    out.print("<div class='card' style='margin-bottom:4px'>");
    out.print("<div style='font-size:13px;color:var(--muted);margin-bottom:10px'>"
              "Select your motor to load recommended starting values. "
              "Run calibration afterwards to refine them.</div>");
    out.print("<div class='g2' style='margin-bottom:8px'>");
    out.print("<button class='primary' onclick='loadDef(\"greg\")'>6.25:1 (Pololu 4757)</button>");
    out.print("<button class='primary' onclick='loadDef(\"10\")'>10:1 (Pololu 4758)</button>");
    out.print("</div>");
    out.print("<div class='g2'>");
    out.print("<button class='primary' onclick='loadDef(\"1875\")'>18.75:1 (Pololu 4751)</button>");
    out.print("<button class='primary' onclick='loadDef(\"iaparts\")'>IA-Parts</button>");
    out.print("</div>");
    out.print("</div>");

    out.print("<div class='form-actions'>"
              "<button class='success' onclick='save()'>save</button>"
              "<button onclick=\"location.href='/setup'\">back</button>"
              "</div>");
    out.print("<div id='msg'></div>");
    out.print("</div>");
    printTabBar(out, "setup");
    out.print("<script>");
    out.print("function loadDef(p){"
              "fetch('/api/param/loaddefaults?preset='+p)"
              ".then(r=>r.json()).then(d=>{"
              "if(!d.ok)return;"
              "document.getElementById('lmp').value=d.lmp;"
              "document.getElementById('sbp').value=d.sbp;"
              "document.getElementById('rmp').value=d.rmp;"
              "document.getElementById('ld').value=d.ld;"
              "var pct=d.ld>0?Math.round(d.ssh*100/d.ld):0;"
              "document.getElementById('ssh').value=pct;"
              "document.getElementById('sshv').textContent=pct;"
              "document.getElementById('dc').value=d.dc;"
              "document.getElementById('dcv').textContent=d.dc;"
              "}).catch(()=>{})}");
    out.print("function save(){"
              "var p='lmp='+document.getElementById('lmp').value"
              "+'&sbp='+document.getElementById('sbp').value"
              "+'&rmp='+document.getElementById('rmp').value"
              "+'&ld='+document.getElementById('ld').value"
              "+'&ssh='+document.getElementById('ssh').value"
              "+'&dc='+document.getElementById('dc').value"
              "+'&rotd='+(document.getElementById('rotd').checked?1:0);"
              "fetch('/api/save/parameters?'+p)"
              ".then(r=>r.json()).then(d=>{"
              "var m=document.getElementById('msg');"
              "m.className='msg '+(d.ok?'ok':'err');"
              "m.textContent=d.ok?'Saved.':('Error: '+d.err);"
              "}).catch(()=>{"
              "document.getElementById('msg').className='msg err';"
              "document.getElementById('msg').textContent='Network error';});}");
    out.print("function _sc(id,cls,txt){var e=document.getElementById(id);"
              "if(!e)return;e.textContent=txt;e.className='sv '+cls;}");
    out.print("function _badge(id,on){"
              "var e=document.getElementById(id);if(!e)return;"
              "e.textContent=on?'ON':'OFF';"
              "e.style.background=on?'rgba(74,222,128,.15)':'rgba(107,114,128,.15)';"
              "e.style.color=on?'var(--green)':'var(--muted)';}");
    out.print("function _limBtn(ncId,noId,isNC){"
              "var nc=document.getElementById(ncId);"
              "var no=document.getElementById(noId);"
              "if(!nc||!no)return;"
              "nc.style.borderColor=isNC?'rgba(79,195,247,.6)':'rgba(42,45,58,1)';"
              "nc.style.color=isNC?'var(--accent)':'var(--muted)';"
              "no.style.borderColor=isNC?'rgba(42,45,58,1)':'rgba(79,195,247,.6)';"
              "no.style.color=isNC?'var(--muted)':'var(--accent)';}");
    out.print("function sendCmd(c){"
              "fetch('/api/cmd?c='+encodeURIComponent(c))"
              ".then(function(){setTimeout(_sb,400);})"
              ".catch(function(){});}");
    out.print("function _sb(){fetch('/api/status').then(r=>r.json()).then(d=>{"
              "_sc('st_h','c-b',d.height+'%');"
              "var rd=Math.round(d.rotDeg||0);"
              "_sc('st_r','c-b',rd<5||rd>355?'home':rd+'\\u00b0');"
              "_sc('st_s',d.fault?'c-r':(d.safety?'c-g':'c-y'),"
              "d.fault?'FAULT':(d.safety?'Ready':'WAIT'));"
              "_sc('st_m',d.motors?'c-g':'c-r',d.motors?'ON':'off');"
              "_badge('pilm_b',d.invertMotor);"
              "_badge('pile_b',d.invertEncoder);"
              "_limBtn('btn_lnc','btn_lno',d.lifterLimitNC);"
              "_limBtn('btn_rnc','btn_rno',d.rotaryLimitNC);"
              "}).catch(()=>{})}");
    out.print("setInterval(_sb,1000);_sb();");
    out.print("</script></body></html>");
}

///////////////////////////////////////////////////////////////////////////////
// WIFI PAGE
///////////////////////////////////////////////////////////////////////////////
static void printWifiPage(Print& out)
{
    printPageHead(out, "R2 WiFi");
    printTopNav(out, "setup");
    printStatusBar(out);
    out.print("<div id='page'>");
    out.print("<div class='pg-title'>WiFi</div>");
    bool en  = preferences.getBool(PREFERENCE_WIFI_ENABLED, WIFI_ENABLED);
    bool ap  = preferences.getBool(PREFERENCE_WIFI_AP, WIFI_ACCESS_POINT);
    String ss = preferences.getString(PREFERENCE_WIFI_SSID, WIFI_AP_NAME);
    String pw = preferences.getString(PREFERENCE_WIFI_PASS, WIFI_AP_PASSPHRASE);
    out.print("<div class='cb-row'>"
              "<input type='checkbox' id='wen'");
    if (en) out.print(" checked");
    out.print("><div><div class='cb-lbl'>WiFi enabled</div></div></div>");
    out.print("<div class='cb-row'>"
              "<input type='checkbox' id='wap'");
    if (ap) out.print(" checked");
    out.print("><div><div class='cb-lbl'>Access point mode</div>"
              "<div class='cb-hint'>Unchecked = join an existing network</div>"
              "</div></div>");
    out.print("<div class='fg' style='margin-top:14px'>"
              "<label class='fl'>Network name (SSID)</label>"
              "<input type='text' id='wssid' value='");
    out.print(ss.c_str());
    out.print("'></div>");
    out.print("<div class='fg'><label class='fl'>Password</label>"
              "<input type='password' id='wpwd' value='");
    out.print(pw.c_str());
    out.print("'></div>");
    out.print("<p class='hint' style='margin-bottom:14px'>"
              "Changes require a reboot to take effect.</p>");
    out.print("<div class='form-actions'>"
              "<button class='success' onclick='save()'>save &amp; reboot</button>"
              "<button onclick=\"location.href='/setup'\">back</button>"
              "</div>");
    out.print("<div id='msg'></div>");
    out.print("</div>");
    printTabBar(out, "setup");
    out.print("<script>");
    out.print("function save(){"
              "var p='en='+(document.getElementById('wen').checked?1:0)"
              "+'&ap='+(document.getElementById('wap').checked?1:0)"
              "+'&ssid='+encodeURIComponent(document.getElementById('wssid').value)"
              "+'&pwd='+encodeURIComponent(document.getElementById('wpwd').value);"
              "fetch('/api/save/wifi?'+p)"
              ".then(r=>r.json()).then(d=>{"
              "var m=document.getElementById('msg');"
              "m.className='msg '+(d.ok?'ok':'err');"
              "m.textContent=d.ok?'Saved \u2014 rebooting\u2026':('Error: '+d.err);"
              "}).catch(()=>{"
              "document.getElementById('msg').className='msg err';"
              "document.getElementById('msg').textContent='Network error';});}");
    out.print("function _sc(id,cls,txt){var e=document.getElementById(id);"
              "if(!e)return;e.textContent=txt;e.className='sv '+cls;}");
    out.print("function _sb(){fetch('/api/status').then(r=>r.json()).then(d=>{"
              "_sc('st_h','c-b',d.height+'%');"
              "var rd=Math.round(d.rotDeg||0);"
              "_sc('st_r','c-b',rd<5||rd>355?'home':rd+'\\u00b0');"
              "_sc('st_s',d.fault?'c-r':(d.safety?'c-g':'c-y'),"
              "d.fault?'FAULT':(d.safety?'Ready':'WAIT'));"
              "_sc('st_m',d.motors?'c-g':'c-r',d.motors?'ON':'off');"
              "}).catch(()=>{})}");
    out.print("setInterval(_sb,1000);_sb();");
    out.print("</script></body></html>");
}

///////////////////////////////////////////////////////////////////////////////
// REMOTE PAGE
///////////////////////////////////////////////////////////////////////////////
static void printRemotePage(Print& out)
{
    printPageHead(out, "R2 Remote");
    printTopNav(out, "setup");
    printStatusBar(out);
    out.print("<div id='page'>");
    out.print("<div class='pg-title'>Droid Remote</div>");
    String rhost = preferences.getString(PREFERENCE_REMOTE_HOSTNAME, SMQ_HOSTNAME);
    String rsec  = preferences.getString(PREFERENCE_REMOTE_SECRET, SMQ_SECRET);
    out.print("<div class='cb-row'>"
              "<input type='checkbox' id='ren'");
    if (remoteEnabled) out.print(" checked");
    out.print("><div><div class='cb-lbl'>Droid remote enabled</div></div></div>");
    out.print("<div class='fg' style='margin-top:14px'>"
              "<label class='fl'>Device name</label>"
              "<input type='text' id='rhost' value='");
    out.print(rhost.c_str());
    out.print("'></div>");
    out.print("<div class='fg'><label class='fl'>Secret</label>"
              "<input type='password' id='rsec' value='");
    out.print(rsec.c_str());
    out.print("'></div>");
    out.print("<p class='hint' style='margin-bottom:14px'>"
              "Changes require a reboot to take effect.</p>");
    out.print("<div class='form-actions'>"
              "<button class='success' onclick='save()'>save &amp; reboot</button>"
              "<button onclick=\"location.href='/setup'\">back</button>"
              "</div>");
    out.print("<div id='msg'></div>");
    out.print("</div>");
    printTabBar(out, "setup");
    out.print("<script>");
    out.print("function save(){"
              "var p='en='+(document.getElementById('ren').checked?1:0)"
              "+'&host='+encodeURIComponent(document.getElementById('rhost').value)"
              "+'&sec='+encodeURIComponent(document.getElementById('rsec').value);"
              "fetch('/api/save/remote?'+p)"
              ".then(r=>r.json()).then(d=>{"
              "var m=document.getElementById('msg');"
              "m.className='msg '+(d.ok?'ok':'err');"
              "m.textContent=d.ok?'Saved \u2014 rebooting\u2026':('Error: '+d.err);"
              "}).catch(()=>{"
              "document.getElementById('msg').className='msg err';"
              "document.getElementById('msg').textContent='Network error';});}");
    out.print("function _sc(id,cls,txt){var e=document.getElementById(id);"
              "if(!e)return;e.textContent=txt;e.className='sv '+cls;}");
    out.print("function _sb(){fetch('/api/status').then(r=>r.json()).then(d=>{"
              "_sc('st_h','c-b',d.height+'%');"
              "var rd=Math.round(d.rotDeg||0);"
              "_sc('st_r','c-b',rd<5||rd>355?'home':rd+'\\u00b0');"
              "_sc('st_s',d.fault?'c-r':(d.safety?'c-g':'c-y'),"
              "d.fault?'FAULT':(d.safety?'Ready':'WAIT'));"
              "_sc('st_m',d.motors?'c-g':'c-r',d.motors?'ON':'off');"
              "}).catch(()=>{})}");
    out.print("setInterval(_sb,1000);_sb();");
    out.print("</script></body></html>");
}

///////////////////////////////////////////////////////////////////////////////
// FIRMWARE PAGE — custom dark-theme wrapper around the OTA upload form
///////////////////////////////////////////////////////////////////////////////

static void printFirmwarePage(Print& out)
{
    printPageHead(out, "R2 Firmware Update");
    printTopNav(out, "setup");
    printStatusBar(out);
    out.print("<div id='page'>");
    out.print("<div class='pg-title'>Firmware Update</div>");

    out.print("<div class='sec'>current build</div>");
    out.print("<div class='card' style='margin-bottom:14px'>");
    out.print("<div style='display:flex;justify-content:space-between;padding:8px 0;"
              "border-bottom:1px solid var(--border)'>");
    out.print("<div style='font-size:14px;color:var(--muted)'>Version</div>");
    out.print("<div style='font-size:14px;font-weight:700;color:var(--accent)'>"
              FIRMWARE_VERSION "</div></div>");
    out.print("<div style='display:flex;justify-content:space-between;padding:8px 0;"
              "border-bottom:1px solid var(--border)'>");
    out.print("<div style='font-size:14px;color:var(--muted)'>Build date</div>");
    out.print("<div style='font-size:14px;color:var(--text)'>"); out.print(__DATE__); out.print("</div></div>");
#ifdef BUILD_VERSION
    out.print("<div style='display:flex;justify-content:space-between;padding:8px 0'>");
    out.print("<div style='font-size:14px;color:var(--muted)'>Sources</div>");
    out.print("<a href='" BUILD_VERSION "' style='font-size:14px;color:var(--accent)'>GitHub</a></div>");
#endif
    out.print("</div>");

    out.print("<div class='sec'>upload new firmware</div>");
    out.print("<div class='card' style='margin-bottom:14px'>");
    out.print("<p style='font-size:14px;color:var(--text);margin-bottom:12px;line-height:1.6'>"
              "Select a <code style='color:var(--accent)'>.bin</code> file compiled for this board, "
              "then press <b>Flash</b>. The board will reboot automatically when done.</p>");
    out.print("<p style='font-size:13px;color:var(--red);margin-bottom:14px'>"
              "&#9888; Do not power off during flashing.</p>");
    // The actual OTA upload form — must use action /update, enctype multipart, name=firmware
    out.print("<form method='POST' action='/update' enctype='multipart/form-data' id='otaform'>");
    out.print("<div style='margin-bottom:12px'>");
    out.print("<input type='file' name='firmware' accept='.bin' id='fw_file' "
              "style='display:none' onchange='_fwSel(this)'>");
    out.print("<div style='display:flex;gap:8px;align-items:center'>");
    out.print("<button type='button' class='primary' style='width:auto;padding:10px 20px' "
              "onclick='document.getElementById(\"fw_file\").click()'>Choose file</button>");
    out.print("<span id='fw_name' style='font-size:13px;color:var(--muted)'>No file chosen</span>");
    out.print("</div></div>");
    out.print("<div id='fw_prog' style='display:none;margin-bottom:12px'>");
    out.print("<div style='background:var(--border);border-radius:4px;height:8px;overflow:hidden'>");
    out.print("<div id='fw_bar' style='background:var(--accent);height:8px;width:0;transition:width .2s'></div>");
    out.print("</div>");
    out.print("<div id='fw_pct' style='font-size:12px;color:var(--muted);margin-top:4px;text-align:center'>0%</div>");
    out.print("</div>");
    out.print("<button type='button' id='fw_btn' class='danger' "
              "style='font-size:16px;padding:15px;width:100%;display:none' "
              "onclick='_fwFlash()'>&#9889; Flash firmware</button>");
    out.print("</form>");
    out.print("<div id='fw_msg' style='margin-top:10px'></div>");
    out.print("</div>");

    out.print("<div style='text-align:center;padding-bottom:8px'>"
              "<a href='/setup' class='back-link'>&#8592; back to setup</a></div>");
    out.print("</div>"); // #page
    printTabBar(out, "setup");

    out.print("<script>");
    out.print("function _sc(id,cls,txt){var e=document.getElementById(id);"
              "if(!e)return;e.textContent=txt;e.className='sv '+cls;}");
    out.print("function _poll(){fetch('/api/status')"
              ".then(function(r){return r.json();})"
              ".then(function(d){"
              "_sc('st_h','c-b',d.height+'%');"
              "var rd=Math.round(d.rotDeg||0);"
              "_sc('st_r','c-b',rd<5||rd>355?'home':rd+'\\u00b0');"
              "_sc('st_s',d.fault?'c-r':(d.safety?'c-g':'c-y'),"
              "d.fault?'FAULT':(d.safety?'Ready':'WAIT'));"
              "_sc('st_m',d.motors?'c-g':'c-r',d.motors?'ON':'off');"
              "}).catch(function(){});}");
    out.print("setInterval(_poll,1000);_poll();");
    out.print("function _fwSel(inp){"
              "var n=inp.files[0]?inp.files[0].name:'No file chosen';"
              "document.getElementById('fw_name').textContent=n;"
              "document.getElementById('fw_btn').style.display=inp.files[0]?'block':'none';}");
    out.print("function _fwFlash(){"
              "var f=document.getElementById('fw_file').files[0];"
              "if(!f)return;"
              "var fd=new FormData();"
              "fd.append('firmware',f);"
              "var xhr=new XMLHttpRequest();"
              "xhr.open('POST','/update',true);"
              "document.getElementById('fw_prog').style.display='block';"
              "document.getElementById('fw_btn').disabled=true;"
              "xhr.upload.onprogress=function(e){"
              "if(e.lengthComputable){"
              "var pct=Math.round(e.loaded*100/e.total);"
              "document.getElementById('fw_bar').style.width=pct+'%';"
              "document.getElementById('fw_pct').textContent=pct+'%';}};"
              "xhr.onload=function(){"
              "var m=document.getElementById('fw_msg');"
              "if(xhr.status===200){"
              "m.className='msg ok';"
              "m.textContent='Flash complete — rebooting\u2026';}"
              "else{"
              "m.className='msg err';"
              "m.textContent='Flash failed ('+xhr.status+')';}"
              "document.getElementById('fw_btn').disabled=false;};"
              "xhr.onerror=function(){"
              "document.getElementById('fw_msg').className='msg err';"
              "document.getElementById('fw_msg').textContent='Network error';};"
              "xhr.send(fd);}");
    out.print("</script></body></html>");
}

// Keep WElement array for the /update POST handler (OTA engine still registered by ReelTwo)
WElement firmwareContents[] = {
    WFirmwareFile("Firmware file:", "firmware"),
    WFirmwareUpload("Flash firmware", "firmware"),
    WVerticalAlign(),
};

///////////////////////////////////////////////////////////////////////////////
// PAGE TABLE
///////////////////////////////////////////////////////////////////////////////

WPage pages[] = {
    WAPI("/",
        [](Print& out, String qs) {
            out.println("HTTP/1.0 302 Found");
            out.println("Location: /periscope");
            out.println("Content-Length: 0");
            out.println();
        }),
    WAPI("/periscope",
        [](Print& out, String qs) {
            out.println("HTTP/1.0 200 OK");
            out.println("Content-type:text/html");
            out.println("Cache-Control:no-cache");
            out.println();
            printPeriscopePage(out);
        }),
    WAPI("/rescue",
        [](Print& out, String qs) {
            out.println("HTTP/1.0 200 OK");
            out.println("Content-type:text/html");
            out.println("Cache-Control:no-cache");
            out.println();
            printRescuePage(out);
        }),
    WAPI("/log",
        [](Print& out, String qs) {
            out.println("HTTP/1.0 200 OK");
            out.println("Content-type:text/html");
            out.println("Cache-Control:no-cache");
            out.println();
            printLogPage(out);
        }),
    WAPI("/setup",
        [](Print& out, String qs) {
            out.println("HTTP/1.0 200 OK");
            out.println("Content-type:text/html");
            out.println("Cache-Control:no-cache");
            out.println();
            printSetupPage(out);
        }),
    WAPI("/calibrate",
        [](Print& out, String qs) {
            out.println("HTTP/1.0 200 OK");
            out.println("Content-type:text/html");
            out.println("Cache-Control:no-cache");
            out.println();
            printCalibratePage(out);
        }),
    WAPI("/marcduino",
        [](Print& out, String qs) {
            out.println("HTTP/1.0 200 OK");
            out.println("Content-type:text/html");
            out.println("Cache-Control:no-cache");
            out.println();
            printMarcduinoPage(out);
        }),
    WAPI("/parameters",
        [](Print& out, String qs) {
            out.println("HTTP/1.0 200 OK");
            out.println("Content-type:text/html");
            out.println("Cache-Control:no-cache");
            out.println();
            printParametersPage(out);
        }),
    WAPI("/wifi",
        [](Print& out, String qs) {
            out.println("HTTP/1.0 200 OK");
            out.println("Content-type:text/html");
            out.println("Cache-Control:no-cache");
            out.println();
            printWifiPage(out);
        }),
    WAPI("/remote",
        [](Print& out, String qs) {
            out.println("HTTP/1.0 200 OK");
            out.println("Content-type:text/html");
            out.println("Cache-Control:no-cache");
            out.println();
            printRemotePage(out);
        }),
    WAPI("/firmware",
        [](Print& out, String qs) {
            out.println("HTTP/1.0 200 OK");
            out.println("Content-type:text/html");
            out.println("Cache-Control:no-cache");
            out.println();
            printFirmwarePage(out);
        }),
    // Hidden path — registers the /update POST OTA handler from ReelTwo's WFirmwareUpload.
    // Our custom /firmware page POSTs directly to /update; this page is never shown to users.
    WPage("/__fw",   firmwareContents,   SizeOfArray(firmwareContents)),

    // /api/save/marcduino
    WAPI("/api/save/marcduino",
        [](Print& out, String qs) {
            auto gp = [&](const char* key, String def) -> String {
                int i = qs.indexOf(String(key)+"=");
                if (i < 0) return def;
                int s = i + strlen(key) + 1;
                int e = qs.indexOf('&', s);
                return e < 0 ? qs.substring(s) : qs.substring(s, e);
            };
            int id   = gp("id",   String(sSettings.fID)).toInt();
            int baud = gp("baud", String(sSettings.fBaudRate)).toInt();
            bool mwifi = gp("wifi", "0") == "1";
            preferences.putBool(PREFERENCE_MARCWIFI_ENABLED, mwifi);
            if (id != (int)sSettings.fID)
                { sSettings.fID = id; sUpdateSettings = true; }
            if (baud != (int)sSettings.fBaudRate)
                { sSettings.fBaudRate = baud; sUpdateSettings = true; }
            out.println("HTTP/1.0 200 OK");
            out.println("Content-type:application/json");
            out.println("Cache-Control:no-cache");
            out.println();
            out.print("{\"ok\":true}");
        }),

    // /api/save/parameters
    WAPI("/api/save/parameters",
        [](Print& out, String qs) {
            auto gp = [&](const char* key, int def) -> int {
                int i = qs.indexOf(String(key)+"=");
                if (i < 0) return def;
                int s = i + strlen(key) + 1;
                int e = qs.indexOf('&', s);
                String v = e < 0 ? qs.substring(s) : qs.substring(s, e);
                return v.toInt();
            };
            int lmp  = gp("lmp",  LIFTER_MINIMUM_POWER);
            int sbp  = gp("sbp",  LIFTER_SEEKBOTTTOM_POWER);
            int rmp  = gp("rmp",  ROTARY_MINIMUM_POWER);
            int ld   = gp("ld",   LIFTER_DISTANCE);
            int sshPct = gp("ssh", 0);
            int dc   = gp("dc",   sLifterParameters.fDriftCorrectionPct);
            bool rotd = gp("rotd", 0) == 1;
            LIFTER_MINIMUM_POWER     = constrain(lmp, 0, 100);
            LIFTER_SEEKBOTTTOM_POWER = constrain(sbp, 0, 100);
            ROTARY_MINIMUM_POWER     = constrain(rmp, 0, 100);
            LIFTER_DISTANCE          = max(ld, 0);
            ROTARY_MINIMUM_HEIGHT    = constrain(sshPct * max(ld, 1) / 100, 0, max(ld, 0));
            sLifterParameters.fDriftCorrectionPct = constrain(dc, 0, 20);
            if (rotd != sSettings.fDisableRotary)
                { sSettings.fDisableRotary = rotd; sUpdateSettings = true; }
            sLifterParameters.save();
            out.println("HTTP/1.0 200 OK");
            out.println("Content-type:application/json");
            out.println("Cache-Control:no-cache");
            out.println();
            out.print("{\"ok\":true}");
        }),

    // /api/param/loaddefaults — returns preset values as JSON
    WAPI("/api/param/loaddefaults",
        [](Print& out, String qs) {
            int i = qs.indexOf("preset=");
            String preset = (i >= 0) ? qs.substring(i + 7) : "";
            int lmp, sbp, rmp, ld, ssh, dc;
            if (preset == "greg" || preset == "6.25") {
                // Pololu 4757 — 6.25:1 gear ratio (fastest, fewest ticks)
                lmp = GREG_LIFTER_MINIMUM_POWER;
                sbp = GREG_LIFTER_SEEKBOTTTOM_POWER;
                rmp = GREG_ROTARY_MINIMUM_POWER;
                ld  = GREG_LIFTER_DISTANCE;
                ssh = ld / 2;
                dc  = DEFAULT_DRIFT_CORRECTION_PCT;
            } else if (preset == "10:1" || preset == "10") {
                // Pololu 4758 — 10:1 gear ratio
                lmp = GREG_LIFTER_10_MINIMUM_POWER;
                sbp = GREG_LIFTER_10_SEEKBOTTTOM_POWER;
                rmp = GREG_LIFTER_10_ROTARY_MIN_POWER;
                ld  = GREG_LIFTER_10_DISTANCE;
                ssh = ld / 2;
                dc  = DEFAULT_DRIFT_CORRECTION_PCT;
            } else if (preset == "18.75" || preset == "1875") {
                // Pololu 4751 — 18.75:1 gear ratio (most torque, most ticks)
                lmp = GREG_LIFTER_1875_MINIMUM_POWER;
                sbp = GREG_LIFTER_1875_SEEKBOTTTOM_POWER;
                rmp = GREG_LIFTER_1875_ROTARY_MIN_POWER;
                ld  = GREG_LIFTER_1875_DISTANCE;
                ssh = ld / 2;
                dc  = DEFAULT_DRIFT_CORRECTION_PCT;
            } else if (preset == "iaparts") {
                lmp = IAPARTS_LIFTER_MINIMUM_POWER;
                sbp = IAPARTS_LIFTER_SEEKBOTTTOM_POWER;
                rmp = IAPARTS_ROTARY_MINIMUM_POWER;
                ld  = IAPARTS_LIFTER_DISTANCE;
                ssh = ld / 2;
                dc  = DEFAULT_DRIFT_CORRECTION_PCT;
            } else {
                out.println("HTTP/1.0 400 Bad Request");
                out.println("Content-type:application/json");
                out.println();
                out.print("{\"ok\":false,\"err\":\"unknown preset\"}");
                return;
            }
            out.println("HTTP/1.0 200 OK");
            out.println("Content-type:application/json");
            out.println("Cache-Control:no-cache");
            out.println();
            out.print("{\"ok\":true,\"lmp\":"); out.print(lmp);
            out.print(",\"sbp\":"); out.print(sbp);
            out.print(",\"rmp\":"); out.print(rmp);
            out.print(",\"ld\":"); out.print(ld);
            out.print(",\"ssh\":"); out.print(ssh);
            out.print(",\"dc\":"); out.print(dc);
            out.print("}");
        }),

    // /api/save/wifi
    WAPI("/api/save/wifi",
        [](Print& out, String qs) {
            auto gp = [&](const char* key, String def) -> String {
                int i = qs.indexOf(String(key)+"=");
                if (i < 0) return def;
                int s = i + strlen(key) + 1;
                int e = qs.indexOf('&', s);
                String enc = e < 0 ? qs.substring(s) : qs.substring(s, e);
                String dec = "";
                for (int j = 0; j < (int)enc.length(); j++) {
                    char ch = enc[j];
                    if (ch == '+') { dec += ' '; }
                    else if (ch == '%' && j+2 < (int)enc.length()) {
                        char hi = enc[++j], lo = enc[++j];
                        auto hx=[](char c)->int{return(c>='0'&&c<='9')?c-'0':(c>='A'&&c<='F')?c-'A'+10:(c>='a'&&c<='f')?c-'a'+10:0;};
                        dec += (char)((hx(hi)<<4)|hx(lo));
                    } else { dec += ch; }
                }
                return dec;
            };
            bool en  = gp("en",  "0") == "1";
            bool ap  = gp("ap",  "0") == "1";
            String ss = gp("ssid", WIFI_AP_NAME);
            String pw = gp("pwd",  WIFI_AP_PASSPHRASE);
            preferences.putBool(PREFERENCE_WIFI_ENABLED, en);
            preferences.putBool(PREFERENCE_WIFI_AP, ap);
            preferences.putString(PREFERENCE_WIFI_SSID, ss);
            preferences.putString(PREFERENCE_WIFI_PASS, pw);
            out.println("HTTP/1.0 200 OK");
            out.println("Content-type:application/json");
            out.println("Cache-Control:no-cache");
            out.println();
            out.print("{\"ok\":true}");
            reboot();
        }),

    // /api/save/remote
    WAPI("/api/save/remote",
        [](Print& out, String qs) {
            auto gp = [&](const char* key, String def) -> String {
                int i = qs.indexOf(String(key)+"=");
                if (i < 0) return def;
                int s = i + strlen(key) + 1;
                int e = qs.indexOf('&', s);
                String enc = e < 0 ? qs.substring(s) : qs.substring(s, e);
                String dec = "";
                for (int j = 0; j < (int)enc.length(); j++) {
                    char ch = enc[j];
                    if (ch == '+') { dec += ' '; }
                    else if (ch == '%' && j+2 < (int)enc.length()) {
                        char hi = enc[++j], lo = enc[++j];
                        auto hx=[](char c)->int{return(c>='0'&&c<='9')?c-'0':(c>='A'&&c<='F')?c-'A'+10:(c>='a'&&c<='f')?c-'a'+10:0;};
                        dec += (char)((hx(hi)<<4)|hx(lo));
                    } else { dec += ch; }
                }
                return dec;
            };
            bool en  = gp("en",   "0") == "1";
            String host = gp("host", SMQ_HOSTNAME);
            String sec  = gp("sec",  SMQ_SECRET);
            preferences.putBool(PREFERENCE_REMOTE_ENABLED, en);
            preferences.putString(PREFERENCE_REMOTE_HOSTNAME, host);
            preferences.putString(PREFERENCE_REMOTE_SECRET, sec);
            remoteEnabled = en;
            out.println("HTTP/1.0 200 OK");
            out.println("Content-type:application/json");
            out.println("Cache-Control:no-cache");
            out.println();
            out.print("{\"ok\":true}");
            reboot();
        }),

    // /api/cmd — URL-decoded command string → executeCommand
    WAPI("/api/cmd",
        [](Print& out, String qs) {
            int idx = qs.indexOf("c=");
            if (idx >= 0) {
                String enc = qs.substring(idx + 2);
                String dec = "";
                for (int i = 0; i < (int)enc.length(); i++) {
                    char ch = enc[i];
                    if (ch == '+') { dec += ' '; }
                    else if (ch == '%' && i + 2 < (int)enc.length()) {
                        char hi = enc[++i], lo = enc[++i];
                        auto hex = [](char c)->int {
                            return (c>='0'&&c<='9')?c-'0':
                                   (c>='A'&&c<='F')?c-'A'+10:
                                   (c>='a'&&c<='f')?c-'a'+10:0;
                        };
                        dec += (char)((hex(hi)<<4)|hex(lo));
                    } else { dec += ch; }
                }
                if (dec.length() > 0) executeCommand("%s", dec.c_str());
            }
            out.println("HTTP/1.0 200 OK");
            out.println("Content-type:application/json");
            out.println("Cache-Control:no-cache");
            out.println();
            out.print("{\"ok\":true}");
        }),

    WAPI("/api/rotary",
        [](Print& out, String qs) {
            float speed = 0;
            int idx = qs.indexOf("s=");
            if (idx >= 0) speed = qs.substring(idx + 2).toFloat();
            speed = max(-1.0f, min(1.0f, speed));
            bool allowed = true;
            if (speed != 0.0f && !lifter.rotaryAllowed()
                && !(sRescueOverrideExpiry && millis() < sRescueOverrideExpiry))
            {
                allowed = false;
                DEBUG_PRINTLN("ROTARY JOG BLOCKED: safety override not active");
            }
            if (speed == 0.0f) {
                lifter.rotaryMotorStop(); sRotaryJogExpiry = 0;
            } else if (allowed) {
                lifter.enableMotors();
                lifter.rotaryMotorSpeedForce(speed);
                sRotaryJogExpiry = millis() + 10000;
            }
            out.println("HTTP/1.0 200 OK");
            out.println("Content-type:application/json");
            out.println("Cache-Control:no-cache");
            out.println();
            out.print(allowed ? "{\"ok\":true}" : "{\"ok\":false,\"blocked\":\"rotary\"}");
        }),

    WAPI("/api/lift",
        [](Print& out, String qs) {
            float throttle = 0;
            int idx = qs.indexOf("t=");
            if (idx >= 0) throttle = qs.substring(idx + 2).toFloat();
            throttle = max(-1.0f, min(1.0f, throttle));
            // Block descent if rotary is not homed and lifter is above safe height,
            // unless the rescue safety override is active.
            bool allowed = true;
            bool overrideActive = sRescueOverrideExpiry && millis() < sRescueOverrideExpiry;
            if (throttle < 0 && !overrideActive
                && !lifter.rotaryHomeLimit()
                && lifter.getLifterPositionClamped() > sLifterParameters.fRotaryMinHeight)
            {
                allowed = false;
                DEBUG_PRINTLN("LIFT JOG DOWN BLOCKED: rotary not homed, override not active");
            }
            // Raw throttle control clears drift correction hold — the user
            // is manually driving the lifter so position hold would fight them.
            lifter.fLifterTargetPos = -1;
            lifter.fLifterHolding = false;
            if (throttle == 0.0f) {
                lifter.lifterMotorStop(); sLifterJogExpiry = 0;
            } else if (allowed) {
                lifter.moveModeEnd();
                sSafetyManeuver = true;
                lifter.enableMotors();
                lifter.lifterMotorMove(throttle);
                sLifterJogExpiry = millis() + 10000;
            }
            out.println("HTTP/1.0 200 OK");
            out.println("Content-type:application/json");
            out.println("Cache-Control:no-cache");
            out.println();
            out.print(allowed ? "{\"ok\":true}" : "{\"ok\":false,\"blocked\":\"lift\"}");
        }),

    WAPI("/api/estop",
        [](Print& out, String qs) {
            sWebAbort = true;
            sRescueOverrideExpiry = 0;
            lifter.moveModeEnd();
            lifter.lifterMotorStop();
            lifter.rotaryMotorStop();
            lifter.disableMotors();
            sLifterJogExpiry = 0; sRotaryJogExpiry = 0; sCalibrating = false;
            DEBUG_PRINTLN("ESTOP fired");
            out.println("HTTP/1.0 200 OK");
            out.println("Content-type:application/json");
            out.println("Cache-Control:no-cache");
            out.println();
            out.print("{\"ok\":true}");
        }),

    WAPI("/api/action",
        [](Print& out, String qs) {
            int idx = qs.indexOf("do=");
            String action = (idx >= 0) ? qs.substring(idx + 3) : "";
            int amp = action.indexOf('&');
            if (amp >= 0) action = action.substring(0, amp);
            if (action == "clearfault")         { sSafetyManeuverFailed = false;
                                                  sSafetyManeuver = true;
                                                  lifter.enableMotors();
                                                  DEBUG_PRINTLN("Fault cleared"); }
            else if (action == "enablemotors")  { sSafetyManeuverFailed = false;
                                                  sSafetyManeuver = true;
                                                  lifter.enableMotors(); }
            else if (action == "homerotary")    { sSafetyManeuverFailed = false;
                                                  sSafetyManeuver = true;
                                                  lifter.enableMotors();
                                                  lifter.rotateHome(); }
            else if (action == "safety")        { sSafetyManeuverFailed = false;
                                                  sSafetyManeuver = false;
                                                  lifter.enableMotors();
                                                  executeCommand(":PH"); }
            else if (action == "sethome")       { lifter.setRotaryHome();
                                                  DEBUG_PRINTLN("Rotary home set."); }
            else if (action == "setsafeheight") { lifter.setSafeSpinHeightToCurrent(); }
            else if (action == "clearenc")      { lifter.clearRotaryEncoderCount(); }
            else if (action == "override_on")   { sRescueOverrideExpiry = millis() + 60000;
                                                  Serial.println("SAFETY OVERRIDE ACTIVATED (60s)"); }
            else if (action == "override_off")  { sRescueOverrideExpiry = 0;
                                                  Serial.println("SAFETY OVERRIDE CANCELLED"); }
            else if (action == "clearprefs")    { DEBUG_PRINTLN("Clear prefs");
                                                  preferences.clear(); }
            else if (action == "reboot")        { reboot(); }
            out.println("HTTP/1.0 200 OK");
            out.println("Content-type:application/json");
            out.println("Cache-Control:no-cache");
            out.println();
            out.print("{\"ok\":true}");
        }),

    // /api/status — now includes rotDeg (0-359) for rotary dial sync
    WAPI("/api/status",
        [](Print& out, String qs) {
            out.println("HTTP/1.0 200 OK");
            out.println("Content-type:application/json");
            out.println("Cache-Control:no-cache");
            out.println();
            out.print("{\"height\":");    out.print(lifter.getLifterHeightPercent());
            out.print(",\"ticks\":");     out.print(lifter.getLifterPosition());
            out.print(",\"rotPos\":");    out.print(lifter.getRotaryPosition());
            // rotDeg: the rotary position in degrees 0-359 (computed by the firmware)
            out.print(",\"rotDeg\":");    out.print(lifter.rotaryMotorCurrentPosition());
            out.print(",\"safety\":");    out.print(sSafetyManeuver?"true":"false");
            out.print(",\"fault\":");     out.print(sSafetyManeuverFailed?"true":"false");
            out.print(",\"motors\":");    out.print(lifter.motorsEnabled()?"true":"false");
            out.print(",\"topLimit\":"); out.print(lifter.lifterTopLimit()?"true":"false");
            out.print(",\"botLimit\":"); out.print(lifter.lifterBottomLimit()?"true":"false");
            out.print(",\"rotHome\":"); out.print(lifter.rotaryHomeLimit()?"true":"false");
            out.print(",\"liftFault\":"); out.print(lifter.lifterMotorFault()?"true":"false");
            out.print(",\"rotFault\":"); out.print(lifter.rotaryMotorFault()?"true":"false");
            out.print(",\"lightkit\":"); out.print(lifter.getLightShow());
            out.print(",\"holding\":"); out.print(lifter.fLifterHolding?"true":"false");
            {
                int dist = (int)sSettings.getLifterDistance();
                if (dist <= 0) dist = sLifterParameters.fLifterDistance;
                int minHPct = (dist > 0) ? (int)(sLifterParameters.fRotaryMinHeight * 100L / dist) : 0;
                out.print(",\"minHeightPct\":"); out.print(minHPct);
            }
            out.print(",\"invertMotor\":"); out.print(sSettings.fInvertLifterMotor?"true":"false");
            out.print(",\"invertEncoder\":"); out.print(sSettings.fInvertLifterEncoder?"true":"false");
            out.print(",\"lifterLimitNC\":"); out.print(!sSettings.fLifterLimitSetting?"true":"false");
            out.print(",\"rotaryLimitNC\":"); out.print(!sSettings.fRotaryLimitSetting?"true":"false");
            out.print(",\"rssi\":"); out.print(WiFi.status() == WL_CONNECTED ? WiFi.RSSI() : 0);
            out.print(",\"apClients\":"); out.print(WiFi.softAPgetStationNum());
            out.print(",\"override\":");
            if (sRescueOverrideExpiry && millis() < sRescueOverrideExpiry) {
                out.print((sRescueOverrideExpiry - millis()) / 1000);
            } else {
                out.print("0");
            }
            out.print(",\"lastCmd\":\"");
            for (const char* p = sCopyBuffer; *p; p++) {
                if (*p == '"') out.print("\\\"");
                else out.print(*p);
            }
            out.print("\"}");
        }),

    WAPI("/api/log",
        [](Print& out, String qs) {
            int since = -1;
            int idx = qs.indexOf("since=");
            if (idx >= 0) since = qs.substring(idx + 6).toInt();
            out.println("HTTP/1.0 200 OK");
            out.println("Content-type:text/plain; charset=utf-8");
            out.println("Cache-Control:no-cache");
            out.println();
            out.print(webLogBuffer.getLinesAfter(since));
        }),

    WUpload("/upload/firmware",
        [](Client& client) {
            client.println(Update.hasError() ? "HTTP/1.0 200 FAIL" : "HTTP/1.0 200 OK");
            client.println("Content-type:text/html");
            client.println("Vary: Accept-Encoding");
            client.println(); client.println(); client.stop();
            if (!Update.hasError()) { delay(1000); preferences.end(); ESP.restart(); }
            sOTAInProgress = false;
        },
        [](WUploader& upload) {
            if (upload.status == UPLOAD_FILE_START) {
                sOTAInProgress = true; unmountFileSystems();
                Serial.printf("Update: %s\n", upload.filename.c_str());
                if (!Update.begin(upload.fileSize)) Update.printError(Serial);
            } else if (upload.status == UPLOAD_FILE_WRITE) {
#ifdef USE_DEBUG
                DEBUG_PRINTLN("Received: "
                    + String((float)upload.receivedSize/(float)upload.fileSize*100) + "%");
#endif
                if (Update.write(upload.buf, upload.currentSize) != upload.currentSize)
                    Update.printError(Serial);
            } else if (upload.status == UPLOAD_FILE_END) {
                DEBUG_PRINTLN("GAME OVER");
                if (Update.end(true))
                    Serial.printf("Update Success: %u\nRebooting...\n", upload.receivedSize);
                else Update.printError(Serial);
            }
        }),
};

WifiWebServer<20, SizeOfArray(pages)> webServer(pages, wifiAccess);
#endif // USE_WIFI_WEB
