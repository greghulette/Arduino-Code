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
    // Allow user scaling on tablet/desktop; phone keeps no-scale for control precision
    out.print("<meta name='viewport' content='width=device-width,initial-scale=1,viewport-fit=cover'>");
    out.print("<title>"); out.print(title); out.print(" v" FIRMWARE_VERSION "</title><style>");

    // ── Reset & base ──────────────────────────────────────────────────────
    out.print("*{box-sizing:border-box;margin:0;padding:0}");
    out.print("html{height:100%}");
    out.print("body{min-height:100%;background:#f0f2f5;color:#1a1a1a;"
              "font-family:'Helvetica Neue',Helvetica,Arial,sans-serif;"
              "font-size:17px;-webkit-tap-highlight-color:transparent}");

    // ── Shell: phone = simple stack, desktop = sidebar + content ─────────
    // #shell wraps everything below the status bar
    out.print("#shell{display:flex;min-height:calc(100vh - 60px)}");
    // #page: the scrollable content area
    out.print("#page{flex:1;padding:12px 14px 90px;min-width:0}");
    out.print("@supports(padding:env(safe-area-inset-bottom)){"
              "#page{padding-bottom:calc(80px + env(safe-area-inset-bottom))}}");

    // ── Status bar ────────────────────────────────────────────────────────
    out.print("#sbar{display:grid;grid-template-columns:repeat(6,1fr);gap:4px;"
              "padding:9px 10px 7px;background:#fff;"
              "border-bottom:1px solid #dde1e7;"
              "position:sticky;top:0;z-index:100;"
              "box-shadow:0 1px 4px rgba(0,0,0,.08)}");
    out.print(".ss{background:#f7f8fa;border:1px solid #e4e6ea;"
              "border-radius:9px;padding:6px 4px;text-align:center}");
    out.print(".sv{font-size:14px;font-weight:700;line-height:1.2}");
    out.print(".sl{font-size:9px;color:#888;margin-top:2px;"
              "text-transform:uppercase;letter-spacing:.05em}");
    out.print(".c-g{color:#16a34a}.c-y{color:#d97706}"
              ".c-r{color:#dc2626}.c-b{color:#2563eb}.c-m{color:#888;font-size:10px}");

    // ── Section labels ────────────────────────────────────────────────────
    out.print(".sec{font-size:12px;color:#888;font-weight:600;"
              "letter-spacing:.08em;text-transform:uppercase;margin:16px 0 8px}");
    out.print(".sec:first-child{margin-top:6px}");

    // ── Buttons ───────────────────────────────────────────────────────────
    out.print("button{appearance:none;-webkit-appearance:none;"
              "border:1px solid #cdd1d9;background:#fff;color:#333;"
              "border-radius:10px;padding:14px 8px;font-size:16px;"
              "font-family:inherit;cursor:pointer;text-align:center;"
              "width:100%;font-weight:500;transition:background .1s}");
    out.print("button:hover{background:#f4f6f8}");
    out.print("button:active{background:#e8eaf0;opacity:.85}");
    out.print("button.danger{border-color:#fca5a5;background:#fef2f2;color:#dc2626}");
    out.print("button.danger:hover{background:#fee2e2}");
    out.print("button.primary{border-color:#93c5fd;background:#eff6ff;color:#2563eb}");
    out.print("button.primary:hover{background:#dbeafe}");
    out.print("button.estop{border-color:#dc2626;background:#dc2626;"
              "color:#fff;font-size:18px;font-weight:700;"
              "padding:18px;width:100%;border-radius:12px;"
              "margin-top:12px}");
    out.print("button.estop:hover{background:#b91c1c}");

    // ── Grids ─────────────────────────────────────────────────────────────
    out.print(".g2{display:grid;grid-template-columns:1fr 1fr;gap:8px}");
    out.print(".g3{display:grid;grid-template-columns:1fr 1fr 1fr;gap:8px}");
    out.print(".g5{display:grid;grid-template-columns:repeat(5,1fr);gap:6px}");

    // ── Light-kit pills ───────────────────────────────────────────────────
    out.print(".lk{display:flex;gap:7px;flex-wrap:wrap;padding-bottom:2px}");
    out.print(".lp{font-size:14px;padding:8px 14px;"
              "border-radius:20px;border:1px solid #cdd1d9;"
              "color:#555;background:#fff;cursor:pointer;"
              "white-space:nowrap;font-weight:500}");
    out.print(".lp:hover{background:#f4f6f8}");
    out.print(".lp.on{background:#2563eb;border-color:#2563eb;color:#fff}");

    // ── Sequence buttons ──────────────────────────────────────────────────
    out.print(".sq{background:#fff;border:1px solid #cdd1d9;"
              "border-radius:10px;color:#555;font-size:16px;"
              "padding:12px 0;cursor:pointer;text-align:center;font-weight:600}");
    out.print(".sq:hover{background:#f4f6f8}");
    out.print(".sq.on{background:#2563eb;border-color:#2563eb;color:#fff}");

    // ── Position row (phone default) ──────────────────────────────────────
    out.print(".pos-row{display:flex;gap:12px;align-items:stretch;height:190px}");
    out.print(".lift-col{display:flex;flex-direction:column;"
              "align-items:center;gap:6px;width:70px}");
    out.print(".lift-lbl{font-size:12px;color:#888;"
              "text-transform:uppercase;letter-spacing:.05em;font-weight:600}");
    out.print(".lift-val{font-size:17px;font-weight:700;color:#2563eb}");
    out.print("canvas{display:block;touch-action:none;cursor:grab}");
    out.print("canvas:active{cursor:grabbing}");
    out.print(".dial-col{flex:1;display:flex;flex-direction:column;"
              "align-items:center;justify-content:center;gap:5px}");
    out.print(".dial-val{font-size:17px;font-weight:700;color:#2563eb}");
    out.print(".dial-hint{font-size:12px;color:#888;text-align:center}");

    // ── Speed sliders ─────────────────────────────────────────────────────
    out.print(".spd-row{display:flex;gap:10px;margin-top:10px}");
    out.print(".spd-item{flex:1;background:#fff;border:1px solid #dde1e7;"
              "border-radius:10px;padding:10px 12px}");
    out.print(".spd-lbl{font-size:12px;color:#888;text-transform:uppercase;"
              "letter-spacing:.05em;margin-bottom:8px;font-weight:600}");
    out.print(".spd-val{font-size:13px;color:#2563eb;text-align:right;margin-top:5px}");
    out.print("input[type=range]{-webkit-appearance:none;appearance:none;"
              "width:100%;height:5px;background:#dde1e7;"
              "border-radius:3px;outline:none;cursor:pointer}");
    out.print("input[type=range]::-webkit-slider-thumb{"
              "-webkit-appearance:none;width:22px;height:22px;"
              "background:#2563eb;border-radius:50%;border:none}");
    out.print("input[type=range]::-moz-range-thumb{"
              "width:22px;height:22px;background:#2563eb;"
              "border-radius:50%;border:none}");

    // ── Status chips ──────────────────────────────────────────────────────
    out.print(".chip-grid{display:grid;grid-template-columns:repeat(3,1fr);gap:6px}");
    out.print(".chip{background:#fff;border:1px solid #dde1e7;"
              "border-radius:10px;padding:8px 10px}");
    out.print(".chip .sv{font-size:15px;font-weight:700}");
    out.print(".chip .sl{font-size:10px;color:#888;margin-top:2px;"
              "text-transform:uppercase;letter-spacing:.04em}");

    // ── Card ──────────────────────────────────────────────────────────────
    out.print(".card{background:#fff;border:1px solid #dde1e7;"
              "border-radius:12px;padding:14px}");

    // ── Settings list ─────────────────────────────────────────────────────
    out.print(".set-row{display:flex;align-items:center;padding:14px 0;"
              "border-bottom:1px solid #e8eaed;"
              "cursor:pointer;text-decoration:none;color:inherit}");
    out.print(".set-row:hover{background:#fafbfc}");
    out.print(".set-row:last-child{border-bottom:none}");
    out.print(".set-icon{width:36px;height:36px;border-radius:10px;"
              "display:flex;align-items:center;justify-content:center;"
              "margin-right:14px;flex-shrink:0}");
    out.print(".set-main{flex:1;min-width:0}");
    out.print(".set-name{font-size:16px;color:#1a1a1a;font-weight:500}");
    out.print(".set-val{font-size:13px;color:#888;margin-top:3px;"
              "white-space:nowrap;overflow:hidden;text-overflow:ellipsis}");
    out.print(".set-arr{color:#bbb;font-size:20px;flex-shrink:0;margin-left:6px}");
    out.print(".danger-zone{margin-top:20px;border-top:1px solid #e8eaed;padding-top:14px}");
    out.print(".dz-lbl{font-size:12px;color:#888;text-transform:uppercase;"
              "letter-spacing:.07em;margin-bottom:10px;font-weight:600}");

    // ── Form inputs ───────────────────────────────────────────────────────
    out.print("input[type=text],input[type=password],input[type=number]{"
              "width:100%;padding:11px 14px;font-size:16px;"
              "border:1px solid #cdd1d9;border-radius:10px;"
              "background:#fff;color:#1a1a1a;font-family:inherit;"
              "appearance:none;-webkit-appearance:none}");
    out.print("input[type=text]:focus,input[type=password]:focus,"
              "input[type=number]:focus{outline:none;border-color:#2563eb}");
    out.print("select{width:100%;padding:11px 14px;font-size:16px;"
              "border:1px solid #cdd1d9;border-radius:10px;"
              "background:#fff;color:#1a1a1a;font-family:inherit;"
              "appearance:none;-webkit-appearance:none}");
    out.print(".form-actions{display:flex;gap:8px;margin-top:6px}");
    out.print(".form-actions button{flex:1}");
    out.print(".back-link{display:inline-flex;align-items:center;gap:6px;"
              "font-size:15px;color:#2563eb;text-decoration:none;"
              "font-weight:500;padding:14px 0}");

    // ── Misc ──────────────────────────────────────────────────────────────
    out.print(".jog{touch-action:none;user-select:none;-webkit-user-select:none}");
    out.print("hr{border:none;border-top:1px solid #e8eaed;margin:12px 0}");

    // ── Log viewer ────────────────────────────────────────────────────────
    out.print("#lb{background:#1a1a2e;color:#4ade80;font-family:monospace;"
              "font-size:13px;height:calc(100vh - 200px);overflow-y:auto;"
              "padding:12px;border-radius:10px;border:1px solid #dde1e7}");

    // ── PHONE: bottom tab bar (default, < 640 px) ─────────────────────────
    out.print("#tabs{display:flex;background:#fff;"
              "border-top:1px solid #dde1e7;"
              "padding:10px 0 16px;"
              "position:fixed;bottom:0;left:0;right:0;"
              "z-index:200;box-shadow:0 -1px 6px rgba(0,0,0,.07)}");
    // On modern iPhones with notch/island, pad the tab bar extra via @supports
    out.print("@supports(padding:env(safe-area-inset-bottom)){"
              "#tabs{padding-bottom:calc(10px + env(safe-area-inset-bottom))}}");
    out.print(".tab{flex:1;display:flex;flex-direction:column;"
              "align-items:center;gap:4px;text-decoration:none;cursor:pointer}");
    out.print(".tab-ic{width:28px;height:28px;border-radius:8px;"
              "display:flex;align-items:center;justify-content:center}");
    out.print(".tab-lbl{font-size:10px;color:#aaa;"
              "text-transform:uppercase;letter-spacing:.05em;font-weight:600}");
    out.print(".tab.active .tab-ic{background:#dbeafe}");
    out.print(".tab.active .tab-lbl{color:#2563eb}");
    out.print(".tab .tab-ic svg{stroke:#aaa}");
    out.print(".tab.active .tab-ic svg{stroke:#2563eb}");
    // Sidebar nav hidden on phone
    out.print("#sidenav{display:none}");

    // ── TABLET: >= 640 px ─────────────────────────────────────────────────
    // Light kit wraps instead of scrolls; position row taller; chip grid 6-wide
    out.print("@media(min-width:640px){");
    out.print("#page{padding:16px 20px 90px}");
    out.print(".sv{font-size:16px}");
    out.print(".sl{font-size:10px}");
    out.print(".ss{padding:8px 6px}");
    out.print(".pos-row{height:240px}");
    out.print(".lift-col{width:80px}");
    out.print(".chip-grid{grid-template-columns:repeat(5,1fr)}");
    out.print(".g5{grid-template-columns:repeat(9,1fr)}");
    out.print("#lb{height:calc(100vh - 220px)}");
    out.print("}");

    // ── DESKTOP: >= 1024 px ───────────────────────────────────────────────
    // Left sidebar replaces bottom tab bar; content centered; canvas bigger
    out.print("@media(min-width:1024px){");
    // Body layout
    out.print("body{font-size:16px}");
    // Status bar
    out.print("#sbar{padding:12px 24px 10px;gap:8px;position:sticky;top:0}");
    out.print(".ss{padding:10px 8px;border-radius:10px}");
    out.print(".sv{font-size:18px}");
    out.print(".sl{font-size:10px;margin-top:4px}");
    // Shell: sidebar + scrollable content
    out.print("#shell{display:flex;align-items:flex-start}");
    // Sidebar nav
    out.print("#sidenav{display:flex;flex-direction:column;gap:2px;"
              "width:180px;flex-shrink:0;"
              "padding:16px 12px;"
              "position:sticky;top:60px;"
              "height:calc(100vh - 60px);"
              "background:#fff;border-right:1px solid #dde1e7;"
              "overflow-y:auto}");
    out.print(".snav-title{font-size:10px;font-weight:700;color:#aaa;"
              "text-transform:uppercase;letter-spacing:.1em;"
              "padding:8px 12px 6px;margin-top:8px}");
    out.print(".snav-title:first-child{margin-top:0}");
    out.print(".snav-a{display:flex;align-items:center;gap:10px;"
              "padding:10px 12px;border-radius:10px;"
              "text-decoration:none;color:#555;font-size:15px;font-weight:500;"
              "cursor:pointer;transition:background .1s}");
    out.print(".snav-a:hover{background:#f0f2f5;color:#1a1a1a}");
    out.print(".snav-a.active{background:#dbeafe;color:#2563eb}");
    out.print(".snav-a svg{flex-shrink:0}");
    // Hide phone tab bar on desktop
    out.print("#tabs{display:none}");
    // Content area
    out.print("#page{padding:20px 32px 40px;max-width:860px}");
    // Larger position row
    out.print(".pos-row{height:280px}");
    out.print(".lift-col{width:90px}");
    // Chip grid: 6 columns on desktop
    out.print(".chip-grid{grid-template-columns:repeat(6,1fr);gap:8px}");
    // All 9 sequences in one row
    out.print(".g5{grid-template-columns:repeat(9,1fr)}");
    // Log
    out.print("#lb{height:calc(100vh - 240px)}");
    out.print("}");

    out.print("</style></head><body>");
}

#define TAB_ICON_PERISCOPE \
    "<svg width='18' height='18' viewBox='0 0 18 18' fill='none' stroke-width='1.6' stroke-linecap='round'>" \
    "<rect x='2' y='10' width='14' height='6' rx='2'/><rect x='6' y='2' width='6' height='9' rx='2'/></svg>"

#define TAB_ICON_RESCUE \
    "<svg width='18' height='18' viewBox='0 0 18 18' fill='none' stroke-width='1.7' stroke-linecap='round'>" \
    "<circle cx='9' cy='9' r='6.5'/><line x1='9' y1='5' x2='9' y2='9.5'/>" \
    "<line x1='9' y1='12' x2='9' y2='13'/></svg>"

#define TAB_ICON_LOG \
    "<svg width='18' height='18' viewBox='0 0 18 18' fill='none' stroke-width='1.6' stroke-linecap='round'>" \
    "<rect x='2' y='2' width='14' height='14' rx='3'/><line x1='5' y1='6.5' x2='13' y2='6.5'/>" \
    "<line x1='5' y1='9.5' x2='10' y2='9.5'/><line x1='5' y1='12.5' x2='8' y2='12.5'/></svg>"

#define TAB_ICON_SETUP \
    "<svg width='18' height='18' viewBox='0 0 18 18' fill='none' stroke-width='1.6' stroke-linecap='round'>" \
    "<circle cx='9' cy='9' r='2.5'/>" \
    "<path d='M9 1.5v3M9 13.5v3M1.5 9h3M13.5 9h3" \
    "M3.9 3.9l2.12 2.12M12 12l2.1 2.1M3.9 14.1l2.12-2.12M12 6l2.1-2.1'/></svg>"

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
    out.print("<div class='ss'><div class='sv c-m'>v" FIRMWARE_VERSION "</div>"
              "<div class='sl'>firmware</div></div>");
    out.print("</div>");
    // Shell opens here — on desktop it becomes the flex container for sidebar + content
    out.print("<div id='shell'><div id='sidenav'></div>");
}

static void printTabBar(Print& out, const char* active)
{
    struct { const char* href; const char* id; const char* icon; const char* label; } tabs[] = {
        { "/periscope", "periscope", TAB_ICON_PERISCOPE, "periscope" },
        { "/rescue",    "rescue",    TAB_ICON_RESCUE,    "rescue"    },
        { "/log",       "log",       TAB_ICON_LOG,       "log"       },
        { "/setup",     "setup",     TAB_ICON_SETUP,     "setup"     },
    };
    // Close #shell (each page function closes its own #page before calling us)
    out.print("</div>"); // close #shell

    // ── Phone: bottom tab bar ─────────────────────────────────────────────
    out.print("<nav id='tabs'>");
    for (auto& t : tabs) {
        bool on = (strcmp(t.id, active) == 0);
        out.print("<a class='tab"); if (on) out.print(" active");
        out.print("' href='"); out.print(t.href); out.print("'>");
        out.print("<div class='tab-ic'>"); out.print(t.icon); out.print("</div>");
        out.print("<div class='tab-lbl'>"); out.print(t.label); out.print("</div>");
        out.print("</a>");
    }
    out.print("</nav>");

    // ── Desktop: sidebar nav (injected into #sidenav via JS on large screens) ─
    // We emit a hidden template and the JS moves it into the sidebar at load time.
    out.print("<script>");
    out.print("(function(){");
    out.print("var w=window.innerWidth;");
    out.print("if(w<1024)return;");
    // Build the sidebar
    out.print("var sn=document.getElementById('sidenav');");
    out.print("if(!sn)return;");
    out.print("var links=[");
    for (int i = 0; i < 4; i++) {
        if (i) out.print(",");
        out.print("{href:'"); out.print(tabs[i].href);
        out.print("',icon:'"); out.print(tabs[i].icon);
        out.print("',label:'"); out.print(tabs[i].label);
        out.print("',active:"); out.print(strcmp(tabs[i].id, active) == 0 ? "true" : "false");
        out.print("}");
    }
    out.print("];");
    out.print("var title=document.createElement('div');");
    out.print("title.className='snav-title';title.textContent='R2 Uppity';");
    out.print("sn.appendChild(title);");
    out.print("links.forEach(function(l){");
    out.print("var a=document.createElement('a');");
    out.print("a.className='snav-a'+(l.active?' active':'');");
    out.print("a.href=l.href;");
    out.print("a.innerHTML=l.icon+'<span>'+l.label+'</span>';");
    out.print("sn.appendChild(a);});");
    out.print("})();");
    out.print("</script>");
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
    out.print("<div style='font-size:12px;color:#888;line-height:1.6;"
              "background:#fff;border:1px solid #dde1e7;border-radius:10px;"
              "padding:10px 12px;margin-bottom:8px'>");
    out.print("<b style='color:#555'>Safety</b> — the startup maneuver: "
              "the board lowers the periscope and finds the rotary home position "
              "before allowing any movement. Shows <b style='color:#16a34a'>Ready</b> "
              "once complete, <b style='color:#d97706'>WAIT</b> while running, "
              "<b style='color:#dc2626'>FAULT</b> if it failed.<br>"
              "<b style='color:#555'>Fault</b> — a motor driver fault: "
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
    out.print("_lctx.fillStyle='#f0f2f5';"
              "_lctx.beginPath();_lctx.roundRect(0,0,_lcW,_lcH,14);"
              "_lctx.fill();");
    out.print("_lctx.strokeStyle='#dde1e7';_lctx.lineWidth=1;"
              "_lctx.beginPath();_lctx.moveTo(8,_lcH/2);"
              "_lctx.lineTo(_lcW-8,_lcH/2);_lctx.stroke();");
    out.print("_lctx.fillStyle='rgba(37,99,235,0.07)';"
              "_lctx.beginPath();"
              "_lctx.roundRect(1,1,_lcW-2,_lcH/2-1,"
              "{upperLeft:13,upperRight:13});_lctx.fill();");
    out.print("_lctx.fillStyle='rgba(220,38,38,0.07)';"
              "_lctx.beginPath();"
              "_lctx.roundRect(1,_lcH/2,_lcW-2,_lcH/2-1,"
              "{lowerLeft:13,lowerRight:13});_lctx.fill();");
    out.print("_lctx.font='bold 20px sans-serif';_lctx.textAlign='center';"
              "_lctx.fillStyle='#93c5fd';"
              "_lctx.fillText('\\u25B2',_lcW/2,26);"
              "_lctx.fillStyle='#fca5a5';"
              "_lctx.fillText('\\u25BC',_lcW/2,_lcH-8);");
    out.print("if(_lcActive&&py!==null){"
              "_lctx.beginPath();_lctx.arc(_lcW/2,py,16,0,6.283);"
              "_lctx.fillStyle=_lcDir>0?'#2563eb':'#dc2626';"
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
              "_dctx.fillStyle='#f7f8fa';_dctx.fill();"
              "_dctx.strokeStyle='#dde1e7';_dctx.lineWidth=2;_dctx.stroke();");
    out.print("for(var i=0;i<12;i++){"
              "var a=i*Math.PI/6,r1=_dcr-5,r2=_dcr-13;"
              "_dctx.beginPath();"
              "_dctx.moveTo(_dcx+r1*Math.sin(a),_dcy-r1*Math.cos(a));"
              "_dctx.lineTo(_dcx+r2*Math.sin(a),_dcy-r2*Math.cos(a));"
              "_dctx.strokeStyle='#cdd1d9';_dctx.lineWidth=1.5;"
              "_dctx.stroke();}");
    // Home tick (top, 0°) highlighted green
    out.print("_dctx.beginPath();"
              "_dctx.moveTo(_dcx,_dcy-(_dcr-5));"
              "_dctx.lineTo(_dcx,_dcy-(_dcr-14));"
              "_dctx.strokeStyle='#16a34a';_dctx.lineWidth=2.5;"
              "_dctx.stroke();");
    out.print("var na=_dang*Math.PI/180;"
              "_dctx.beginPath();_dctx.moveTo(_dcx,_dcy);"
              "_dctx.lineTo(_dcx+(_dcr-10)*Math.sin(na),"
              "_dcy-(_dcr-10)*Math.cos(na));"
              "_dctx.strokeStyle='#2563eb';_dctx.lineWidth=3;"
              "_dctx.lineCap='round';_dctx.stroke();");
    out.print("_dctx.beginPath();_dctx.arc(_dcx,_dcy,6,0,6.283);"
              "_dctx.fillStyle='#2563eb';_dctx.fill();");
    // "home" label at top
    out.print("_dctx.fillStyle='#16a34a';_dctx.font='bold 10px sans-serif';"
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
    printStatusBar(out);
    out.print("<div id='page'>");

    out.print("<button class='estop' "
              "onclick='fetch(\"/api/estop\")'>&#9632; EMERGENCY STOP</button>");

    out.print("<div class='sec'>lifter jog</div>");
    out.print("<div class='card' style='margin-bottom:10px'>");
    out.print("<div style='display:flex;align-items:center;"
              "gap:10px;margin-bottom:10px'>");
    out.print("<span style='font-size:13px;color:#888;"
              "white-space:nowrap;font-weight:600'>jog speed</span>");
    out.print("<input type='range' id='jspd' min='0' max='100' "
              "value='80' step='1' style='flex:1'>");
    out.print("<span style='font-size:15px;color:#2563eb;"
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
    out.print("<div style='font-size:13px;color:#888'>"
              "drag left/right &bull; release to stop</div>");
    out.print("<div style='font-size:16px;color:#2563eb;font-weight:700'>"
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
    out.print("<button onclick='fetch(\"/api/action?do=setsafeheight\")'>"
              "set safe spin ht</button>");
    out.print("<button onclick='fetch(\"/api/action?do=clearenc\")'>"
              "clear encoder</button>");
    out.print("</div></div>");

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
              "_rx.fillStyle='#f7f8fa';_rx.fill();"
              "_rx.strokeStyle='#dde1e7';_rx.lineWidth=2;_rx.stroke();");
    out.print("_rx.save();_rx.beginPath();_rx.moveTo(_rcx,_rcy);"
              "_rx.arc(_rcx,_rcy,_rcr,1.571,4.712);_rx.closePath();"
              "_rx.fillStyle='rgba(37,99,235,0.08)';_rx.fill();_rx.restore();");
    out.print("_rx.save();_rx.beginPath();_rx.moveTo(_rcx,_rcy);"
              "_rx.arc(_rcx,_rcy,_rcr,-1.571,1.571);_rx.closePath();"
              "_rx.fillStyle='rgba(220,38,38,0.08)';_rx.fill();_rx.restore();");
    out.print("_rx.strokeStyle='#dde1e7';_rx.lineWidth=1;"
              "_rx.beginPath();"
              "_rx.moveTo(_rcx-_rcr,_rcy);_rx.lineTo(_rcx+_rcr,_rcy);"
              "_rx.stroke();");
    out.print("_rx.fillStyle='#aaa';_rx.font='bold 15px sans-serif';"
              "_rx.textAlign='center';"
              "_rx.fillText('\\u21b6 L',_rcx-Math.round(_rcr*0.54),_rcy+5);"
              "_rx.fillText('R \\u21b7',_rcx+Math.round(_rcr*0.54),_rcy+5);");
    out.print("_rx.beginPath();_rx.arc(_rcx,_rcy,14,0,6.283);"
              "_rx.fillStyle='#e8eaed';_rx.fill();"
              "_rx.strokeStyle='#cdd1d9';_rx.lineWidth=1;_rx.stroke();");
    out.print("if(_rda){"
              "_rx.beginPath();"
              "_rx.arc(_rcx+_rds*_rcr,_rcy,14,0,6.283);"
              "_rx.fillStyle=_rds<0?'#2563eb':(_rds>0?'#dc2626':'#aaa');"
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
    printStatusBar(out);
    out.print("<div id='page' style='padding-bottom:90px'>");
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
    printStatusBar(out);
    out.print("<div id='page'>");
    out.print("<div style='font-size:13px;color:#888;font-weight:600;"
              "padding:10px 0 12px;border-bottom:1px solid #e8eaed;"
              "margin-bottom:6px;text-transform:uppercase;letter-spacing:.07em'>"
              "configuration</div>");

    // Calibrate
    out.print("<a class='set-row' href='/calibrate'>");
    out.print("<div class='set-icon' style='background:#dbeafe'>"
              "<svg width='18' height='18' viewBox='0 0 18 18' fill='none' "
              "stroke='#2563eb' stroke-width='1.6' stroke-linecap='round'>"
              "<circle cx='9' cy='9' r='6'/>"
              "<path d='M9 5.5v3.5l2.5 1.5'/></svg></div>");
    out.print("<div class='set-main'>"
              "<div class='set-name'>calibrate</div>"
              "<div class='set-val'>run calibration routine</div>"
              "</div><div class='set-arr'>&#8250;</div></a>");

    // Marcduino
    out.print("<a class='set-row' href='/marcduino'>");
    out.print("<div class='set-icon' style='background:#dcfce7'>"
              "<svg width='18' height='18' viewBox='0 0 18 18' fill='none' "
              "stroke='#16a34a' stroke-width='1.6' stroke-linecap='round'>"
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
    out.print("<div class='set-icon' style='background:#f3e8ff'>"
              "<svg width='18' height='18' viewBox='0 0 18 18' fill='none' "
              "stroke='#9333ea' stroke-width='1.6' stroke-linecap='round'>"
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
    out.print("<div class='set-icon' style='background:#fef9c3'>"
              "<svg width='18' height='18' viewBox='0 0 18 18' fill='none' "
              "stroke='#ca8a04' stroke-width='1.6' stroke-linecap='round'>"
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
    out.print("<div class='set-icon' style='background:#e0e7ff'>"
              "<svg width='18' height='18' viewBox='0 0 18 18' fill='none' "
              "stroke='#4f46e5' stroke-width='1.6' stroke-linecap='round'>"
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
    out.print("<div class='set-icon' style='background:#dcfce7'>"
              "<svg width='18' height='18' viewBox='0 0 18 18' fill='none' "
              "stroke='#16a34a' stroke-width='1.6' stroke-linecap='round'>"
              "<path d='M2 14h14M5 14V9l4-6 4 6v5'/></svg></div>");
    out.print("<div class='set-main'><div class='set-name'>firmware</div>"
              "<div class='set-val'>"); out.print(__DATE__);
#ifdef BUILD_VERSION
    out.print(" &bull; <a href='"); out.print(BUILD_VERSION);
    out.print("' style='color:#2563eb'>sources</a>");
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
    printPageHead(out, "R2 Calibrate");
    printStatusBar(out);
    out.print("<div id='page'>");
    out.print("<div class='sec'>calibration</div>");
    out.print("<div class='card' style='margin-bottom:14px'>");
    out.print("<p style='font-size:15px;color:#555;margin-bottom:12px;line-height:1.6'>"
              "Runs the automatic calibration routine. The lifter will move up and "
              "down at increasing speeds to measure the minimum power and full travel "
              "distance. Results are saved to flash.</p>");
    out.print("<p style='font-size:15px;color:#dc2626;margin-bottom:16px;line-height:1.6'>"
              "&#9888; Keep clear of the periscope during calibration.</p>");
    out.print("<button class='primary' style='font-size:17px;padding:16px' "
              "onclick='doCalibrate()'>start calibration</button>");
    out.print("<div id='calmsg' style='margin-top:14px;font-size:15px;"
              "color:#888'></div>");
    out.print("</div></div>");
    printTabBar(out, "setup");
    out.print("<script>");
    out.print("function doCalibrate(){"
              "document.getElementById('calmsg').textContent="
              "'Calibration started — watch the log page for progress.';"
              "fetch('/api/cmd?c='+encodeURIComponent('#PSC')).catch(function(){});}");
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
    out.print("</script></body></html>");
}

///////////////////////////////////////////////////////////////////////////////
// SETUP SUB-PAGES — restyled with new .form-* CSS classes
// These use WElement framework for form widgets but we supply custom CSS above.
///////////////////////////////////////////////////////////////////////////////

int paramLifterMinPower;
int paramLifterSeekBotPower;
int paramRotaryMinPower;
int paramLifterDistance;
int paramMinHeight;
int paramDriftPct;
bool paramRotDisabled;
String wifiSSID;
String wifiPass;
bool wifiAP;
bool marcWifiEnabled;
int marcID;
int marcBaudRate;
String remoteHostName;
String remoteSecret;
String swBaudRates[] = { "2400", "9600" };

WElement marcduinoContents[] = {
    WTextFieldInteger("ID #", "marcID",
        []()->String { return String(marcID = sSettings.fID); },
        [](String val) { marcID = val.toInt(); }),
    WSelect("Serial baud rate", "serialbaud",
        swBaudRates, SizeOfArray(swBaudRates),
        []() { return ((marcBaudRate = sSettings.fBaudRate) == 2400) ? 0 : 1; },
        [](int val) { marcBaudRate = (val == 0) ? 2400 : 9600; }),
    WCheckbox("Marcduino on WiFi (port 2000)", "wifienabled",
        []() { return (marcWifiEnabled =
                   preferences.getBool(PREFERENCE_MARCWIFI_ENABLED, MARC_WIFI_ENABLED)); },
        [](bool val) { marcWifiEnabled = val; }),
    WLabel("Last command:", "cmd"),
    WTextFieldReadonly("", "received",
        []()->String { return sCopyBuffer; },
        []()->bool { return true; }),
    WButton("Save", "save", []() {
        preferences.putBool(PREFERENCE_MARCWIFI_ENABLED, marcWifiEnabled);
        if (marcID != (int)sSettings.fID)
            { sSettings.fID = marcID; sUpdateSettings = true; }
        if (marcBaudRate != (int)sSettings.fBaudRate)
            { sSettings.fBaudRate = marcBaudRate; sUpdateSettings = true; }
    }),
    WHorizontalAlign(),
    WButton("Back", "back", "/setup"),
    WVerticalAlign(),
};

WElement parametersContents[] = {
    WTextFieldInteger("Min lifter power", "lftminpwr",
        []()->String { return String(paramLifterMinPower = LIFTER_MINIMUM_POWER); },
        [](String val) { paramLifterMinPower = val.toInt(); }),
    WTextFieldInteger("Min seek-bottom power", "lftseekbotpwr",
        []()->String { return String(paramLifterSeekBotPower = LIFTER_SEEKBOTTTOM_POWER); },
        [](String val) { paramLifterSeekBotPower = val.toInt(); }),
    WTextFieldInteger("Min rotary power", "rotminpwr",
        []()->String { return String(paramRotaryMinPower = ROTARY_MINIMUM_POWER); },
        [](String val) { paramRotaryMinPower = val.toInt(); }),
    WTextFieldInteger("Lifter distance (ticks)", "lftdist",
        []()->String { return String(paramLifterDistance = LIFTER_DISTANCE); },
        [](String val) { paramLifterDistance = val.toInt(); }),
    WSlider("Safe spin height (% of travel)", "minsafeheight", 0, 100,
        []()->int {
            paramMinHeight = ROTARY_MINIMUM_HEIGHT;
            int dist = max(1, LIFTER_DISTANCE);
            return paramMinHeight * 100 / dist;
        },
        [](int pct) { paramMinHeight = pct * max(1, LIFTER_DISTANCE) / 100; }),
    WButton("Set to current height", "setsafeheight", []() {
        lifter.setSafeSpinHeightToCurrent();
        paramMinHeight = ROTARY_MINIMUM_HEIGHT;
    }),
    WHorizontalAlign(),
    WSlider("Drift correction (% threshold, 0=off)", "driftpct", 0, 20,
        []()->int {
            return paramDriftPct = sLifterParameters.fDriftCorrectionPct;
        },
        [](int val) { paramDriftPct = val; }),
    WVerticalAlign(),
    WCheckbox("Disable rotary unit", "rotdisabled",
        []() { return paramRotDisabled = sSettings.fDisableRotary; },
        [](bool val) { paramRotDisabled = val; }),
    WVerticalAlign(),
    WLabel("Load defaults:", "dflbl"),
    WButtonReload("Greg", "greg", []() {
        LIFTER_MINIMUM_POWER     = GREG_LIFTER_MINIMUM_POWER;
        LIFTER_SEEKBOTTTOM_POWER = GREG_LIFTER_SEEKBOTTTOM_POWER;
        ROTARY_MINIMUM_POWER     = GREG_ROTARY_MINIMUM_POWER;
        LIFTER_DISTANCE          = GREG_LIFTER_DISTANCE;
        ROTARY_MINIMUM_HEIGHT    = LIFTER_DISTANCE / 2;
        sLifterParameters.fDriftCorrectionPct = DEFAULT_DRIFT_CORRECTION_PCT;
    }),
    WHorizontalAlign(),
    WButtonReload("IA-Parts", "iaparts", []() {
        LIFTER_MINIMUM_POWER     = IAPARTS_LIFTER_MINIMUM_POWER;
        LIFTER_SEEKBOTTTOM_POWER = IAPARTS_LIFTER_SEEKBOTTTOM_POWER;
        ROTARY_MINIMUM_POWER     = IAPARTS_ROTARY_MINIMUM_POWER;
        LIFTER_DISTANCE          = IAPARTS_LIFTER_DISTANCE;
        ROTARY_MINIMUM_HEIGHT    = LIFTER_DISTANCE / 2;
        sLifterParameters.fDriftCorrectionPct = DEFAULT_DRIFT_CORRECTION_PCT;
    }),
    WVerticalAlign(),
    WButton("Save", "save", []() {
        LIFTER_MINIMUM_POWER     = min(max(paramLifterMinPower, 0), 100);
        LIFTER_SEEKBOTTTOM_POWER = min(max(paramLifterSeekBotPower, 0), 100);
        ROTARY_MINIMUM_POWER     = min(max(paramRotaryMinPower, 0), 100);
        LIFTER_DISTANCE          = max(paramLifterDistance, 0);
        ROTARY_MINIMUM_HEIGHT    = min(max(paramMinHeight, 0), LIFTER_DISTANCE);
        sLifterParameters.fDriftCorrectionPct = min(max(paramDriftPct, 0), 20);
        sLifterParameters.save();
        if (paramRotDisabled != sSettings.fDisableRotary)
            { sSettings.fDisableRotary = paramRotDisabled; sUpdateSettings = true; }
    }),
    WHorizontalAlign(),
    WButton("Back", "back", "/setup"),
    WVerticalAlign(),
};

WElement wifiContents[] = {
    W1("WiFi setup"),
    WCheckbox("WiFi enabled", "enabled",
        []() { return (wifiEnabled =
                   preferences.getBool(PREFERENCE_WIFI_ENABLED, WIFI_ENABLED)); },
        [](bool val) { wifiEnabled = val; }),
    WCheckbox("Access point mode", "apmode",
        []() { return (wifiAP =
                   preferences.getBool(PREFERENCE_WIFI_AP, WIFI_ACCESS_POINT)); },
        [](bool val) { wifiAP = val; }),
    WTextField("Network name:", "wifi",
        []()->String { return (wifiSSID =
                   preferences.getString(PREFERENCE_WIFI_SSID, WIFI_AP_NAME)); },
        [](String val) { wifiSSID = val; }),
    WPassword("Password:", "password",
        []()->String { return (wifiPass =
                   preferences.getString(PREFERENCE_WIFI_PASS, WIFI_AP_PASSPHRASE)); },
        [](String val) { wifiPass = val; }),
    WButton("Save & reboot", "save", []() {
        preferences.putBool(PREFERENCE_WIFI_ENABLED, wifiEnabled);
        preferences.putBool(PREFERENCE_WIFI_AP, wifiAP);
        preferences.putString(PREFERENCE_WIFI_SSID, wifiSSID);
        preferences.putString(PREFERENCE_WIFI_PASS, wifiPass);
        reboot();
    }),
    WHorizontalAlign(),
    WButton("Back", "back", "/setup"),
    WVerticalAlign(),
};

WElement remoteContents[] = {
    W1("Droid remote setup"),
    WCheckbox("Droid remote enabled", "remoteenabled",
        []() { return remoteEnabled; },
        [](bool val) { remoteEnabled = val; }),
    WHR(),
    WTextField("Device name:", "hostname",
        []()->String { return (remoteHostName =
                   preferences.getString(PREFERENCE_REMOTE_HOSTNAME, SMQ_HOSTNAME)); },
        [](String val) { remoteHostName = val; }),
    WPassword("Secret:", "secret",
        []()->String { return (remoteSecret =
                   preferences.getString(PREFERENCE_REMOTE_SECRET, SMQ_SECRET)); },
        [](String val) { remoteSecret = val; }),
    WButton("Save & reboot", "save", []() {
        preferences.putBool(PREFERENCE_REMOTE_ENABLED, remoteEnabled);
        preferences.putString(PREFERENCE_REMOTE_HOSTNAME, remoteHostName);
        preferences.putString(PREFERENCE_REMOTE_SECRET, remoteSecret);
        reboot();
    }),
    WHorizontalAlign(),
    WButton("Back", "back", "/setup"),
    WVerticalAlign(),
};

WElement firmwareContents[] = {
    W1("Firmware update"),
    WFirmwareFile("Firmware file:", "firmware"),
    WFirmwareUpload("Flash firmware", "firmware"),
    WLabel("Current build date:", "label"),
    WLabel(__DATE__, "date"),
#ifdef BUILD_VERSION
    WHRef(BUILD_VERSION, "Sources"),
#endif
    WButton("Back", "back", "/setup"),
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
    WPage("/marcduino",  marcduinoContents,  SizeOfArray(marcduinoContents)),
    WPage("/parameters", parametersContents, SizeOfArray(parametersContents)),
    WPage("/wifi",       wifiContents,       SizeOfArray(wifiContents)),
    WPage("/remote",     remoteContents,     SizeOfArray(remoteContents)),
    WPage("/firmware",   firmwareContents,   SizeOfArray(firmwareContents)),

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
            // Brief delay to let any in-flight motion loops see sWebAbort and exit,
            // then clear the flag so the next user command isn't eaten by serialAbort().
            delay(50);
            sWebAbort = false;
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
            out.print("{\"version\":\"" FIRMWARE_VERSION "\"");
            out.print(",\"height\":");    out.print(lifter.getLifterHeightPercent());
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
