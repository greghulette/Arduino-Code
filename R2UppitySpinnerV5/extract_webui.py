#!/usr/bin/env python3
"""
extract_webui.py
Extracts the R2 Uppity Spinner web UI from WebPages.h and produces a
standalone preview.html you can open directly in a browser.

Usage:  python extract_webui.py
Output: preview.html  (in the same folder)
"""

import re, sys, os

SRC  = os.path.join(os.path.dirname(__file__), "WebPages.h")
DEST = os.path.join(os.path.dirname(__file__), "preview.html")

# ---------------------------------------------------------------------------
# Placeholder runtime values that the ESP32 would normally supply
# ---------------------------------------------------------------------------
PLACEHOLDERS = {
    # status
    "htPct":          "65",
    "rotDeg":         "0",
    "safetyOk":       "true",
    "motorsOn":       "true",
    "liftFault":      "false",
    "rotFault":       "false",
    "rotHome":        "true",
    "holding":        "false",
    "lightkit":       "0",
    "minHeightPct":   "20",
    "rssi":           "-55",
    "apClients":      "1",
    "override":       "0",
    "lastCmd":        "P100",
    # settings
    "minPwr":         "65",
    "seekBotPwr":     "40",
    "liftDist":       "1225",
    "rotMinH":        "225",
    "rotEncCnt":      "1201",
    "driftPct":       "10",
    "invertMotor":    "1",
    "invertEncoder":  "0",
    "disableRotary":  "0",
    "upCalib":        "1",
    "ssid":           "R2Uppity-833C",
    "wifiEnabled":    "true",
    "apMode":         "true",
    "marcEnabled":    "false",
}

# ---------------------------------------------------------------------------
# Read the source
# ---------------------------------------------------------------------------
with open(SRC, "r", encoding="utf-8", errors="replace") as f:
    src = f.read()

# ---------------------------------------------------------------------------
# Strip C++ line comments but keep the content inside strings intact.
# Simple approach: only strip // comments that appear outside quotes.
# ---------------------------------------------------------------------------
def strip_line_comments(text):
    result = []
    for line in text.splitlines():
        # remove // comment that's not inside a string (good enough for this file)
        in_str = False
        i = 0
        out = []
        while i < len(line):
            c = line[i]
            if c == '"' and (i == 0 or line[i-1] != '\\'):
                in_str = not in_str
            if not in_str and c == '/' and i+1 < len(line) and line[i+1] == '/':
                break
            out.append(c)
            i += 1
        result.append(''.join(out))
    return '\n'.join(result)

src = strip_line_comments(src)

# ---------------------------------------------------------------------------
# Extract every string that appears in an out.print(...) call.
# Handles:
#   out.print("normal string");
#   out.print("multi"
#             "line"
#             "concat");
#   out.print(R"RAW(raw content)RAW");
# Skips lines that pass a C++ variable (e.g. out.print(someVar)).
# ---------------------------------------------------------------------------

# Collapse the entire file into one long string for easier regex work
flat = src.replace('\n', ' ')

html_parts = []

# Pattern: out.print( ... );  — capture the argument(s)
# We'll walk the file character by character to find each out.print(...)
# block, then parse its argument.

pos = 0
L = len(flat)

def skip_whitespace(s, p):
    while p < len(s) and s[p] in ' \t\r\n':
        p += 1
    return p

def read_string_literal(s, p):
    """Read one C string literal starting at p (which must be '"').
    Returns (content, new_pos) or (None, p) if not a string."""
    if p >= len(s) or s[p] != '"':
        return None, p
    p += 1  # skip opening "
    content = []
    while p < len(s):
        c = s[p]
        if c == '\\':
            p += 1
            if p < len(s):
                esc = s[p]
                escmap = {'n':'\n','t':'\t','r':'\r','"':'"','\\':'\\','\'':'\''}
                content.append(escmap.get(esc, esc))
            p += 1
        elif c == '"':
            p += 1  # skip closing "
            return ''.join(content), p
        else:
            content.append(c)
            p += 1
    return None, p  # unterminated

def read_raw_string(s, p):
    """Read a R"RAW(...)RAW" literal. p points to 'R'.
    Returns (content, new_pos) or (None, p)."""
    if s[p:p+2] != 'R"':
        return None, p
    # find the delimiter
    q = p + 2
    delim_end = s.find('(', q)
    if delim_end < 0:
        return None, p
    delim = s[q:delim_end]         # e.g. "RAW"
    close = ')' + delim + '"'
    content_start = delim_end + 1
    close_pos = s.find(close, content_start)
    if close_pos < 0:
        return None, p
    content = s[content_start:close_pos]
    return content, close_pos + len(close)

def read_print_arg(s, p):
    """
    p is just after the '(' of out.print(.
    Read one or more string/raw-string literals (possibly concatenated)
    until we hit ')' or a non-string token.
    Returns (combined_string_or_None, new_pos_after_closing_paren).
    """
    parts = []
    while True:
        p = skip_whitespace(s, p)
        if p >= len(s):
            break
        c = s[p]
        if c == '"':
            txt, p = read_string_literal(s, p)
            if txt is None:
                return None, p
            parts.append(txt)
        elif c == 'R' and p+1 < len(s) and s[p+1] == '"':
            txt, p = read_raw_string(s, p)
            if txt is None:
                return None, p
            parts.append(txt)
        elif c == ')':
            p += 1  # consume ')'
            break
        else:
            # It's a variable reference or something else — skip to closing ')'
            depth = 1
            while p < len(s) and depth > 0:
                if s[p] == '(':
                    depth += 1
                elif s[p] == ')':
                    depth -= 1
                p += 1
            return None, p
    if parts:
        return ''.join(parts), p
    return None, p

# Walk flat looking for "out.print("
search_token = "out.print("
p = 0
while True:
    idx = flat.find(search_token, p)
    if idx < 0:
        break
    p = idx + len(search_token)
    txt, p = read_print_arg(flat, p)
    if txt is not None:
        html_parts.append(txt)

# ---------------------------------------------------------------------------
# Also grab #define string macros (TAB_ICON_*, etc.) used inline
# These are expanded by the C preprocessor; we just inline them here.
# ---------------------------------------------------------------------------
define_map = {}
for m in re.finditer(r'#define\s+(\w+)\s+\\?\s*("(?:[^"\\]|\\.)*"(?:\s*\\\s*\n\s*"(?:[^"\\]|\\.)*")*)', src):
    name = m.group(1)
    raw  = m.group(2)
    # join continuation lines
    raw = re.sub(r'"\s*\\\s*\n\s*"', '', raw)
    raw = raw.strip('"')
    define_map[name] = raw

# ---------------------------------------------------------------------------
# Combine and inject a mock /api/status so the JavaScript live-update works
# ---------------------------------------------------------------------------
combined = ''.join(html_parts)

# Build a fake status JSON
fake_status = (
    '{"htPct":65,"rotDeg":0,"safetyOk":true,"motorsOn":true,'
    '"liftFault":false,"rotFault":false,"rotHome":true,"holding":false,'
    '"lightkit":0,"minHeightPct":20,"rssi":-55,"apClients":1,'
    '"override":0,"lastCmd":"P100"}'
)

# Inject a <base> tag so relative links don't break, and a mock fetch shim
mock_shim = """
<script>
// ── Preview shim: intercepts API calls so the UI renders without hardware ──
(function(){
  var _realFetch = window.fetch;
  window.fetch = function(url, opts) {
    if (typeof url === 'string' && url.indexOf('/api/status') !== -1) {
      var body = JSON.stringify(""" + fake_status + """);
      return Promise.resolve(new Response(body, {status:200,
        headers:{'Content-Type':'application/json'}}));
    }
    if (typeof url === 'string' && url.indexOf('/api/log') !== -1) {
      return Promise.resolve(new Response('1\\tPreview mode — no hardware\\n',
        {status:200, headers:{'Content-Type':'text/plain'}}));
    }
    // For all other fetches (commands etc.) just silently swallow
    return Promise.resolve(new Response('', {status:200}));
  };
  // Also patch XMLHttpRequest just in case
  var _XHR = window.XMLHttpRequest;
  window.XMLHttpRequest = function() {
    var xhr = new _XHR();
    var _open = xhr.open.bind(xhr);
    xhr.open = function(method, url) {
      if (typeof url === 'string' && url.indexOf('/api') !== -1) {
        this._mocked = url;
      }
      return _open.apply(this, arguments);
    };
    return xhr;
  };
})();
</script>
"""

# Insert shim right after <head> (before any scripts run)
if '<head>' in combined:
    combined = combined.replace('<head>', '<head>' + mock_shim, 1)
elif '<!DOCTYPE' in combined.lower():
    combined = combined.replace('<!DOCTYPE html>', '<!DOCTYPE html>' + mock_shim, 1)
else:
    combined = mock_shim + combined

# ---------------------------------------------------------------------------
# Truncate at </html> — API endpoint handlers also use out.print() and
# their JSON would otherwise leak into the file after the page closes.
# ---------------------------------------------------------------------------
end_tag = '</html>'
end_idx = combined.find(end_tag)
if end_idx != -1:
    combined = combined[:end_idx + len(end_tag)]

# ---------------------------------------------------------------------------
# Fix tab-bar links: the C++ loop uses variables for href/icon/label so the
# extractor emits blank values.  Inject them as hardcoded values instead.
# ---------------------------------------------------------------------------
# Replace the blank tab anchor
tab_svg_periscope = ("<svg width='18' height='18' viewBox='0 0 18 18' fill='none' "
    "stroke-width='1.6' stroke-linecap='round'>"
    "<rect x='2' y='10' width='14' height='6' rx='2'/>"
    "<rect x='6' y='2' width='6' height='9' rx='2'/></svg>")
tab_svg_rescue = ("<svg width='18' height='18' viewBox='0 0 18 18' fill='none' "
    "stroke-width='1.7' stroke-linecap='round'>"
    "<circle cx='9' cy='9' r='6.5'/><line x1='9' y1='5' x2='9' y2='9.5'/>"
    "<line x1='9' y1='12' x2='9' y2='13'/></svg>")
tab_svg_log = ("<svg width='18' height='18' viewBox='0 0 18 18' fill='none' "
    "stroke-width='1.6' stroke-linecap='round'>"
    "<rect x='2' y='2' width='14' height='14' rx='3'/>"
    "<line x1='5' y1='6.5' x2='13' y2='6.5'/>"
    "<line x1='5' y1='9.5' x2='10' y2='9.5'/>"
    "<line x1='5' y1='12.5' x2='8' y2='12.5'/></svg>")
tab_svg_setup = ("<svg width='18' height='18' viewBox='0 0 18 18' fill='none' "
    "stroke-width='1.6' stroke-linecap='round'>"
    "<circle cx='9' cy='9' r='2.5'/>"
    "<path d='M9 1.5v3M9 13.5v3M1.5 9h3M13.5 9h3"
    "M3.9 3.9l2.12 2.12M12 12l2.1 2.1M3.9 14.1l2.12-2.12M12 6l2.1-2.1'/></svg>")

fixed_tabs = (
    "<nav id='tabs'>"
    f"<a class='tab active' href='#periscope'>"
      f"<div class='tab-ic'>{tab_svg_periscope}</div>"
      "<div class='tab-lbl'>periscope</div></a>"
    f"<a class='tab' href='#rescue'>"
      f"<div class='tab-ic'>{tab_svg_rescue}</div>"
      "<div class='tab-lbl'>rescue</div></a>"
    f"<a class='tab' href='#log'>"
      f"<div class='tab-ic'>{tab_svg_log}</div>"
      "<div class='tab-lbl'>log</div></a>"
    f"<a class='tab' href='#setup'>"
      f"<div class='tab-ic'>{tab_svg_setup}</div>"
      "<div class='tab-lbl'>setup</div></a>"
    "</nav>"
)

# Replace the broken nav block (has empty hrefs)
combined = re.sub(
    r"<nav id='tabs'>.*?</nav>",
    fixed_tabs,
    combined,
    count=1,
    flags=re.DOTALL
)

# ---------------------------------------------------------------------------
# Write output
# ---------------------------------------------------------------------------
with open(DEST, "w", encoding="utf-8") as f:
    f.write(combined)

print(f"Written {len(combined):,} bytes -> {DEST}")
print("Open preview.html in your browser.")
