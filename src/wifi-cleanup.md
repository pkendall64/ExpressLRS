# WiFi cleanup plan

## Recommendation
Keep this as an HTTP cleanup pass. Do **not** add websockets in the first round.

Why:
- The only obvious push-style flow today is SSID scanning: `lib/WIFI/devWIFI.cpp:690-723` and `html/src/pages/wifi-panel.js:183-205`.
- There is no existing websocket usage in the firmware or web UI.
- Adding a socket layer on the MCU buys little if we still only need one short-lived scan result.

## What I found

### Firmware side
- `lib/WIFI/devWIFI.cpp` is doing several jobs in one file: captive portal/static assets, config JSON assembly, WiFi network actions, scan polling, OTA/update handlers, and service startup.
- WiFi credentials are copied with unbounded `strcpy()` in two places:
  - `lib/WIFI/devWIFI.cpp:755-756`
  - `lib/WIFI/devWIFI.cpp:1059-1060`
- `/networks.json` has a special contract today:
  - `204` while a scan is still pending
  - `200` with a raw JSON array string once ready
  - manual JSON string building with `String` + `std::set<String>` at `lib/WIFI/devWIFI.cpp:695-707`
- `/config` GET already returns the main app state envelope `{settings, options, config}` and is used by the app boot path in `html/src/app.js:163-170`.

### HTML side
- The WiFi page cannot use the shared `loadJSON()` helper because `/networks.json` returns `204` with no JSON body while scanning, so it falls back to manual `XMLHttpRequest` + `JSON.parse()` in `html/src/pages/wifi-panel.js:183-205`.
- Response handling is inconsistent across the UI:
  - shared `fetch(...).json()` helper in `html/src/utils/feedback.js:48-55`
  - shared generic `post()` helper in `html/src/utils/feedback.js:57-96`
  - manual `JSON.parse(xhr.responseText)` in `wifi-panel.js`, `update-panel.js`, `lr1121-updater.js`, and `voltage-calibration-panel.js`
- The dev mock mirrors the current special cases, so any API cleanup needs the mock updated too: `html/dev-plugins/dev-mock-plugin.js:240-340`.

## Proposed scope

### Phase 1 — low-risk cleanup in `devWIFI.cpp`
1. Replace both `strcpy()` credential copies with bounded copies.
2. Add small local helpers inside `devWIFI.cpp` for the repeated WiFi response paths instead of inventing a new module.
3. Keep route names stable. No URL churn.
4. Keep the existing `/config` envelope. Too many pages consume it to justify a broader schema rewrite.

### Phase 2 — make network scan JSON boring
Change `/networks.json` from a transport/state hybrid into one consistent JSON response.

Recommended response shape:

```json
{
  "status": "scanning",
  "networks": []
}
```

and when ready:

```json
{
  "status": "ready",
  "networks": ["ExpressLRS TX", "MockHomeWiFi"]
}
```

Why this is worth doing:
- removes the `204` special case
- lets the UI use shared JSON helpers
- makes the mock and firmware contract explicit
- keeps polling simple; no socket state machine needed

### Phase 3 — trim the HTML request handling
Touch only the WiFi flow and the shared transport helpers it already depends on.

Planned changes:
1. Update `html/src/pages/wifi-panel.js` to use the normalized `/networks.json` response.
2. Move the WiFi scan request onto shared helper code instead of raw `XMLHttpRequest` parsing.
3. If the shared helper needs one small extension, add it in `html/src/utils/feedback.js` rather than creating a new client layer.
4. Update `html/dev-plugins/dev-mock-plugin.js` to match the new scan contract.

## Optional follow-up, not phase 1
If we want one more cleanup pass after the scan contract lands, normalize mutating endpoints to return one JSON ack shape like:

```json
{
  "status": "ok",
  "msg": "..."
}
```

Candidates:
- `/sethome`
- `/forget`
- `/connect`
- `/access`
- `/config` POST
- `/options.json` POST

I would **not** mix this into the first pass unless the helper rewrite is already touching those paths. It is cleanup, not required to simplify WiFi scanning.

## Websocket decision
Do not switch to websockets now.

Use websockets only if we decide to add one of these later:
- live scan progress instead of simple polling
- live OTA/update status pushed from firmware
- ongoing device telemetry on the page

Without one of those, websockets add server/client lifecycle code and failure modes for little gain.

## Files I expect this plan to touch if approved
- `lib/WIFI/devWIFI.cpp`
- `html/src/pages/wifi-panel.js`
- `html/src/utils/feedback.js`
- `html/dev-plugins/dev-mock-plugin.js`
- maybe `html/src/utils/state.js` only if a tiny shared refresh helper actually removes duplicated WiFi state patching

## Non-goals
- no new websocket transport
- no large API redesign beyond the scan response
- no splitting `devWIFI.cpp` into new modules unless the first pass proves a tiny extraction is still too cramped
- no unrelated HTML panel rewrites

## Verification when implemented
- firmware build for at least one ESP32 target and one ESP8285 target
- HTML syntax/lint checks
- manual WiFi page smoke test in the dev mock:
  - scan shows loader, then networks
  - set home / forget still show expected success flow
  - no regressions in initial `/config` load
