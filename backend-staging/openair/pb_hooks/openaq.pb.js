/// <reference path="../pb_data/types.d.ts" />
//
// OpenAQ-format read API. Exposes the OpenAir readings in the OpenAQ measurement
// shape so ExpoTrack (or any OpenAQ-aware consumer) can ingest the stations
// through a stable, standard contract — decoupled from the internal PocketBase
// schema. Read-only, curated fields only (no device IP / heap / reset telemetry).
//
//   GET /api/openaq/locations
//   GET /api/openaq/measurements?location=<serial>&parameter=<p>&date_from=<iso>&date_to=<iso>&limit=<n>&page=<n>
//
// NOTE: PocketBase JSVM runs each routerAdd handler in an isolated context —
// module-level consts/functions are NOT visible inside. Everything must be
// declared inside the handler, hence the duplicated helpers below.

routerAdd("GET", "/api/openaq/locations", (e) => {
  try {
    try { e.response.header().set("Access-Control-Allow-Origin", "*"); } catch (_) {}
    const isoUtc = (s) => (s ? String(s).replace(" ", "T") : null);
    const PARAMS = ["pm1","pm25","pm10","pm1_number","pm25_number","pm10_number",
      "pm_bin_0.3_0.5","pm_bin_0.5_1","pm_bin_1_2.5","pm_bin_2.5_5","pm_bin_5_10",
      "co2","vocindex","noxindex","temperature","relativehumidity"];
    const siteCoords = (device) => {
      const sid = device.getString("site");
      if (!sid) return null;
      try {
        const s = $app.findRecordById("sites", sid);
        const lat = s.getFloat("lat"), lon = s.getFloat("lon");
        if (lat !== 0 || lon !== 0) return { latitude: lat, longitude: lon };
      } catch (_) {}
      return null;
    };

    const devices = $app.findRecordsByFilter("devices", "id != ''", "-last_seen", 200, 0);
    const results = devices.map((d) => ({
      id: d.getString("device_serial"),
      name: d.getString("name") || d.getString("device_serial"),
      locationId: d.getString("device_serial"),
      coordinates: siteCoords(d),
      lastUpdated: isoUtc(d.getString("last_seen")),
      firmwareVersion: d.getString("fw_version"),
      parameters: PARAMS,
      sensorType: "low-cost sensor",
      entity: "community",
      country: "FR",
    }));
    return e.json(200, { meta: { name: "openaq-airsentinels", found: results.length }, results });
  } catch (err) {
    return e.json(500, { error: "locations", detail: String(err) });
  }
});

routerAdd("GET", "/api/openaq/measurements", (e) => {
  try {
    try { e.response.header().set("Access-Control-Allow-Origin", "*"); } catch (_) {}
    const isoUtc = (s) => (s ? String(s).replace(" ", "T") : null);
    const PARAMS = [
      { f: "pm1",        p: "pm1",              u: "ug/m3",        z: false },
      { f: "pm25",       p: "pm25",             u: "ug/m3",        z: false },
      { f: "pm10",       p: "pm10",             u: "ug/m3",        z: false },
      { f: "cntPM1_dL",  p: "pm1_number",       u: "particles/mL", z: false },
      { f: "cntPM25_dL", p: "pm25_number",      u: "particles/mL", z: false },
      { f: "cntPM10_dL", p: "pm10_number",      u: "particles/mL", z: false },
      { f: "c02_05",     p: "pm_bin_0.3_0.5",   u: "particles/L",  z: true },
      { f: "c05_10",     p: "pm_bin_0.5_1",     u: "particles/L",  z: true },
      { f: "c10_25",     p: "pm_bin_1_2.5",     u: "particles/L",  z: true },
      { f: "c25_50",     p: "pm_bin_2.5_5",     u: "particles/L",  z: true },
      { f: "c50_100",    p: "pm_bin_5_10",      u: "particles/L",  z: true },
      { f: "co2",        p: "co2",              u: "ppm",          z: true },
      { f: "tvoc_index", p: "vocindex",         u: "index",        z: true },
      { f: "nox_index",  p: "noxindex",         u: "index",        z: true },
      { f: "atmp",       p: "temperature",      u: "c",            z: true },
      { f: "rhum",       p: "relativehumidity", u: "%",            z: true },
    ];
    const siteCoords = (device) => {
      const sid = device.getString("site");
      if (!sid) return null;
      try {
        const s = $app.findRecordById("sites", sid);
        const lat = s.getFloat("lat"), lon = s.getFloat("lon");
        if (lat !== 0 || lon !== 0) return { latitude: lat, longitude: lon };
      } catch (_) {}
      return null;
    };

    const q = e.request.url.query();
    const loc = q.get("location");
    const param = q.get("parameter");
    const from = q.get("date_from");
    const to = q.get("date_to");
    let limit = parseInt(q.get("limit") || "100", 10);
    if (isNaN(limit) || limit < 1) limit = 100;
    if (limit > 500) limit = 500;
    let page = parseInt(q.get("page") || "1", 10);
    if (isNaN(page) || page < 1) page = 1;

    let device = null;
    if (loc) {
      try { device = $app.findFirstRecordByFilter("devices", "device_serial = {:s}", { s: loc }); }
      catch (_) { return e.json(404, { error: "unknown location" }); }
    }

    const conds = [];
    const params = {};
    if (device) { conds.push("device = {:dev}"); params.dev = device.id; }
    if (from) { conds.push("ts >= {:from}"); params.from = from; }
    if (to)   { conds.push("ts <= {:to}");   params.to = to; }
    const filter = conds.length ? conds.join(" && ") : "id != ''";

    const rows = $app.findRecordsByFilter("readings", filter, "-ts", limit, (page - 1) * limit, params);

    const devCache = {};
    const getDev = (id) => {
      if (id in devCache) return devCache[id];
      try { devCache[id] = $app.findRecordById("devices", id); } catch (_) { devCache[id] = null; }
      return devCache[id];
    };

    const results = [];
    for (const r of rows) {
      const dev = device || getDev(r.getString("device"));
      const serial = dev ? dev.getString("device_serial") : r.getString("device");
      const name = dev ? (dev.getString("name") || serial) : serial;
      const coords = dev ? siteCoords(dev) : null;
      const utc = isoUtc(r.getString("ts"));
      const avg = r.getFloat("postAvgSec") || null;
      for (const pm of PARAMS) {
        if (param && pm.p !== param) continue;
        const v = r.getFloat(pm.f);
        if (pm.z && v === 0) continue;
        results.push({
          locationId: serial,
          location: name,
          parameter: pm.p,
          value: v,
          unit: pm.u,
          date: { utc: utc, local: utc },
          coordinates: coords,
          country: "FR",
          averagingPeriod: avg ? { value: avg, unit: "seconds" } : null,
          sensorType: "low-cost sensor",
          entity: "community",
        });
      }
    }
    return e.json(200, {
      meta: { name: "openaq-airsentinels", page: page, limit: limit, found: results.length },
      results: results,
    });
  } catch (err) {
    return e.json(500, { error: "measurements", detail: String(err) });
  }
});
