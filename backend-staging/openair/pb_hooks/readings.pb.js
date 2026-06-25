/// <reference path="../pb_data/types.d.ts" />

routerAdd("POST", "/api/openair/ingest", (e) => {
  const AUTO_CREATE_DEVICES = true;
  try {
    const expected = $os.getenv("OPENAIR_DEVICE_TOKEN");
    if (!expected || expected.length < 8) {
      return e.json(500, { error: "server misconfigured: OPENAIR_DEVICE_TOKEN missing" });
    }
    const got = e.request.header.get("X-Device-Token");
    if (got !== expected) {
      return e.json(401, { error: "invalid device token" });
    }

    const ALLOWED_NUM = [
      "pm1","pm25","pm10","pm003_dL","cntPM1_dL","cntPM25_dL","cntPM10_dL",
      "c02_05","c05_10","c10_25","c25_50","c50_100",
      "pm_10s_pm1","pm_10s_pm25","pm_10s_pm10","pm_10s_cntPM1_dL","pm_10s_cntPM25_dL","pm_10s_cntPM10_dL",
      "pm_60s_pm1","pm_60s_pm25","pm_60s_pm10","pm_60s_cntPM1_dL","pm_60s_cntPM25_dL","pm_60s_cntPM10_dL",
      "pm_15m_pm1","pm_15m_pm25","pm_15m_pm10","pm_15m_cntPM1_dL","pm_15m_cntPM25_dL","pm_15m_cntPM10_dL",
      "co2","tvoc_index","nox_index","tvoc_index_avg","nox_index_avg","tvoc_raw","nox_raw",
      "atmp","rhum","rssi","data_age_sec","postAvgSec",
    ];
    const ALLOWED_BOOL = [
      "pm_10s_ok","pm_60s_ok","pm_15m_ok","sensor_ok","sgpConditioning",
    ];

    const shape = { device_serial: "", ts: "" };
    for (const k of ALLOWED_NUM)  shape[k] = -0.0;  // float64, not int64
    for (const k of ALLOWED_BOOL) shape[k] = false;

    const data = new DynamicModel(shape);
    e.bindBody(data);

    if (!data.device_serial) {
      return e.json(400, { error: "device_serial is required" });
    }
    // ts is optional: the firmware sends an ISO8601 UTC stamp once its NTP
    // clock is synced; before that it omits ts and we stamp server-side.
    const ts = data.ts && data.ts.length > 0 ? data.ts : new Date().toISOString();

    let device;
    try {
      device = $app.findFirstRecordByFilter(
        "devices",
        "device_serial = {:s}",
        { s: data.device_serial }
      );
    } catch (err) {
      device = null;
    }
    if (!device) {
      if (!AUTO_CREATE_DEVICES) {
        return e.json(404, { error: "unknown device_serial; pre-register it first" });
      }
      const devicesCol = $app.findCollectionByNameOrId("devices");
      device = new Record(devicesCol, {
        device_serial: data.device_serial,
        name: "Station " + data.device_serial.slice(-6),
        active: true,
      });
      $app.save(device);
    }

    const readingsCol = $app.findCollectionByNameOrId("readings");
    const reading = new Record(readingsCol);
    reading.set("device", device.id);
    reading.set("ts", ts);
    for (const k of ALLOWED_NUM)  reading.set(k, data[k]);
    for (const k of ALLOWED_BOOL) reading.set(k, data[k]);
    $app.save(reading);

    device.set("last_seen", ts);
    $app.save(device);

    return e.json(200, { id: reading.id, device: device.id });
  } catch (err) {
    return e.json(500, { error: "hook exception", detail: String(err), stack: err && err.stack ? err.stack : "" });
  }
});
