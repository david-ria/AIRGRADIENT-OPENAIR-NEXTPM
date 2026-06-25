/// <reference path="../pb_data/types.d.ts" />
// Initial schema for the OpenAir AirSentinels backend.
// Three collections:
//   - sites     : où la station est installée (école, mairie, etc.)
//   - devices   : une station OpenAir = un device_serial unique (la MAC)
//   - readings  : ingestion brute push depuis le firmware, 30 s typique
migrate((app) => {
  const sites = new Collection({
    type: "base",
    name: "sites",
    listRule: "",
    viewRule: "",
    createRule: null,
    updateRule: null,
    deleteRule: null,
    fields: [
      { name: "name",     type: "text", required: true, max: 128 },
      { name: "address",  type: "text", max: 256 },
      { name: "city",     type: "text", max: 128 },
      { name: "lat",      type: "number" },
      { name: "lon",      type: "number" },
      { name: "notes",    type: "text", max: 1024 },
      { name: "active",   type: "bool" },
      { name: "created",  type: "autodate", onCreate: true },
      { name: "updated",  type: "autodate", onCreate: true, onUpdate: true },
    ],
    indexes: [
      "CREATE UNIQUE INDEX idx_sites_name ON sites (name)",
    ],
  });
  app.save(sites);

  const devices = new Collection({
    type: "base",
    name: "devices",
    listRule: "",
    viewRule: "",
    createRule: null,
    updateRule: null,
    deleteRule: null,
    fields: [
      { name: "device_serial", type: "text", required: true, max: 64 },  // MAC lowercase, no colons
      { name: "site",          type: "relation", collectionId: sites.id, cascadeDelete: false, maxSelect: 1 },
      { name: "name",          type: "text", max: 128 },
      { name: "location_label",type: "text", max: 128 },  // libellé fin (cour primaire, façade rue, etc.)
      { name: "hw_revision",   type: "text", max: 64 },
      { name: "fw_version",    type: "text", max: 64 },
      { name: "active",        type: "bool" },
      { name: "last_seen",     type: "date" },
      { name: "created",       type: "autodate", onCreate: true },
      { name: "updated",       type: "autodate", onCreate: true, onUpdate: true },
    ],
    indexes: [
      "CREATE UNIQUE INDEX idx_devices_device_serial ON devices (device_serial)",
      "CREATE INDEX idx_devices_site ON devices (site)",
    ],
  });
  app.save(devices);

  const readings = new Collection({
    type: "base",
    name: "readings",
    listRule: "",
    viewRule: "",
    // createRule géré côté hook (vérification token + résolution device_serial → device)
    createRule: null,
    updateRule: null,
    deleteRule: null,
    fields: [
      { name: "device", type: "relation", required: true, collectionId: devices.id, cascadeDelete: true, maxSelect: 1 },
      { name: "ts", type: "date", required: true },

      // PM primary mirror (= la fenêtre actuellement sélectionnée par le firmware)
      { name: "pm1",         type: "number" },
      { name: "pm25",        type: "number" },
      { name: "pm10",        type: "number" },
      { name: "pm003_dL",    type: "number" },
      { name: "cntPM1_dL",   type: "number" },
      { name: "cntPM25_dL",  type: "number" },
      { name: "cntPM10_dL",  type: "number" },

      // Modbus binned counts (souvent vides — voir code firmware, conservés pour futurs FW)
      { name: "c02_05",  type: "number" },
      { name: "c05_10",  type: "number" },
      { name: "c10_25",  type: "number" },
      { name: "c25_50",  type: "number" },
      { name: "c50_100", type: "number" },

      // NextPM native moving averages (10s / 60s / 15m)
      { name: "pm_10s_ok",          type: "bool" },
      { name: "pm_10s_pm1",         type: "number" },
      { name: "pm_10s_pm25",        type: "number" },
      { name: "pm_10s_pm10",        type: "number" },
      { name: "pm_10s_cntPM1_dL",   type: "number" },
      { name: "pm_10s_cntPM25_dL",  type: "number" },
      { name: "pm_10s_cntPM10_dL",  type: "number" },
      { name: "pm_60s_ok",          type: "bool" },
      { name: "pm_60s_pm1",         type: "number" },
      { name: "pm_60s_pm25",        type: "number" },
      { name: "pm_60s_pm10",        type: "number" },
      { name: "pm_60s_cntPM1_dL",   type: "number" },
      { name: "pm_60s_cntPM25_dL",  type: "number" },
      { name: "pm_60s_cntPM10_dL",  type: "number" },
      { name: "pm_15m_ok",          type: "bool" },
      { name: "pm_15m_pm1",         type: "number" },
      { name: "pm_15m_pm25",        type: "number" },
      { name: "pm_15m_pm10",        type: "number" },
      { name: "pm_15m_cntPM1_dL",   type: "number" },
      { name: "pm_15m_cntPM25_dL",  type: "number" },
      { name: "pm_15m_cntPM10_dL",  type: "number" },

      // Gas
      { name: "co2",            type: "number" },
      { name: "tvoc_index",     type: "number" },
      { name: "nox_index",      type: "number" },
      { name: "tvoc_index_avg", type: "number" },
      { name: "nox_index_avg",  type: "number" },
      { name: "tvoc_raw",       type: "number" },
      { name: "nox_raw",        type: "number" },

      // Env + telemetry
      { name: "atmp",            type: "number" },
      { name: "rhum",            type: "number" },
      { name: "rssi",            type: "number" },
      { name: "data_age_sec",    type: "number" },
      { name: "sensor_ok",       type: "bool" },
      { name: "postAvgSec",      type: "number" },
      { name: "sgpConditioning", type: "bool" },

      { name: "created", type: "autodate", onCreate: true },
    ],
    indexes: [
      "CREATE INDEX idx_readings_device_ts ON readings (device, ts)",
      "CREATE INDEX idx_readings_ts ON readings (ts)",
    ],
  });
  app.save(readings);
}, (app) => {
  for (const name of ["readings", "devices", "sites"]) {
    const c = app.findCollectionByNameOrId(name);
    if (c) app.delete(c);
  }
});
