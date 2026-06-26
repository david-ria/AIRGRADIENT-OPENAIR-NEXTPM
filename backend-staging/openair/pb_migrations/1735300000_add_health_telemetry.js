/// <reference path="../pb_data/types.d.ts" />
// Health telemetry from the firmware reliability bundle (Phase 2a): uptime,
// free heap, offline-buffer depth, lifetime POST-failure count, last reset cause.
migrate((app) => {
  const c = app.findCollectionByNameOrId("readings");
  const fields = [
    { name: "uptime_s",     type: "number" },
    { name: "heap_free",    type: "number" },
    { name: "outbox_depth", type: "number" },
    { name: "post_fail",    type: "number" },
    { name: "reset_reason", type: "text", max: 32 },
  ];
  for (const f of fields) if (!c.fields.getByName(f.name)) c.fields.add(new Field(f));
  app.save(c);
}, (app) => {
  const c = app.findCollectionByNameOrId("readings");
  for (const n of ["uptime_s", "heap_free", "outbox_depth", "post_fail", "reset_reason"]) {
    const f = c.fields.getByName(n);
    if (f) c.fields.removeById(f.id);
  }
  app.save(c);
});
