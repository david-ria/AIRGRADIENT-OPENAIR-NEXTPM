/// <reference path="../pb_data/types.d.ts" />
// Station LAN IP, reported each POST — lets fleet ops reach a station's local
// dashboard / re-provisioning endpoints (/settoken, /setwifi) without a scan.
migrate((app) => {
  const c = app.findCollectionByNameOrId("readings");
  if (!c.fields.getByName("ip")) c.fields.add(new Field({ name: "ip", type: "text", max: 45 }));
  app.save(c);
}, (app) => {
  const c = app.findCollectionByNameOrId("readings");
  const f = c.fields.getByName("ip");
  if (f) c.fields.removeById(f.id);
  app.save(c);
});
