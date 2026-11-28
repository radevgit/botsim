# Test data

## VDA 5050 schemas

`order.schema`, `state.schema`, `instantActions.schema`, `responses.schema` and
`zoneSet.schema` are the official JSON schemas of VDA 5050 3.0.1, unchanged.

- Source: <https://github.com/VDA5050/VDA5050>, folder `json_schemas/`
- Branch `dev/3.0.1`, commit `9176e22191cbd9f4b432fdb322f91168282b91c8`
  (2026-11-11). Version 3.0.1 is in development there and not yet tagged (checked
  2026-11-27); copy again when it is published.
- Copied from the `vda5050` crate 0.1.0, which carries them (`spec/schemas/`).
- Licence: MIT, Copyright 2026 Verband der Automobilindustrie, see
  `LICENSE-VDA5050`. It applies to the five schema files.
- `botsim-sim`'s tests check their state messages against `state.schema` here.

## Layout

`three_nodes.lif.json` is a copy of the layout of the same name in
`botsim-sim`'s tests.
