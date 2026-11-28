# botsim-replay

Records what the [fleet server](../server/README.md) decides on, and plays it back.
When a fleet behaves wrongly, the file of its inputs is enough to make the same
decisions again at a desk: no simulator, scenario, layout or factsheet file is
needed.

```sh
cargo run --release -p botsim -- --record-inputs run.inputs scenarios/ring_jobs.toml
cargo run --release -p botsim -- --replay run.inputs
```

(Not to be confused with [botsim-record](../record/README.md), the recording of what
happened in a simulated run, for the viewer.)

## What a recording holds

| Part | |
|---|---|
| header | the format number and the botsim version that wrote it |
| setup | the layout (LIF text), the fleet, zones, lifts, doors, the start time; the traffic rules (the baseline's, or the traffic module with the robot types and conflict map settings it is made from) and the dispatcher; every set-up call (route weights, charging, conflict lists) |
| every cycle | the time and the inbox (state reports, lift states, new jobs, whether somebody watches); a checksum of what came out and one of the state left |
| end | the number of cycles; a file without it was cut off |

A report is written as what changed since the robot's report before. Numbers with a
fraction come back bit for bit. Every part has a check value, so a changed byte is
found.

## Using it

| | |
|---|---|
| Record | make the controller with `controller(&setup)`, then `Recorder::new(output, &setup)`, `recorder.step(&mut controller, now, inbox)` for each cycle, `recorder.finish()` |
| Replay | `replay(input)` gives how many cycles it compared, the first that differs, and what broke the reading, if anything did |
| Read or rewrite | `Reader` (setup, then cycle by cycle), `Writer` |

## What a replay promises

- The same decisions **with the same build on the same kind of machine**. A file of
  another botsim version is replayed all the same; a difference then says no more
  than that the two differ. (Builds of one version cannot be told apart.)
- It **stops at the first cycle that differs** and says whether the messages, the
  state or both do.
- The **messages** checksum covers the whole outbox. The **state** checksum covers
  everything the controller, its dispatcher and its traffic rules keep that can
  change, not the fixed tables made at the start. An input that changes nothing that
  is kept or sent (a report older than the one kept, say) is not seen, since the run
  is the same with it. The state checksum changes whenever the server's inner types
  do; the messages one only with the message types.
- Only the traffic rules and the dispatcher that botsim has can be recorded.
- A file that is damaged, cut off or something else gives an error, not a panic. The
  cycles before a break are replayed.

## Format

Own binary format, number 1; see the documentation of the `file` module.
