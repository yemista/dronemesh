# Welcome to DroneMesh!

DroneMesh is a fork of [Betaflight](https://github.com/betaflight/betaflight) that adds
**fleet (swarm) control**: flying a group of drones together as if they were one, using a
single pilot's transmitter.

> ⚠️ **Work in progress.** The architecture below describes the intended design. Much of it
> is still being built, so details will change as the project evolves. This document is meant
> to capture the concept and will be iterated on.

## The core idea

One drone in the fleet acts as the **leader**. The pilot flies the leader normally with their
transmitter. The leader takes the RC input it receives and **broadcasts it to the rest of the
fleet**, so every drone responds together.

If the leader goes down, the fleet doesn't fall out of the sky — it enters an **election** to
choose a new leader. The newly elected leader then starts acting on RC input and controlling
the fleet, and flying continues.

## Roles

Every drone runs the same firmware and can play either role:

- **Leader** — the single drone currently in command. It acts on incoming RC input and
  broadcasts it to the fleet.
- **Follower** — every other drone. Followers ignore their own RC input and instead fly
  according to what the leader broadcasts over the mesh.

Roles are not fixed: any follower can be promoted to leader through an election.

## How it works

```
        Pilot's transmitter
                │  (RC link)
                ▼
        ┌───────────────┐        mesh broadcast        ┌───────────────┐
        │    LEADER     │ ───────────────────────────► │   FOLLOWER    │
        │ acts on RC +  │ ───────────────────────────► │ applies what  │
        │ broadcasts it │ ───────────────────────────► │ leader sends  │
        └───────────────┘                              └───────────────┘
```

1. **Every drone is bound to the pilot's transmitter** and can receive RC input — but only the
   leader acts on it. This is what makes failover possible: any drone is capable of taking
   command on a moment's notice.
2. **The leader receives RC input and broadcasts it** to the fleet over the mesh link.
3. **Followers apply the broadcast** instead of their own RC, so the fleet moves as one.
4. **If the leader dies, the fleet holds an election** and promotes a new leader, which then
   begins acting on RC input and broadcasting to the others.

### What gets broadcast

Initially the leader forwards its **raw RC inputs** (the unprocessed stick/channel values),
so the fleet simply **mirrors** the leader's movements. The design leaves room to evolve
toward **per-drone setpoints** later (e.g. formation offsets or individual commands) without
changing the overall leader/follower model.

### Leader election & failover

Followers expect the leader to be alive. When the leader stops being heard from, the fleet
transitions into an **election** and selects a new leader, which immediately takes over
control. The exact mechanism for detecting failure and choosing the successor is still being
designed and will be documented here as it solidifies.

## The mesh link (transport)

The medium that carries traffic between drones is deliberately **abstracted** for now. The
firmware talks to a pluggable "link" interface, and the concrete transport (a radio mesh,
DroneCAN, or otherwise) is filled in behind it. This keeps the fleet logic independent of the
hardware choice while it's being figured out.

As a first step toward this, the firmware already has a dedicated relay path that snapshots the
**raw, unprocessed RC values** as they arrive and hands them to a (currently stubbed)
transmitter — the seam where a real mesh link will plug in.

## Relationship to Betaflight

DroneMesh keeps Betaflight's flight-control core intact and layers the fleet behavior on top.
Normal single-drone flight still works as usual; the leader/follower and mesh features are
additive. Upstream Betaflight documentation still applies for everything flight-related.

## Status & roadmap

- [x] Raw RC relay seam (snapshot raw RC, hand off to a pluggable transmitter)
- [ ] Concrete mesh transport
- [ ] Leader → follower broadcast of RC input
- [ ] Followers applying broadcast input instead of local RC
- [ ] Leader heartbeat / failure detection
- [ ] Leader election & failover
- [ ] (Later) per-drone setpoints / formation control
