use super::*;
use thiserror::Error;

#[derive(Clone, Debug, PartialEq)]
pub struct ReplayCommand {
    pub tick: u64,
    pub stable_order: u64,
    pub command: FractureCommand,
}

#[derive(Clone, Debug, PartialEq)]
pub struct ReplayTickTrace {
    pub tick: u64,
    pub family_digest: u64,
    pub fracture_events: Vec<FractureEvent>,
    pub split_events: Vec<SplitEvent>,
}

#[derive(Error, Debug, Clone, PartialEq, Eq)]
pub enum ReplayError {
    #[error("ambiguous replay command key at tick {tick} stable order {stable_order}")]
    AmbiguousDuplicateKey { tick: u64, stable_order: u64 },
}

pub fn run_replay_ticks(
    family: &mut FxFamily,
    start_tick: u64,
    end_tick: u64,
    scheduled: &[ReplayCommand],
) -> Result<Vec<ReplayTickTrace>, ReplayError> {
    validate_replay_commands(scheduled)?;
    let mut commands = scheduled.to_vec();
    commands.sort_by(|a, b| {
        a.tick
            .cmp(&b.tick)
            .then_with(|| a.stable_order.cmp(&b.stable_order))
            .then_with(|| a.command.order_key.cmp(&b.command.order_key))
            .then_with(|| a.command.target.cmp(&b.command.target))
    });
    let mut traces = Vec::new();
    for tick in start_tick..end_tick {
        let tick_commands = commands
            .iter()
            .filter(|command| command.tick == tick)
            .map(|command| command.command.clone())
            .collect::<Vec<_>>();
        let fracture_events = apply_fracture_commands(family, &tick_commands);
        let split_events = split_dirty_actors(family);
        traces.push(ReplayTickTrace {
            tick,
            family_digest: family.deterministic_state_digest(),
            fracture_events,
            split_events,
        });
    }
    Ok(traces)
}

fn validate_replay_commands(commands: &[ReplayCommand]) -> Result<(), ReplayError> {
    let mut keyed = commands
        .iter()
        .map(|command| (replay_key(command), replay_payload(command)))
        .collect::<Vec<_>>();
    keyed.sort_by_key(|(key, _)| *key);
    for window in keyed.windows(2) {
        if window[0].0 == window[1].0 && window[0].1 != window[1].1 {
            let (tick, stable_order, _, _) = window[0].0;
            return Err(ReplayError::AmbiguousDuplicateKey { tick, stable_order });
        }
    }
    Ok(())
}

fn replay_key(command: &ReplayCommand) -> (u64, u64, DeterministicOrderKey, FractureTarget) {
    (
        command.tick,
        command.stable_order,
        command.command.order_key,
        command.command.target,
    )
}

fn replay_payload(command: &ReplayCommand) -> (FxActorId, u32, u32, u8) {
    (
        command.command.actor,
        command.command.health_loss.to_bits(),
        command.command.effective_length_loss.to_bits(),
        damage_source_rank(command.command.source),
    )
}

fn damage_source_rank(source: DamageSource) -> u8 {
    match source {
        DamageSource::Script => 0,
        DamageSource::ContactImpulse => 1,
        DamageSource::JointFeedback => 2,
        DamageSource::Stress => 3,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn asset_from_rows(rows: &[&str], nodes: &[Option<u32>]) -> FxAsset {
        let occupancy = DenseOccupancy::from_rows(rows).unwrap();
        let support_node_map = nodes
            .iter()
            .map(|node| node.map(SupportNodeId))
            .collect::<Vec<_>>();
        let mut desc = FxAssetDesc::new(FxAssetId(1), 1.0, occupancy, support_node_map);
        desc.default_bond_health = 1.0;
        FxAsset::from_desc(desc).unwrap()
    }

    fn break_bond(tick: u64, stable_order: u64, bond: BondId) -> ReplayCommand {
        ReplayCommand {
            tick,
            stable_order,
            command: FractureCommand {
                order_key: DeterministicOrderKey::new(
                    tick,
                    0,
                    FxFamilyId(3),
                    FxActorId(0),
                    CommandId(stable_order as u32),
                ),
                actor: FxActorId(0),
                target: FractureTarget::Bond(bond),
                health_loss: 2.0,
                effective_length_loss: 2.0,
                source: DamageSource::Script,
            },
        }
    }

    #[test]
    fn deterministic_sort_commands() {
        let mut family = FxFamily::instantiate(
            FxFamilyId(3),
            asset_from_rows(&["###"], &[Some(0), Some(1), Some(2)]),
        );
        let traces = run_replay_ticks(
            &mut family,
            0,
            2,
            &[
                break_bond(1, 9, BondId(1)),
                break_bond(0, 2, BondId(1)),
                break_bond(0, 1, BondId(0)),
            ],
        )
        .unwrap();
        assert_eq!(
            traces[0]
                .fracture_events
                .iter()
                .map(|event| event.target)
                .collect::<Vec<_>>(),
            vec![
                FractureTarget::Bond(BondId(0)),
                FractureTarget::Bond(BondId(1))
            ]
        );
    }

    #[test]
    fn replay_split_order() {
        let asset = asset_from_rows(&["####"], &[Some(0), Some(1), Some(2), Some(3)]);
        let mut a = FxFamily::instantiate(FxFamilyId(3), asset.clone());
        let mut b = FxFamily::instantiate(FxFamilyId(3), asset);
        let commands = [break_bond(0, 2, BondId(2)), break_bond(0, 1, BondId(0))];
        let reversed = [commands[1].clone(), commands[0].clone()];
        let a_trace = run_replay_ticks(&mut a, 0, 1, &commands).unwrap();
        let b_trace = run_replay_ticks(&mut b, 0, 1, &reversed).unwrap();
        assert_eq!(a_trace[0].split_events, b_trace[0].split_events);
        assert_eq!(a_trace[0].family_digest, b_trace[0].family_digest);
        assert_eq!(a_trace[0].split_events.len(), 1);
        assert_eq!(a_trace[0].split_events[0].kept_actor, FxActorId(0));
    }

    #[test]
    fn replay_rejects_same_key_different_payload_before_apply() {
        let asset = asset_from_rows(&["###"], &[Some(0), Some(1), Some(2)]);
        let mut family = FxFamily::instantiate(FxFamilyId(3), asset);
        let before = family.deterministic_state_digest();
        let mut ambiguous = break_bond(0, 1, BondId(0));
        ambiguous.command.actor = FxActorId(1);
        ambiguous.command.health_loss = 3.0;
        ambiguous.command.effective_length_loss = 0.5;
        ambiguous.command.source = DamageSource::Stress;

        assert_eq!(
            run_replay_ticks(&mut family, 0, 1, &[break_bond(0, 1, BondId(0)), ambiguous])
                .unwrap_err(),
            ReplayError::AmbiguousDuplicateKey {
                tick: 0,
                stable_order: 1
            }
        );
        assert_eq!(family.deterministic_state_digest(), before);
    }
}
