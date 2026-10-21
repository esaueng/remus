//! Measure explicit native compaction over repeated allocate/restore sessions.
//!
//! Run the built executable in separate processes with and without `--compact`.
//! This is an arena-lifecycle diagnostic, not a CAD edit-latency benchmark.

use std::error::Error;
use std::io::Write;
use std::time::Instant;

use remus_io::arena_io::{CompactionRoots, compact_document, serialize_solids};
use remus_operations::primitives::make_box;
use remus_topology::Topology;

fn main() -> Result<(), Box<dyn Error>> {
    let mut compact = false;
    let mut cycles = 20usize;
    let mut ticks = 100usize;
    let mut args = std::env::args().skip(1);
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--compact" => compact = true,
            "--cycles" => cycles = args.next().ok_or("missing cycles")?.parse()?,
            "--ticks" => ticks = args.next().ok_or("missing ticks")?.parse()?,
            _ => return Err(format!("unknown argument: {arg}").into()),
        }
    }
    if cycles == 0 || ticks == 0 {
        return Err("cycles and ticks must be positive".into());
    }

    let mut topology = Topology::new();
    let mut solid = make_box(&mut topology, 2.0, 3.0, 4.0)?;
    let original = serialize_solids(&topology, &[solid])?;
    let mut peak_slots = topology.allocated_slot_count();
    let mut compact_time = std::time::Duration::ZERO;
    let start = Instant::now();
    for _ in 0..cycles {
        {
            let checkpoint = topology.clone();
            for _ in 0..ticks {
                make_box(&mut topology, 1.0, 1.0, 1.0)?;
                peak_slots = peak_slots.max(topology.allocated_slot_count());
                topology.restore_preserving_handle_slots(&checkpoint);
            }
        }
        if compact {
            let compact_start = Instant::now();
            let copied = compact_document(
                &topology,
                CompactionRoots {
                    solids: &[solid],
                    ..CompactionRoots::default()
                },
            )?;
            solid = copied.roots.solids[0];
            topology = copied.topology;
            compact_time += compact_start.elapsed();
        }
        if serialize_solids(&topology, &[solid])? != original {
            return Err("selected geometry changed during the session".into());
        }
    }
    let elapsed = start.elapsed();
    let rss_kib = std::fs::read_to_string("/proc/self/status")
        .ok()
        .and_then(|status| {
            status.lines().find_map(|line| {
                line.strip_prefix("VmRSS:")?
                    .split_whitespace()
                    .next()?
                    .parse::<u64>()
                    .ok()
            })
        });
    writeln!(
        std::io::stdout().lock(),
        "{}",
        serde_json::json!({
            "compact": compact,
            "cycles": cycles,
            "ticksPerCycle": ticks,
            "peakSlots": peak_slots,
            "retainedSlots": topology.allocated_slot_count(),
            "rssKiB": rss_kib,
            "sessionMs": elapsed.as_secs_f64() * 1000.0,
            "compactionMs": compact_time.as_secs_f64() * 1000.0,
            "documentBytes": original.len(),
            "exactDocumentPreserved": true,
        })
    )?;
    Ok(())
}
