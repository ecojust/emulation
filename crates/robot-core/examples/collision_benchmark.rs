//! Actual CAD mesh kernel timings. Excludes process startup, JSON and desktop IPC.
use emulation_core::collision::{
    check_collisions, CollisionCandidate, CollisionRequest, CollisionResult, CollisionZone,
};
use serde_json::json;
use std::hint::black_box;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{Duration, Instant};

fn zone(id: &str, center_mm: [f64; 3], size_mm: [f64; 3]) -> CollisionZone {
    CollisionZone {
        id: id.into(),
        enabled: true,
        center_mm,
        size_mm,
    }
}

fn count(results: &[CollisionResult]) -> (usize, usize) {
    results
        .iter()
        .flat_map(|result| &result.zones)
        .fold((0, 0), |(hit, uncertain), zone| {
            (
                hit + zone.colliding_links.len(),
                uncertain + zone.uncertain_links.len(),
            )
        })
}

fn timed(request: &CollisionRequest) -> (f64, (usize, usize)) {
    let started = Instant::now();
    let results = check_collisions(black_box(request), &AtomicBool::new(false)).unwrap();
    let elapsed = started.elapsed().as_secs_f64() * 1000.;
    (elapsed, count(black_box(&results)))
}

fn distribution(request: &CollisionRequest, samples: usize) -> serde_json::Value {
    let mut timings: Vec<_> = (0..samples).map(|_| timed(request).0).collect();
    timings.sort_by(f64::total_cmp);
    json!({ "samples": samples, "meanMs": timings.iter().sum::<f64>() / samples as f64,
        "p50Ms": timings[samples / 2], "p95Ms": timings[(samples * 95 / 100).min(samples - 1)],
        "maximumMs": timings[samples - 1] })
}

fn cancel_after_one_ms(request: &CollisionRequest) -> serde_json::Value {
    let flag = AtomicBool::new(false);
    let started = Instant::now();
    let (result, requested_at) = std::thread::scope(|scope| {
        let canceller = scope.spawn(|| {
            std::thread::sleep(Duration::from_millis(1));
            let requested = Instant::now();
            flag.store(true, Ordering::Relaxed);
            requested
        });
        let result = check_collisions(request, &flag);
        (result, canceller.join().unwrap())
    });
    let finished = Instant::now();
    json!({ "cancelled": result.is_err(), "totalMs": (finished - started).as_secs_f64() * 1000.,
        "afterCancelMs": (finished - requested_at).as_secs_f64() * 1000., "error": result.err() })
}

fn main() {
    let base = CollisionRequest {
        base_pose: None,
        model_id: "irb1100-4-0.58".into(),
        zones: vec![
            zone("empty-obb-corner", [75., 75., 80.], [4.; 3]),
            zone("actual-surface", [0., 70., 100.], [4.; 3]),
            zone("contained-box", [0., 0., 50.], [2.; 3]),
        ],
        solutions: vec![CollisionCandidate {
            id: "zero".into(),
            joints: [0.; 6],
        }],
    };
    let base_cold = timed(&base);
    let base_warm = distribution(&base, 100);
    let ready = CollisionRequest {
        base_pose: None,
        model_id: base.model_id.clone(),
        zones: vec![
            zone(
                "application-default",
                [220., -160., 350.],
                [160., 160., 200.],
            ),
            zone("multiple-links", [30., 0., 350.], [200., 200., 400.]),
            zone("wrist", [450., 0., 620.], [150.; 3]),
        ],
        solutions: vec![CollisionCandidate {
            id: "ready".into(),
            joints: [0., 20., -30., 0., 40., 0.],
        }],
    };
    let ready_initial = timed(&ready);
    let ready_warm = distribution(&ready, 100);
    let mut maximum = base.clone();
    maximum.zones = (0..64)
        .map(|i| {
            let mut z = match i % 4 {
                0 => zone(&i.to_string(), [75., 75., 80.], [4.; 3]),
                1 => zone(&i.to_string(), [0., 0., 50.], [2.; 3]),
                2 => zone(&i.to_string(), [0., 70., 100.], [4.; 3]),
                _ => zone(&i.to_string(), [251., 0., 617.], [5.; 3]),
            };
            z.center_mm[1] += (i / 4) as f64 * 0.001;
            z
        })
        .collect();
    maximum.solutions = (0..256)
        .map(|i| CollisionCandidate {
            id: i.to_string(),
            joints: [
                ((i % 16) as f64 - 7.5) * 0.02,
                ((i / 16) as f64 - 7.5) * 0.02,
                0.,
                0.,
                0.,
                0.,
            ],
        })
        .collect();
    let maximum_result = timed(&maximum);
    let maximum_warm = distribution(&maximum, 5);
    let warm_cancel = cancel_after_one_ms(&maximum);
    let mut cold_cancel_request = maximum.clone();
    cold_cancel_request.model_id = "irb1100-4-0.475".into();
    let cold_cancel = cancel_after_one_ms(&cold_cancel_request);
    let retry = timed(&cold_cancel_request);
    println!("{}", serde_json::to_string_pretty(&json!({
        "scope": "Rust release kernel only; actual pinned STL; excludes JSON/process startup/IPC",
        "baseOnePoseThreeZones": { "coldMs": base_cold.0, "confirmedPairs": base_cold.1.0, "uncertainPairs": base_cold.1.1, "warm": base_warm },
        "readyOnePoseThreeZones": { "remainingMeshesColdMs": ready_initial.0, "confirmedPairs": ready_initial.1.0, "uncertainPairs": ready_initial.1.1, "warm": ready_warm },
        "maximum256Poses64Zones": { "firstMs": maximum_result.0, "confirmedPairs": maximum_result.1.0, "uncertainPairs": maximum_result.1.1, "warm": maximum_warm },
        "warmCancellation": warm_cancel, "cold475Cancellation": cold_cancel,
        "retryAfterColdCancellation": { "ms": retry.0, "confirmedPairs": retry.1.0, "uncertainPairs": retry.1.1 }
    })).unwrap());
}
