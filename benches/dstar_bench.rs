use criterion::{BenchmarkId, Criterion, criterion_group, criterion_main};
use std::hint::black_box;

use dstar_trajectory_planner::dstar::DStar;
use dstar_trajectory_planner::planner::DStarGlobalPlanner;
use dstar_trajectory_planner::state_map::StateMap;

fn build_uniform_map(width: i64, height: i64, cost: u8) -> Vec<u8> {
    vec![cost; (width * height) as usize]
}

fn bench_dstar_core(c: &mut Criterion) {
    let width = 100;
    let height = 100;

    let mut map = StateMap::new(width, height);

    for x in 0..width {
        for y in 0..height {
            if let Some(p) = map.point(x, y) {
                p.weight = 1;
                p.weight_previous = 1;
                p.cost_actual = 1;
                p.cost_previous = 1;
            }
        }
    }

    let mut dstar = DStar::new(map);
    dstar.init_targets(0, 0, width - 1, height - 1, true);

    c.bench_function("dstar_core_loop(generate_trajectory)", |b| {
        b.iter(|| {
            let _ = black_box(dstar.generate_trajectory());
        })
    });
}

fn bench_generate_trajectory(c: &mut Criterion) {
    let width = 100;
    let height = 100;

    let mut map = StateMap::new(width, height);

    for x in 0..width {
        for y in 0..height {
            if let Some(p) = map.point(x, y) {
                p.weight = 1;
                p.weight_previous = 1;
                p.cost_actual = 1;
                p.cost_previous = 1;
            }
        }
    }

    let mut dstar = DStar::new(map);
    dstar.init_targets(0, 0, width - 1, height - 1, true);

    c.bench_function("dstar_generate_trajectory", |b| {
        b.iter(|| {
            let traj = dstar.generate_trajectory().unwrap();
            black_box(traj);
        })
    });
}

fn bench_initialize_costmap(c: &mut Criterion) {
    let mut group = c.benchmark_group("planner_initialize_costmap_group");
    group.sample_size(40);

    let width = 200;
    let height = 200;
    let map_data = build_uniform_map(width, height, 1);

    group.bench_function(BenchmarkId::new("planner_initialize_costmap", width), |b| {
        b.iter(|| {
            let mut planner = DStarGlobalPlanner::new();
            planner.set_verbose(false);
            planner.initialize(width, height, &map_data, false, "");
            black_box(planner);
        })
    });

    group.finish();
}

fn bench_make_plan(c: &mut Criterion) {
    let mut group = c.benchmark_group("planner_make_plan_group");
    group.sample_size(20);
    group.measurement_time(std::time::Duration::from_secs(20));

    let width = 200;
    let height = 200;
    let map_data = build_uniform_map(width, height, 1);

    let mut planner = DStarGlobalPlanner::new();
    planner.set_verbose(false);
    planner.initialize(width, height, &map_data, false, "");

    let start = (0.0, 0.0);
    let goal = ((width - 1) as f64, (height - 1) as f64);

    group.bench_function(BenchmarkId::new("planner_make_plan", width), |b| {
        b.iter(|| {
            let path = planner.make_plan(start, goal).unwrap();
            black_box(path);
        })
    });

    group.finish();
}

fn bench_large_map(c: &mut Criterion) {
    let mut group = c.benchmark_group("planner_large_map_group");
    group.sample_size(10);
    group.measurement_time(std::time::Duration::from_secs(30));

    let width = 500;
    let height = 500;
    let map_data = build_uniform_map(width, height, 1);

    let mut planner = DStarGlobalPlanner::new();
    planner.set_verbose(false);
    planner.initialize(width, height, &map_data, false, "");

    let start = (0.0, 0.0);
    let goal = ((width - 1) as f64, (height - 1) as f64);

    group.bench_function(BenchmarkId::new("planner_large_map_500x500", width), |b| {
        b.iter(|| {
            let path = planner.make_plan(start, goal).unwrap();
            black_box(path);
        })
    });

    group.finish();
}

criterion_group!(
    benches,
    bench_dstar_core,
    bench_generate_trajectory,
    bench_initialize_costmap,
    bench_make_plan,
    bench_large_map,
);

criterion_main!(benches);
