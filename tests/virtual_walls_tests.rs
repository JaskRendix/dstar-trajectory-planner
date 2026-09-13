use dstar_trajectory_planner::virtual_walls::VirtualWallsModule;
use std::io::Write;
use tempfile::NamedTempFile;

#[test]
fn test_virtual_walls_module_initialization() {
    let _module = VirtualWallsModule::new(50);
}

#[test]
fn test_parse_missing_file() {
    let mut module = VirtualWallsModule::new(0);
    module.parse_walls_json("non_existent_file_12345.json");
}

#[test]
fn test_parse_valid_walls_and_paths_json() {
    let json_data = r#"{
        "vwalls": [
            {
                "name": "wall_1",
                "polygon": [1.0, 1.0, 2.0, 2.0, 3.0, 3.0]
            }
        ],
        "vpaths": [
            {
                "name": "path_1",
                "polygon": [0.0, 0.0, 5.0, 5.0]
            }
        ]
    }"#;

    let mut temp_file = NamedTempFile::new().unwrap();
    temp_file.write_all(json_data.as_bytes()).unwrap();
    let path_str = temp_file.path().to_str().unwrap();

    let mut module = VirtualWallsModule::new(10);
    module.parse_walls_json(path_str);
}

#[test]
fn test_apply_to_grid_modifications() {
    let width = 10;
    let height = 10;
    let mut grid = vec![0u8; width * height];

    let module = VirtualWallsModule::new(20);

    module.apply_to_grid(&mut grid, width, height, 1.0, 0.0, 0.0, 254, 0);

    assert_eq!(grid[0], 20);
}

#[test]
fn test_malformed_polygon_coordinates() {
    let json_data = r#"{
        "vwalls": [
            {
                "name": "bad_wall",
                "polygon": [1.0, 2.0, 3.0] 
            }
        ],
        "vpaths": [
            {
                "name": "short_path",
                "polygon": [1.0, 1.0]
            }
        ]
    }"#;

    let mut temp_file = NamedTempFile::new().unwrap();
    temp_file.write_all(json_data.as_bytes()).unwrap();
    let path_str = temp_file.path().to_str().unwrap();

    let mut module = VirtualWallsModule::new(0);
    module.parse_walls_json(path_str);
}

#[test]
fn test_virtual_wall_application() {
    let json_data = r#"{
        "vwalls": [
            { "name": "wall", "polygon": [0.0, 0.0, 1.0, 1.0, 2.0, 2.0] }
        ]
    }"#;

    let mut temp = NamedTempFile::new().unwrap();
    temp.write_all(json_data.as_bytes()).unwrap();

    let mut module = VirtualWallsModule::new(0);
    module.parse_walls_json(temp.path().to_str().unwrap());

    let mut grid = vec![0u8; 100];
    module.apply_to_grid(&mut grid, 10, 10, 1.0, 0.0, 0.0, 200, 0);

    assert_eq!(grid[0], 200);
    assert_eq!(grid[11], 200);
    assert_eq!(grid[22], 200);
}

#[test]
fn test_virtual_path_bresenham() {
    let json_data = r#"{
        "vpaths": [
            { "name": "p", "polygon": [0.0, 0.0, 5.0, 5.0] }
        ]
    }"#;

    let mut temp = NamedTempFile::new().unwrap();
    temp.write_all(json_data.as_bytes()).unwrap();

    let mut module = VirtualWallsModule::new(0);
    module.parse_walls_json(temp.path().to_str().unwrap());

    let mut grid = vec![255u8; 100];
    module.apply_to_grid(&mut grid, 10, 10, 1.0, 0.0, 0.0, 254, 0);

    // Bresenham diagonal should mark (0,0), (1,1), (2,2), ...
    for i in 0..6 {
        let idx = i * 10 + i;
        assert_eq!(grid[idx], 0);
    }
}

#[test]
fn test_invalid_json_structure() {
    let json_data = r#"{ "vwalls": "not_an_array" }"#;

    let mut temp = NamedTempFile::new().unwrap();
    temp.write_all(json_data.as_bytes()).unwrap();

    let mut module = VirtualWallsModule::new(0);
    module.parse_walls_json(temp.path().to_str().unwrap());
}

#[test]
fn test_no_walls_no_paths_do_not_modify_grid() {
    let module = VirtualWallsModule::new(0);

    let mut grid = vec![0u8; 100];
    module.apply_to_grid(&mut grid, 10, 10, 1.0, 0.0, 0.0, 200, 0);

    assert!(grid.iter().all(|&v| v == 0));
}

#[test]
fn test_empty_json_produces_no_walls_or_paths() {
    let json_data = r#"{
        "vwalls": [],
        "vpaths": []
    }"#;

    let mut temp = NamedTempFile::new().unwrap();
    temp.write_all(json_data.as_bytes()).unwrap();

    let mut module = VirtualWallsModule::new(0);
    module.parse_walls_json(temp.path().to_str().unwrap());

    let mut grid = vec![0u8; 100];
    module.apply_to_grid(&mut grid, 10, 10, 1.0, 0.0, 0.0, 200, 0);

    assert!(grid.iter().all(|&v| v == 0));
}

#[test]
fn test_world_to_map_bounds() {
    let module = VirtualWallsModule::new(0);
    let mut grid = vec![0u8; 100];

    module.apply_to_grid(&mut grid, 10, 10, 1.0, 0.0, 0.0, 200, 0);
}
