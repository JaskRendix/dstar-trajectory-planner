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
