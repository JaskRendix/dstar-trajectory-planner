use crate::VirtualWallsModule;

pub fn load_mock_map() -> (i64, i64, Vec<u8>) {
    let width: i64 = 50;
    let height: i64 = 50;
    let mut map_data = vec![0u8; (width * height) as usize];

    // Create a mock obstacle wall down the middle of the grid
    for y in 10..40 {
        let idx = (y * width + 25) as usize;
        map_data[idx] = 100; // Lethal obstacle cost
    }

    // Initialize the virtual walls module with a default cost
    let walls_module = VirtualWallsModule::new(0);
    walls_module.apply_to_grid(
        &mut map_data,
        width as usize,
        height as usize,
        1.0, // resolution (m/cell)
        0.0, // origin x
        0.0, // origin y
        100, // lethal cost
        0,   // free space cost
    );

    (width, height, map_data)
}
