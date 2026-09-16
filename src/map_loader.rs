use crate::VirtualWallsModule;

pub fn load_mock_map() -> (i64, i64, Vec<u8>) {
    const WIDTH: i64 = 50;
    const HEIGHT: i64 = 50;
    const LETHAL_COST: u8 = 100;
    const WALL_X: i64 = 25;

    let mut map_data = vec![0u8; (WIDTH * HEIGHT) as usize];

    // Create a mock obstacle wall down the middle of the grid
    for y in 10..40 {
        let idx = (y * WIDTH + WALL_X) as usize;
        map_data[idx] = LETHAL_COST;
    }

    // Initialize the virtual walls module with a default cost
    let walls_module = VirtualWallsModule::new(0);
    walls_module.apply_to_grid(
        &mut map_data,
        WIDTH as usize,
        HEIGHT as usize,
        1.0,         // resolution (m/cell)
        0.0,         // origin x
        0.0,         // origin y
        LETHAL_COST, // lethal cost (u8)
        0,           // free space cost
    );

    (WIDTH, HEIGHT, map_data)
}
