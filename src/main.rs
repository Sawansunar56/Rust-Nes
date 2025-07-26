extern crate sdl2;

pub mod cpu;
pub mod opcodes;

use sdl2::{event::Event, keyboard::Keycode, pixels::Color};

use crate::cpu::CPU;

fn main() {
    let mut cpu = CPU::new();

    cpu.load_and_run(vec![0xa9, 0x05, 0x00]);

    let sdl_context = sdl2::init().unwrap_or_else(|e| {
        eprintln!("SDL_Error: Problem with sdl initialization \n{}", e);
        std::process::exit(1);
    });
    let video_subsystem = sdl_context.video().unwrap_or_else(|e| {
        eprintln!("SDL_Error: Problem with sdl video subsystem \n{}", e);
        std::process::exit(1);
    });

    let window = video_subsystem
        .window("Rust NES", 800, 600)
        .position_centered()
        .opengl()
        .build()
        .unwrap_or_else(|e| {
            eprintln!("SDL_Error: Problem for windows setup \n {}", e);
            std::process::exit(1);
        });

    let mut canvas = window.into_canvas().build().unwrap_or_else(|e| {
        eprintln!("Failed to create canvas: {}", e);
        std::process::exit(1);
    });


    canvas.set_draw_color(Color::RGB(200, 200, 0));
    canvas.clear();
    canvas.present();
    
    let mut event_pump = sdl_context.event_pump().unwrap();

    'running: loop {
        for event in event_pump.poll_iter() {
            match event {
                Event::Quit { .. }
                | Event::KeyDown { 
                    keycode: Some(Keycode::Escape),
                    ..
                } => break 'running,
                _ => {}
            }
        }

        canvas.clear();
        canvas.present();
    }
}
