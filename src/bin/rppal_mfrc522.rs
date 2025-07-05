use core::fmt::Arguments;
use rppal_mfrc522::{error, RppalMfrc522Log, RppalMfrc522Tool};
use termion::color;

struct RppalMfrc522Logger;

impl RppalMfrc522Logger {
    fn new() -> RppalMfrc522Logger {
        RppalMfrc522Logger {}
    }
}

impl RppalMfrc522Log for RppalMfrc522Logger {
    fn output(self: &Self, args: Arguments) {
        println!("{}", args);
    }
    fn warning(self: &Self, args: Arguments) {
        eprintln!("{}warning: {}", color::Fg(color::Yellow), args);
    }
    fn error(self: &Self, args: Arguments) {
        eprintln!("{}error: {}", color::Fg(color::Red), args);
    }
}

fn main() {
    let logger = RppalMfrc522Logger::new();

    if let Err(error) = RppalMfrc522Tool::new(&logger).run(std::env::args_os()) {
        error!(logger, "{}", error);
        std::process::exit(1);
    }
}
