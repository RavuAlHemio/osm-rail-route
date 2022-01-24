use std::time::Instant;

use log::{Level, Log, Metadata, Record};


pub struct TimingLogger {
    start_time: Instant,
    log_level: Level,
}
impl TimingLogger {
    pub fn new(log_level: Level) -> Self {
        let start_time = Instant::now();
        Self {
            start_time,
            log_level,
        }
    }

    pub fn init(log_level: Level) {
        let logger = Self::new(log_level);
        log::set_boxed_logger(Box::new(logger))
            .expect("failed to set logger");
        log::set_max_level(log_level.to_level_filter());
    }
}
impl Log for TimingLogger {
    fn enabled(&self, metadata: &Metadata) -> bool {
        metadata.level() <= self.log_level
    }

    fn log(&self, record: &Record) {
        if !self.enabled(record.metadata()) {
            return;
        }
        let duration = Instant::now() - self.start_time;
        eprintln!("[{:.6}] {}", duration.as_secs_f64(), record.args());
    }

    fn flush(&self) {}
}
