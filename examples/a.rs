use std::{thread::sleep, time::Duration};

use magiclaw_rs::{
    franka::{ConnectConfig, Franka},
    RobotTrait,
};
use nalgebra::Isometry3;

fn main() {
    let franka = Franka::connect(ConnectConfig {
        address: "192.168.1.100".into(),
        realtime: false,
        logsize: None,
    })
    .unwrap();

    let mut control = franka.start_impedance_control(300., 20.);

    for _ in 0..30 {
        sleep(Duration::from_secs_f64(0.1));
        let mut i = Isometry3::identity();
        i.translation.z += 0.01;
        control.move_delta_cartesian(i);
    }

    for _ in 0..30 {
        sleep(Duration::from_secs_f64(0.1));
        let mut i = Isometry3::identity();
        i.translation.z -= 0.01;
        control.move_delta_cartesian(i);
    }

    control.move_delta_cartesian(Isometry3::identity());
    sleep(Duration::from_secs_f64(1.));

    control.stop();
}
