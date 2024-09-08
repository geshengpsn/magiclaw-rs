use std::{
    sync::mpsc::{channel, Sender},
    thread::{spawn, JoinHandle},
};

use franka::{
    exception::FrankaException, Frame, MotionFinished, RealtimeConfig, RobotState, Torques,
};
use nalgebra::{
    Isometry3, Matrix3, Matrix4, Matrix6, Matrix6x1, Rotation3, SMatrix, UnitQuaternion, Vector3,
};

use crate::RobotTrait;

pub struct Franka {
    inner: franka::Robot,
    model: franka::Model,
}

pub struct ConnectConfig {
    pub address: String,
    pub realtime: bool,
    pub logsize: Option<usize>,
}

impl RobotTrait for Franka {
    type Config = ConnectConfig;
    type ConnectError = FrankaException;
    fn connect(connect_config: ConnectConfig) -> Result<Self, FrankaException> {
        let mut inner = franka::Robot::new(
            &connect_config.address,
            if connect_config.realtime {
                None
            } else {
                Some(RealtimeConfig::Ignore)
            },
            connect_config.logsize,
        )?;

        let model = inner.load_model(true)?;

        Ok(Franka { inner, model })
    }

    type State = Result<RobotState, FrankaException>;

    fn read_state(&mut self) -> Self::State {
        // let mut robot = self.inner.lock().unwrap();
        self.inner.read_once()
    }
}

enum ControlMsg {
    Cartesian(Isometry3<f64>),
    Stop,
}

impl Franka {
    pub fn start_impedance_control(mut self, stiffness: f64, damping: f64) -> ControlSession {
        let (tx, rx) = channel::<ControlMsg>();
        self.inner
            .set_collision_behavior(
                [100.; 7], [100.; 7], [100.; 7], [100.; 7], [100.; 6], [100.; 6], [100.; 6],
                [100.; 6],
            )
            .unwrap();
        self.inner
            .set_joint_impedance([3000., 3000., 3000., 2500., 2500., 2000., 2000.])
            .unwrap();
        self.inner
            .set_cartesian_impedance([3000., 3000., 3000., 300., 300., 300.])
            .unwrap();
        let (stiffness_matrix, damping_matrix) = stiffness_damping(stiffness, damping);
        let state = self.read_state().unwrap();
        let mut robot_ee_pose_d = Isometry3::from_parts(
            Vector3::new(state.O_T_EE[12], state.O_T_EE[13], state.O_T_EE[14]).into(),
            Rotation3::<f64>::from_matrix(
                &Matrix4::from_column_slice(&state.O_T_EE)
                    .remove_column(3)
                    .remove_row(3),
            )
            .into(),
        );

        let handle = spawn(move || {
            // let mut locked_robot = robot.lock().unwrap();
            self.inner
                .control_torques(
                    |state, _time| {
                        match rx.try_recv() {
                            Ok(ControlMsg::Cartesian(delta)) => {
                                robot_ee_pose_d *= delta;
                            }
                            Err(std::sync::mpsc::TryRecvError::Disconnected)
                            | Ok(ControlMsg::Stop) => {
                                return Torques::new([0., 0., 0., 0., 0., 0., 0.])
                                    .motion_finished();
                            }
                            _ => {}
                        };

                        let robot_ee_pose = Isometry3::from_parts(
                            Vector3::new(state.O_T_EE[12], state.O_T_EE[13], state.O_T_EE[14])
                                .into(),
                            Rotation3::<f64>::from_matrix(
                                &Matrix4::from_column_slice(&state.O_T_EE)
                                    .remove_column(3)
                                    .remove_row(3),
                            )
                            .into(),
                        );

                        let position = robot_ee_pose.translation.vector;
                        let mut orientation = *robot_ee_pose.rotation.quaternion();

                        let position_d = robot_ee_pose_d.translation.vector;
                        let orientation_d: UnitQuaternion<f64> = robot_ee_pose_d.rotation;

                        let coriolis: SMatrix<f64, 7, 1> =
                            self.model.coriolis_from_state(state).into();
                        let jacobian_array = self
                            .model
                            .zero_jacobian_from_state(&Frame::EndEffector, state);
                        let jacobian = SMatrix::<f64, 6, 7>::from_column_slice(&jacobian_array);
                        // let _q = Vector7::from_column_slice(&state.q);
                        let dq = SMatrix::<f64, 7, 1>::from_column_slice(&state.dq);

                        let mut error: Matrix6x1<f64> = Matrix6x1::<f64>::zeros();
                        {
                            let mut error_head = error.fixed_view_mut::<3, 1>(0, 0);
                            error_head.set_column(0, &(position - position_d));
                        }

                        if orientation_d.coords.dot(&orientation.coords) < 0. {
                            orientation.coords = -orientation.coords;
                        }
                        let orientation = UnitQuaternion::new_normalize(orientation);
                        let error_quaternion: UnitQuaternion<f64> =
                            orientation.inverse() * orientation_d;
                        {
                            let mut error_tail = error.fixed_view_mut::<3, 1>(3, 0);
                            error_tail.copy_from(
                                &-(robot_ee_pose.rotation.to_rotation_matrix()
                                    * Vector3::new(
                                        error_quaternion.i,
                                        error_quaternion.j,
                                        error_quaternion.k,
                                    )),
                            );
                        }
                        let tau_task = jacobian.transpose()
                            * (-stiffness_matrix * error - damping_matrix * (jacobian * dq));
                        let tau_d = tau_task + coriolis;
                        Torques::new([
                            tau_d[0], tau_d[1], tau_d[2], tau_d[3], tau_d[4], tau_d[5], tau_d[6],
                        ])
                    },
                    None,
                    None,
                )
                .unwrap();
            (self.inner, self.model)
        });
        ControlSession {
            sender: tx,
            control_job_handle: handle,
        }
    }
}

fn stiffness_damping(
    translational_stiffness: f64,
    rotational_stiffness: f64,
) -> (Matrix6<f64>, Matrix6<f64>) {
    let mut stiffness = SMatrix::<f64, 6, 6>::zeros();
    let mut damping = SMatrix::<f64, 6, 6>::zeros();
    {
        let mut top_left_corner = stiffness.fixed_view_mut::<3, 3>(0, 0);
        top_left_corner.copy_from(&(Matrix3::identity() * translational_stiffness));
        let mut top_left_corner = damping.fixed_view_mut::<3, 3>(0, 0);
        top_left_corner.copy_from(&(2. * f64::sqrt(translational_stiffness) * Matrix3::identity()));
    }
    {
        let mut bottom_right_corner = stiffness.fixed_view_mut::<3, 3>(3, 3);
        bottom_right_corner.copy_from(&(Matrix3::identity() * rotational_stiffness));
        let mut bottom_right_corner = damping.fixed_view_mut::<3, 3>(3, 3);
        bottom_right_corner
            .copy_from(&(2. * f64::sqrt(rotational_stiffness) * Matrix3::identity()));
    }
    (stiffness, damping)
}

pub struct ControlSession {
    // franka
    sender: Sender<ControlMsg>,
    control_job_handle: JoinHandle<(franka::Robot, franka::Model)>,
}

impl ControlSession {
    pub fn stop(self) -> Franka {
        self.sender.send(ControlMsg::Stop).unwrap();
        let (inner, model) = self.control_job_handle.join().unwrap();
        Franka { inner, model }
    }

    pub fn move_delta_cartesian(&mut self, delta_cartesian: Isometry3<f64>) {
        self.sender
            .send(ControlMsg::Cartesian(delta_cartesian))
            .unwrap();
    }
}
