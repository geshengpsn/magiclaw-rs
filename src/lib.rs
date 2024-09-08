pub mod franka;

pub mod prelude {
    pub use crate::RobotTrait;
    pub use crate::Cartesian;
    pub use crate::JointControl;
    pub use crate::TorqueControl;
    pub use crate::CartesianControl;
    pub use crate::DynamicModel;
}

pub trait RobotTrait: Sized {
    type Config;
    type ConnectError;
    fn connect(connect_config: Self::Config) -> Result<Self, Self::ConnectError>;

    type State;
    fn read_state(&mut self) -> Self::State;
}

pub trait Cartesian {
    type Translation;
    fn set_translation(&mut self, translation: Self::Translation);
    fn get_translation(&self) -> Self::Translation;

    type RotMatrix;
    fn set_rotation_matrix(&mut self, matrix: Self::RotMatrix);
    fn get_rotation_matrix(&self) -> Self::RotMatrix;

    type Quaternion;
    fn set_quaternion(&mut self, quaternion: Self::Quaternion);
    fn get_quaternion(&self) -> Self::Quaternion;
}

pub trait JointControl {
    type Joints;
    
    fn joint(&mut self, joint: Self::Joints);
}

pub trait TorqueControl {
    type Torque;
    fn torque(&mut self, torque: Self::Torque);
}

pub trait CartesianControl {
    fn move_cartesian(&mut self, cartesian: impl Cartesian);

    type Jacobian;
    fn jacobian(&mut self) -> Self::Jacobian;
}

pub trait DynamicModel {
    fn cartesian_impedance(&mut self, cartesian: impl Cartesian);
}
