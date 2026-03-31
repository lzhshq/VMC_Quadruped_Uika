#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};


#[link(name = "interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__interfaces__msg__BaseState() -> *const std::ffi::c_void;
}

#[link(name = "interfaces__rosidl_generator_c")]
extern "C" {
    fn interfaces__msg__BaseState__init(msg: *mut BaseState) -> bool;
    fn interfaces__msg__BaseState__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<BaseState>, size: usize) -> bool;
    fn interfaces__msg__BaseState__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<BaseState>);
    fn interfaces__msg__BaseState__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<BaseState>, out_seq: *mut rosidl_runtime_rs::Sequence<BaseState>) -> bool;
}

// Corresponds to interfaces__msg__BaseState
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct BaseState {
    /// 消息头：包含时间戳和坐标系名称
    pub header: std_msgs::msg::rmw::Header,

    /// 机身位置，单位 m，建议在 world 坐标系下表达
    pub position: geometry_msgs::msg::rmw::Point,

    /// 机身姿态，四元数表示
    pub orientation: geometry_msgs::msg::rmw::Quaternion,

    /// 机身线速度，单位 m/s
    pub linear_velocity: geometry_msgs::msg::rmw::Vector3,

    /// 机身角速度，单位 rad/s
    pub angular_velocity: geometry_msgs::msg::rmw::Vector3,

}



impl Default for BaseState {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !interfaces__msg__BaseState__init(&mut msg as *mut _) {
        panic!("Call to interfaces__msg__BaseState__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for BaseState {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { interfaces__msg__BaseState__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { interfaces__msg__BaseState__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { interfaces__msg__BaseState__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for BaseState {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for BaseState where Self: Sized {
  const TYPE_NAME: &'static str = "interfaces/msg/BaseState";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__interfaces__msg__BaseState() }
  }
}


#[link(name = "interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__interfaces__msg__ContactState() -> *const std::ffi::c_void;
}

#[link(name = "interfaces__rosidl_generator_c")]
extern "C" {
    fn interfaces__msg__ContactState__init(msg: *mut ContactState) -> bool;
    fn interfaces__msg__ContactState__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ContactState>, size: usize) -> bool;
    fn interfaces__msg__ContactState__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ContactState>);
    fn interfaces__msg__ContactState__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ContactState>, out_seq: *mut rosidl_runtime_rs::Sequence<ContactState>) -> bool;
}

// Corresponds to interfaces__msg__ContactState
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ContactState {
    /// 消息头：包含时间戳和坐标系名称
    pub header: std_msgs::msg::rmw::Header,

    /// 左前腿是否接触地面，true 表示接触
    pub fl_contact: bool,

    /// 右前腿是否接触地面，true 表示接触
    pub fr_contact: bool,

    /// 左后腿是否接触地面，true 表示接触
    pub rl_contact: bool,

    /// 右后腿是否接触地面，true 表示接触
    pub rr_contact: bool,

}



impl Default for ContactState {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !interfaces__msg__ContactState__init(&mut msg as *mut _) {
        panic!("Call to interfaces__msg__ContactState__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ContactState {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { interfaces__msg__ContactState__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { interfaces__msg__ContactState__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { interfaces__msg__ContactState__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ContactState {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ContactState where Self: Sized {
  const TYPE_NAME: &'static str = "interfaces/msg/ContactState";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__interfaces__msg__ContactState() }
  }
}


#[link(name = "interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__interfaces__msg__GaitPhase() -> *const std::ffi::c_void;
}

#[link(name = "interfaces__rosidl_generator_c")]
extern "C" {
    fn interfaces__msg__GaitPhase__init(msg: *mut GaitPhase) -> bool;
    fn interfaces__msg__GaitPhase__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<GaitPhase>, size: usize) -> bool;
    fn interfaces__msg__GaitPhase__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<GaitPhase>);
    fn interfaces__msg__GaitPhase__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<GaitPhase>, out_seq: *mut rosidl_runtime_rs::Sequence<GaitPhase>) -> bool;
}

// Corresponds to interfaces__msg__GaitPhase
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct GaitPhase {
    /// 消息头：包含时间戳和坐标系名称
    pub header: std_msgs::msg::rmw::Header,

    /// 左前腿当前相位，取值：STANCE 或 SWING
    pub fl_phase: u8,

    /// 右前腿当前相位，取值：STANCE 或 SWING
    pub fr_phase: u8,

    /// 左后腿当前相位，取值：STANCE 或 SWING
    pub rl_phase: u8,

    /// 右后腿当前相位，取值：STANCE 或 SWING
    pub rr_phase: u8,

}

impl GaitPhase {
    /// 支撑相
    pub const STANCE: u8 = 0;

    /// 摆动相
    pub const SWING: u8 = 1;

}


impl Default for GaitPhase {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !interfaces__msg__GaitPhase__init(&mut msg as *mut _) {
        panic!("Call to interfaces__msg__GaitPhase__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for GaitPhase {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { interfaces__msg__GaitPhase__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { interfaces__msg__GaitPhase__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { interfaces__msg__GaitPhase__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for GaitPhase {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for GaitPhase where Self: Sized {
  const TYPE_NAME: &'static str = "interfaces/msg/GaitPhase";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__interfaces__msg__GaitPhase() }
  }
}


#[link(name = "interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__interfaces__msg__MotionCommand() -> *const std::ffi::c_void;
}

#[link(name = "interfaces__rosidl_generator_c")]
extern "C" {
    fn interfaces__msg__MotionCommand__init(msg: *mut MotionCommand) -> bool;
    fn interfaces__msg__MotionCommand__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<MotionCommand>, size: usize) -> bool;
    fn interfaces__msg__MotionCommand__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<MotionCommand>);
    fn interfaces__msg__MotionCommand__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<MotionCommand>, out_seq: *mut rosidl_runtime_rs::Sequence<MotionCommand>) -> bool;
}

// Corresponds to interfaces__msg__MotionCommand
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct MotionCommand {
    /// 消息头：主要用于时间戳
    pub header: std_msgs::msg::rmw::Header,

    /// 机器人期望前向速度，单位 m/s
    pub vx: f64,

    /// 机器人期望侧向速度，单位 m/s
    pub vy: f64,

    /// 机器人期望偏航角速度，单位 rad/s
    pub wz: f64,

    /// 机器人期望机身高度，单位 m
    pub body_height: f64,

}



impl Default for MotionCommand {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !interfaces__msg__MotionCommand__init(&mut msg as *mut _) {
        panic!("Call to interfaces__msg__MotionCommand__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for MotionCommand {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { interfaces__msg__MotionCommand__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { interfaces__msg__MotionCommand__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { interfaces__msg__MotionCommand__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for MotionCommand {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for MotionCommand where Self: Sized {
  const TYPE_NAME: &'static str = "interfaces/msg/MotionCommand";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__interfaces__msg__MotionCommand() }
  }
}


#[link(name = "interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__interfaces__msg__SingleFootState() -> *const std::ffi::c_void;
}

#[link(name = "interfaces__rosidl_generator_c")]
extern "C" {
    fn interfaces__msg__SingleFootState__init(msg: *mut SingleFootState) -> bool;
    fn interfaces__msg__SingleFootState__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<SingleFootState>, size: usize) -> bool;
    fn interfaces__msg__SingleFootState__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<SingleFootState>);
    fn interfaces__msg__SingleFootState__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<SingleFootState>, out_seq: *mut rosidl_runtime_rs::Sequence<SingleFootState>) -> bool;
}

// Corresponds to interfaces__msg__SingleFootState
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SingleFootState {
    /// 单条腿足端位置，单位 m
    pub position: geometry_msgs::msg::rmw::Point,

    /// 单条腿足端速度，单位 m/s
    pub velocity: geometry_msgs::msg::rmw::Vector3,

    /// 单条腿足端受力或期望力，单位 N
    pub force: geometry_msgs::msg::rmw::Vector3,

}



impl Default for SingleFootState {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !interfaces__msg__SingleFootState__init(&mut msg as *mut _) {
        panic!("Call to interfaces__msg__SingleFootState__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for SingleFootState {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { interfaces__msg__SingleFootState__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { interfaces__msg__SingleFootState__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { interfaces__msg__SingleFootState__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for SingleFootState {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for SingleFootState where Self: Sized {
  const TYPE_NAME: &'static str = "interfaces/msg/SingleFootState";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__interfaces__msg__SingleFootState() }
  }
}


#[link(name = "interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__interfaces__msg__FootState() -> *const std::ffi::c_void;
}

#[link(name = "interfaces__rosidl_generator_c")]
extern "C" {
    fn interfaces__msg__FootState__init(msg: *mut FootState) -> bool;
    fn interfaces__msg__FootState__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<FootState>, size: usize) -> bool;
    fn interfaces__msg__FootState__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<FootState>);
    fn interfaces__msg__FootState__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<FootState>, out_seq: *mut rosidl_runtime_rs::Sequence<FootState>) -> bool;
}

// Corresponds to interfaces__msg__FootState
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct FootState {
    /// 消息头：包含时间戳和坐标系名称
    pub header: std_msgs::msg::rmw::Header,

    /// 左前腿足端状态
    pub fl: super::super::msg::rmw::SingleFootState,

    /// 右前腿足端状态
    pub fr: super::super::msg::rmw::SingleFootState,

    /// 左后腿足端状态
    pub rl: super::super::msg::rmw::SingleFootState,

    /// 右后腿足端状态
    pub rr: super::super::msg::rmw::SingleFootState,

}



impl Default for FootState {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !interfaces__msg__FootState__init(&mut msg as *mut _) {
        panic!("Call to interfaces__msg__FootState__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for FootState {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { interfaces__msg__FootState__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { interfaces__msg__FootState__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { interfaces__msg__FootState__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for FootState {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for FootState where Self: Sized {
  const TYPE_NAME: &'static str = "interfaces/msg/FootState";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__interfaces__msg__FootState() }
  }
}


#[link(name = "interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__interfaces__msg__MotorCommand() -> *const std::ffi::c_void;
}

#[link(name = "interfaces__rosidl_generator_c")]
extern "C" {
    fn interfaces__msg__MotorCommand__init(msg: *mut MotorCommand) -> bool;
    fn interfaces__msg__MotorCommand__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<MotorCommand>, size: usize) -> bool;
    fn interfaces__msg__MotorCommand__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<MotorCommand>);
    fn interfaces__msg__MotorCommand__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<MotorCommand>, out_seq: *mut rosidl_runtime_rs::Sequence<MotorCommand>) -> bool;
}

// Corresponds to interfaces__msg__MotorCommand
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct MotorCommand {
    /// 电机前馈力矩，单位 N·m
    pub torque: f64,

    /// 电机目标位置，单位 rad
    pub position: f64,

    /// 电机目标速度，单位 rad/s
    pub velocity: f64,

    /// 位置环增益
    pub kp: f64,

    /// 速度环增益
    pub kd: f64,

}



impl Default for MotorCommand {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !interfaces__msg__MotorCommand__init(&mut msg as *mut _) {
        panic!("Call to interfaces__msg__MotorCommand__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for MotorCommand {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { interfaces__msg__MotorCommand__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { interfaces__msg__MotorCommand__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { interfaces__msg__MotorCommand__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for MotorCommand {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for MotorCommand where Self: Sized {
  const TYPE_NAME: &'static str = "interfaces/msg/MotorCommand";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__interfaces__msg__MotorCommand() }
  }
}


#[link(name = "interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__interfaces__msg__MotorCommand12() -> *const std::ffi::c_void;
}

#[link(name = "interfaces__rosidl_generator_c")]
extern "C" {
    fn interfaces__msg__MotorCommand12__init(msg: *mut MotorCommand12) -> bool;
    fn interfaces__msg__MotorCommand12__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<MotorCommand12>, size: usize) -> bool;
    fn interfaces__msg__MotorCommand12__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<MotorCommand12>);
    fn interfaces__msg__MotorCommand12__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<MotorCommand12>, out_seq: *mut rosidl_runtime_rs::Sequence<MotorCommand12>) -> bool;
}

// Corresponds to interfaces__msg__MotorCommand12
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct MotorCommand12 {
    /// 消息头：包含时间戳
    pub header: std_msgs::msg::rmw::Header,

    /// 左前髋关节电机命令
    pub fl_hip: super::super::msg::rmw::MotorCommand,

    /// 左前大腿关节电机命令
    pub fl_thigh: super::super::msg::rmw::MotorCommand,

    /// 左前小腿关节电机命令
    pub fl_calf: super::super::msg::rmw::MotorCommand,

    /// 右前髋关节电机命令
    pub fr_hip: super::super::msg::rmw::MotorCommand,

    /// 右前大腿关节电机命令
    pub fr_thigh: super::super::msg::rmw::MotorCommand,

    /// 右前小腿关节电机命令
    pub fr_calf: super::super::msg::rmw::MotorCommand,

    /// 左后髋关节电机命令
    pub rl_hip: super::super::msg::rmw::MotorCommand,

    /// 左后大腿关节电机命令
    pub rl_thigh: super::super::msg::rmw::MotorCommand,

    /// 左后小腿关节电机命令
    pub rl_calf: super::super::msg::rmw::MotorCommand,

    /// 右后髋关节电机命令
    pub rr_hip: super::super::msg::rmw::MotorCommand,

    /// 右后大腿关节电机命令
    pub rr_thigh: super::super::msg::rmw::MotorCommand,

    /// 右后小腿关节电机命令
    pub rr_calf: super::super::msg::rmw::MotorCommand,

}



impl Default for MotorCommand12 {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !interfaces__msg__MotorCommand12__init(&mut msg as *mut _) {
        panic!("Call to interfaces__msg__MotorCommand12__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for MotorCommand12 {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { interfaces__msg__MotorCommand12__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { interfaces__msg__MotorCommand12__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { interfaces__msg__MotorCommand12__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for MotorCommand12 {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for MotorCommand12 where Self: Sized {
  const TYPE_NAME: &'static str = "interfaces/msg/MotorCommand12";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__interfaces__msg__MotorCommand12() }
  }
}


#[link(name = "interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__interfaces__msg__MotorFeedback() -> *const std::ffi::c_void;
}

#[link(name = "interfaces__rosidl_generator_c")]
extern "C" {
    fn interfaces__msg__MotorFeedback__init(msg: *mut MotorFeedback) -> bool;
    fn interfaces__msg__MotorFeedback__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<MotorFeedback>, size: usize) -> bool;
    fn interfaces__msg__MotorFeedback__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<MotorFeedback>);
    fn interfaces__msg__MotorFeedback__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<MotorFeedback>, out_seq: *mut rosidl_runtime_rs::Sequence<MotorFeedback>) -> bool;
}

// Corresponds to interfaces__msg__MotorFeedback
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct MotorFeedback {
    /// 反馈力矩 (N·m)
    pub torque: f64,

    /// 反馈位置 (rad)
    pub position: f64,

    /// 反馈速度 (rad/s)
    pub velocity: f64,

    /// 电机温度 (°C)
    pub temperature: f64,

}



impl Default for MotorFeedback {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !interfaces__msg__MotorFeedback__init(&mut msg as *mut _) {
        panic!("Call to interfaces__msg__MotorFeedback__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for MotorFeedback {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { interfaces__msg__MotorFeedback__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { interfaces__msg__MotorFeedback__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { interfaces__msg__MotorFeedback__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for MotorFeedback {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for MotorFeedback where Self: Sized {
  const TYPE_NAME: &'static str = "interfaces/msg/MotorFeedback";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__interfaces__msg__MotorFeedback() }
  }
}


#[link(name = "interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__interfaces__msg__MotorFeedback12() -> *const std::ffi::c_void;
}

#[link(name = "interfaces__rosidl_generator_c")]
extern "C" {
    fn interfaces__msg__MotorFeedback12__init(msg: *mut MotorFeedback12) -> bool;
    fn interfaces__msg__MotorFeedback12__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<MotorFeedback12>, size: usize) -> bool;
    fn interfaces__msg__MotorFeedback12__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<MotorFeedback12>);
    fn interfaces__msg__MotorFeedback12__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<MotorFeedback12>, out_seq: *mut rosidl_runtime_rs::Sequence<MotorFeedback12>) -> bool;
}

// Corresponds to interfaces__msg__MotorFeedback12
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct MotorFeedback12 {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::rmw::Header,


    // This member is not documented.
    #[allow(missing_docs)]
    pub fl_hip: super::super::msg::rmw::MotorFeedback,


    // This member is not documented.
    #[allow(missing_docs)]
    pub fl_thigh: super::super::msg::rmw::MotorFeedback,


    // This member is not documented.
    #[allow(missing_docs)]
    pub fl_calf: super::super::msg::rmw::MotorFeedback,


    // This member is not documented.
    #[allow(missing_docs)]
    pub fr_hip: super::super::msg::rmw::MotorFeedback,


    // This member is not documented.
    #[allow(missing_docs)]
    pub fr_thigh: super::super::msg::rmw::MotorFeedback,


    // This member is not documented.
    #[allow(missing_docs)]
    pub fr_calf: super::super::msg::rmw::MotorFeedback,


    // This member is not documented.
    #[allow(missing_docs)]
    pub rl_hip: super::super::msg::rmw::MotorFeedback,


    // This member is not documented.
    #[allow(missing_docs)]
    pub rl_thigh: super::super::msg::rmw::MotorFeedback,


    // This member is not documented.
    #[allow(missing_docs)]
    pub rl_calf: super::super::msg::rmw::MotorFeedback,


    // This member is not documented.
    #[allow(missing_docs)]
    pub rr_hip: super::super::msg::rmw::MotorFeedback,


    // This member is not documented.
    #[allow(missing_docs)]
    pub rr_thigh: super::super::msg::rmw::MotorFeedback,


    // This member is not documented.
    #[allow(missing_docs)]
    pub rr_calf: super::super::msg::rmw::MotorFeedback,

}



impl Default for MotorFeedback12 {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !interfaces__msg__MotorFeedback12__init(&mut msg as *mut _) {
        panic!("Call to interfaces__msg__MotorFeedback12__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for MotorFeedback12 {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { interfaces__msg__MotorFeedback12__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { interfaces__msg__MotorFeedback12__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { interfaces__msg__MotorFeedback12__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for MotorFeedback12 {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for MotorFeedback12 where Self: Sized {
  const TYPE_NAME: &'static str = "interfaces/msg/MotorFeedback12";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__interfaces__msg__MotorFeedback12() }
  }
}


