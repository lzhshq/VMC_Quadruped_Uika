#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};



// Corresponds to interfaces__msg__BaseState

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct BaseState {
    /// 消息头：包含时间戳和坐标系名称
    pub header: std_msgs::msg::Header,

    /// 机身位置，单位 m，建议在 world 坐标系下表达
    pub position: geometry_msgs::msg::Point,

    /// 机身姿态，四元数表示
    pub orientation: geometry_msgs::msg::Quaternion,

    /// 机身线速度，单位 m/s
    pub linear_velocity: geometry_msgs::msg::Vector3,

    /// 机身角速度，单位 rad/s
    pub angular_velocity: geometry_msgs::msg::Vector3,

}



impl Default for BaseState {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::BaseState::default())
  }
}

impl rosidl_runtime_rs::Message for BaseState {
  type RmwMsg = super::msg::rmw::BaseState;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Owned(msg.header)).into_owned(),
        position: geometry_msgs::msg::Point::into_rmw_message(std::borrow::Cow::Owned(msg.position)).into_owned(),
        orientation: geometry_msgs::msg::Quaternion::into_rmw_message(std::borrow::Cow::Owned(msg.orientation)).into_owned(),
        linear_velocity: geometry_msgs::msg::Vector3::into_rmw_message(std::borrow::Cow::Owned(msg.linear_velocity)).into_owned(),
        angular_velocity: geometry_msgs::msg::Vector3::into_rmw_message(std::borrow::Cow::Owned(msg.angular_velocity)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Borrowed(&msg.header)).into_owned(),
        position: geometry_msgs::msg::Point::into_rmw_message(std::borrow::Cow::Borrowed(&msg.position)).into_owned(),
        orientation: geometry_msgs::msg::Quaternion::into_rmw_message(std::borrow::Cow::Borrowed(&msg.orientation)).into_owned(),
        linear_velocity: geometry_msgs::msg::Vector3::into_rmw_message(std::borrow::Cow::Borrowed(&msg.linear_velocity)).into_owned(),
        angular_velocity: geometry_msgs::msg::Vector3::into_rmw_message(std::borrow::Cow::Borrowed(&msg.angular_velocity)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      header: std_msgs::msg::Header::from_rmw_message(msg.header),
      position: geometry_msgs::msg::Point::from_rmw_message(msg.position),
      orientation: geometry_msgs::msg::Quaternion::from_rmw_message(msg.orientation),
      linear_velocity: geometry_msgs::msg::Vector3::from_rmw_message(msg.linear_velocity),
      angular_velocity: geometry_msgs::msg::Vector3::from_rmw_message(msg.angular_velocity),
    }
  }
}


// Corresponds to interfaces__msg__ContactState

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ContactState {
    /// 消息头：包含时间戳和坐标系名称
    pub header: std_msgs::msg::Header,

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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::ContactState::default())
  }
}

impl rosidl_runtime_rs::Message for ContactState {
  type RmwMsg = super::msg::rmw::ContactState;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Owned(msg.header)).into_owned(),
        fl_contact: msg.fl_contact,
        fr_contact: msg.fr_contact,
        rl_contact: msg.rl_contact,
        rr_contact: msg.rr_contact,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Borrowed(&msg.header)).into_owned(),
      fl_contact: msg.fl_contact,
      fr_contact: msg.fr_contact,
      rl_contact: msg.rl_contact,
      rr_contact: msg.rr_contact,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      header: std_msgs::msg::Header::from_rmw_message(msg.header),
      fl_contact: msg.fl_contact,
      fr_contact: msg.fr_contact,
      rl_contact: msg.rl_contact,
      rr_contact: msg.rr_contact,
    }
  }
}


// Corresponds to interfaces__msg__GaitPhase

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct GaitPhase {
    /// 消息头：包含时间戳和坐标系名称
    pub header: std_msgs::msg::Header,

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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::GaitPhase::default())
  }
}

impl rosidl_runtime_rs::Message for GaitPhase {
  type RmwMsg = super::msg::rmw::GaitPhase;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Owned(msg.header)).into_owned(),
        fl_phase: msg.fl_phase,
        fr_phase: msg.fr_phase,
        rl_phase: msg.rl_phase,
        rr_phase: msg.rr_phase,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Borrowed(&msg.header)).into_owned(),
      fl_phase: msg.fl_phase,
      fr_phase: msg.fr_phase,
      rl_phase: msg.rl_phase,
      rr_phase: msg.rr_phase,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      header: std_msgs::msg::Header::from_rmw_message(msg.header),
      fl_phase: msg.fl_phase,
      fr_phase: msg.fr_phase,
      rl_phase: msg.rl_phase,
      rr_phase: msg.rr_phase,
    }
  }
}


// Corresponds to interfaces__msg__MotionCommand

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct MotionCommand {
    /// 消息头：主要用于时间戳
    pub header: std_msgs::msg::Header,

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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::MotionCommand::default())
  }
}

impl rosidl_runtime_rs::Message for MotionCommand {
  type RmwMsg = super::msg::rmw::MotionCommand;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Owned(msg.header)).into_owned(),
        vx: msg.vx,
        vy: msg.vy,
        wz: msg.wz,
        body_height: msg.body_height,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Borrowed(&msg.header)).into_owned(),
      vx: msg.vx,
      vy: msg.vy,
      wz: msg.wz,
      body_height: msg.body_height,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      header: std_msgs::msg::Header::from_rmw_message(msg.header),
      vx: msg.vx,
      vy: msg.vy,
      wz: msg.wz,
      body_height: msg.body_height,
    }
  }
}


// Corresponds to interfaces__msg__SingleFootState

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SingleFootState {
    /// 单条腿足端位置，单位 m
    pub position: geometry_msgs::msg::Point,

    /// 单条腿足端速度，单位 m/s
    pub velocity: geometry_msgs::msg::Vector3,

    /// 单条腿足端受力或期望力，单位 N
    pub force: geometry_msgs::msg::Vector3,

}



impl Default for SingleFootState {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::SingleFootState::default())
  }
}

impl rosidl_runtime_rs::Message for SingleFootState {
  type RmwMsg = super::msg::rmw::SingleFootState;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        position: geometry_msgs::msg::Point::into_rmw_message(std::borrow::Cow::Owned(msg.position)).into_owned(),
        velocity: geometry_msgs::msg::Vector3::into_rmw_message(std::borrow::Cow::Owned(msg.velocity)).into_owned(),
        force: geometry_msgs::msg::Vector3::into_rmw_message(std::borrow::Cow::Owned(msg.force)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        position: geometry_msgs::msg::Point::into_rmw_message(std::borrow::Cow::Borrowed(&msg.position)).into_owned(),
        velocity: geometry_msgs::msg::Vector3::into_rmw_message(std::borrow::Cow::Borrowed(&msg.velocity)).into_owned(),
        force: geometry_msgs::msg::Vector3::into_rmw_message(std::borrow::Cow::Borrowed(&msg.force)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      position: geometry_msgs::msg::Point::from_rmw_message(msg.position),
      velocity: geometry_msgs::msg::Vector3::from_rmw_message(msg.velocity),
      force: geometry_msgs::msg::Vector3::from_rmw_message(msg.force),
    }
  }
}


// Corresponds to interfaces__msg__FootState

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct FootState {
    /// 消息头：包含时间戳和坐标系名称
    pub header: std_msgs::msg::Header,

    /// 左前腿足端状态
    pub fl: super::msg::SingleFootState,

    /// 右前腿足端状态
    pub fr: super::msg::SingleFootState,

    /// 左后腿足端状态
    pub rl: super::msg::SingleFootState,

    /// 右后腿足端状态
    pub rr: super::msg::SingleFootState,

}



impl Default for FootState {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::FootState::default())
  }
}

impl rosidl_runtime_rs::Message for FootState {
  type RmwMsg = super::msg::rmw::FootState;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Owned(msg.header)).into_owned(),
        fl: super::msg::SingleFootState::into_rmw_message(std::borrow::Cow::Owned(msg.fl)).into_owned(),
        fr: super::msg::SingleFootState::into_rmw_message(std::borrow::Cow::Owned(msg.fr)).into_owned(),
        rl: super::msg::SingleFootState::into_rmw_message(std::borrow::Cow::Owned(msg.rl)).into_owned(),
        rr: super::msg::SingleFootState::into_rmw_message(std::borrow::Cow::Owned(msg.rr)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Borrowed(&msg.header)).into_owned(),
        fl: super::msg::SingleFootState::into_rmw_message(std::borrow::Cow::Borrowed(&msg.fl)).into_owned(),
        fr: super::msg::SingleFootState::into_rmw_message(std::borrow::Cow::Borrowed(&msg.fr)).into_owned(),
        rl: super::msg::SingleFootState::into_rmw_message(std::borrow::Cow::Borrowed(&msg.rl)).into_owned(),
        rr: super::msg::SingleFootState::into_rmw_message(std::borrow::Cow::Borrowed(&msg.rr)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      header: std_msgs::msg::Header::from_rmw_message(msg.header),
      fl: super::msg::SingleFootState::from_rmw_message(msg.fl),
      fr: super::msg::SingleFootState::from_rmw_message(msg.fr),
      rl: super::msg::SingleFootState::from_rmw_message(msg.rl),
      rr: super::msg::SingleFootState::from_rmw_message(msg.rr),
    }
  }
}


// Corresponds to interfaces__msg__MotorCommand

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::MotorCommand::default())
  }
}

impl rosidl_runtime_rs::Message for MotorCommand {
  type RmwMsg = super::msg::rmw::MotorCommand;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        torque: msg.torque,
        position: msg.position,
        velocity: msg.velocity,
        kp: msg.kp,
        kd: msg.kd,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      torque: msg.torque,
      position: msg.position,
      velocity: msg.velocity,
      kp: msg.kp,
      kd: msg.kd,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      torque: msg.torque,
      position: msg.position,
      velocity: msg.velocity,
      kp: msg.kp,
      kd: msg.kd,
    }
  }
}


// Corresponds to interfaces__msg__MotorCommand12

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct MotorCommand12 {
    /// 消息头：包含时间戳
    pub header: std_msgs::msg::Header,

    /// 左前髋关节电机命令
    pub fl_hip: super::msg::MotorCommand,

    /// 左前大腿关节电机命令
    pub fl_thigh: super::msg::MotorCommand,

    /// 左前小腿关节电机命令
    pub fl_calf: super::msg::MotorCommand,

    /// 右前髋关节电机命令
    pub fr_hip: super::msg::MotorCommand,

    /// 右前大腿关节电机命令
    pub fr_thigh: super::msg::MotorCommand,

    /// 右前小腿关节电机命令
    pub fr_calf: super::msg::MotorCommand,

    /// 左后髋关节电机命令
    pub rl_hip: super::msg::MotorCommand,

    /// 左后大腿关节电机命令
    pub rl_thigh: super::msg::MotorCommand,

    /// 左后小腿关节电机命令
    pub rl_calf: super::msg::MotorCommand,

    /// 右后髋关节电机命令
    pub rr_hip: super::msg::MotorCommand,

    /// 右后大腿关节电机命令
    pub rr_thigh: super::msg::MotorCommand,

    /// 右后小腿关节电机命令
    pub rr_calf: super::msg::MotorCommand,

}



impl Default for MotorCommand12 {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::MotorCommand12::default())
  }
}

impl rosidl_runtime_rs::Message for MotorCommand12 {
  type RmwMsg = super::msg::rmw::MotorCommand12;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Owned(msg.header)).into_owned(),
        fl_hip: super::msg::MotorCommand::into_rmw_message(std::borrow::Cow::Owned(msg.fl_hip)).into_owned(),
        fl_thigh: super::msg::MotorCommand::into_rmw_message(std::borrow::Cow::Owned(msg.fl_thigh)).into_owned(),
        fl_calf: super::msg::MotorCommand::into_rmw_message(std::borrow::Cow::Owned(msg.fl_calf)).into_owned(),
        fr_hip: super::msg::MotorCommand::into_rmw_message(std::borrow::Cow::Owned(msg.fr_hip)).into_owned(),
        fr_thigh: super::msg::MotorCommand::into_rmw_message(std::borrow::Cow::Owned(msg.fr_thigh)).into_owned(),
        fr_calf: super::msg::MotorCommand::into_rmw_message(std::borrow::Cow::Owned(msg.fr_calf)).into_owned(),
        rl_hip: super::msg::MotorCommand::into_rmw_message(std::borrow::Cow::Owned(msg.rl_hip)).into_owned(),
        rl_thigh: super::msg::MotorCommand::into_rmw_message(std::borrow::Cow::Owned(msg.rl_thigh)).into_owned(),
        rl_calf: super::msg::MotorCommand::into_rmw_message(std::borrow::Cow::Owned(msg.rl_calf)).into_owned(),
        rr_hip: super::msg::MotorCommand::into_rmw_message(std::borrow::Cow::Owned(msg.rr_hip)).into_owned(),
        rr_thigh: super::msg::MotorCommand::into_rmw_message(std::borrow::Cow::Owned(msg.rr_thigh)).into_owned(),
        rr_calf: super::msg::MotorCommand::into_rmw_message(std::borrow::Cow::Owned(msg.rr_calf)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Borrowed(&msg.header)).into_owned(),
        fl_hip: super::msg::MotorCommand::into_rmw_message(std::borrow::Cow::Borrowed(&msg.fl_hip)).into_owned(),
        fl_thigh: super::msg::MotorCommand::into_rmw_message(std::borrow::Cow::Borrowed(&msg.fl_thigh)).into_owned(),
        fl_calf: super::msg::MotorCommand::into_rmw_message(std::borrow::Cow::Borrowed(&msg.fl_calf)).into_owned(),
        fr_hip: super::msg::MotorCommand::into_rmw_message(std::borrow::Cow::Borrowed(&msg.fr_hip)).into_owned(),
        fr_thigh: super::msg::MotorCommand::into_rmw_message(std::borrow::Cow::Borrowed(&msg.fr_thigh)).into_owned(),
        fr_calf: super::msg::MotorCommand::into_rmw_message(std::borrow::Cow::Borrowed(&msg.fr_calf)).into_owned(),
        rl_hip: super::msg::MotorCommand::into_rmw_message(std::borrow::Cow::Borrowed(&msg.rl_hip)).into_owned(),
        rl_thigh: super::msg::MotorCommand::into_rmw_message(std::borrow::Cow::Borrowed(&msg.rl_thigh)).into_owned(),
        rl_calf: super::msg::MotorCommand::into_rmw_message(std::borrow::Cow::Borrowed(&msg.rl_calf)).into_owned(),
        rr_hip: super::msg::MotorCommand::into_rmw_message(std::borrow::Cow::Borrowed(&msg.rr_hip)).into_owned(),
        rr_thigh: super::msg::MotorCommand::into_rmw_message(std::borrow::Cow::Borrowed(&msg.rr_thigh)).into_owned(),
        rr_calf: super::msg::MotorCommand::into_rmw_message(std::borrow::Cow::Borrowed(&msg.rr_calf)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      header: std_msgs::msg::Header::from_rmw_message(msg.header),
      fl_hip: super::msg::MotorCommand::from_rmw_message(msg.fl_hip),
      fl_thigh: super::msg::MotorCommand::from_rmw_message(msg.fl_thigh),
      fl_calf: super::msg::MotorCommand::from_rmw_message(msg.fl_calf),
      fr_hip: super::msg::MotorCommand::from_rmw_message(msg.fr_hip),
      fr_thigh: super::msg::MotorCommand::from_rmw_message(msg.fr_thigh),
      fr_calf: super::msg::MotorCommand::from_rmw_message(msg.fr_calf),
      rl_hip: super::msg::MotorCommand::from_rmw_message(msg.rl_hip),
      rl_thigh: super::msg::MotorCommand::from_rmw_message(msg.rl_thigh),
      rl_calf: super::msg::MotorCommand::from_rmw_message(msg.rl_calf),
      rr_hip: super::msg::MotorCommand::from_rmw_message(msg.rr_hip),
      rr_thigh: super::msg::MotorCommand::from_rmw_message(msg.rr_thigh),
      rr_calf: super::msg::MotorCommand::from_rmw_message(msg.rr_calf),
    }
  }
}


// Corresponds to interfaces__msg__MotorFeedback

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::MotorFeedback::default())
  }
}

impl rosidl_runtime_rs::Message for MotorFeedback {
  type RmwMsg = super::msg::rmw::MotorFeedback;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        torque: msg.torque,
        position: msg.position,
        velocity: msg.velocity,
        temperature: msg.temperature,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      torque: msg.torque,
      position: msg.position,
      velocity: msg.velocity,
      temperature: msg.temperature,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      torque: msg.torque,
      position: msg.position,
      velocity: msg.velocity,
      temperature: msg.temperature,
    }
  }
}


// Corresponds to interfaces__msg__MotorFeedback12

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct MotorFeedback12 {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::Header,


    // This member is not documented.
    #[allow(missing_docs)]
    pub fl_hip: super::msg::MotorFeedback,


    // This member is not documented.
    #[allow(missing_docs)]
    pub fl_thigh: super::msg::MotorFeedback,


    // This member is not documented.
    #[allow(missing_docs)]
    pub fl_calf: super::msg::MotorFeedback,


    // This member is not documented.
    #[allow(missing_docs)]
    pub fr_hip: super::msg::MotorFeedback,


    // This member is not documented.
    #[allow(missing_docs)]
    pub fr_thigh: super::msg::MotorFeedback,


    // This member is not documented.
    #[allow(missing_docs)]
    pub fr_calf: super::msg::MotorFeedback,


    // This member is not documented.
    #[allow(missing_docs)]
    pub rl_hip: super::msg::MotorFeedback,


    // This member is not documented.
    #[allow(missing_docs)]
    pub rl_thigh: super::msg::MotorFeedback,


    // This member is not documented.
    #[allow(missing_docs)]
    pub rl_calf: super::msg::MotorFeedback,


    // This member is not documented.
    #[allow(missing_docs)]
    pub rr_hip: super::msg::MotorFeedback,


    // This member is not documented.
    #[allow(missing_docs)]
    pub rr_thigh: super::msg::MotorFeedback,


    // This member is not documented.
    #[allow(missing_docs)]
    pub rr_calf: super::msg::MotorFeedback,

}



impl Default for MotorFeedback12 {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::MotorFeedback12::default())
  }
}

impl rosidl_runtime_rs::Message for MotorFeedback12 {
  type RmwMsg = super::msg::rmw::MotorFeedback12;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Owned(msg.header)).into_owned(),
        fl_hip: super::msg::MotorFeedback::into_rmw_message(std::borrow::Cow::Owned(msg.fl_hip)).into_owned(),
        fl_thigh: super::msg::MotorFeedback::into_rmw_message(std::borrow::Cow::Owned(msg.fl_thigh)).into_owned(),
        fl_calf: super::msg::MotorFeedback::into_rmw_message(std::borrow::Cow::Owned(msg.fl_calf)).into_owned(),
        fr_hip: super::msg::MotorFeedback::into_rmw_message(std::borrow::Cow::Owned(msg.fr_hip)).into_owned(),
        fr_thigh: super::msg::MotorFeedback::into_rmw_message(std::borrow::Cow::Owned(msg.fr_thigh)).into_owned(),
        fr_calf: super::msg::MotorFeedback::into_rmw_message(std::borrow::Cow::Owned(msg.fr_calf)).into_owned(),
        rl_hip: super::msg::MotorFeedback::into_rmw_message(std::borrow::Cow::Owned(msg.rl_hip)).into_owned(),
        rl_thigh: super::msg::MotorFeedback::into_rmw_message(std::borrow::Cow::Owned(msg.rl_thigh)).into_owned(),
        rl_calf: super::msg::MotorFeedback::into_rmw_message(std::borrow::Cow::Owned(msg.rl_calf)).into_owned(),
        rr_hip: super::msg::MotorFeedback::into_rmw_message(std::borrow::Cow::Owned(msg.rr_hip)).into_owned(),
        rr_thigh: super::msg::MotorFeedback::into_rmw_message(std::borrow::Cow::Owned(msg.rr_thigh)).into_owned(),
        rr_calf: super::msg::MotorFeedback::into_rmw_message(std::borrow::Cow::Owned(msg.rr_calf)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Borrowed(&msg.header)).into_owned(),
        fl_hip: super::msg::MotorFeedback::into_rmw_message(std::borrow::Cow::Borrowed(&msg.fl_hip)).into_owned(),
        fl_thigh: super::msg::MotorFeedback::into_rmw_message(std::borrow::Cow::Borrowed(&msg.fl_thigh)).into_owned(),
        fl_calf: super::msg::MotorFeedback::into_rmw_message(std::borrow::Cow::Borrowed(&msg.fl_calf)).into_owned(),
        fr_hip: super::msg::MotorFeedback::into_rmw_message(std::borrow::Cow::Borrowed(&msg.fr_hip)).into_owned(),
        fr_thigh: super::msg::MotorFeedback::into_rmw_message(std::borrow::Cow::Borrowed(&msg.fr_thigh)).into_owned(),
        fr_calf: super::msg::MotorFeedback::into_rmw_message(std::borrow::Cow::Borrowed(&msg.fr_calf)).into_owned(),
        rl_hip: super::msg::MotorFeedback::into_rmw_message(std::borrow::Cow::Borrowed(&msg.rl_hip)).into_owned(),
        rl_thigh: super::msg::MotorFeedback::into_rmw_message(std::borrow::Cow::Borrowed(&msg.rl_thigh)).into_owned(),
        rl_calf: super::msg::MotorFeedback::into_rmw_message(std::borrow::Cow::Borrowed(&msg.rl_calf)).into_owned(),
        rr_hip: super::msg::MotorFeedback::into_rmw_message(std::borrow::Cow::Borrowed(&msg.rr_hip)).into_owned(),
        rr_thigh: super::msg::MotorFeedback::into_rmw_message(std::borrow::Cow::Borrowed(&msg.rr_thigh)).into_owned(),
        rr_calf: super::msg::MotorFeedback::into_rmw_message(std::borrow::Cow::Borrowed(&msg.rr_calf)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      header: std_msgs::msg::Header::from_rmw_message(msg.header),
      fl_hip: super::msg::MotorFeedback::from_rmw_message(msg.fl_hip),
      fl_thigh: super::msg::MotorFeedback::from_rmw_message(msg.fl_thigh),
      fl_calf: super::msg::MotorFeedback::from_rmw_message(msg.fl_calf),
      fr_hip: super::msg::MotorFeedback::from_rmw_message(msg.fr_hip),
      fr_thigh: super::msg::MotorFeedback::from_rmw_message(msg.fr_thigh),
      fr_calf: super::msg::MotorFeedback::from_rmw_message(msg.fr_calf),
      rl_hip: super::msg::MotorFeedback::from_rmw_message(msg.rl_hip),
      rl_thigh: super::msg::MotorFeedback::from_rmw_message(msg.rl_thigh),
      rl_calf: super::msg::MotorFeedback::from_rmw_message(msg.rl_calf),
      rr_hip: super::msg::MotorFeedback::from_rmw_message(msg.rr_hip),
      rr_thigh: super::msg::MotorFeedback::from_rmw_message(msg.rr_thigh),
      rr_calf: super::msg::MotorFeedback::from_rmw_message(msg.rr_calf),
    }
  }
}


