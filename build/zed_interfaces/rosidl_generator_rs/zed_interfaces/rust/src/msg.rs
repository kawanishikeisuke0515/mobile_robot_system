#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};



// Corresponds to zed_interfaces__msg__ZedHeading

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ZedHeading {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::Header,


    // This member is not documented.
    #[allow(missing_docs)]
    pub raw_x: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub raw_z: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub corrected_x: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub corrected_z: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub magnetic_heading_deg: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub robot_yaw_deg: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub robot_yaw_rad: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub valid: bool,

}



impl Default for ZedHeading {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::ZedHeading::default())
  }
}

impl rosidl_runtime_rs::Message for ZedHeading {
  type RmwMsg = super::msg::rmw::ZedHeading;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Owned(msg.header)).into_owned(),
        raw_x: msg.raw_x,
        raw_z: msg.raw_z,
        corrected_x: msg.corrected_x,
        corrected_z: msg.corrected_z,
        magnetic_heading_deg: msg.magnetic_heading_deg,
        robot_yaw_deg: msg.robot_yaw_deg,
        robot_yaw_rad: msg.robot_yaw_rad,
        valid: msg.valid,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Borrowed(&msg.header)).into_owned(),
      raw_x: msg.raw_x,
      raw_z: msg.raw_z,
      corrected_x: msg.corrected_x,
      corrected_z: msg.corrected_z,
      magnetic_heading_deg: msg.magnetic_heading_deg,
      robot_yaw_deg: msg.robot_yaw_deg,
      robot_yaw_rad: msg.robot_yaw_rad,
      valid: msg.valid,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      header: std_msgs::msg::Header::from_rmw_message(msg.header),
      raw_x: msg.raw_x,
      raw_z: msg.raw_z,
      corrected_x: msg.corrected_x,
      corrected_z: msg.corrected_z,
      magnetic_heading_deg: msg.magnetic_heading_deg,
      robot_yaw_deg: msg.robot_yaw_deg,
      robot_yaw_rad: msg.robot_yaw_rad,
      valid: msg.valid,
    }
  }
}


