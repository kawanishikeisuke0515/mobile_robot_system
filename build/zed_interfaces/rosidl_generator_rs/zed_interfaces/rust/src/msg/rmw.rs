#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};


#[link(name = "zed_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__zed_interfaces__msg__ZedHeading() -> *const std::ffi::c_void;
}

#[link(name = "zed_interfaces__rosidl_generator_c")]
extern "C" {
    fn zed_interfaces__msg__ZedHeading__init(msg: *mut ZedHeading) -> bool;
    fn zed_interfaces__msg__ZedHeading__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ZedHeading>, size: usize) -> bool;
    fn zed_interfaces__msg__ZedHeading__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ZedHeading>);
    fn zed_interfaces__msg__ZedHeading__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ZedHeading>, out_seq: *mut rosidl_runtime_rs::Sequence<ZedHeading>) -> bool;
}

// Corresponds to zed_interfaces__msg__ZedHeading
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ZedHeading {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::rmw::Header,


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
    unsafe {
      let mut msg = std::mem::zeroed();
      if !zed_interfaces__msg__ZedHeading__init(&mut msg as *mut _) {
        panic!("Call to zed_interfaces__msg__ZedHeading__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ZedHeading {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { zed_interfaces__msg__ZedHeading__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { zed_interfaces__msg__ZedHeading__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { zed_interfaces__msg__ZedHeading__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ZedHeading {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ZedHeading where Self: Sized {
  const TYPE_NAME: &'static str = "zed_interfaces/msg/ZedHeading";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__zed_interfaces__msg__ZedHeading() }
  }
}


