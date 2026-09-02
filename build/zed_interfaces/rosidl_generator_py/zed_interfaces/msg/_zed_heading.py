# generated from rosidl_generator_py/resource/_idl.py.em
# with input from zed_interfaces:msg/ZedHeading.idl
# generated code does not contain a copyright notice

# This is being done at the module level and not on the instance level to avoid looking
# for the same variable multiple times on each instance. This variable is not supposed to
# change during runtime so it makes sense to only look for it once.
from os import getenv

ros_python_check_fields = getenv('ROS_PYTHON_CHECK_FIELDS', default='')


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_ZedHeading(type):
    """Metaclass of message 'ZedHeading'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('zed_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'zed_interfaces.msg.ZedHeading')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__zed_heading
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__zed_heading
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__zed_heading
            cls._TYPE_SUPPORT = module.type_support_msg__msg__zed_heading
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__zed_heading

            from std_msgs.msg import Header
            if Header.__class__._TYPE_SUPPORT is None:
                Header.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class ZedHeading(metaclass=Metaclass_ZedHeading):
    """Message class 'ZedHeading'."""

    __slots__ = [
        '_header',
        '_raw_x',
        '_raw_z',
        '_corrected_x',
        '_corrected_z',
        '_magnetic_heading_deg',
        '_robot_yaw_deg',
        '_robot_yaw_rad',
        '_valid',
        '_check_fields',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'raw_x': 'float',
        'raw_z': 'float',
        'corrected_x': 'float',
        'corrected_z': 'float',
        'magnetic_heading_deg': 'float',
        'robot_yaw_deg': 'float',
        'robot_yaw_rad': 'float',
        'valid': 'boolean',
    }

    # This attribute is used to store an rosidl_parser.definition variable
    # related to the data type of each of the components the message.
    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        if 'check_fields' in kwargs:
            self._check_fields = kwargs['check_fields']
        else:
            self._check_fields = ros_python_check_fields == '1'
        if self._check_fields:
            assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
                'Invalid arguments passed to constructor: %s' % \
                ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.raw_x = kwargs.get('raw_x', float())
        self.raw_z = kwargs.get('raw_z', float())
        self.corrected_x = kwargs.get('corrected_x', float())
        self.corrected_z = kwargs.get('corrected_z', float())
        self.magnetic_heading_deg = kwargs.get('magnetic_heading_deg', float())
        self.robot_yaw_deg = kwargs.get('robot_yaw_deg', float())
        self.robot_yaw_rad = kwargs.get('robot_yaw_rad', float())
        self.valid = kwargs.get('valid', bool())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.get_fields_and_field_types().keys(), self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    if self._check_fields:
                        assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.header != other.header:
            return False
        if self.raw_x != other.raw_x:
            return False
        if self.raw_z != other.raw_z:
            return False
        if self.corrected_x != other.corrected_x:
            return False
        if self.corrected_z != other.corrected_z:
            return False
        if self.magnetic_heading_deg != other.magnetic_heading_deg:
            return False
        if self.robot_yaw_deg != other.robot_yaw_deg:
            return False
        if self.robot_yaw_rad != other.robot_yaw_rad:
            return False
        if self.valid != other.valid:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def header(self):
        """Message field 'header'."""
        return self._header

    @header.setter
    def header(self, value):
        if self._check_fields:
            from std_msgs.msg import Header
            assert \
                isinstance(value, Header), \
                "The 'header' field must be a sub message of type 'Header'"
        self._header = value

    @builtins.property
    def raw_x(self):
        """Message field 'raw_x'."""
        return self._raw_x

    @raw_x.setter
    def raw_x(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'raw_x' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'raw_x' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._raw_x = value

    @builtins.property
    def raw_z(self):
        """Message field 'raw_z'."""
        return self._raw_z

    @raw_z.setter
    def raw_z(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'raw_z' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'raw_z' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._raw_z = value

    @builtins.property
    def corrected_x(self):
        """Message field 'corrected_x'."""
        return self._corrected_x

    @corrected_x.setter
    def corrected_x(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'corrected_x' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'corrected_x' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._corrected_x = value

    @builtins.property
    def corrected_z(self):
        """Message field 'corrected_z'."""
        return self._corrected_z

    @corrected_z.setter
    def corrected_z(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'corrected_z' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'corrected_z' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._corrected_z = value

    @builtins.property
    def magnetic_heading_deg(self):
        """Message field 'magnetic_heading_deg'."""
        return self._magnetic_heading_deg

    @magnetic_heading_deg.setter
    def magnetic_heading_deg(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'magnetic_heading_deg' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'magnetic_heading_deg' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._magnetic_heading_deg = value

    @builtins.property
    def robot_yaw_deg(self):
        """Message field 'robot_yaw_deg'."""
        return self._robot_yaw_deg

    @robot_yaw_deg.setter
    def robot_yaw_deg(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'robot_yaw_deg' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'robot_yaw_deg' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._robot_yaw_deg = value

    @builtins.property
    def robot_yaw_rad(self):
        """Message field 'robot_yaw_rad'."""
        return self._robot_yaw_rad

    @robot_yaw_rad.setter
    def robot_yaw_rad(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'robot_yaw_rad' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'robot_yaw_rad' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._robot_yaw_rad = value

    @builtins.property
    def valid(self):
        """Message field 'valid'."""
        return self._valid

    @valid.setter
    def valid(self, value):
        if self._check_fields:
            assert \
                isinstance(value, bool), \
                "The 'valid' field must be of type 'bool'"
        self._valid = value
