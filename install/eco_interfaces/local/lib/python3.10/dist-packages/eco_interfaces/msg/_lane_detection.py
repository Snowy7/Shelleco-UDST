# generated from rosidl_generator_py/resource/_idl.py.em
# with input from eco_interfaces:msg/LaneDetection.idl
# generated code does not contain a copyright notice


# Import statements for member types

# Member 'left_line_x'
# Member 'left_line_y'
# Member 'right_line_x'
# Member 'right_line_y'
import array  # noqa: E402, I100

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_LaneDetection(type):
    """Metaclass of message 'LaneDetection'."""

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
            module = import_type_support('eco_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'eco_interfaces.msg.LaneDetection')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__lane_detection
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__lane_detection
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__lane_detection
            cls._TYPE_SUPPORT = module.type_support_msg__msg__lane_detection
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__lane_detection

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


class LaneDetection(metaclass=Metaclass_LaneDetection):
    """Message class 'LaneDetection'."""

    __slots__ = [
        '_header',
        '_left_line_x',
        '_left_line_y',
        '_right_line_x',
        '_right_line_y',
        '_lane_center_offset',
        '_lane_heading_error',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'left_line_x': 'sequence<float>',
        'left_line_y': 'sequence<float>',
        'right_line_x': 'sequence<float>',
        'right_line_y': 'sequence<float>',
        'lane_center_offset': 'float',
        'lane_heading_error': 'float',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('float')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('float')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('float')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('float')),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.left_line_x = array.array('f', kwargs.get('left_line_x', []))
        self.left_line_y = array.array('f', kwargs.get('left_line_y', []))
        self.right_line_x = array.array('f', kwargs.get('right_line_x', []))
        self.right_line_y = array.array('f', kwargs.get('right_line_y', []))
        self.lane_center_offset = kwargs.get('lane_center_offset', float())
        self.lane_heading_error = kwargs.get('lane_heading_error', float())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
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
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.header != other.header:
            return False
        if self.left_line_x != other.left_line_x:
            return False
        if self.left_line_y != other.left_line_y:
            return False
        if self.right_line_x != other.right_line_x:
            return False
        if self.right_line_y != other.right_line_y:
            return False
        if self.lane_center_offset != other.lane_center_offset:
            return False
        if self.lane_heading_error != other.lane_heading_error:
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
        if __debug__:
            from std_msgs.msg import Header
            assert \
                isinstance(value, Header), \
                "The 'header' field must be a sub message of type 'Header'"
        self._header = value

    @builtins.property
    def left_line_x(self):
        """Message field 'left_line_x'."""
        return self._left_line_x

    @left_line_x.setter
    def left_line_x(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'f', \
                "The 'left_line_x' array.array() must have the type code of 'f'"
            self._left_line_x = value
            return
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -3.402823466e+38 or val > 3.402823466e+38) or math.isinf(val) for val in value)), \
                "The 'left_line_x' field must be a set or sequence and each value of type 'float' and each float in [-340282346600000016151267322115014000640.000000, 340282346600000016151267322115014000640.000000]"
        self._left_line_x = array.array('f', value)

    @builtins.property
    def left_line_y(self):
        """Message field 'left_line_y'."""
        return self._left_line_y

    @left_line_y.setter
    def left_line_y(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'f', \
                "The 'left_line_y' array.array() must have the type code of 'f'"
            self._left_line_y = value
            return
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -3.402823466e+38 or val > 3.402823466e+38) or math.isinf(val) for val in value)), \
                "The 'left_line_y' field must be a set or sequence and each value of type 'float' and each float in [-340282346600000016151267322115014000640.000000, 340282346600000016151267322115014000640.000000]"
        self._left_line_y = array.array('f', value)

    @builtins.property
    def right_line_x(self):
        """Message field 'right_line_x'."""
        return self._right_line_x

    @right_line_x.setter
    def right_line_x(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'f', \
                "The 'right_line_x' array.array() must have the type code of 'f'"
            self._right_line_x = value
            return
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -3.402823466e+38 or val > 3.402823466e+38) or math.isinf(val) for val in value)), \
                "The 'right_line_x' field must be a set or sequence and each value of type 'float' and each float in [-340282346600000016151267322115014000640.000000, 340282346600000016151267322115014000640.000000]"
        self._right_line_x = array.array('f', value)

    @builtins.property
    def right_line_y(self):
        """Message field 'right_line_y'."""
        return self._right_line_y

    @right_line_y.setter
    def right_line_y(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'f', \
                "The 'right_line_y' array.array() must have the type code of 'f'"
            self._right_line_y = value
            return
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -3.402823466e+38 or val > 3.402823466e+38) or math.isinf(val) for val in value)), \
                "The 'right_line_y' field must be a set or sequence and each value of type 'float' and each float in [-340282346600000016151267322115014000640.000000, 340282346600000016151267322115014000640.000000]"
        self._right_line_y = array.array('f', value)

    @builtins.property
    def lane_center_offset(self):
        """Message field 'lane_center_offset'."""
        return self._lane_center_offset

    @lane_center_offset.setter
    def lane_center_offset(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'lane_center_offset' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'lane_center_offset' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._lane_center_offset = value

    @builtins.property
    def lane_heading_error(self):
        """Message field 'lane_heading_error'."""
        return self._lane_heading_error

    @lane_heading_error.setter
    def lane_heading_error(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'lane_heading_error' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'lane_heading_error' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._lane_heading_error = value
