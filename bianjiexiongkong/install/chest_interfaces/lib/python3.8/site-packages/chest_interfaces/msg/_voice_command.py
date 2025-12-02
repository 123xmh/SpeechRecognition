# generated from rosidl_generator_py/resource/_idl.py.em
# with input from chest_interfaces:msg/VoiceCommand.idl
# generated code does not contain a copyright notice


# Import statements for member types

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_VoiceCommand(type):
    """Metaclass of message 'VoiceCommand'."""

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
            module = import_type_support('chest_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'chest_interfaces.msg.VoiceCommand')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__voice_command
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__voice_command
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__voice_command
            cls._TYPE_SUPPORT = module.type_support_msg__msg__voice_command
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__voice_command

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


class VoiceCommand(metaclass=Metaclass_VoiceCommand):
    """Message class 'VoiceCommand'."""

    __slots__ = [
        '_header',
        '_frame_header',
        '_command_id',
        '_param1',
        '_param2',
        '_param3',
        '_is_valid',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'frame_header': 'uint16',
        'command_id': 'uint32',
        'param1': 'int32',
        'param2': 'int32',
        'param3': 'uint8',
        'is_valid': 'boolean',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint16'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.frame_header = kwargs.get('frame_header', int())
        self.command_id = kwargs.get('command_id', int())
        self.param1 = kwargs.get('param1', int())
        self.param2 = kwargs.get('param2', int())
        self.param3 = kwargs.get('param3', int())
        self.is_valid = kwargs.get('is_valid', bool())

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
        if self.frame_header != other.frame_header:
            return False
        if self.command_id != other.command_id:
            return False
        if self.param1 != other.param1:
            return False
        if self.param2 != other.param2:
            return False
        if self.param3 != other.param3:
            return False
        if self.is_valid != other.is_valid:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @property
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

    @property
    def frame_header(self):
        """Message field 'frame_header'."""
        return self._frame_header

    @frame_header.setter
    def frame_header(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'frame_header' field must be of type 'int'"
            assert value >= 0 and value < 65536, \
                "The 'frame_header' field must be an unsigned integer in [0, 65535]"
        self._frame_header = value

    @property
    def command_id(self):
        """Message field 'command_id'."""
        return self._command_id

    @command_id.setter
    def command_id(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'command_id' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'command_id' field must be an unsigned integer in [0, 4294967295]"
        self._command_id = value

    @property
    def param1(self):
        """Message field 'param1'."""
        return self._param1

    @param1.setter
    def param1(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'param1' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'param1' field must be an integer in [-2147483648, 2147483647]"
        self._param1 = value

    @property
    def param2(self):
        """Message field 'param2'."""
        return self._param2

    @param2.setter
    def param2(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'param2' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'param2' field must be an integer in [-2147483648, 2147483647]"
        self._param2 = value

    @property
    def param3(self):
        """Message field 'param3'."""
        return self._param3

    @param3.setter
    def param3(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'param3' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'param3' field must be an unsigned integer in [0, 255]"
        self._param3 = value

    @property
    def is_valid(self):
        """Message field 'is_valid'."""
        return self._is_valid

    @is_valid.setter
    def is_valid(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'is_valid' field must be of type 'bool'"
        self._is_valid = value
