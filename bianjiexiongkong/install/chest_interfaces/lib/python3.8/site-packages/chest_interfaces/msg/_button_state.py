# generated from rosidl_generator_py/resource/_idl.py.em
# with input from chest_interfaces:msg/ButtonState.idl
# generated code does not contain a copyright notice


# Import statements for member types

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_ButtonState(type):
    """Metaclass of message 'ButtonState'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'FRONT_SCREEN_DEFAULT': 0,
        'FRONT_SCREEN_MODE_A': 1,
        'FRONT_SCREEN_MODE_B': 2,
        'FRONT_SCREEN_MODE_C': 3,
        'HMD_OVERLAY_NONE': 0,
        'HMD_OVERLAY_MINIMAL': 1,
        'HMD_OVERLAY_FULL': 2,
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
                'chest_interfaces.msg.ButtonState')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__button_state
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__button_state
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__button_state
            cls._TYPE_SUPPORT = module.type_support_msg__msg__button_state
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__button_state

            from std_msgs.msg import Header
            if Header.__class__._TYPE_SUPPORT is None:
                Header.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'FRONT_SCREEN_DEFAULT': cls.__constants['FRONT_SCREEN_DEFAULT'],
            'FRONT_SCREEN_MODE_A': cls.__constants['FRONT_SCREEN_MODE_A'],
            'FRONT_SCREEN_MODE_B': cls.__constants['FRONT_SCREEN_MODE_B'],
            'FRONT_SCREEN_MODE_C': cls.__constants['FRONT_SCREEN_MODE_C'],
            'HMD_OVERLAY_NONE': cls.__constants['HMD_OVERLAY_NONE'],
            'HMD_OVERLAY_MINIMAL': cls.__constants['HMD_OVERLAY_MINIMAL'],
            'HMD_OVERLAY_FULL': cls.__constants['HMD_OVERLAY_FULL'],
        }

    @property
    def FRONT_SCREEN_DEFAULT(self):
        """Message constant 'FRONT_SCREEN_DEFAULT'."""
        return Metaclass_ButtonState.__constants['FRONT_SCREEN_DEFAULT']

    @property
    def FRONT_SCREEN_MODE_A(self):
        """Message constant 'FRONT_SCREEN_MODE_A'."""
        return Metaclass_ButtonState.__constants['FRONT_SCREEN_MODE_A']

    @property
    def FRONT_SCREEN_MODE_B(self):
        """Message constant 'FRONT_SCREEN_MODE_B'."""
        return Metaclass_ButtonState.__constants['FRONT_SCREEN_MODE_B']

    @property
    def FRONT_SCREEN_MODE_C(self):
        """Message constant 'FRONT_SCREEN_MODE_C'."""
        return Metaclass_ButtonState.__constants['FRONT_SCREEN_MODE_C']

    @property
    def HMD_OVERLAY_NONE(self):
        """Message constant 'HMD_OVERLAY_NONE'."""
        return Metaclass_ButtonState.__constants['HMD_OVERLAY_NONE']

    @property
    def HMD_OVERLAY_MINIMAL(self):
        """Message constant 'HMD_OVERLAY_MINIMAL'."""
        return Metaclass_ButtonState.__constants['HMD_OVERLAY_MINIMAL']

    @property
    def HMD_OVERLAY_FULL(self):
        """Message constant 'HMD_OVERLAY_FULL'."""
        return Metaclass_ButtonState.__constants['HMD_OVERLAY_FULL']


class ButtonState(metaclass=Metaclass_ButtonState):
    """
    Message class 'ButtonState'.

    Constants:
      FRONT_SCREEN_DEFAULT
      FRONT_SCREEN_MODE_A
      FRONT_SCREEN_MODE_B
      FRONT_SCREEN_MODE_C
      HMD_OVERLAY_NONE
      HMD_OVERLAY_MINIMAL
      HMD_OVERLAY_FULL
    """

    __slots__ = [
        '_header',
        '_head_tracking_enabled',
        '_voice_command_enabled',
        '_front_screen_enabled',
        '_front_screen_mode',
        '_hmd_video_source',
        '_hmd_overlay_mode',
        '_hmd_enabled',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'head_tracking_enabled': 'boolean',
        'voice_command_enabled': 'boolean',
        'front_screen_enabled': 'boolean',
        'front_screen_mode': 'uint8',
        'hmd_video_source': 'uint8',
        'hmd_overlay_mode': 'uint8',
        'hmd_enabled': 'boolean',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.head_tracking_enabled = kwargs.get('head_tracking_enabled', bool())
        self.voice_command_enabled = kwargs.get('voice_command_enabled', bool())
        self.front_screen_enabled = kwargs.get('front_screen_enabled', bool())
        self.front_screen_mode = kwargs.get('front_screen_mode', int())
        self.hmd_video_source = kwargs.get('hmd_video_source', int())
        self.hmd_overlay_mode = kwargs.get('hmd_overlay_mode', int())
        self.hmd_enabled = kwargs.get('hmd_enabled', bool())

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
        if self.head_tracking_enabled != other.head_tracking_enabled:
            return False
        if self.voice_command_enabled != other.voice_command_enabled:
            return False
        if self.front_screen_enabled != other.front_screen_enabled:
            return False
        if self.front_screen_mode != other.front_screen_mode:
            return False
        if self.hmd_video_source != other.hmd_video_source:
            return False
        if self.hmd_overlay_mode != other.hmd_overlay_mode:
            return False
        if self.hmd_enabled != other.hmd_enabled:
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
    def head_tracking_enabled(self):
        """Message field 'head_tracking_enabled'."""
        return self._head_tracking_enabled

    @head_tracking_enabled.setter
    def head_tracking_enabled(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'head_tracking_enabled' field must be of type 'bool'"
        self._head_tracking_enabled = value

    @property
    def voice_command_enabled(self):
        """Message field 'voice_command_enabled'."""
        return self._voice_command_enabled

    @voice_command_enabled.setter
    def voice_command_enabled(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'voice_command_enabled' field must be of type 'bool'"
        self._voice_command_enabled = value

    @property
    def front_screen_enabled(self):
        """Message field 'front_screen_enabled'."""
        return self._front_screen_enabled

    @front_screen_enabled.setter
    def front_screen_enabled(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'front_screen_enabled' field must be of type 'bool'"
        self._front_screen_enabled = value

    @property
    def front_screen_mode(self):
        """Message field 'front_screen_mode'."""
        return self._front_screen_mode

    @front_screen_mode.setter
    def front_screen_mode(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'front_screen_mode' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'front_screen_mode' field must be an unsigned integer in [0, 255]"
        self._front_screen_mode = value

    @property
    def hmd_video_source(self):
        """Message field 'hmd_video_source'."""
        return self._hmd_video_source

    @hmd_video_source.setter
    def hmd_video_source(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'hmd_video_source' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'hmd_video_source' field must be an unsigned integer in [0, 255]"
        self._hmd_video_source = value

    @property
    def hmd_overlay_mode(self):
        """Message field 'hmd_overlay_mode'."""
        return self._hmd_overlay_mode

    @hmd_overlay_mode.setter
    def hmd_overlay_mode(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'hmd_overlay_mode' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'hmd_overlay_mode' field must be an unsigned integer in [0, 255]"
        self._hmd_overlay_mode = value

    @property
    def hmd_enabled(self):
        """Message field 'hmd_enabled'."""
        return self._hmd_enabled

    @hmd_enabled.setter
    def hmd_enabled(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'hmd_enabled' field must be of type 'bool'"
        self._hmd_enabled = value
