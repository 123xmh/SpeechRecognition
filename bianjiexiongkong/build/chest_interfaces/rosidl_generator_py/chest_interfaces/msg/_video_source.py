# generated from rosidl_generator_py/resource/_idl.py.em
# with input from chest_interfaces:msg/VideoSource.idl
# generated code does not contain a copyright notice


# Import statements for member types

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_VideoSource(type):
    """Metaclass of message 'VideoSource'."""

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
                'chest_interfaces.msg.VideoSource')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__video_source
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__video_source
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__video_source
            cls._TYPE_SUPPORT = module.type_support_msg__msg__video_source
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__video_source

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


class VideoSource(metaclass=Metaclass_VideoSource):
    """Message class 'VideoSource'."""

    __slots__ = [
        '_header',
        '_id',
        '_ip_address',
        '_source_name',
        '_fps',
        '_width',
        '_height',
        '_encoding',
        '_is_available',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'id': 'uint8',
        'ip_address': 'string',
        'source_name': 'string',
        'fps': 'uint32',
        'width': 'uint32',
        'height': 'uint32',
        'encoding': 'string',
        'is_available': 'boolean',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.id = kwargs.get('id', int())
        self.ip_address = kwargs.get('ip_address', str())
        self.source_name = kwargs.get('source_name', str())
        self.fps = kwargs.get('fps', int())
        self.width = kwargs.get('width', int())
        self.height = kwargs.get('height', int())
        self.encoding = kwargs.get('encoding', str())
        self.is_available = kwargs.get('is_available', bool())

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
        if self.id != other.id:
            return False
        if self.ip_address != other.ip_address:
            return False
        if self.source_name != other.source_name:
            return False
        if self.fps != other.fps:
            return False
        if self.width != other.width:
            return False
        if self.height != other.height:
            return False
        if self.encoding != other.encoding:
            return False
        if self.is_available != other.is_available:
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

    @property  # noqa: A003
    def id(self):  # noqa: A003
        """Message field 'id'."""
        return self._id

    @id.setter  # noqa: A003
    def id(self, value):  # noqa: A003
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'id' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'id' field must be an unsigned integer in [0, 255]"
        self._id = value

    @property
    def ip_address(self):
        """Message field 'ip_address'."""
        return self._ip_address

    @ip_address.setter
    def ip_address(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'ip_address' field must be of type 'str'"
        self._ip_address = value

    @property
    def source_name(self):
        """Message field 'source_name'."""
        return self._source_name

    @source_name.setter
    def source_name(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'source_name' field must be of type 'str'"
        self._source_name = value

    @property
    def fps(self):
        """Message field 'fps'."""
        return self._fps

    @fps.setter
    def fps(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'fps' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'fps' field must be an unsigned integer in [0, 4294967295]"
        self._fps = value

    @property
    def width(self):
        """Message field 'width'."""
        return self._width

    @width.setter
    def width(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'width' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'width' field must be an unsigned integer in [0, 4294967295]"
        self._width = value

    @property
    def height(self):
        """Message field 'height'."""
        return self._height

    @height.setter
    def height(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'height' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'height' field must be an unsigned integer in [0, 4294967295]"
        self._height = value

    @property
    def encoding(self):
        """Message field 'encoding'."""
        return self._encoding

    @encoding.setter
    def encoding(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'encoding' field must be of type 'str'"
        self._encoding = value

    @property
    def is_available(self):
        """Message field 'is_available'."""
        return self._is_available

    @is_available.setter
    def is_available(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'is_available' field must be of type 'bool'"
        self._is_available = value
