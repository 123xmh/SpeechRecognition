# generated from rosidl_generator_py/resource/_idl.py.em
# with input from chest_interfaces:msg/EquipmentState.idl
# generated code does not contain a copyright notice


# Import statements for member types

# Member 'ammo_counts'
# Member 'state_values'
import array  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_EquipmentState(type):
    """Metaclass of message 'EquipmentState'."""

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
                'chest_interfaces.msg.EquipmentState')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__equipment_state
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__equipment_state
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__equipment_state
            cls._TYPE_SUPPORT = module.type_support_msg__msg__equipment_state
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__equipment_state

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class EquipmentState(metaclass=Metaclass_EquipmentState):
    """Message class 'EquipmentState'."""

    __slots__ = [
        '_ammo_types',
        '_ammo_counts',
        '_gimbal_pitch',
        '_gimbal_yaw',
        '_gimbal_is_active',
        '_longitude',
        '_latitude',
        '_altitude',
        '_ground_altitude',
        '_heading',
        '_speed',
        '_fuel_level',
        '_battery_level',
        '_roll',
        '_pitch',
        '_yaw',
        '_warnings',
        '_state_names',
        '_state_values',
    ]

    _fields_and_field_types = {
        'ammo_types': 'sequence<string>',
        'ammo_counts': 'sequence<uint8>',
        'gimbal_pitch': 'float',
        'gimbal_yaw': 'float',
        'gimbal_is_active': 'boolean',
        'longitude': 'double',
        'latitude': 'double',
        'altitude': 'double',
        'ground_altitude': 'double',
        'heading': 'float',
        'speed': 'float',
        'fuel_level': 'float',
        'battery_level': 'float',
        'roll': 'float',
        'pitch': 'float',
        'yaw': 'float',
        'warnings': 'sequence<string>',
        'state_names': 'sequence<string>',
        'state_values': 'sequence<float>',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.UnboundedString()),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('uint8')),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.UnboundedString()),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.UnboundedString()),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('float')),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.ammo_types = kwargs.get('ammo_types', [])
        self.ammo_counts = array.array('B', kwargs.get('ammo_counts', []))
        self.gimbal_pitch = kwargs.get('gimbal_pitch', float())
        self.gimbal_yaw = kwargs.get('gimbal_yaw', float())
        self.gimbal_is_active = kwargs.get('gimbal_is_active', bool())
        self.longitude = kwargs.get('longitude', float())
        self.latitude = kwargs.get('latitude', float())
        self.altitude = kwargs.get('altitude', float())
        self.ground_altitude = kwargs.get('ground_altitude', float())
        self.heading = kwargs.get('heading', float())
        self.speed = kwargs.get('speed', float())
        self.fuel_level = kwargs.get('fuel_level', float())
        self.battery_level = kwargs.get('battery_level', float())
        self.roll = kwargs.get('roll', float())
        self.pitch = kwargs.get('pitch', float())
        self.yaw = kwargs.get('yaw', float())
        self.warnings = kwargs.get('warnings', [])
        self.state_names = kwargs.get('state_names', [])
        self.state_values = array.array('f', kwargs.get('state_values', []))

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
        if self.ammo_types != other.ammo_types:
            return False
        if self.ammo_counts != other.ammo_counts:
            return False
        if self.gimbal_pitch != other.gimbal_pitch:
            return False
        if self.gimbal_yaw != other.gimbal_yaw:
            return False
        if self.gimbal_is_active != other.gimbal_is_active:
            return False
        if self.longitude != other.longitude:
            return False
        if self.latitude != other.latitude:
            return False
        if self.altitude != other.altitude:
            return False
        if self.ground_altitude != other.ground_altitude:
            return False
        if self.heading != other.heading:
            return False
        if self.speed != other.speed:
            return False
        if self.fuel_level != other.fuel_level:
            return False
        if self.battery_level != other.battery_level:
            return False
        if self.roll != other.roll:
            return False
        if self.pitch != other.pitch:
            return False
        if self.yaw != other.yaw:
            return False
        if self.warnings != other.warnings:
            return False
        if self.state_names != other.state_names:
            return False
        if self.state_values != other.state_values:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @property
    def ammo_types(self):
        """Message field 'ammo_types'."""
        return self._ammo_types

    @ammo_types.setter
    def ammo_types(self, value):
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
                 all(isinstance(v, str) for v in value) and
                 True), \
                "The 'ammo_types' field must be a set or sequence and each value of type 'str'"
        self._ammo_types = value

    @property
    def ammo_counts(self):
        """Message field 'ammo_counts'."""
        return self._ammo_counts

    @ammo_counts.setter
    def ammo_counts(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'B', \
                "The 'ammo_counts' array.array() must have the type code of 'B'"
            self._ammo_counts = value
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
                 all(isinstance(v, int) for v in value) and
                 all(val >= 0 and val < 256 for val in value)), \
                "The 'ammo_counts' field must be a set or sequence and each value of type 'int' and each unsigned integer in [0, 255]"
        self._ammo_counts = array.array('B', value)

    @property
    def gimbal_pitch(self):
        """Message field 'gimbal_pitch'."""
        return self._gimbal_pitch

    @gimbal_pitch.setter
    def gimbal_pitch(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'gimbal_pitch' field must be of type 'float'"
        self._gimbal_pitch = value

    @property
    def gimbal_yaw(self):
        """Message field 'gimbal_yaw'."""
        return self._gimbal_yaw

    @gimbal_yaw.setter
    def gimbal_yaw(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'gimbal_yaw' field must be of type 'float'"
        self._gimbal_yaw = value

    @property
    def gimbal_is_active(self):
        """Message field 'gimbal_is_active'."""
        return self._gimbal_is_active

    @gimbal_is_active.setter
    def gimbal_is_active(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'gimbal_is_active' field must be of type 'bool'"
        self._gimbal_is_active = value

    @property
    def longitude(self):
        """Message field 'longitude'."""
        return self._longitude

    @longitude.setter
    def longitude(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'longitude' field must be of type 'float'"
        self._longitude = value

    @property
    def latitude(self):
        """Message field 'latitude'."""
        return self._latitude

    @latitude.setter
    def latitude(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'latitude' field must be of type 'float'"
        self._latitude = value

    @property
    def altitude(self):
        """Message field 'altitude'."""
        return self._altitude

    @altitude.setter
    def altitude(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'altitude' field must be of type 'float'"
        self._altitude = value

    @property
    def ground_altitude(self):
        """Message field 'ground_altitude'."""
        return self._ground_altitude

    @ground_altitude.setter
    def ground_altitude(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'ground_altitude' field must be of type 'float'"
        self._ground_altitude = value

    @property
    def heading(self):
        """Message field 'heading'."""
        return self._heading

    @heading.setter
    def heading(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'heading' field must be of type 'float'"
        self._heading = value

    @property
    def speed(self):
        """Message field 'speed'."""
        return self._speed

    @speed.setter
    def speed(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'speed' field must be of type 'float'"
        self._speed = value

    @property
    def fuel_level(self):
        """Message field 'fuel_level'."""
        return self._fuel_level

    @fuel_level.setter
    def fuel_level(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'fuel_level' field must be of type 'float'"
        self._fuel_level = value

    @property
    def battery_level(self):
        """Message field 'battery_level'."""
        return self._battery_level

    @battery_level.setter
    def battery_level(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'battery_level' field must be of type 'float'"
        self._battery_level = value

    @property
    def roll(self):
        """Message field 'roll'."""
        return self._roll

    @roll.setter
    def roll(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'roll' field must be of type 'float'"
        self._roll = value

    @property
    def pitch(self):
        """Message field 'pitch'."""
        return self._pitch

    @pitch.setter
    def pitch(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'pitch' field must be of type 'float'"
        self._pitch = value

    @property
    def yaw(self):
        """Message field 'yaw'."""
        return self._yaw

    @yaw.setter
    def yaw(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'yaw' field must be of type 'float'"
        self._yaw = value

    @property
    def warnings(self):
        """Message field 'warnings'."""
        return self._warnings

    @warnings.setter
    def warnings(self, value):
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
                 all(isinstance(v, str) for v in value) and
                 True), \
                "The 'warnings' field must be a set or sequence and each value of type 'str'"
        self._warnings = value

    @property
    def state_names(self):
        """Message field 'state_names'."""
        return self._state_names

    @state_names.setter
    def state_names(self, value):
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
                 all(isinstance(v, str) for v in value) and
                 True), \
                "The 'state_names' field must be a set or sequence and each value of type 'str'"
        self._state_names = value

    @property
    def state_values(self):
        """Message field 'state_values'."""
        return self._state_values

    @state_values.setter
    def state_values(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'f', \
                "The 'state_values' array.array() must have the type code of 'f'"
            self._state_values = value
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
                 True), \
                "The 'state_values' field must be a set or sequence and each value of type 'float'"
        self._state_values = array.array('f', value)
