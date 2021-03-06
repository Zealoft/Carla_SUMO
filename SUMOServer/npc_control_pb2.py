# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: npc_control.proto

import sys
_b=sys.version_info[0]<3 and (lambda x:x) or (lambda x:x.encode('latin1'))
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='npc_control.proto',
  package='',
  syntax='proto3',
  serialized_options=None,
  serialized_pb=_b('\n\x11npc_control.proto\".\n\x08Waypoint\x12\x10\n\x08Location\x18\x01 \x03(\x01\x12\x10\n\x08Rotation\x18\x02 \x03(\x01\"\x11\n\x0f\x63onnect_request\"C\n\x10\x63onnect_response\x12\x1b\n\x08init_pos\x18\x01 \x01(\x0b\x32\t.Waypoint\x12\x12\n\nvehicle_id\x18\x02 \x01(\t\"X\n\x0e\x61\x63tion_package\x12\x12\n\nvehicle_id\x18\x01 \x01(\t\x12\x1c\n\twaypoints\x18\x02 \x03(\x0b\x32\t.Waypoint\x12\x14\n\x0ctarget_speed\x18\x03 \x03(\x01\"Z\n\raction_result\x12\x12\n\nvehicle_id\x18\x01 \x01(\t\x12\x1e\n\x0b\x63urrent_pos\x18\x02 \x01(\x0b\x32\t.Waypoint\x12\x15\n\rcurrent_speed\x18\x03 \x03(\x01\"$\n\x0e\x65nd_connection\x12\x12\n\nvehicle_id\x18\x01 \x01(\t\"H\n\x12suspend_simulation\x12\x12\n\nvehicle_id\x18\x01 \x01(\t\x12\x1e\n\x0b\x63urrent_pos\x18\x02 \x01(\x0b\x32\t.Waypointb\x06proto3')
)




_WAYPOINT = _descriptor.Descriptor(
  name='Waypoint',
  full_name='Waypoint',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='Location', full_name='Waypoint.Location', index=0,
      number=1, type=1, cpp_type=5, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='Rotation', full_name='Waypoint.Rotation', index=1,
      number=2, type=1, cpp_type=5, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=21,
  serialized_end=67,
)


_CONNECT_REQUEST = _descriptor.Descriptor(
  name='connect_request',
  full_name='connect_request',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=69,
  serialized_end=86,
)


_CONNECT_RESPONSE = _descriptor.Descriptor(
  name='connect_response',
  full_name='connect_response',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='init_pos', full_name='connect_response.init_pos', index=0,
      number=1, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='vehicle_id', full_name='connect_response.vehicle_id', index=1,
      number=2, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=88,
  serialized_end=155,
)


_ACTION_PACKAGE = _descriptor.Descriptor(
  name='action_package',
  full_name='action_package',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='vehicle_id', full_name='action_package.vehicle_id', index=0,
      number=1, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='waypoints', full_name='action_package.waypoints', index=1,
      number=2, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='target_speed', full_name='action_package.target_speed', index=2,
      number=3, type=1, cpp_type=5, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=157,
  serialized_end=245,
)


_ACTION_RESULT = _descriptor.Descriptor(
  name='action_result',
  full_name='action_result',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='vehicle_id', full_name='action_result.vehicle_id', index=0,
      number=1, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='current_pos', full_name='action_result.current_pos', index=1,
      number=2, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='current_speed', full_name='action_result.current_speed', index=2,
      number=3, type=1, cpp_type=5, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=247,
  serialized_end=337,
)


_END_CONNECTION = _descriptor.Descriptor(
  name='end_connection',
  full_name='end_connection',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='vehicle_id', full_name='end_connection.vehicle_id', index=0,
      number=1, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=339,
  serialized_end=375,
)


_SUSPEND_SIMULATION = _descriptor.Descriptor(
  name='suspend_simulation',
  full_name='suspend_simulation',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='vehicle_id', full_name='suspend_simulation.vehicle_id', index=0,
      number=1, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='current_pos', full_name='suspend_simulation.current_pos', index=1,
      number=2, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=377,
  serialized_end=449,
)

_CONNECT_RESPONSE.fields_by_name['init_pos'].message_type = _WAYPOINT
_ACTION_PACKAGE.fields_by_name['waypoints'].message_type = _WAYPOINT
_ACTION_RESULT.fields_by_name['current_pos'].message_type = _WAYPOINT
_SUSPEND_SIMULATION.fields_by_name['current_pos'].message_type = _WAYPOINT
DESCRIPTOR.message_types_by_name['Waypoint'] = _WAYPOINT
DESCRIPTOR.message_types_by_name['connect_request'] = _CONNECT_REQUEST
DESCRIPTOR.message_types_by_name['connect_response'] = _CONNECT_RESPONSE
DESCRIPTOR.message_types_by_name['action_package'] = _ACTION_PACKAGE
DESCRIPTOR.message_types_by_name['action_result'] = _ACTION_RESULT
DESCRIPTOR.message_types_by_name['end_connection'] = _END_CONNECTION
DESCRIPTOR.message_types_by_name['suspend_simulation'] = _SUSPEND_SIMULATION
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

Waypoint = _reflection.GeneratedProtocolMessageType('Waypoint', (_message.Message,), {
  'DESCRIPTOR' : _WAYPOINT,
  '__module__' : 'npc_control_pb2'
  # @@protoc_insertion_point(class_scope:Waypoint)
  })
_sym_db.RegisterMessage(Waypoint)

connect_request = _reflection.GeneratedProtocolMessageType('connect_request', (_message.Message,), {
  'DESCRIPTOR' : _CONNECT_REQUEST,
  '__module__' : 'npc_control_pb2'
  # @@protoc_insertion_point(class_scope:connect_request)
  })
_sym_db.RegisterMessage(connect_request)

connect_response = _reflection.GeneratedProtocolMessageType('connect_response', (_message.Message,), {
  'DESCRIPTOR' : _CONNECT_RESPONSE,
  '__module__' : 'npc_control_pb2'
  # @@protoc_insertion_point(class_scope:connect_response)
  })
_sym_db.RegisterMessage(connect_response)

action_package = _reflection.GeneratedProtocolMessageType('action_package', (_message.Message,), {
  'DESCRIPTOR' : _ACTION_PACKAGE,
  '__module__' : 'npc_control_pb2'
  # @@protoc_insertion_point(class_scope:action_package)
  })
_sym_db.RegisterMessage(action_package)

action_result = _reflection.GeneratedProtocolMessageType('action_result', (_message.Message,), {
  'DESCRIPTOR' : _ACTION_RESULT,
  '__module__' : 'npc_control_pb2'
  # @@protoc_insertion_point(class_scope:action_result)
  })
_sym_db.RegisterMessage(action_result)

end_connection = _reflection.GeneratedProtocolMessageType('end_connection', (_message.Message,), {
  'DESCRIPTOR' : _END_CONNECTION,
  '__module__' : 'npc_control_pb2'
  # @@protoc_insertion_point(class_scope:end_connection)
  })
_sym_db.RegisterMessage(end_connection)

suspend_simulation = _reflection.GeneratedProtocolMessageType('suspend_simulation', (_message.Message,), {
  'DESCRIPTOR' : _SUSPEND_SIMULATION,
  '__module__' : 'npc_control_pb2'
  # @@protoc_insertion_point(class_scope:suspend_simulation)
  })
_sym_db.RegisterMessage(suspend_simulation)


# @@protoc_insertion_point(module_scope)
