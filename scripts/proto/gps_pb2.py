# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: gps.proto
# Protobuf Python Version: 4.25.2
"""Generated protocol buffer code."""
from google.protobuf import descriptor as _descriptor
from google.protobuf import descriptor_pool as _descriptor_pool
from google.protobuf import symbol_database as _symbol_database
from google.protobuf.internal import builder as _builder
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor_pool.Default().AddSerializedFile(b'\n\tgps.proto\"\x8f\x03\n\x08GpsFrame\x12\x11\n\ttimestamp\x18\x01 \x01(\x04\x12\x0c\n\x04year\x18\x02 \x01(\r\x12\r\n\x05month\x18\x03 \x01(\r\x12\x0b\n\x03\x64\x61y\x18\x04 \x01(\r\x12\x0c\n\x04hour\x18\x05 \x01(\r\x12\x0b\n\x03min\x18\x06 \x01(\r\x12\x0b\n\x03sec\x18\x07 \x01(\r\x12\x13\n\x0bvalid_flags\x18\x08 \x01(\x04\x12\x10\n\x08num_sats\x18\t \x01(\r\x12\x0b\n\x03lon\x18\n \x01(\x02\x12\x0b\n\x03lat\x18\x0b \x01(\x02\x12\x0e\n\x06height\x18\x0c \x01(\x02\x12\x12\n\nheight_msl\x18\r \x01(\x02\x12\x16\n\x0e\x61\x63\x63uracy_horiz\x18\x0e \x01(\x02\x12\x19\n\x11\x61\x63\x63uracy_vertical\x18\x0f \x01(\x02\x12\x11\n\tvel_north\x18\x10 \x01(\x02\x12\x10\n\x08vel_east\x18\x11 \x01(\x02\x12\x10\n\x08vel_down\x18\x12 \x01(\x02\x12\x14\n\x0cground_speed\x18\x13 \x01(\x02\x12\x0b\n\x03hdg\x18\x14 \x01(\x02\x12\x16\n\x0e\x61\x63\x63uracy_speed\x18\x15 \x01(\x02\x12\x14\n\x0c\x61\x63\x63uracy_hdg\x18\x16 \x01(\x02\x62\x06proto3')

_globals = globals()
_builder.BuildMessageAndEnumDescriptors(DESCRIPTOR, _globals)
_builder.BuildTopDescriptorsAndMessages(DESCRIPTOR, 'gps_pb2', _globals)
if _descriptor._USE_C_DESCRIPTORS == False:
  DESCRIPTOR._options = None
  _globals['_GPSFRAME']._serialized_start=14
  _globals['_GPSFRAME']._serialized_end=413
# @@protoc_insertion_point(module_scope)
