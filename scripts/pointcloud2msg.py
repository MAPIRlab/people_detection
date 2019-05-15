#!/usr/bin/env python


## sensor_msgs/PointCloud2 to separated fields

import ctypes
import math
import struct
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
_DATATYPES = {}
_DATATYPES[PointField.INT8]    = ('b', 1)
_DATATYPES[PointField.UINT8]   = ('B', 1)
_DATATYPES[PointField.INT16]   = ('h', 2)
_DATATYPES[PointField.UINT16]  = ('H', 2)
_DATATYPES[PointField.INT32]   = ('i', 4)
_DATATYPES[PointField.UINT32]  = ('I', 4)
_DATATYPES[PointField.FLOAT32] = ('f', 4)
_DATATYPES[PointField.FLOAT64] = ('d', 8)

class PointCloudClass():

    def __init__(self, cloud, field_names = ["x", "y", "z"]):
        self._skip_nans = True
        self._field_names = field_names
        self._fmt = self._get_struct_fmt(cloud.is_bigendian, cloud.fields, self._field_names)
        self._width = cloud.width
        self._height = cloud.height
        self._point_step = cloud.point_step 
        self._row_step = cloud.row_step
        self._data = cloud.data
        self._isnan = math.isnan
        self._cloud_correct = True
        ##print self._width, self._height

        if self._width > 0 and self._height > 0:
            buff = struct.Struct(self._fmt)
            if buff.size >= 12:
                self._unpack_from = buff.unpack_from
            else:
                self._cloud_correct = False
                
        else:
            self._cloud_correct = False

    def read_point (self, u, v):
        points_read = []
        has_nan = False
        if self._cloud_correct:
            
            if u is not None and v is not None:

                p = self._unpack_from(self._data, (self._row_step * int(v)) + (self._point_step * int(u)))
                ##print p
                if len(p) == 0:
                    has_nan = True
                    return points_read, has_nan
                if self._skip_nans:
                    for pv in p:
                        if np.isnan(pv):
                            has_nan = True
                            break
                    if not has_nan:
                        points_read = p
                        
                else:
                    points_read = p
        else:
            has_nan = True
        return points_read, has_nan

    def _get_struct_fmt(self, is_bigendian, fields, field_names=None):
        fmt = '>' if is_bigendian else '<'

        offset = 0
        for field in (f for f in sorted(fields, key=lambda f: f.offset) if field_names is None or f.name in field_names):
            if offset < field.offset:
                fmt += 'x' * (field.offset - offset)
                offset = field.offset
            if field.datatype not in _DATATYPES:
                print >> sys.stderr, 'Skipping unknown PointField datatype [%d]' % field.datatype
            else:
                datatype_fmt, datatype_length = _DATATYPES[field.datatype]
                fmt    += field.count * datatype_fmt
                offset += field.count * datatype_length

        return fmt