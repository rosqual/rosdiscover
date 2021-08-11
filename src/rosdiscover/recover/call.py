# -*- coding: utf-8 -*-
import abc
import typing as t

import attr

from .symbolic import SymbolicStatement, SymbolicString


class SymbolicRosApiCall(abc.ABC, SymbolicStatement):
    """Represents a symbolic call to a ROS API."""


@attr.s(frozen=True, auto_attribs=True, slots=True)
class RosInit(SymbolicRosApiCall):
    name: SymbolicString


@attr.s(frozen=True, auto_attribs=True, slots=True)
class Publisher(SymbolicRosApiCall):
    topic: SymbolicString
    format_: str


@attr.s(frozen=True, auto_attribs=True, slots=True)
class Subscriber(SymbolicRosApiCall):
    topic: SymbolicString
    format_: str


@attr.s(frozen=True, auto_attribs=True, slots=True)
class ServiceProvider(SymbolicRosApiCall):
    service: SymbolicString
    format_: str


@attr.s(frozen=True, auto_attribs=True, slots=True)
class ServiceCaller(SymbolicRosApiCall):
    service: SymbolicValue
    format_: str


@attr.s(frozen=True, auto_attribs=True, slots=True)
class WriteParam(SymbolicRosApiCall):
    param: SymbolicString
    value: SymbolicValue


@attr.s(frozen=True, auto_attribs=True, slots=True)
class ReadParam(SymbolicRosApiCall):
    param: SymbolicString
    value: SymbolicValue


@attr.s(frozen=True, auto_attribs=True, slots=True)
class ReadParamWithDefault(SymbolicRosApiCall):
    param: SymbolicString
    value: SymbolicValue
    default: SymbolicValue


@attr.s(frozen=True, auto_attribs=True, slots=True)
class HasParam(SymbolicRosApiCall):
    param: SymbolicString
