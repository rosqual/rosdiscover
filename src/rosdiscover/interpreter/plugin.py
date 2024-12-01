# -*- coding: utf-8 -*-
__all__ = ('ModelPlugin',)

import abc
import typing

from loguru import logger

from . import NodeContext

if typing.TYPE_CHECKING:
    from .interpreter import Interpreter


class ModelPlugin(abc.ABC):
    """Models the architectural effects of a dynamically-loaded node plugin
    (e.g., a Gazebo plugin)."""
    @abc.abstractmethod
    def load(self, interpreter: 'Interpreter', context: NodeContext) -> None:
        """Simulates the effects of loading this plugin in a given context."""
        ...


class DynamicPlugin(type, ModelPlugin):
    """
    Models dynamically loaded plugins, taking in a name of the plygin that
    will be recovered when the plugin is loaded
    """

    def load(self, interpreter: 'Interpreter', context: NodeContext) -> None:
        logger.warning("Dynamic loading of plugins for {name} not implemented")


def generate_dynamic_plugin(plugin_name: str) -> type:
    return type(f"{plugin_name}DynamicPlugin", (DynamicPlugin), {'plugin_name': plugin_name})
