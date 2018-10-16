# for now, we need to include prebaked modules
# - a node may be composed of multiple components
from typing import Dict, Iterator, Any, Optional, Tuple, Callable, Set
import logging

import attr
# import roslaunch

from .workspace import Workspace

logger = logging.getLogger(__name__)  # type: logging.Logger
logger.setLevel(logging.DEBUG)

FullName = str


class ParameterServer(object):
    def __init__(self) -> None:
        self.__contents = {}  # type: Dict[str, Any]

    def __getitem__(self, key: str) -> Any:
        return self.__contents[key]

    def __contains__(self, key: str) -> bool:
        return key in self.__contents

    def __setitem__(self, key: str, val: Any) -> None:
        self.__contents[key] = val


@attr.s
class Topic(object):
    name = attr.ib(type=str)
    publishers = attr.ib(type=Set[str], default=set())
    subscribers = attr.ib(type=Set[str], default=set())


class NodeContext(object):
    def __init__(self,
                 name: str,
                 params: ParameterServer,
                 topics: Dict[str, Topic]
                 ) -> None:
        self.__name = name
        self.__params = params
        self.__topics = topics

    def resolve(self, name: str) -> FullName:
        """
        Resolves a given name within the context of this node.

        Returns:
            the fully qualified form of a given name.
        """
        if name[0] == '/':
            return name
        elif name[0] == '~':
            return '/{}/{}'.join(self.__name, name)
        # FIXME
        else:
            return name

    def provide(self,
                service: str,
                fmt: str
                ) -> None:
        """
        Instructs the node to provide a service.
        """
        logger.debug("node [%s] provides service [%s] using format [%s]",
                     self.__name, service, fmt)

    def sub(self,
            topic: str,
            fmt: str
            ) -> None:
        """
        Subscribes the node to a given topic.

        Parameters:
            topic: the unqualified name of the topic.
            fmt: the message format used by the topic.
        """
        logger.debug("node [%s] subscribes to topic [%s] with format [%s]",
                     self.__name, topic, fmt)
        qualified_topic = self.resolve(topic)
        if qualified_topic not in self.__topics:
            self.__topics[qualified_topic] = Topic(qualified_topic)
        self.__topics[qualified_topic].subscribers.add(self.__name)

    def pub(self,
            topic: str,
            fmt: str
            ) -> None:
        """
        Instructs the node to publish to a given topic.

        Parameters:
            topic: the unqualified name of the topic.
            fmt: the message format used by the topic.
        """
        logger.debug("node [%s] publishes to topic [%s] with format [%s]",
                     self.__name, topic, fmt)

    def read(self,
             param: str,
             default: Optional[Any]
             ) -> Any:
        """
        Obtains the value of a given parameter from the parameter
        server.
        """
        logger.debug("node [%s] reads parameter [%s]",
                     self.__name, param)

        # FIXME
        return default


class Model(object):
    """
    Models the architectural interactions of a node type.
    """
    _models = {}  # type: Dict[Tuple[str, str], Model]

    @staticmethod
    def register(package: str,
                 name: str,
                 definition: Callable[[NodeContext], None]
                 ) -> None:
        key = (package, name)
        models = Model._models
        if key in models:
            m = "model [{}] already registered for package [{}]"
            m.format(name, package)
            raise Exception(m)
        models[key] = Model(package, name, definition)
        logger.debug("registered model [%s] for package [%s]",
                     name, package)

    @staticmethod
    def find(package: str, name: str) -> 'Model':
        return Model._models[(package, name)]

    def __init__(self,
                 package: str,
                 name: str,
                 definition: Callable[[NodeContext], None]
                 ) -> None:
        self.__package = package
        self.__name = name
        self.__definition = definition

    def eval(self, context: NodeContext) -> None:
        return self.__definition(context)


def model(package: str, name: str):
    def register(m: Callable[[NodeContext], None]):
        Model.register(package, name, m)
        return m
    return register


class VM(object):
    def __init__(self,
                 workspace: Workspace
                 ) -> None:
        self.__workspace = workspace
        self.__params = ParameterServer()
        self.__topics = {}  # type: Dict[str, Topic]

    @property
    def parameters(self) -> ParameterServer:
        return self.__params

    @property
    def topics(self) -> Iterator[Topic]:
        yield from self.__topics.values()

    # def nodes(self) -> Iterator[Node]:
    #     return

    def launch(self, fn: str) -> None:
        """
        Simulates the effects of `roslaunch` using a given launch file.
        """
        config = roslaunch.config.ROSLaunchConfig()
        loader = roslaunch.xmlloader.XmlLoader()
        loader.load(fn, config)

        for node in config.nodes:
            logger.debug("launching node: %s", node)

    def load(self,
             pkg: str,
             nodetype: str,
             name: str
             ) -> None:
        if nodetype == 'nodelet':
            raise Exception('nodelets are not currently supported.')

        try:
            model = Model.find(pkg, nodetype)
        except Exception:
            m = "failed to find model for node type [{}] in package [{}]"
            m = m.format(nodetype, pkg)
            raise Exception(m)

        ctx = NodeContext(name, self.__params, self.__topics)
        model.eval(ctx)
